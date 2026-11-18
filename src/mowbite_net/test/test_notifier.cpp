#include <gtest/gtest.h>

#include "mowbite_api/catalog.h"
#include "mowbite_net/notifier.h"
#include "mowbite_net/notify_config.h"

using namespace mowbite_net;

namespace {

NotifyRules All(const std::string& lang = "en") {
  NotifyRules rules;
  rules.lang = lang;
  rules.name = "Robi";
  for (const char* key : {"emergency", "dock_failed", "undock_failed", "nav_error", "spinup", "standstill", "refused",
                          "done", "rain", "battery", "dark", "maintenance", "schedule_skipped"}) {
    rules.events.insert(key);
  }
  rules.remind_s = 1800;
  rules.emergency_wait_s = 10;
  return rules;
}

StateView At(double s, const std::string& state = "IDLE", bool emergency = false, bool charging = false, double x = 0) {
  StateView view;
  view.wall = 1794564000 + s;
  view.mono = s;
  view.state = state;
  view.emergency = emergency;
  view.charging = charging;
  view.pose = true;
  view.x = x;
  return view;
}

Notifier Make(const NotifyRules& rules = All()) {
  Notifier n;
  n.SetRules(rules);
  return n;
}

}  // namespace

TEST(Notifier, EmergencyAfterTheWaitWithReminderAndClear) {
  Notifier n = Make();
  n.OnState(At(0, "MOWING", true, false));
  n.OnState(At(9, "MOWING", true, false));
  EXPECT_TRUE(n.Take().empty());
  n.OnState(At(10, "MOWING", true, false));
  std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, "emergency");
  EXPECT_EQ(out[0].phase, "new");
  EXPECT_EQ(out[0].prio, 5);
  EXPECT_EQ(out[0].message, "Robi is in emergency stop.");
  n.OnState(At(1809, "MOWING", true, false));
  EXPECT_TRUE(n.Take().empty());
  n.OnState(At(1810, "MOWING", true, false));
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].phase, "again");
  EXPECT_EQ(out[0].prio, 4);
  EXPECT_EQ(out[0].message, "Robi is still in emergency stop (for 30 min).");
  n.OnState(At(1811, "IDLE", false, false));
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].phase, "clear");
  EXPECT_EQ(out[0].prio, 2);
  EXPECT_EQ(out[0].title, "Emergency stop cleared");
}

TEST(Notifier, AShortEmergencyIsNotTold) {
  Notifier n = Make();
  n.OnState(At(0, "DOCKING", true));
  n.OnState(At(5, "DOCKING", false));
  n.OnState(At(20, "DOCKING", false));
  EXPECT_TRUE(n.Take().empty());
}

TEST(Notifier, SpinupGoesIntoTheEmergency) {
  Notifier n = Make();
  n.OnEvent({{"type", "MOW_MOTOR_SPINUP_FAILED"}}, 0);
  n.OnState(At(5, "MOWING", true));
  n.OnState(At(15, "MOWING", true));
  const std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "Robi is in emergency stop. The mow motor didn't start.");
  NotifyRules rules = All();
  rules.events.erase("emergency");
  Notifier alone = Make(rules);
  alone.OnEvent({{"type", "MOW_MOTOR_SPINUP_FAILED"}}, 0);
  ASSERT_EQ(alone.Take().size(), 1u);
}

TEST(Notifier, SnoozeHoldsReminders) {
  NotifyRules rules = All();
  rules.snooze_until = 1794564000 + 4000;
  Notifier n = Make(rules);
  n.OnEvent({{"type", "UNDOCKING_FAILED"}, {"reason", "path_failed"}}, 0);
  EXPECT_EQ(n.Take().size(), 1u);
  n.OnState(At(1800, "IDLE"));
  EXPECT_TRUE(n.Take().empty());
  n.OnState(At(4000, "IDLE"));
  const std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "Robi still hasn't got out of the dock (for 66 min).");
}

TEST(Notifier, DockFailedVariantsAndTheirEnd) {
  Notifier n = Make();
  n.OnEvent({{"type", "DOCKING_FAILED"}, {"reason", "approach_failed"}, {"attempts", 3}}, 0);
  std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].title, "Docking failed");
  n.OnEvent({{"type", "DOCKED"}}, 5);
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].title, "Back in the dock");
  EXPECT_EQ(out[0].message, "Robi is charging again.");

  n.OnEvent({{"type", "DOCKING_FAILED"}, {"reason", "NO_POSITION"}}, 10);
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].title, "Can't go home");
  n.OnState(At(11, "IDLE"));
  n.OnState(At(12, "MOWING"));
  EXPECT_EQ(n.open_problem(), "");

  n.OnEvent({{"type", "CHARGING_LOST"}, {"reason", "NO_DOCK_CONTACT"}, {"battery_percentage", 64}}, 20);
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "Robi lost contact with the docking station and isn't charging (64 %).");
  n.OnState(At(1900, "IDLE"));
  EXPECT_TRUE(n.Take().empty()) << "no reminder for not charging";
  n.OnEvent({{"type", "DOCKED"}}, 1901);
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].title, "Charging again");
}

TEST(Notifier, OneProblemAtATime) {
  Notifier n = Make();
  n.OnEvent({{"type", "UNDOCKING_FAILED"}, {"reason", "no_gps"}}, 0);
  n.OnEvent({{"type", "DOCKING_FAILED"}, {"reason", "approach_failed"}, {"attempts", 3}}, 1);
  std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].title, "No GPS after undocking");
  // An emergency replaces any.
  n.OnState(At(2, "IDLE", true));
  n.OnState(At(12, "IDLE", true));
  EXPECT_EQ(n.open_problem(), "emergency");
  // Its clear leaves none open: the problem it replaced is gone.
  n.OnState(At(13, "IDLE", false, true));
  EXPECT_EQ(n.open_problem(), "");
}

TEST(Notifier, StandstillOnceThenItMoves) {
  Notifier n = Make();
  n.OnState(At(0, "MOWING", false, false, 0));
  n.OnState(At(300, "MOWING", false, false, 0.4));
  n.OnState(At(599, "MOWING", false, false, 0.3));
  EXPECT_TRUE(n.Take().empty());
  n.OnState(At(600, "MOWING", false, false, 0.3));
  std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "Robi has been standing still for 10 min and isn't charging. Not getting on, maybe stuck.");
  n.OnState(At(5000, "MOWING", false, false, 0.3));
  EXPECT_TRUE(n.Take().empty());
  n.OnState(At(5001, "MOWING", false, false, 1.0));
  out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].message, "Robi is moving again.");
}

TEST(Notifier, NoStandstillWhileChargingRecordingOrPausedByHand) {
  Notifier n = Make();
  n.OnState(At(0, "IDLE", false, true));
  n.OnState(At(700, "IDLE", false, true));
  n.OnState(At(701, "AREA_RECORDING"));
  n.OnState(At(1400, "AREA_RECORDING"));
  n.OnEvent({{"type", "STATE"}, {"state", "PAUSED"}, {"reason", "MANUAL"}, {"resume", "user"}}, 1401);
  n.OnState(At(1401, "PAUSED"));
  n.OnState(At(2100, "PAUSED"));
  EXPECT_TRUE(n.Take().empty());
}

TEST(Notifier, AnObstacleIsToldAfterAMinuteAPersonAfterFive) {
  const auto held = [](double s, const char* kind) {
    StateView view = At(s, "PAUSED");
    view.pause_reason = "OBSTACLE";
    view.pause_kind = kind;
    return view;
  };
  Notifier n = Make();
  n.OnEvent({{"type", "STATE"}, {"state", "PAUSED"}, {"reason", "OBSTACLE"}, {"kind", "OBJECT"}, {"resume", "auto"}},
            0);
  n.OnState(held(0, "OBJECT"));
  n.OnState(held(59, "OBJECT"));
  EXPECT_TRUE(n.Take().empty());
  n.OnState(held(60, "OBJECT"));
  std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, "standstill");
  EXPECT_EQ(out[0].message,
            "Robi has been standing still for 1 min and isn't charging. Paused: Obstacle in the way.");

  Notifier living = Make();
  living.OnState(held(0, "PERSON"));
  living.OnState(held(299, "PERSON"));
  EXPECT_TRUE(living.Take().empty());
  living.OnState(held(300, "PERSON"));
  EXPECT_EQ(living.Take().size(), 1u);
}

TEST(Notifier, PausedForAReason) {
  Notifier n = Make(All("de"));
  n.OnEvent({{"type", "STATE"}, {"state", "PAUSED"}, {"reason", "RAIN"}, {"resume", "user"}}, 0);
  n.OnEvent({{"type", "STATE"}, {"state", "PAUSED"}, {"reason", "OUTSIDE"}, {"resume", "user"},
             {"message", "outside"}},
            1);
  n.OnEvent({{"type", "STATE"}, {"state", "PAUSED"}, {"reason", "NO_POSITION"}, {"resume", "auto"}}, 2);
  n.OnEvent({{"type", "STATE"}, {"state", "PAUSED"}, {"reason", "MANUAL"}, {"resume", "user"}}, 3);
  const std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].kind, "rain");
  EXPECT_EQ(out[0].message, "Robi hat wegen Regen angehalten und wartet an Ort und Stelle, allein kann er nicht heim.");
  EXPECT_EQ(out[1].kind, "nav_error");
  EXPECT_TRUE(out[1].problem);
  EXPECT_EQ(out[1].prio, 3);
  EXPECT_EQ(out[1].title, "Pausiert, wartet");
}

TEST(Notifier, OnceEvents) {
  Notifier n = Make();
  n.OnEvent({{"type", "JOB_COMPLETE"}}, 0);
  n.OnEvent({{"type", "DOCKING"}, {"reason", "BATTERY_LOW"}}, 0);
  n.OnEvent({{"type", "DOCKING"}, {"reason", "DARK"}}, 0);
  n.OnEvent({{"type", "DOCKING"}, {"reason", "MANUAL"}}, 0);
  n.OnEvent({{"type", "BATTERY_LOW"}, {"battery_voltage", 21.04}}, 0);
  n.OnEvent({{"type", "MOWING_REFUSED"}, {"reason", "BUSY"}}, 0);
  n.OnEvent({{"type", "MOWING_REFUSED"}, {"reason", "SOMETHING_NEW"}}, 0);
  const std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 5u);
  EXPECT_EQ(out[0].title, "Done mowing");
  EXPECT_EQ(out[1].message, "Robi is going home to charge.");
  EXPECT_EQ(out[2].title, "Sunset");
  EXPECT_EQ(out[3].message, "Robi is low on battery (21.0 V) and can't go home, it stays where it is.");
  EXPECT_EQ(out[4].message, "Robi didn't start mowing: something new.");
}

TEST(Notifier, OnlyPickedEvents) {
  NotifyRules rules = All();
  rules.events = {"done"};
  Notifier n = Make(rules);
  n.OnEvent({{"type", "DOCKING"}, {"reason", "RAIN"}}, 0);
  n.OnEvent({{"type", "UNDOCKING_FAILED"}, {"reason", "no_gps"}}, 0);
  n.OnState(At(0, "IDLE", true));
  n.OnState(At(20, "IDLE", true));
  EXPECT_TRUE(n.Take().empty());
  EXPECT_EQ(n.open_problem(), "");
  // These two always go.
  n.OnScheduleChanged();
  n.OnNotifyChanged();
  EXPECT_EQ(n.Take().size(), 2u);
}

TEST(Notifier, MaintenanceOnceUntilSeenToOrChanged) {
  Notifier n = Make();
  n.OnParts({{"blade", 80.4, 80, true}, {"clean", 80.4, 200, false}});
  std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, "maintenance_blade");
  EXPECT_EQ(out[0].message, "Robi has mowed 80 h since the blades were last changed.");
  n.OnParts({{"blade", 81, 80, true}});
  EXPECT_TRUE(n.Take().empty());
  n.OnParts({{"blade", 81, 75, true}});
  EXPECT_EQ(n.Take().size(), 1u);
  Notifier again = Make();
  again.RestoreMaintenance(n.maintenance_told());
  again.OnParts({{"blade", 81, 75, true}});
  EXPECT_TRUE(again.Take().empty()) << "not again after a restart";
  n.OnParts({{"blade", 0, 75, false}});
  EXPECT_TRUE(n.maintenance_told().empty());
}

TEST(Notifier, ScheduleSkipsMarkedForPushOnly) {
  Notifier n = Make(All("de"));
  n.OnScheduleLogged("skip_clock", "08:30");
  n.OnScheduleLogged("skip_rain", "09:00");
  n.OnScheduleLogged("started", "10:00");
  const std::vector<Push> out = n.Take();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, "schedule_skipped");
  EXPECT_EQ(out[0].message, "Robi ist um 08:30 nicht losgefahren: die Uhr war nicht sicher.");
}

TEST(Notifier, EveryEventKeyHasItsWay) {
  // Each key of the catalog is one notify.set takes and the notifier knows.
  NotifyConfig config = DefaultConfig();
  json all = json::array();
  for (const auto& key : All().events) all.push_back(key);
  EXPECT_FALSE(ApplySet({{"events", all}}, &config));
}

TEST(Notifier, EveryPushWordAndPartHasItsTexts) {
  for (const char* lang : {"de", "en"}) {
    for (const auto& word : mowbite_api::ScheduleLogWords()) {
      if (word.push) {
        EXPECT_NE(Say(lang, word.word), word.word) << lang << " " << word.word;
      }
    }
    for (const char* part : {"maintenance_blade", "maintenance_clean"}) {
      EXPECT_NE(Say(lang, part), part) << lang;
      EXPECT_NE(Say(lang, std::string(part) + ".title"), std::string(part) + ".title") << lang;
    }
  }
}
