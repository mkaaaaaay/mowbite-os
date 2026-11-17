#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mowbite_stats/incident_keeper.h"

namespace mowbite_stats {
namespace {

constexpr int64_t kS = 1000000000;

Trigger Make(const std::string& kind, int64_t t_s, bool safety = false) {
  Trigger trigger;
  trigger.kind = kind;
  trigger.safety = safety;
  trigger.t_ns = t_s * kS;
  trigger.utc = "2026-11-15T12:00:00Z";
  return trigger;
}

TEST(TriggerOfEvent, OnlyTheEventsThatSetOffAnIncident) {
  const auto docking = TriggerOfEvent(json{{"type", "DOCKING_FAILED"}, {"reason", "dock_failed"}}, 0, "u");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->kind, "DOCKING_FAILED");
  EXPECT_EQ(docking->reason, "dock_failed");
  EXPECT_FALSE(docking->safety);

  const auto restart = TriggerOfEvent(json{{"type", "UNEXPECTED_RESTART"}, {"reason", "OS_CRASH"}}, 0, "u");
  ASSERT_TRUE(restart);
  EXPECT_TRUE(restart->safety);
  EXPECT_EQ(TriggerOfEvent(json{{"type", "POSITION_JUMP"}, {"reason", "GAP"}}, 0, "u")->kind, "POSITION_JUMP");
  EXPECT_EQ(TriggerOfEvent(json{{"type", "STATE"}, {"reason", "BLADE_REFUSED"}}, 0, "u")->kind, "BLADE_REFUSED");
  EXPECT_EQ(TriggerOfEvent(json{{"type", "MOWING_REFUSED"}, {"reason", "FOLLOWER_MISSING"}}, 0, "u")->kind,
            "FOLLOWER_MISSING");

  EXPECT_FALSE(TriggerOfEvent(json{{"type", "STATE"}, {"reason", "MANUAL"}}, 0, "u"));
  EXPECT_FALSE(TriggerOfEvent(json{{"type", "DOCKING_RETRY"}, {"reason", "dock_failed"}}, 0, "u"));
  EXPECT_FALSE(TriggerOfEvent(json{{"type", 3}, {"reason", 4}}, 0, "u"));
  EXPECT_FALSE(TriggerOfEvent(json::array(), 0, "u"));
  // Emergencies come from hw/status.
  EXPECT_FALSE(TriggerOfEvent(json{{"type", "EMERGENCY"}, {"emergency", true}, {"reason", 5}}, 0, "u"));
}

TEST(EmergencyTrigger, EveryRisingBit) {
  const auto first = EmergencyTrigger(0, 5, 7, "u");
  ASSERT_TRUE(first);
  EXPECT_EQ(first->kind, "EMERGENCY");
  EXPECT_EQ(first->reason, "LATCH, STOP");
  EXPECT_TRUE(first->safety);
  EXPECT_EQ(first->t_ns, 7);
  // A further bit while others stay set.
  const auto second = EmergencyTrigger(5, 7, 8, "u");
  ASSERT_TRUE(second);
  EXPECT_EQ(second->detail["rising"], 2);
  EXPECT_FALSE(EmergencyTrigger(7, 7, 9, "u"));
  EXPECT_FALSE(EmergencyTrigger(7, 1, 9, "u"));
  EXPECT_FALSE(EmergencyTrigger(0, 0, 9, "u"));
}

TEST(IncidentKeeper, ATriggerOpensAWindowLaterOnesGrowIt) {
  IncidentKeeper keeper{IncidentLimits{}};
  const std::string id = keeper.OnTrigger(Make("DOCKING_FAILED", 1000));
  EXPECT_EQ(id, "20261115T120000Z");
  const Incident* incident = keeper.Find(id);
  ASSERT_NE(incident, nullptr);
  EXPECT_EQ(incident->from_ns, 940 * kS);
  EXPECT_EQ(incident->to_ns, 1015 * kS);

  EXPECT_EQ(keeper.OnTrigger(Make("POSITION_JUMP", 1010)), id);
  EXPECT_EQ(keeper.Find(id)->to_ns, 1025 * kS);
  EXPECT_EQ(keeper.Find(id)->triggers.size(), 2u);
  EXPECT_TRUE(keeper.OnTick(1025 * kS).empty());

  // Growing ends max_s after the first trigger.
  for (int64_t t = 1020; t <= 1700; t += 10) keeper.OnTrigger(Make("POSITION_JUMP", t));
  EXPECT_EQ(keeper.Find(id)->to_ns, 1600 * kS);
}

TEST(IncidentKeeper, OverWindowsGoToSealingOnce) {
  IncidentKeeper keeper{IncidentLimits{}};
  const std::string id = keeper.OnTrigger(Make("EMERGENCY", 100, true));
  EXPECT_TRUE(keeper.OnTick(115 * kS).empty());
  EXPECT_EQ(keeper.OnTick(116 * kS), std::vector<std::string>{id});
  EXPECT_TRUE(keeper.OnTick(117 * kS).empty());
  ASSERT_NE(keeper.Find(id), nullptr);
  EXPECT_EQ(keeper.Find(id)->state, IncidentState::kSealing);
  EXPECT_FALSE(keeper.collecting());
  keeper.Sealed(id);
  EXPECT_EQ(keeper.Find(id), nullptr);
}

TEST(IncidentKeeper, ATriggerAfterTheWindowSealsTheLastOneAndOpensANew) {
  IncidentKeeper keeper{IncidentLimits{}};
  const std::string first = keeper.OnTrigger(Make("EMERGENCY", 100, true));
  Trigger later = Make("EMERGENCY", 200, true);
  later.utc = "2026-11-15T12:01:40Z";
  const std::string second = keeper.OnTrigger(later);
  EXPECT_NE(first, second);
  EXPECT_EQ(keeper.OnTick(201 * kS), std::vector<std::string>{first});
  EXPECT_EQ(keeper.Find(second)->state, IncidentState::kCollecting);
}

TEST(IncidentKeeper, KindsThatAreNoSafetyAreThrottled) {
  IncidentLimits limits;
  limits.kind_per_day = 3;
  IncidentKeeper keeper{limits};
  EXPECT_FALSE(keeper.OnTrigger(Make("DOCKING_FAILED", 0)).empty());
  keeper.OnTick(20 * kS);
  // Within kind_gap_s of the last opening of its kind.
  EXPECT_TRUE(keeper.OnTrigger(Make("DOCKING_FAILED", 30)).empty());
  EXPECT_EQ(keeper.throttled().at("DOCKING_FAILED"), 1u);
  // Another kind is not.
  EXPECT_FALSE(keeper.OnTrigger(Make("POSITION_JUMP", 31)).empty());
  keeper.OnTick(50 * kS);
  // Safety never is, however often.
  for (int64_t t = 60; t < 200; t += 20) {
    EXPECT_FALSE(keeper.OnTrigger(Make("EMERGENCY", t, true)).empty()) << t;
    keeper.OnTick((t + 16) * kS);
  }
  EXPECT_FALSE(keeper.OnTrigger(Make("DOCKING_FAILED", 300)).empty());
  keeper.OnTick(400 * kS);
  EXPECT_FALSE(keeper.OnTrigger(Make("DOCKING_FAILED", 500)).empty());
  keeper.OnTick(600 * kS);
  // The fourth in 24 h.
  EXPECT_TRUE(keeper.OnTrigger(Make("DOCKING_FAILED", 700)).empty());
  // A day after the first ones they count no more.
  EXPECT_FALSE(keeper.OnTrigger(Make("DOCKING_FAILED", 86400 + 301)).empty());
}

TEST(IncidentKeeper, AMarkKeepsItsOwnTimeBefore) {
  IncidentKeeper keeper{IncidentLimits{}};
  Trigger mark = Make("MARK", 1000);
  mark.before_s = 300;
  const std::string id = keeper.OnTrigger(mark);
  EXPECT_EQ(keeper.Find(id)->from_ns, 700 * kS);
}

TEST(IncidentKeeper, AMarkInAnOpenWindowMovesItsBeginning) {
  IncidentKeeper keeper{IncidentLimits{}};
  const std::string id = keeper.OnTrigger(Make("DOCKING_FAILED", 1000));
  Trigger mark = Make("MARK", 1010);
  mark.before_s = 300;
  EXPECT_EQ(keeper.OnTrigger(mark), id);
  EXPECT_EQ(keeper.Find(id)->from_ns, 710 * kS);
  EXPECT_EQ(keeper.Find(id)->start_ns, 1000 * kS);
  EXPECT_EQ(keeper.Find(id)->to_ns, 1025 * kS);
}

TEST(IncidentKeeper, AnOrdinaryTriggerKeepsTheBeginning) {
  IncidentKeeper keeper{IncidentLimits{}};
  Trigger mark = Make("MARK", 1000);
  mark.before_s = 300;
  const std::string id = keeper.OnTrigger(mark);
  EXPECT_EQ(keeper.OnTrigger(Make("POSITION_JUMP", 1010)), id);
  EXPECT_EQ(keeper.Find(id)->from_ns, 700 * kS);
  EXPECT_EQ(keeper.OnTrigger(Make("DOCKING_FAILED", 1020)), id);
  EXPECT_EQ(keeper.Find(id)->from_ns, 700 * kS);
}

TEST(IncidentKeeper, IdsDifferFromThoseOnTheDisk) {
  IncidentKeeper keeper{IncidentLimits{}};
  keeper.Known("20261115T120000Z");
  const std::string first = keeper.OnTrigger(Make("EMERGENCY", 0, true));
  EXPECT_EQ(first, "20261115T120000Z-2");
  keeper.OnTick(100 * kS);
  EXPECT_EQ(keeper.OnTrigger(Make("EMERGENCY", 200, true)), "20261115T120000Z-3");
  Trigger no_clock = Make("EMERGENCY", 400, true);
  no_clock.utc.clear();
  keeper.OnTick(300 * kS);
  EXPECT_EQ(keeper.OnTrigger(no_clock), "incident");
}

TEST(IncidentKeeper, TriggersBeyondTheLimitAreCounted) {
  IncidentKeeper keeper{IncidentLimits{}};
  const std::string id = keeper.OnTrigger(Make("EMERGENCY", 0, true));
  for (size_t i = 0; i < kMaxTriggers + 5; ++i) keeper.OnTrigger(Make("EMERGENCY", 1, true));
  EXPECT_EQ(keeper.Find(id)->triggers.size(), kMaxTriggers);
  EXPECT_EQ(keeper.Find(id)->more, 6u);
}

StoredIncident Stored(const std::string& id, double created_s, bool safety, std::vector<StoredFile> files,
                      bool open = false) {
  StoredIncident incident;
  incident.id = id;
  incident.created_s = created_s;
  incident.safety = safety;
  incident.open = open;
  incident.files = std::move(files);
  return incident;
}

TEST(IncidentsToRemove, ExpiredOnesGoBySafety) {
  IncidentLimits limits;
  const double day = 86400;
  const double now = 100 * day;
  const auto removed = IncidentsToRemove({Stored("a", now - 31 * day, false, {}), Stored("b", now - 31 * day, true, {}),
                                          Stored("c", now - 91 * day, true, {}), Stored("d", now - 29 * day, false, {}),
                                          Stored("e", now - 200 * day, false, {}, true)},
                                         now, limits);
  EXPECT_EQ(removed, (std::vector<std::string>{"a", "c"}));
}

TEST(IncidentsToRemove, OverTheBudgetTheOldestNoSafetyFirst) {
  IncidentLimits limits;
  limits.max_mb = 3;
  // Inode 1 is shared and counts once: 5 MB in all.
  const std::vector<StoredIncident> stored = {
      Stored("old-safety", 10, true, {{1, 1000000}, {2, 1000000}}),
      Stored("old", 20, false, {{1, 1000000}, {3, 1000000}}),
      Stored("new", 30, false, {{4, 1000000}}),
      Stored("open", 5, false, {{5, 1000000}}, true),
  };
  // "old" frees only inode 3 (1 stays with old-safety), "new" frees 4: 3 MB.
  EXPECT_EQ(IncidentsToRemove(stored, 40, limits), (std::vector<std::string>{"old", "new"}));
  limits.max_mb = 1;
  EXPECT_EQ(IncidentsToRemove(stored, 40, limits), (std::vector<std::string>{"old", "new", "old-safety"}));
}

TEST(IncidentsToRemove, AtMostMaxCount) {
  IncidentLimits limits;
  limits.max_count = 2;
  const auto removed = IncidentsToRemove(
      {Stored("s", 1, true, {}), Stored("a", 2, false, {}), Stored("b", 3, false, {}), Stored("c", 4, false, {})}, 5,
      limits);
  EXPECT_EQ(removed, (std::vector<std::string>{"a", "b"}));
}

}  // namespace
}  // namespace mowbite_stats
