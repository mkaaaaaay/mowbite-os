#include "mowbite_net/notifier.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mowbite_api/catalog.h"
#include "mowbite_net/texts.h"

namespace mowbite_net {

namespace {

constexpr int kPrioEmergency = 5;
constexpr int kPrioProblem = 4;
constexpr int kPrioInfo = 3;
constexpr int kPrioClear = 2;

// The kinds the app shows as a problem, after the catalog.
bool IsProblem(const std::string& kind) {
  const auto& events = mowbite_api::NotifyEvents();
  return std::any_of(events.begin(), events.end(), [&](const auto& e) { return e.key == kind && e.problem; });
}

std::string Minutes(double seconds) {
  return std::to_string(static_cast<int64_t>(std::max(0.0, seconds) / 60.0));
}

std::string Text(const json& event, const char* key) {
  const auto it = event.find(key);
  return it != event.end() && it->is_string() ? it->get<std::string>() : "";
}

bool PushWord(const std::string& what) {
  for (const auto& word : mowbite_api::ScheduleLogWords()) {
    if (word.word == what) return word.push;
  }
  return false;
}

}  // namespace

void Notifier::SetRules(const NotifyRules& rules) {
  rules_ = rules;
}

std::string Notifier::Name() const {
  return rules_.name.empty() ? Say(rules_.lang, "mower") : rules_.name;
}

void Notifier::Emit(const std::string& kind, const std::string& phase, int prio, const std::string& tag,
                    const std::string& title, const std::string& message, const std::string& path) {
  out_.push_back(Push{kind, phase, IsProblem(kind), prio, tag, title, message, path});
}

void Notifier::Once(const std::string& key, const std::string& kind, const std::string& tag, const std::string& title,
                    const std::string& message) {
  if (Wanted(key)) Emit(kind, "", kPrioInfo, tag, title, message);
}

bool Notifier::OpenProblem(const std::string& key, const std::string& variant, double since, double now, bool reminds,
                           const std::string& title, const std::string& message, int prio, const std::string& tag) {
  if (!Wanted(key)) return false;
  if (open_ && (open_->key == "emergency" || (open_->key != "standstill" && key != "emergency"))) return false;
  open_ = Open{key, variant, since, now, reminds};
  Emit(key, "new", prio, tag, title, message);
  return true;
}

void Notifier::Clear(const std::string& title, const std::string& message) {
  if (!open_) return;
  Emit(open_->key, "clear", kPrioClear, "white_check_mark", title, message);
  open_.reset();
}

void Notifier::OnEvent(const json& event, double mono) {
  if (!event.is_object()) return;
  const std::string type = Text(event, "type");
  const std::string reason = Text(event, "reason");
  const std::string& lang = rules_.lang;
  if (type == "STATE") {
    OnStateEvent(event);
  } else if (type == "MOW_MOTOR_SPINUP_FAILED") {
    spinup_at_ = mono;
    // With emergency picked, its text says it.
    if (!Wanted("emergency")) Once("spinup", "spinup", "warning", Say(lang, "spinup.title"), Say(lang, "spinup", N()));
  } else if (type == "DOCKING_FAILED") {
    // Not charging in the dock, could not set off for it, or gave up after its tries.
    const std::string id = reason == "NO_CHARGE_CURRENT"   ? "not_charging"
                           : !event.contains("attempts") ? "cant_go_home"
                                                         : "dock_failed";
    const Vars vars = {{"n", Name()}, {"reason", ReasonName(lang, type, reason, Text(event, "message"))}};
    OpenProblem("dock_failed", id, mono, mono, id != "not_charging", Say(lang, id + ".title"),
                Say(lang, id + ".new", vars), kPrioProblem, "warning");
  } else if (type == "CHARGING_LOST") {
    const auto pct = event.find("battery_percentage");
    const Vars vars = {{"n", Name()},
                       {"pct", pct != event.end() && pct->is_number() ? std::to_string(pct->get<int64_t>()) : "?"}};
    OpenProblem("dock_failed", "charging_lost", mono, mono, false, Say(lang, "not_charging.title"),
                Say(lang, reason == "NO_DOCK_CONTACT" ? "charging_lost.contact" : "charging_lost.power", vars),
                kPrioProblem, "warning");
  } else if (type == "DOCKED") {
    if (open_ && open_->key == "dock_failed") {
      const bool charging = open_->variant == "not_charging" || open_->variant == "charging_lost";
      Clear(Say(lang, charging ? "charging_again.title" : "back_in_dock.title"), Say(lang, "charging_again", N()));
    }
  } else if (type == "UNDOCKING_FAILED") {
    const bool no_gps = reason == "no_gps";
    const std::string id = no_gps ? "no_gps" : "undock_failed";
    OpenProblem("undock_failed", id, mono, mono, true, Say(lang, id + ".title"), Say(lang, id + ".new", N()),
                kPrioProblem, "warning");
  } else if (type == "UNDOCKED") {
    if (open_ && open_->key == "undock_failed") Clear(Say(lang, "on_its_way.title"), Say(lang, "undocked", N()));
  } else if (type == "JOB_COMPLETE") {
    Once("done", "done", "seedling", Say(lang, "done.title"), Say(lang, "done", N()));
  } else if (type == "DOCKING") {
    if (reason == "RAIN") {
      Once("rain", "rain", "cloud_with_rain", Say(lang, "rain.title"), Say(lang, "rain.home", N()));
    } else if (reason == "BATTERY_LOW" || reason == "BATTERY_CRITICAL") {
      Once("battery", "battery", "battery", Say(lang, "battery.title"), Say(lang, "battery.home", N()));
    } else if (reason == "DARK") {
      Once("dark", "dark", "city_sunset", Say(lang, "dark.title"), Say(lang, "dark.home", N()));
    }
  } else if (type == "BATTERY_LOW") {
    const auto volt = event.find("battery_voltage");
    char text[16] = "?";
    if (volt != event.end() && volt->is_number()) std::snprintf(text, sizeof(text), "%.1f", volt->get<double>());
    Once("battery", "battery", "battery", Say(lang, "battery.title"),
         Say(lang, "battery.stays", {{"n", Name()}, {"volt", text}}));
  } else if (type == "MOWING_REFUSED") {
    if (reason == "BUSY" || reason == "STILL_STOPPING") return;
    Once("refused", "refused", "warning", Say(lang, "refused.title"),
         Say(lang, "refused", {{"n", Name()}, {"reason", ReasonName(lang, type, reason, Text(event, "message"))}}));
  } else if (type == "MAP_DATUM") {
    Once("refused", "refused", "warning", Say(lang, "map_datum.title"),
         Say(lang, "map_datum", {{"n", Name()}, {"reason", ReasonName(lang, type, reason, Text(event, "message"))}}));
  }
}

void Notifier::OnStateEvent(const json& event) {
  const std::string state = Text(event, "state");
  if (state != "PAUSED") {
    paused_reason_.clear();
    return;
  }
  const std::string reason = Text(event, "reason");
  paused_reason_ = reason;
  if (Text(event, "resume") != "user") return;
  const std::string& lang = rules_.lang;
  const Vars why = {{"n", Name()}, {"reason", ReasonName(lang, "STATE", reason, Text(event, "message"))}};
  if (reason == "RAIN") {
    Once("rain", "rain", "cloud_with_rain", Say(lang, "rain.title"), Say(lang, "rain.stopped", N()));
  } else if (reason == "DARK") {
    Once("dark", "dark", "city_sunset", Say(lang, "dark.title"), Say(lang, "dark.stopped", N()));
  } else if (reason == "BLADE_REFUSED") {
    Once("spinup", "spinup", "warning", Say(lang, "spinup.title"), Say(lang, "paused", why));
  } else if (reason != "MANUAL" && reason != "EMERGENCY" && reason != "BLADE_SPINUP_FAILED" &&
             reason != "SCHEDULE") {
    Once("nav_error", "nav_error", "warning", Say(lang, "paused.title"), Say(lang, "paused", why));
  }
}

void Notifier::OnState(const StateView& state) {
  const std::string& lang = rules_.lang;
  if (state.emergency) {
    if (!emergency_since_) emergency_since_ = state.mono;
    if ((!open_ || open_->key != "emergency") && state.mono - *emergency_since_ >= rules_.emergency_wait_s &&
        !emergency_told_) {
      std::string message = Say(lang, "emergency.new", N());
      if (spinup_at_ && *spinup_at_ >= *emergency_since_ - kSpinupBefore) message += Say(lang, "emergency.spinup");
      // Reminders count from now, their minutes from the start of the emergency.
      emergency_told_ = OpenProblem("emergency", "emergency", *emergency_since_, state.mono, true,
                                    Say(lang, "emergency.title"), message, kPrioEmergency, "rotating_light");
    }
  } else {
    emergency_since_.reset();
    emergency_told_ = false;
    if (open_ && open_->key == "emergency") {
      Clear(Say(lang, "emergency.clear.title"), Say(lang, "emergency.clear", N()));
    }
  }
  const bool changed = !last_ || last_->state != state.state;
  if (open_ && open_->key == "dock_failed" && changed) {
    if (open_->variant == "charging_lost" ? state.state != "IDLE"
                                          : state.state == "UNDOCKING" || state.state == "MOWING") {
      Close();
    }
  }
  if (open_ && open_->key == "undock_failed") {
    if (changed && state.state == "MOWING") {
      Clear(Say(lang, "on_its_way.title"), Say(lang, "undocked", N()));
    } else if (open_->variant != "no_gps" && state.charging && last_ && !last_->charging) {
      Close();
    }
  }
  Remind(state);
  Standstill(state);
  last_ = state;
}

void Notifier::Remind(const StateView& state) {
  if (!open_ || !open_->reminds || rules_.remind_s <= 0) return;
  if (state.mono - open_->told < rules_.remind_s || state.wall < rules_.snooze_until) return;
  open_->told = state.mono;
  const Vars vars = {{"n", Name()}, {"min", Minutes(state.mono - open_->since)}};
  const std::string& v = open_->variant;
  // cant_go_home and dock_failed share the text of a reminder.
  const std::string id = v == "emergency" || v == "no_gps" || v == "undock_failed" ? v : "dock_failed";
  Emit(open_->key, "again", kPrioProblem, open_->key == "emergency" ? "rotating_light" : "warning",
       Say(rules_.lang, v == "cant_go_home" ? "cant_go_home.title" : id + ".title"),
       Say(rules_.lang, id + ".again", vars));
}

void Notifier::Standstill(const StateView& state) {
  const std::string& lang = rules_.lang;
  const bool moved = state.pose && still_since_ && std::hypot(state.x - still_x_, state.y - still_y_) > kStandstillMove;
  if (open_ && open_->key == "standstill") {
    if (state.charging) {
      Clear(Say(lang, "charging_again.title"), Say(lang, "charging_again", N()));
    } else if (moved) {
      Clear(Say(lang, "on_its_way.title"), Say(lang, "moving_again", N()));
    }
  }
  const bool counts = !state.emergency && state.state != "AREA_RECORDING" && !state.charging &&
                      !(state.state == "PAUSED" && paused_reason_ == "MANUAL");
  if (!counts) {
    still_since_.reset();
    still_told_ = false;
    return;
  }
  if (!still_since_ || moved) {
    still_since_ = state.mono;
    still_told_ = false;
    still_x_ = state.x;
    still_y_ = state.y;
    still_pose_ = state.pose;
    return;
  }
  if (!still_pose_ && state.pose) {
    still_x_ = state.x;
    still_y_ = state.y;
    still_pose_ = true;
  }
  if (still_told_ || open_ || state.mono - *still_since_ < kStandstillAfter) return;
  std::string message = Say(lang, "standstill.new", {{"n", Name()}, {"min", Minutes(state.mono - *still_since_)}});
  if (state.state == "PAUSED") {
    message += Say(lang, "standstill.paused", {{"reason", ReasonName(lang, "STATE", paused_reason_)}});
  } else if (state.state == "IDLE") {
    message += Say(lang, "standstill.idle");
  } else {
    message += Say(lang, "standstill.stuck");
  }
  still_told_ = true;
  OpenProblem("standstill", "standstill", state.mono, state.mono, false, Say(lang, "standstill.title"), message,
              kPrioProblem, "warning");
}

void Notifier::OnParts(const std::vector<PartView>& parts) {
  for (const PartView& part : parts) {
    if (!part.due) {
      maintenance_told_.erase(part.key);
      continue;
    }
    const auto told = maintenance_told_.find(part.key);
    if (told != maintenance_told_.end() && told->second == part.due_h) continue;
    if (!Wanted("maintenance")) continue;
    maintenance_told_[part.key] = part.due_h;
    const std::string kind = "maintenance_" + part.key;
    Emit(kind, "", kPrioInfo, part.key == "blade" ? "wrench" : "broom", Say(rules_.lang, kind + ".title"),
         Say(rules_.lang, kind,
             {{"n", Name()}, {"hours", std::to_string(static_cast<int64_t>(std::floor(part.used_h)))}}));
  }
}

void Notifier::OnScheduleChanged() {
  Emit("schedule_changed", "", kPrioInfo, "calendar", Say(rules_.lang, "schedule_changed.title"),
       Say(rules_.lang, "schedule_changed", N()));
}

void Notifier::OnScheduleLogged(const std::string& what, const std::string& at) {
  if (!PushWord(what)) return;
  Once("schedule_skipped", "schedule_skipped", "calendar", Say(rules_.lang, "schedule_skipped.title"),
       Say(rules_.lang, "schedule_skipped",
           {{"n", Name()}, {"at", at.empty() ? "?" : at}, {"reason", Say(rules_.lang, what)}}));
}

void Notifier::OnNotifyChanged() {
  Emit("notify_changed", "", kPrioInfo, "bell", Say(rules_.lang, "notify_changed.title"),
       Say(rules_.lang, "notify_changed", N()), "/settings/");
}

std::vector<Push> Notifier::Take() {
  std::vector<Push> out;
  out.swap(out_);
  return out;
}

}  // namespace mowbite_net
