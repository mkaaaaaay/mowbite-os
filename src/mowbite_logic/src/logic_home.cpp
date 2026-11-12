#include "mowbite_logic/logic_core.h"

#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_time/sun.h"

namespace mowbite_logic {

HomeSample HomeSampleOf(const HardwareSnapshot& hw, double now_s) {
  HomeSample sample;
  sample.time = now_s;
  sample.telemetry_ok = hw.telemetry_ok;
  sample.battery_ok = hw.battery_ok;
  sample.battery_voltage = hw.battery_voltage;
  sample.battery_temperature = hw.battery_temperature;
  sample.blade_ok = hw.blade.ok;
  sample.blade_temperature = hw.blade.temperature;
  sample.rain = hw.rain;
  return sample;
}

HomeConfig LogicCore::HomeRules() const {
  HomeConfig rules;
  rules.critical_voltage = config_.battery_critical_voltage;
  rules.critical_time = config_.battery_critical_time;
  rules.empty_voltage = config_.battery_empty_voltage;
  rules.low_window = config_.battery_low_window;
  rules.battery_hot_temperature = config_.battery_hot_temperature;
  rules.motor_hot_temperature = config_.motor_hot_temperature;
  rules.rain_mode = config_.rain_mode;
  rules.rain_check_seconds = config_.rain_check_seconds;
  return rules;
}

void LogicCore::SetHomeSettings(int rain_mode, double rain_check_seconds, double motor_hot_temperature,
                                double motor_cold_temperature) {
  config_.rain_mode = rain_mode;
  config_.rain_check_seconds = rain_check_seconds;
  config_.motor_hot_temperature = motor_hot_temperature;
  config_.motor_cold_temperature = motor_cold_temperature;
  home_.SetConfig(HomeRules());
}

Reason LogicCore::HomeWhy(HomeReason reason) const {
  char text[160];
  switch (reason) {
    case HomeReason::kBatteryCritical:
      std::snprintf(text, sizeof(text), "the battery at %.1f V stayed below %.1f V for %.0f s",
                    hw_.battery_voltage, config_.battery_critical_voltage, config_.battery_critical_time);
      return Reason("BATTERY_CRITICAL", text)
          .With("battery_voltage", hw_.battery_voltage)
          .With("limit", config_.battery_critical_voltage);
    case HomeReason::kBatteryHot:
      std::snprintf(text, sizeof(text), "the battery at %d °C, %.0f °C or more", hw_.battery_temperature,
                    config_.battery_hot_temperature);
      return Reason("BATTERY_HOT", text)
          .With("temperature", static_cast<int>(hw_.battery_temperature))
          .With("limit", config_.battery_hot_temperature);
    case HomeReason::kBatteryLow:
      std::snprintf(text, sizeof(text), "the battery stayed below %.1f V for %.0f s (at most %.1f V)",
                    config_.battery_empty_voltage, config_.battery_low_window, home_.WindowVoltage());
      return Reason("BATTERY_LOW", text)
          .With("battery_voltage", home_.WindowVoltage())
          .With("limit", config_.battery_empty_voltage);
    case HomeReason::kMotorHot:
      std::snprintf(text, sizeof(text), "the blade motor at %d °C, %.0f °C or more",
                    hw_.blade.temperature, config_.motor_hot_temperature);
      return Reason("MOTOR_HOT", text)
          .With("temperature", static_cast<int>(hw_.blade.temperature))
          .With("limit", config_.motor_hot_temperature);
    case HomeReason::kRain:
    default:
      std::snprintf(text, sizeof(text), "rain for %.0f s", config_.rain_check_seconds);
      return Reason("RAIN", text);
  }
}

void LogicCore::BatteryEndsMowing(HomeReason reason) {
  const bool critical = reason == HomeReason::kBatteryCritical;
  const double voltage = critical ? hw_.battery_voltage : home_.WindowVoltage();
  const double limit = critical ? config_.battery_critical_voltage : config_.battery_empty_voltage;
  AddEvent("BATTERY_LOW", {{"battery_voltage", voltage}, {"limit", limit}});
  Notify("Mowing ends: the battery is low and the mower cannot go home");
  EndJob(false, false);
}

void LogicCore::WatchHome(double now_s) {
  home_.OnSample(HomeSampleOf(hw_, now_s));
  if (sunset_try_) TrySunset(now_s);
  ActOnHome(sunset_due_ && now_s >= *sunset_due_ ? DarkWhy(now_s) : Reason());
}

void LogicCore::ActOnHome(const Reason& dark) {
  const std::vector<HomeReason> due = home_.Due();
  if (due.empty() && dark.empty()) return;
  const Reason first = due.empty() ? dark : HomeWhy(due.front());
  // A pause that waits for the user never goes home: the user may stand at the mower (G11).
  const bool waits_for_user = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  if (!waits_for_user && HomeReachable()) {
    Reason why = first;
    why.message = "Going home: " + why.message;
    EndJob(false, true, why);
    return;
  }
  for (HomeReason reason : due) {
    if (reason == HomeReason::kBatteryCritical || reason == HomeReason::kBatteryLow) {
      BatteryEndsMowing(reason);
      return;
    }
  }
  // A pause for GPS turns into this one in the tick it ends; any other pause stays as it is.
  if (pause_ != 0) return;
  Reason why = first;
  why.message = "It cannot go home: " + why.message;
  Pause(kPauseFault, why);
}

Reason LogicCore::ContinueProblem() const {
  if (HomeReachable()) return {};
  for (HomeReason reason : home_.Due()) {
    if (reason == HomeReason::kRain || reason == HomeReason::kBatteryHot || reason == HomeReason::kMotorHot) {
      Reason why = HomeWhy(reason);
      why.message = "Not while it cannot go home: " + why.message;
      return why;
    }
  }
  return {};
}

void LogicCore::ResumedByUser() {
  sunset_due_.reset();
  sunset_try_ = true;
  ActOnHome(Reason());
}

void LogicCore::SetSunsetSettings(bool dock_at_sunset) {
  if (!dock_at_sunset) {
    sunset_due_.reset();
    sunset_tried_ = sunset_told_ = false;
  }
  sunset_on_ = dock_at_sunset;
  sunset_try_ = true;
}

void LogicCore::OnClock(bool safe, const std::string& reason, double unix_minus_mono_s) {
  clock_safe_ = safe;
  clock_reason_ = reason;
  clock_offset_ = unix_minus_mono_s;
  sunset_try_ = true;
}

void LogicCore::OnDatum(double latitude, double longitude) {
  datum_ = std::make_pair(latitude, longitude);
  sunset_try_ = true;
}

void LogicCore::TrySunset(double now_s) {
  sunset_try_ = false;
  if (!sunset_on_ || sunset_due_) return;
  const bool first = !std::exchange(sunset_tried_, true);
  std::string missing;
  if (!clock_safe_) missing = "the clock is not safe (" + clock_reason_ + ")";
  else if (!datum_) missing = "no datum";
  if (!missing.empty()) {
    if (first) {
      sunset_told_ = true;
      Notify("No going home at sunset yet: " + missing);
    }
    return;
  }
  // The first sunset after now; before it lies daylight, whenever the run started.
  const double unix_s = now_s + clock_offset_;
  for (const double day : {unix_s, unix_s + 86400.0}) {
    const mowbite_time::SunDay sun = mowbite_time::SunTimes(datum_->first, datum_->second, day);
    if (sun.set && *sun.set > unix_s) {
      sunset_due_ = *sun.set - clock_offset_;
      if (std::exchange(sunset_told_, false)) Notify("Going home at sunset after all");
      return;
    }
  }
  // No sunset within two days (polar day): the next clock tries again.
}

Reason LogicCore::DarkWhy(double now_s) const {
  const double elevation = mowbite_time::SunElevation(datum_->first, datum_->second, now_s + clock_offset_);
  return Reason("DARK", "the sun has set")
      .With("sun_elevation", elevation)
      .With("limit", mowbite_time::kSunsetElevation);
}

}  // namespace mowbite_logic
