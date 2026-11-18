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
  home_.watch().SetConfig(HomeRules());
}

void LogicCore::BatteryEndsMowing(HomeReason reason) {
  const bool critical = reason == HomeReason::kBatteryCritical;
  const double voltage = critical ? hw_.battery_voltage : home_.watch().WindowVoltage();
  const double limit = critical ? config_.battery_critical_voltage : config_.battery_empty_voltage;
  AddEvent("BATTERY_LOW", {{"battery_voltage", voltage}, {"limit", limit}});
  RunEnds("BATTERY_LOW");
  Notify("Mowing ends: the battery is low and the mower cannot go home");
  EndJob(false, false);
}

void LogicCore::WatchHome(double now_s) {
  home_.watch().OnSample(HomeSampleOf(hw_, now_s));
  if (home_.sunset_try()) TrySunset(now_s);
  ActOnHome(DeadlineDue(now_s));
}

void LogicCore::ActOnHome(const Reason& dark) {
  const std::vector<HomeReason> due = home_.watch().Due();
  if (due.empty() && dark.empty()) return;
  const Reason first = due.empty() ? dark : HomeWhy(due.front());
  // A pause that waits for the user or for an obstacle never goes home: someone may stand at the mower (G11, G45).
  const bool keeps_place = (mowing_.pause() & kPauseKeepsPlace) != 0;
  if (!keeps_place && HomeReachable()) {
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
  if (mowing_.pause() != 0) return;
  Reason why = first;
  why.message = "It cannot go home: " + why.message;
  Pause(kPauseFault, why);
}

Reason LogicCore::ContinueProblem() const {
  if (HomeReachable()) return {};
  for (HomeReason reason : home_.watch().Due()) {
    if (reason == HomeReason::kRain || reason == HomeReason::kBatteryHot || reason == HomeReason::kMotorHot) {
      Reason why = HomeWhy(reason);
      why.message = "Not while it cannot go home: " + why.message;
      return why;
    }
  }
  return {};
}

void LogicCore::ResumedByUser() {
  auto& active = schedule_.run();
  // The user took over: deadlines that came are dropped, those to come stay.
  if (active && active->end_passed) active->end_due.reset();
  home_.Resumed();
  ActOnHome(Reason());
}

void LogicCore::OnClock(bool safe, const std::string& reason, double unix_minus_mono_s) {
  clock_safe_ = safe;
  clock_reason_ = reason;
  clock_offset_ = unix_minus_mono_s;
  home_.TryAgain();
}

void LogicCore::OnDatum(double latitude, double longitude) {
  datum_ = std::make_pair(latitude, longitude);
  home_.TryAgain();
}

Reason LogicCore::DarkWhy(double now_s) const {
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): sunset_due_ is set only with datum_, which is never cleared
  const double elevation = mowbite_time::SunElevation(datum_->first, datum_->second, now_s + clock_offset_);
  return Reason("DARK", "the sun has set")
      .With("sun_elevation", elevation)
      .With("limit", mowbite_time::kSunsetElevation);
}

}  // namespace mowbite_logic
