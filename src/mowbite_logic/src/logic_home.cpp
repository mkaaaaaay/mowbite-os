#include "mowbite_logic/logic_core.h"

#include <cstdio>
#include <string>
#include <vector>

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
  const std::vector<HomeReason> due = home_.Due();
  if (due.empty()) return;
  const auto battery = [](HomeReason r) { return r == HomeReason::kBatteryCritical || r == HomeReason::kBatteryLow; };
  // A pause that waits for the user never goes home: the user may stand at the mower (G11).
  const bool waits_for_user = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  if (!waits_for_user && HomeReachable()) {
    Reason why = HomeWhy(due.front());
    why.message = "Going home: " + why.message;
    EndJob(false, true, why);
    return;
  }
  for (HomeReason reason : due) {
    if (battery(reason)) {
      BatteryEndsMowing(reason);
      return;
    }
  }
  // Without a way home heat and rain change nothing yet (O11, question 47 iii); told once per run.
  const int bit = 1 << static_cast<int>(due.front());
  if (waits_for_user || (home_told_ & bit) != 0) return;
  home_told_ |= bit;
  Notify("Mowing goes on, it cannot go home: " + HomeWhy(due.front()).message);
}

}  // namespace mowbite_logic
