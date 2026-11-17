// The rules of going home while mowing and the deadline of the sunset.
#include <cstdio>
#include <string>
#include <utility>

#include "mowbite_logic/going_home_part.h"
#include "mowbite_time/sun.h"

namespace mowbite_logic {

Reason GoingHomePart::HomeWhy(CoreFacts core, HomeReason reason) const {
  char text[160];
  switch (reason) {
    case HomeReason::kBatteryCritical:
      std::snprintf(text, sizeof(text), "the battery at %.1f V stayed below %.1f V for %.0f s",
                    core.hw().battery_voltage, core.config().battery_critical_voltage,
                    core.config().battery_critical_time);
      return Reason("BATTERY_CRITICAL", text)
          .With("battery_voltage", core.hw().battery_voltage)
          .With("limit", core.config().battery_critical_voltage);
    case HomeReason::kBatteryHot:
      std::snprintf(text, sizeof(text), "the battery at %d °C, %.0f °C or more", core.hw().battery_temperature,
                    core.config().battery_hot_temperature);
      return Reason("BATTERY_HOT", text)
          .With("temperature", static_cast<int>(core.hw().battery_temperature))
          .With("limit", core.config().battery_hot_temperature);
    case HomeReason::kBatteryLow:
      std::snprintf(text, sizeof(text), "the battery stayed below %.1f V for %.0f s (at most %.1f V)",
                    core.config().battery_empty_voltage, core.config().battery_low_window, home_.WindowVoltage());
      return Reason("BATTERY_LOW", text)
          .With("battery_voltage", home_.WindowVoltage())
          .With("limit", core.config().battery_empty_voltage);
    case HomeReason::kMotorHot:
      std::snprintf(text, sizeof(text), "the blade motor at %d °C, %.0f °C or more",
                    core.hw().blade.temperature, core.config().motor_hot_temperature);
      return Reason("MOTOR_HOT", text)
          .With("temperature", static_cast<int>(core.hw().blade.temperature))
          .With("limit", core.config().motor_hot_temperature);
    case HomeReason::kRain:
    default:
      std::snprintf(text, sizeof(text), "rain for %.0f s", core.config().rain_check_seconds);
      return Reason("RAIN", text);
  }
}

void GoingHomePart::SetSunsetSettings(CoreFacts core, bool dock_at_sunset) {
  sunset_on_ = dock_at_sunset;
  if (!core.SunsetWanted()) {
    sunset_due_.reset();
    sunset_tried_ = sunset_told_ = false;
  }
  sunset_try_ = true;
}

void GoingHomePart::TrySunset(CoreAccess core, double now_s) {
  sunset_try_ = false;
  if (!core.SunsetWanted() || sunset_due_) return;
  const bool first = !std::exchange(sunset_tried_, true);
  std::string missing;
  if (!core.clock_safe()) missing = "the clock is not safe (" + core.clock_reason() + ")";
  else if (!core.datum()) missing = "no datum";
  if (!missing.empty()) {
    if (first) {
      sunset_told_ = true;
      core.Notify("No going home at sunset yet: " + missing);
    }
    return;
  }
  // The first sunset after now; before it lies daylight, whenever the run started.
  const double unix_s = now_s + core.clock_offset();
  for (const double day : {unix_s, unix_s + 86400.0}) {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): missing is set above when datum_ is empty
    const mowbite_time::SunDay sun = mowbite_time::SunTimes(core.datum()->first, core.datum()->second, day);
    if (sun.set && *sun.set > unix_s) {
      sunset_due_ = *sun.set - core.clock_offset();
      if (std::exchange(sunset_told_, false)) core.Notify("Going home at sunset after all");
      return;
    }
  }
  // No sunset within two days (polar day or night): the next clock tries again; until then the run saw no light to
  // lose, like a start in the dark.
}

}  // namespace mowbite_logic
