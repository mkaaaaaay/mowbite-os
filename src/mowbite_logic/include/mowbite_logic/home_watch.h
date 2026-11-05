#ifndef MOWBITE_LOGIC_HOME_WATCH_H
#define MOWBITE_LOGIC_HOME_WATCH_H

#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace mowbite_logic {

// Why the mower goes home by itself, in the order the first one counts (MANUAL, a command, comes before all).
enum class HomeReason { kBatteryCritical, kBatteryHot, kBatteryLow, kMotorHot, kRain };

// The rules of HomeWatch. A temperature counts from its limit on; 0 for battery_hot_temperature switches that rule off,
// rain_mode 0 the rain.
struct HomeConfig {
  // Below critical_voltage for critical_time s without a break.
  double critical_voltage = 19.7;
  double critical_time = 2.0;
  // The highest voltage of the last low_window s below empty_voltage, with samples over the whole window.
  double empty_voltage = 21.0;
  double low_window = 20.0;
  double battery_hot_temperature = 0;
  double motor_hot_temperature = 70.0;
  // s a temperature holds before it counts, against single wrong values.
  double hot_time = 3.0;
  int rain_mode = 0;
  // Wet in every sample for rain_check_seconds s.
  double rain_check_seconds = 20.0;
  // A longer gap between samples (s) starts every rule anew.
  double max_gap = 2.0;
};

// What one sample of hw/status says for the rules; a sample without telemetry counts as a gap.
struct HomeSample {
  // Monotonic s.
  double time = 0;
  bool telemetry_ok = false;
  bool battery_ok = false;
  double battery_voltage = 0;
  double battery_temperature = 0;
  // The blade motor reports.
  bool blade_ok = false;
  double blade_temperature = 0;
  bool rain = false;
};

// The rules for going home while mowing, without ROS: each holds once its time or window is full.
class HomeWatch {
 public:
  explicit HomeWatch(const HomeConfig& config = HomeConfig()) : config_(config) {}

  void OnSample(const HomeSample& sample);
  // Every reason that holds now, in the order of HomeReason.
  std::vector<HomeReason> Due() const;
  bool Holds(HomeReason reason) const;
  // The highest voltage of the window, for BATTERY_LOW.
  double WindowVoltage() const;
  // Starts every rule anew.
  void Reset();
  const HomeConfig& config() const {
    return config_;
  }
  void SetConfig(const HomeConfig& config);

 private:
  HomeConfig config_;
  std::optional<double> last_;
  double now_ = 0;
  std::optional<double> critical_since_;
  std::optional<double> battery_hot_since_;
  std::optional<double> motor_hot_since_;
  std::optional<double> wet_since_;
  // The samples of the battery since window_since_, without a gap, the last low_window s of them.
  std::optional<double> window_since_;
  std::deque<std::pair<double, double>> window_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_HOME_WATCH_H
