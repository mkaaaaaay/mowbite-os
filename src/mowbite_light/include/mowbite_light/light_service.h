#ifndef MOWBITE_LIGHT_LIGHT_SERVICE_H
#define MOWBITE_LIGHT_LIGHT_SERVICE_H

#include <optional>
#include <string>
#include <vector>

#include "mowbite_light/effect.h"

namespace mowbite_light {

// Longest light.preview in s.
constexpr double kMaxPreview = 10.0;

struct RpcOutcome {
  bool ok = true;
  Json result;
  int code = 0;
  std::string message;
};

struct LightConfig {
  size_t leds = 55;
  // Of full scale per channel; the effects scale within it.
  double max_brightness = 0.47;
  double max_current_ma = 1200.0;
  // What one channel of an LED draws at 255.
  double ma_per_channel = 20.0;
  // Below this battery (0..1) IDLE shows BATTERY_LOW.
  double low_battery = 0.2;
  // Where the profile is kept; empty: only in memory.
  std::string profile_path;
};

// The lights of the mower without ROS: the profile per state and its file, the RPC methods light.*, a preview that
// ends by itself, and the emergency over all of it.
class LightService {
 public:
  explicit LightService(LightConfig config);

  // Reads the profile file; the defaults and why for one that is not usable, "" when it is fine or missing.
  std::string Load();

  static const std::vector<std::string>& Methods();
  // nullopt for a method of another node.
  std::optional<RpcOutcome> Call(const std::string& method, const Json& params, double now);

  void SetRobotState(const std::string& current_state, bool emergency, bool is_charging, double battery);

  // The colours of the chain at now (s), within max_brightness and max_current_ma.
  std::vector<Rgb> Frame(double now);

  const Profile& profile() const { return profile_; }

 private:
  RpcOutcome GetProfile() const;
  RpcOutcome SetProfile(const Json& params);
  RpcOutcome Preview(const Json& params, double now);
  RpcOutcome Reset();

  LightConfig config_;
  Profile profile_;
  std::string state_ = "IDLE";
  bool emergency_ = false;
  bool charging_ = false;
  double battery_ = 1.0;
  std::optional<Effect> preview_;
  double preview_until_ = 0;
  unsigned previews_ = 0;
  // What is shown and since when, so that every effect starts at its beginning.
  std::string shown_;
  double shown_since_ = 0;
};

}  // namespace mowbite_light

#endif  // MOWBITE_LIGHT_LIGHT_SERVICE_H
