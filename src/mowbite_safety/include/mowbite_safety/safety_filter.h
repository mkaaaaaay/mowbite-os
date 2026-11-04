#ifndef MOWBITE_SAFETY_SAFETY_FILTER_H
#define MOWBITE_SAFETY_SAFETY_FILTER_H

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_safety/region.h"

namespace mowbite_safety {

// Sources of drive commands, numbered like DriveCommand; kStop is the logic stopping the mower.
enum class Source : uint8_t { kStop = 0, kTeleop = 1, kFollower = 2 };

// The reason bits of hw/status the filter latches over hw/emergency.
constexpr uint32_t kReasonTilt = 1u << 13;
constexpr uint32_t kReasonFence = 1u << 14;

struct SafetyConfig {
  // TILT: the body more than tilt_limit degrees off the vertical for tilt_time s; a release needs it below
  // tilt_release. The Tron mows slopes up to 31 degrees (vendor), its MCU reports a lift from about 37.
  double tilt_limit = 31.0;
  double tilt_time = 1.0;
  double tilt_release = 26.0;
  // Nothing moves without an IMU sample for imu_timeout s or without hw/status for hw_timeout s.
  double imu_timeout = 0.5;
  double hw_timeout = 1.0;
  // Autonomous drives need a pose not older than pose_timeout s, max_position_accuracy m or better, with a heading.
  double pose_timeout = 0.5;
  double max_position_accuracy = 0.1;
  // FENCE: an autonomous drive with base_link farther than fence_margin m outside of the allowed areas, or deeper than
  // obstacle_margin m in an obstacle. The follower stops at 0.5 m already, so this only acts when it does not.
  double fence_margin = 0.6;
  double obstacle_margin = 0.1;
  // The zone of the docking station: the longest approach and undocking of params.set (2 m) and the corridor of the
  // logic (0.1 m) on its line, and the overshoot of docking behind it.
  double station_ahead = 2.1;
  double station_behind = 0.15;
  double station_half_width = 0.1;
  // After an emergency, and for the others when a source starts to move, a source moves only after a zero or arm_quiet
  // s without commands of its own; the app sends teleop every 0.1 s while the joystick is held.
  double arm_quiet = 1.0;
  // A source keeps the drive for owner_timeout s after its last moving command, the cmd_vel timeout of the driver.
  double owner_timeout = 0.5;
};

struct Twist2 {
  double linear = 0;
  double angular = 0;
};

// What to send on hw/cmd_vel for a command (a zero stops), nothing if empty; why a moving command did not pass.
struct Decision {
  std::optional<Twist2> out;
  std::string refusal;
  // The refusal is a second source while another one drives (G14).
  bool conflict = false;
};

// A latch for hw/emergency; again: sent before, hw/status does not show it yet.
struct LatchRequest {
  uint32_t bits = 0;
  std::string reason;
  bool again = false;
};

struct Release {
  bool allowed = false;
  std::string message;
  // Why not: STILL_TILTED with the angle and the limit in degrees, or TILT_UNKNOWN with the seconds since the last
  // IMU sample (NaN without one).
  std::string code;
  double value = std::numeric_limits<double>::quiet_NaN();
  double limit = std::numeric_limits<double>::quiet_NaN();
};

// S2 between all sources of drive commands and the driver: one source at a time, armed after every emergency, a
// tilt limit for all of them, and for autonomous drives a recent accurate pose with a heading and the fence of the
// map. Time is passed in (monotonic s), nothing here talks to ROS.
class SafetyFilter {
 public:
  explicit SafetyFilter(const SafetyConfig& config);

  // hw/status: emergency is set while the driver blocks or has no telemetry; reason as it shows it.
  void OnHardware(bool emergency, uint32_t reason, double now);
  // hw/capabilities: the safety measurement of the platform and the limits of the driver.
  void OnCapabilities(bool safety_measured, double max_linear, double max_angular);
  // The angle of the body to the vertical (degrees) of an IMU sample.
  void OnTilt(double tilt_deg, double now);
  void OnPose(double x, double y, double accuracy, bool heading_valid, double now);
  // map/json, and whether map/status says it fits the datum.
  void SetMap(const std::string& map_json);
  void SetMapFits(bool fits);
  void SetMaxPositionAccuracy(double accuracy);

  Decision OnCommand(Source source, double linear, double angular, double now);
  // A stop when an emergency began or the IMU went silent while the mower moves.
  std::optional<Twist2> Tick(double now);
  // The own latch for hw/emergency: once when it is new, again every second while hw/status does not show it.
  std::optional<LatchRequest> TakeLatchRequest(double now);
  // A release asked for by the logic: refused while tilted or without an IMU sample; else the own latch is gone, every
  // source has to arm again, and the node asks the driver.
  Release OnRelease(double now);

  uint32_t Latched() const;
  // NaN without a recent IMU sample.
  double TiltDeg(double now) const;
  // m base_link lies outside of the allowed areas (0 inside), NaN without a map that fits or a recent pose.
  double FenceDistance(double now) const;
  bool AutonomousOk(double now) const;
  // The source that moves the mower, kStop while none does.
  Source Owner(double now) const;
  uint32_t Refused() const;
  const std::string& LastRefusal() const;

 private:
  struct SourceState {
    bool armed = false;
    std::optional<double> last_s;
  };
  struct Pose {
    double x;
    double y;
    double accuracy;
    bool heading_valid;
    double s;
  };

  bool Emergency(double now) const;
  // Disarms every source when an emergency begins or ends.
  void UpdateEmergency(double now);
  void DisarmAll();
  bool ImuFresh(double now) const;
  std::string AutonomousProblem(double now) const;
  // m (x, y) lies outside of the allowed areas and the zone of the station, 0 inside.
  double OutsideDistance(double x, double y) const;
  // Empty if base_link may be at (x, y), else why not.
  std::string FenceProblem(double x, double y) const;
  // Why a moving command of source may not pass, empty if it may; latches FENCE.
  std::string MoveProblem(Source source, double quiet, double now, bool* conflict);
  void Latch(uint32_t bits, const std::string& reason);
  SourceState& State(Source source);

  SafetyConfig config_;
  std::optional<double> hw_s_;
  bool hw_emergency_ = true;
  uint32_t hw_reason_ = 0;
  bool emergency_ = true;
  bool safety_measured_ = false;
  bool capabilities_ = false;
  double max_linear_ = 0;
  double max_angular_ = 0;
  std::optional<double> imu_s_;
  double tilt_deg_ = 0;
  std::optional<double> tilt_since_;
  std::optional<Pose> pose_;
  std::optional<Region> region_;
  // The allowed areas of the region and the zone of its station.
  std::vector<Ring> allowed_;
  std::string region_error_ = "no map yet";
  bool map_fits_ = false;
  std::array<SourceState, 3> sources_;
  Source owner_ = Source::kStop;
  double owner_s_ = 0;
  // The source whose moving command went out last, empty after a stop.
  std::optional<Source> moving_;
  uint32_t latched_ = 0;
  std::string latch_reason_;
  bool latch_new_ = false;
  std::optional<double> latch_sent_s_;
  uint32_t refused_ = 0;
  std::string last_refusal_;
};

const char* SourceName(Source source);

// Angle of the body to the vertical in degrees, from the orientation of an IMU sample (normalized here) or from
// gravity in its acceleration; NaN for an orientation or acceleration of length 0.
double TiltFromOrientation(double x, double y, double z, double w);
double TiltFromGravity(double ax, double ay, double az);

}  // namespace mowbite_safety

#endif  // MOWBITE_SAFETY_SAFETY_FILTER_H
