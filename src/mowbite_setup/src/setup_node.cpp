#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <map>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/clock_status.hpp>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_inputs.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/ntrip_status.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/setup_motion.hpp>
#include <mowbite_msgs/srv/end_driver.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mowbite_msgs/srv/silence_driver.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <set>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_bridge/event_builder.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_hold/lock.h"
#include "mowbite_hold/restart.h"
#include "mowbite_map/sha256.h"
#include "mowbite_ntrip/ntrip_protocol.h"
#include "mowbite_setup/layer.h"
#include "mowbite_setup/program.h"
#include "mowbite_setup/steps.h"
#include "mowbite_watch/attach.h"

namespace mowbite_setup {

namespace {

using mowbite_msgs::msg::HardwareStatus;
using mowbite_msgs::msg::RpcError;
using mowbite_msgs::srv::EndDriver;
using mowbite_msgs::srv::SetEmergency;
using mowbite_msgs::srv::SilenceDriver;

constexpr const char* kTopic = "setup/json";
constexpr const char* kRefused = "SETUP_REFUSED";
constexpr const char* kGeometryNode = "/mowbite_geometry";
// setup/json goes out again this often without a change (s), for a bridge that came later.
constexpr double kRepeat = 30.0;
// A write that failed is tried again this often (s); the lock of autonomous driving sooner.
constexpr double kWriteRetry = 5.0;
constexpr double kLockRetry = 1.0;
// Correction data counts as arriving with a frame this young (s).
constexpr double kFrameAge = 10.0;
// In data_dir: the setup program asked for a restart of the OS; the launch ends when it sees it (stack.py).
constexpr const char* kRestartFile = "setup_restart.json";
// In data_dir: the start noted the last end of the driver there (stack.py), read this often (s).
constexpr const char* kDriverExitFile = "driver_exit.json";
constexpr double kRestartsEvery = 1.0;
// A note older than its stable_s and this (s): the driver ran long enough since, the start counts anew.
constexpr double kStableMargin = 5.0;
// setup/motion goes out this often while a step moves the wheels (s); setup/cmd_vel with every tick.
constexpr double kMotionPeriod = 0.25;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

double WallSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

struct Image {
  std::string bytes;
  std::string sha256;
};

}  // namespace

// The setup program of the platform: answers setup.*, keeps <data_dir>/setup.json, sends the state retained on
// setup/json and the events SETUP_STEP and SETUP. What it asks of the mower: the release of an emergency a part of
// its own brought (Program::OwnLatch), over the checks of the safety filter; a restart of the OS for a step that
// changed a setting of the driver, with the mower in IDLE (Program::TakeRestart); and in a step that moves the
// wheels of the mower jacked up, setup/motion and commands on setup/cmd_vel, which the logic passes only in SETUP and
// mowbite_safety caps, and hw/silence or hw/end of the driver.
class SetupNode : public rclcpp::Node {
 public:
  explicit SetupNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_setup", mowbite_component::StartupOnly(options)) {
    data_dir_ = declare_parameter("data_dir", std::string());
    const std::string platform = declare_parameter("platform", std::string());
    const std::string steps = declare_parameter("steps", std::string());
    const std::string image_dir = declare_parameter("image_dir", std::string());
    layer_path_ = declare_parameter("layer", std::string());
    const std::string shadowed = declare_parameter("shadowed", std::string());
    const std::string restarts = declare_parameter("driver_restarts", std::string("0"));
    driver_restarts_ = std::atoi(restarts.c_str());

    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<RpcError>("bridge/rpc/error", 100);
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_setup";
    if (steps.empty()) {
      RCLCPP_INFO(get_logger(), "The platform has no step list: setup.* is not offered");
    } else if (data_dir_.empty()) {
      // Without a place for the lock a failed safety check could not close it.
      RCLCPP_WARN(get_logger(), "No data_dir: setup.* is not offered");
    } else {
      std::string why;
      const std::optional<StepList> list = ParseSteps(json::parse(steps, nullptr, false), &why);
      if (!list) {
        RCLCPP_ERROR(get_logger(), "The step list of the platform is not valid, setup.* is not offered: %s",
                     why.c_str());
      } else {
        LoadImages(*list, image_dir);
        program_.emplace(*list, platform);
        LoadRecord();
        LoadLock();
        LoadLayer(shadowed);
        for (const auto& method : mowbite_api::Methods()) {
          if (method.node == methods.node_id) methods.methods.push_back(method.name);
        }
      }
    }
    methods_pub_->publish(methods);
    watch_ = mowbite_watch::Attach(*this);
    if (!program_) return;

    hw_sub_ = create_subscription<HardwareStatus>("hw/status", 10, [this](HardwareStatus::ConstSharedPtr msg) {
      Hardware hardware;
      hardware.emergency_reason = msg->emergency_reason;
      hardware.latch_count = msg->latch_count;
      hardware.telemetry_ok = msg->telemetry_ok;
      hardware.stop = msg->stop;
      hardware.lift = msg->lift;
      hardware.bumper = msg->bumper;
      hardware.bumper_left = msg->bumper_left;
      hardware.bumper_right = msg->bumper_right;
      hardware.dock_contact = msg->dock_contact;
      hardware.hold = msg->hold;
      hardware.battery = msg->battery_source != HardwareStatus::BATTERY_SOURCE_NONE;
      hardware.battery_percentage = msg->battery_percentage;
      if (!msg->blade_motor.ok) hardware.motors_down.push_back("blade");
      hardware.blade_ok = msg->blade_motor.ok;
      hardware.blade_rpm = msg->blade_motor.rpm;
      if (!msg->left_motor.ok) hardware.motors_down.push_back("left");
      if (!msg->right_motor.ok) hardware.motors_down.push_back("right");
      hardware.link_errors = msg->link_errors;
      std::lock_guard<std::mutex> lock(mutex_);
      program_->OnHardware(hardware, MonotonicSeconds());
    });
    inputs_sub_ = create_subscription<mowbite_msgs::msg::HardwareInputs>(
        "hw/inputs", LatchedQos(1), [this](mowbite_msgs::msg::HardwareInputs::ConstSharedPtr msg) {
          std::vector<Line> lines;
          lines.reserve(msg->inputs.size());
          for (const auto& input : msg->inputs) {
            Line line;
            line.name = input.name;
            line.role = input.role;
            line.reason = input.reason;
            line.active = input.active;
            line.assigned = input.assigned;
            line.held = input.held;
            line.active_high = input.active_high;
            line.delay_ms = input.delay_ms;
            lines.push_back(std::move(line));
          }
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnInputs(lines, msg->learning);
        });
    release_client_ = create_client<SetEmergency>("safety/release_emergency");
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          Capabilities capabilities;
          for (const auto& entry : msg->firmware) capabilities.firmware.emplace_back(entry.key, entry.value);
          capabilities.safety_measurement_date = msg->safety_measurement_date;
          capabilities.silence_max_s = msg->silence_max_s;
          capabilities.driver_end = msg->driver_end;
          capabilities.blade_allowed = msg->blade_allowed;
          capabilities.blade_lease_max_s = msg->blade_lease_max_s;
          capabilities.failsafe_without_host_s = msg->failsafe_without_host_s;
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnCapabilities(capabilities);
          state_due_ = true;
        });
    imu_in_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          const ImuSample sample{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                                 msg->angular_velocity.z};
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnImu(sample, MonotonicSeconds());
        });
    wheels_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "hw/wheel_odom", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          const double v = msg->twist.linear.x;
          const double turn = msg->twist.angular.z * track_ / 2;
          program_->OnWheels(v - turn, v + turn, MonotonicSeconds());
        });
    motion_pub_ = create_publisher<mowbite_msgs::msg::SetupMotion>("setup/motion", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("setup/cmd_vel", 10);
    silence_client_ = create_client<SilenceDriver>("hw/silence");
    end_client_ = create_client<EndDriver>("hw/end");
    gnss_sub_ = create_subscription<mowbite_msgs::msg::GnssFix>(
        "gps/absolute_pose", 10, [this](mowbite_msgs::msg::GnssFix::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnFix(msg->solution, msg->accuracy, msg->x, msg->y, MonotonicSeconds());
        });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) {
          // Driving by hand and calibrating are IDLE with a sub state: the program takes IDLE as standing.
          std::string state = mowbite_hold::SessionState(msg->current_state, msg->current_sub_state);
          std::lock_guard<std::mutex> lock(mutex_);
          if (world_.state == state) return;
          world_.state = std::move(state);
          program_->OnWorld(world_);
          state_due_ = true;
        });
    map_sub_ = create_subscription<mowbite_msgs::msg::MapStatus>(
        "map/status", LatchedQos(1), [this](mowbite_msgs::msg::MapStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          world_.areas = static_cast<int>(msg->areas);
          world_.docking_stations = static_cast<int>(msg->docking_stations);
          world_.mowing_allowed = msg->mowing_allowed;
          program_->OnWorld(world_);
        });
    ntrip_sub_ = create_subscription<mowbite_msgs::msg::NtripStatus>(
        "gnss/ntrip_status", 10, [this](mowbite_msgs::msg::NtripStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          world_.ntrip = msg->state == "connected" && msg->frames > 0 && msg->last_frame_age >= 0 &&
                         msg->last_frame_age < kFrameAge;
          program_->OnWorld(world_);
        });
    // The date a summary writes needs a safe wall clock.
    clock_sub_ = create_subscription<mowbite_msgs::msg::ClockStatus>(
        "time/clock", LatchedQos(1), [this](mowbite_msgs::msg::ClockStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnClock(msg->ok, msg->reason);
        });
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", LatchedQos(50), [this](std_msgs::msg::String::ConstSharedPtr msg) { OnParams(msg->data); });
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    tick_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { OnTick(); });
  }

 private:
  std::string RecordPath() const { return data_dir_ + "/setup.json"; }

  // The images the steps name, from the folder of the platform; one that is missing or breaks the rules is left out.
  void LoadImages(const StepList& list, const std::string& folder) {
    for (const std::string& name : list.Images()) {
      std::string bytes;
      const std::string wrong = LoadImage(folder, name, &bytes);
      if (!wrong.empty()) {
        RCLCPP_ERROR(get_logger(), "The image %s is not used: %s", name.c_str(), wrong.c_str());
        continue;
      }
      images_[name] = {bytes, mowbite_map::Sha256Hex(bytes)};
      image_infos_[name] = {images_[name].sha256, static_cast<int>(bytes.size())};
    }
  }

  void LoadRecord() {
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(RecordPath(), 4 * 1024 * 1024, &text, &error);
    if (read == mowbite_hold::FileRead::kMissing) return;
    std::string why = error;
    if (read != mowbite_hold::FileRead::kText || !program_->Load(text, WallSeconds(), &why)) {
      RCLCPP_ERROR(get_logger(), "%s is not used, the program starts anew: %s", RecordPath().c_str(), why.c_str());
    }
  }

  std::string LockPath() const { return data_dir_ + "/" + mowbite_hold::kSafetyLockFile; }

  // The lock as the last start left it; one that cannot be read closes with a reset.
  void LoadLock() {
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(LockPath(), 1024 * 1024, &text, &error);
    if (read == mowbite_hold::FileRead::kMissing) return;
    if (read != mowbite_hold::FileRead::kText) text.clear();
    program_->LoadLock(text, WallSeconds());
    if (!mowbite_hold::ParseSafetyLock(text)) {
      RCLCPP_ERROR(get_logger(), "%s cannot be read (%s): the lock stays closed until a summary passes",
                   LockPath().c_str(), error.empty() ? "not a lock" : error.c_str());
    }
  }

  // Writes or removes the lock; a failure keeps it due and blocks every change. Called with the mutex held.
  void WriteLock(double now) {
    if (!program_->lock_writable() && now - lock_tried_ < kLockRetry) return;
    const auto text = program_->TakeLock();
    if (!text) return;
    std::string error;
    const bool written = text->empty() ? mowbite_hold::RemoveFileSynced(LockPath(), &error)
                                       : mowbite_hold::WriteFileAtomic(LockPath(), *text, &error);
    const bool before = program_->lock_writable();
    if (!written && before) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", LockPath().c_str(), error.c_str());
    if (written && !before) RCLCPP_INFO(get_logger(), "%s is written again", LockPath().c_str());
    lock_tried_ = now;
    program_->LockWritten(written);
    if (written != before) state_due_ = true;
  }

  // The parameter file the program wrote for this mower, and what the file kept by hand overrules of it
  // (<node>.<parameter>, with commas, from the start).
  void LoadLayer(const std::string& shadowed) {
    if (layer_path_.empty()) return;
    Layer layer;
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(layer_path_, 1024 * 1024, &text, &error);
    if (read == mowbite_hold::FileRead::kText) {
      if (!layer.Parse(text, &error)) {
        RCLCPP_ERROR(get_logger(), "%s is not as the setup program writes it (%s): its values are left out of the "
                                   "next one",
                     layer_path_.c_str(), error.c_str());
      }
    } else if (read != mowbite_hold::FileRead::kMissing) {
      RCLCPP_ERROR(get_logger(), "%s cannot be read: %s", layer_path_.c_str(), error.c_str());
    }
    std::set<std::string> names;
    for (size_t begin = 0; begin < shadowed.size();) {
      const size_t end = std::min(shadowed.find(',', begin), shadowed.size());
      if (end > begin) names.insert(shadowed.substr(begin, end - begin));
      begin = end + 1;
    }
    program_->SetLayer(std::move(layer), std::move(names));
  }

  void OnParams(const std::string& text) {
    const json parsed = json::parse(text, nullptr, false);
    if (!parsed.is_object() || !parsed.contains("node") || parsed["node"] != kGeometryNode) return;
    if (!parsed.contains("params")) return;
    const json& params = parsed["params"];
    if (!params.is_object()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (params.contains("track") && params["track"].is_number()) {
      track_ = params["track"].get<double>();
      program_->SetTrack(track_);
    }
    if (!params.contains("confirmed") || !params["confirmed"].is_array()) return;
    world_.confirmed.clear();
    for (const auto& key : params["confirmed"]) {
      if (key.is_string()) world_.confirmed.insert(key.get<std::string>());
    }
    program_->OnWorld(world_);
  }

  // Called with the mutex held.
  json State(double now) const { return program_->State(now, image_infos_); }

  // Saves the record; a failure blocks every change until a write goes through again. Called with the mutex held.
  void Save(double now) {
    std::string error;
    const bool written = mowbite_hold::WriteFileSafely(RecordPath(), program_->Text(), &error);
    const bool before = program_->writable();
    if (!written && before) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", RecordPath().c_str(), error.c_str());
    if (written && !before) RCLCPP_INFO(get_logger(), "%s is written again", RecordPath().c_str());
    write_tried_ = now;
    program_->SetWritable(written);
    state_due_ = true;
  }

  // What the program has to tell after a change: the lock first, then the record and the events. Called with the
  // mutex held.
  void Flush(double now) {
    WriteLock(now);
    if (program_->TakeChanged()) {
      state_due_ = true;
      Save(now);
    }
    if (const auto layer = program_->TakeLayer()) {
      std::string error;
      const bool written = layer->empty() ? mowbite_hold::RemoveRecordFile(layer_path_, &error)
                                          : mowbite_hold::WriteFileSafely(layer_path_, *layer, &error);
      if (!written) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", layer_path_.c_str(), error.c_str());
      program_->LayerWritten(written, now, WallSeconds());
      if (program_->TakeChanged()) Save(now);
    }
    for (const Event& event : program_->TakeEvents()) {
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::BuildEvent(event.type, event.details, WallSeconds())
                        .dump(-1, ' ', false, json::error_handler_t::replace);
      publish_pub_->publish(msg);
    }
  }

  // The restart a step asked for, by the way of the step list: self leaves a note for the launch and ends this
  // process in order, so that whatever runs the OS starts it again; host asks mowbite-host over the broker.
  void Restart() {
    if (program_->restart_way() == "host") {
      RCLCPP_INFO(get_logger(), "The setup program asks mowbite-host to restart the OS");
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "rpc/request";
      msg.payload = json{{"jsonrpc", "2.0"}, {"method", "host.restart_os"}}.dump();
      publish_pub_->publish(msg);
      return;
    }
    std::string error;
    const std::string note = data_dir_ + "/" + kRestartFile;
    if (!mowbite_hold::WriteFileSafely(note, json{{"t", std::floor(WallSeconds())}}.dump() + "\n", &error)) {
      RCLCPP_ERROR(get_logger(), "Cannot write %s, the OS does not restart: %s", note.c_str(), error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "The setup program restarts the OS");
    rclcpp::shutdown();
  }

  void OnTick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    if (now - restarts_read_ >= kRestartsEvery) {
      restarts_read_ = now;
      program_->SetDriverRestarts(RestartsLeft(now));
    }
    program_->Tick(now, WallSeconds());
    if (const auto release = program_->TakeRelease()) SendRelease(*release);
    Drive(now);
    Flush(now);
    if (program_->TakeRestart(now)) Restart();
    if (!program_->writable() && now - write_tried_ >= kWriteRetry) Save(now);
    // What blocks a step changes without a change of the record: the app has to see it as well.
    std::string blocked = program_->BlockedNow(now);
    if (blocked != blocked_) {
      blocked_ = std::move(blocked);
      state_due_ = true;
    }
    // The state waits for the bridge; while a step runs it goes out at the live rate.
    if (publish_pub_->get_subscription_count() == 0) return;
    const double age = state_sent_ < 0 ? kRepeat : now - state_sent_;
    const bool due = state_due_ || age >= (program_->running() ? mowbite_api::kSetupLivePeriod : kRepeat);
    if (!due || age < mowbite_api::kSetupLivePeriod) return;
    state_due_ = false;
    state_sent_ = now;
    mowbite_msgs::msg::MqttPublish msg;
    msg.topic = kTopic;
    msg.payload = State(now).dump(-1, ' ', false, json::error_handler_t::replace);
    msg.retain = true;
    publish_pub_->publish(msg);
  }

  // How often the start would start an ended driver again from now on, by the note of its last end: none, the
  // restarts of the platform; down, none; older than its stable_s, all again.
  int RestartsLeft(double now) const {
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(data_dir_ + "/" + kDriverExitFile, 64 * 1024, &text, &error);
    if (read == mowbite_hold::FileRead::kMissing) return driver_restarts_;
    const json note = read == mowbite_hold::FileRead::kText ? json::parse(text, nullptr, false) : json();
    if (!note.is_object()) return 0;
    const auto number = [&note](const char* key) {
      return note.contains(key) && note[key].is_number() ? note[key].get<double>() : -1.0;
    };
    const double max = number("max_restarts");
    // A note without its times may be of a driver that is down already.
    if (note.value("down", false) || max < 0 || number("mono") < 0 || number("stable_s") < 0) return 0;
    if (now - number("mono") > number("stable_s") + kStableMargin) return static_cast<int>(max);
    return std::max(0, static_cast<int>(max - number("restart")));
  }

  // A step that moves the wheels or the blade: setup/motion at its rate, at once when the blade changes, and once with
  // active false at its end, its command with every tick, hw/silence and hw/end when it asks for them. Called with the
  // mutex held.
  void Drive(double now) {
    EndMotion();
    if (const auto motion = program_->MotionNow()) {
      if (motion_sent_ < 0 || now - motion_sent_ >= kMotionPeriod || motion->blade != motion_blade_) {
        motion_sent_ = now;
        motion_blade_ = motion->blade;
        mowbite_msgs::msg::SetupMotion msg;
        msg.stamp = get_clock()->now();
        msg.active = true;
        msg.step = motion->step;
        msg.raised = motion->raised;
        msg.max_linear = static_cast<float>(motion->max_linear);
        msg.max_angular = static_cast<float>(motion->max_angular);
        msg.blade = static_cast<uint8_t>(motion->blade);
        msg.blade_percent = static_cast<uint8_t>(motion->blade_percent);
        msg.blade_height_mm = static_cast<uint16_t>(motion->blade_height_mm);
        msg.blade_lease_s = static_cast<float>(motion->blade_lease_s);
        motion_pub_->publish(msg);
      }
      if (const auto command = program_->CommandNow()) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = command->linear;
        twist.angular.z = command->angular;
        cmd_pub_->publish(twist);
      }
    }
    if (const auto seconds = program_->TakeSilence()) {
      if (!silence_client_->service_is_ready()) {
        program_->OnSilenceAnswer(false, "NO_SERVICE", now, WallSeconds());
      } else {
        auto request = std::make_shared<SilenceDriver::Request>();
        request->seconds = static_cast<float>(*seconds);
        request->blade = program_->SilenceWithBlade();
        silence_client_->async_send_request(request, [this](rclcpp::Client<SilenceDriver>::SharedFuture future) {
          const auto& response = future.get();
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnSilenceAnswer(response->success, response->code, MonotonicSeconds(), WallSeconds());
        });
      }
    }
    if (program_->TakeEnd()) {
      if (!end_client_->service_is_ready()) {
        program_->OnEndAnswer(false, "NO_SERVICE", now, WallSeconds());
      } else {
        end_client_->async_send_request(
            std::make_shared<EndDriver::Request>(), [this](rclcpp::Client<EndDriver>::SharedFuture future) {
              const auto& response = future.get();
              std::lock_guard<std::mutex> lock(mutex_);
              program_->OnEndAnswer(response->success, response->code, MonotonicSeconds(), WallSeconds());
            });
      }
    }
  }

  // A step that ended: a zero on setup/cmd_vel, then setup/motion with active false. Called with the mutex held.
  void EndMotion() {
    const auto step = program_->TakeMotionEnd();
    if (!step) return;
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    mowbite_msgs::msg::SetupMotion msg;
    msg.stamp = get_clock()->now();
    msg.active = false;
    msg.step = *step;
    motion_pub_->publish(msg);
    motion_sent_ = -1;
  }

  // The answer is only logged: whether the emergency ended shows in hw/status alone.
  void SendRelease(const ReleaseRequest& release) {
    if (!release_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "safety/release_emergency is not available");
      return;
    }
    release_client_->prune_pending_requests();
    auto request = std::make_shared<SetEmergency::Request>();
    request->latch = false;
    request->reason = release.reason;
    request->latch_count = release.latch_count;
    release_client_->async_send_request(request, [this](rclcpp::Client<SetEmergency>::SharedFuture future) {
      const auto& response = future.get();
      if (!response->success) {
        RCLCPP_INFO(get_logger(), "The emergency of the checked input stays (%s): %s", response->code.c_str(),
                    response->message.c_str());
      }
    });
  }

  void Reply(const std::string& id, const json& result) {
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = result.dump(-1, ' ', false, json::error_handler_t::replace);
    msg.id = id;
    response_pub_->publish(msg);
  }

  void ReplyError(const std::string& id, int code, const std::string& message, const json& data) {
    RpcError msg;
    msg.code = code;
    msg.message = message;
    msg.id = id;
    msg.data = data.dump(-1, ' ', false, json::error_handler_t::replace);
    error_pub_->publish(msg);
  }

  void ReplyRefusal(const std::string& id, const Refusal& refusal) {
    if (!refusal.field.empty()) {
      ReplyError(id, RpcError::ERROR_INVALID_PARAMS, refusal.field + ": " + refusal.message,
                 {{"field", refusal.field}, {"reason", "INVALID"}});
      return;
    }
    const mowbite_api::Rating rating = mowbite_api::RateReason(kRefused, refusal.reason);
    json data = {{"event", kRefused},
                 {"reason", refusal.reason},
                 {"message", refusal.message},
                 {"severity", rating.severity},
                 {"resume", rating.resume}};
    for (const auto& item : refusal.data.items()) data[item.key()] = item.value();
    ReplyError(id, RpcError::ERROR_INVALID_REQUEST, refusal.message, data);
  }

  // setup.image: a piece of an image, base64.
  void OnImage(const std::string& id, const json& params) {
    const auto bad = [this, &id](const std::string& field, const std::string& message) {
      ReplyError(id, RpcError::ERROR_INVALID_PARAMS, field + ": " + message,
                 {{"field", field}, {"reason", "INVALID"}});
    };
    if (!params.is_object() || !params.contains("name") || !params["name"].is_string()) {
      return bad("name", "the name of an image is missing");
    }
    const auto found = images_.find(params["name"].get<std::string>());
    if (found == images_.end()) return bad("name", "no such image");
    const std::string& bytes = found->second.bytes;
    int64_t offset = 0;
    int64_t size = mowbite_api::kSetupImageBytes;
    for (const char* key : {"offset", "size"}) {
      if (!params.contains(key)) continue;
      if (!params[key].is_number_integer()) return bad(key, "no whole number");
      (std::string(key) == "offset" ? offset : size) = params[key].get<int64_t>();
    }
    if (offset < 0 || offset > static_cast<int64_t>(bytes.size())) return bad("offset", "outside of the image");
    if (size < 1 || size > mowbite_api::kSetupImageBytes) return bad("size", "outside of 1 to the largest piece");
    Reply(id, {{"size", bytes.size()},
               {"offset", offset},
               {"data", mowbite_ntrip::Base64(bytes.substr(static_cast<size_t>(offset), static_cast<size_t>(size)))},
               {"sha256", found->second.sha256},
               {"type", "image/svg+xml"}});
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    if (request.method.rfind("setup.", 0) != 0) return;
    json params;
    if (!request.params.empty()) {
      params = json::parse(request.params, nullptr, false);
      if (params.is_discarded()) {
        ReplyError(request.id, RpcError::ERROR_INVALID_PARAMS, "The parameters are not valid JSON", json::object());
        return;
      }
    }
    if (request.method == "setup.image") {
      OnImage(request.id, params);
      return;
    }
    if (request.method == "setup.keep") {
      json result;
      std::optional<Refusal> refused;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        refused = program_->Keep(params, MonotonicSeconds(), &result);
      }
      if (refused) {
        ReplyRefusal(request.id, *refused);
      } else {
        Reply(request.id, result);
      }
      return;
    }
    std::optional<Refusal> refusal;
    json state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = MonotonicSeconds();
      const double wall = WallSeconds();
      if (request.method == "setup.start") {
        refusal = program_->Start(params, now, wall);
      } else if (request.method == "setup.answer") {
        refusal = program_->Answer(params, now, wall);
      } else if (request.method == "setup.skip") {
        refusal = program_->Skip(params, wall);
      } else if (request.method == "setup.abort") {
        refusal = program_->Abort(params, now, wall);
      } else if (request.method == "setup.reset") {
        refusal = program_->Reset(params, WallSeconds());
      } else if (request.method != "setup.get") {
        return;
      }
      if (!refusal) {
        // A step that moves ends before any file is written.
        EndMotion();
        Flush(now);
        state = State(now);
      }
    }
    if (refusal) {
      ReplyRefusal(request.id, *refusal);
    } else {
      Reply(request.id, state);
    }
  }

  std::mutex mutex_;
  std::string data_dir_;
  std::string layer_path_;
  std::optional<Program> program_;
  std::map<std::string, Image> images_;
  std::map<std::string, std::pair<std::string, int>> image_infos_;
  World world_;
  double write_tried_ = 0;
  double lock_tried_ = 0;
  bool state_due_ = true;
  std::string blocked_;
  double state_sent_ = -1;
  int driver_restarts_ = 0;
  double restarts_read_ = -1;
  double track_ = 0;
  double motion_sent_ = -1;
  int motion_blade_ = 0;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<RpcError>::SharedPtr error_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Subscription<HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareInputs>::SharedPtr inputs_sub_;
  rclcpp::Client<SetEmergency>::SharedPtr release_client_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_in_;
  rclcpp::Subscription<mowbite_msgs::msg::GnssFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheels_sub_;
  rclcpp::Publisher<mowbite_msgs::msg::SetupMotion>::SharedPtr motion_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<SilenceDriver>::SharedPtr silence_client_;
  rclcpp::Client<EndDriver>::SharedPtr end_client_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::NtripStatus>::SharedPtr ntrip_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ClockStatus>::SharedPtr clock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_setup

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_setup::SetupNode)
