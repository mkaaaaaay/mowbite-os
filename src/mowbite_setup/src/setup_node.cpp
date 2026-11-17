#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mowbite_component/node_options.h>
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
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_bridge/event_builder.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_map/sha256.h"
#include "mowbite_ntrip/ntrip_protocol.h"
#include "mowbite_setup/program.h"
#include "mowbite_setup/steps.h"
#include "mowbite_watch/attach.h"

namespace mowbite_setup {

namespace {

using mowbite_msgs::msg::HardwareStatus;
using mowbite_msgs::msg::RpcError;
using mowbite_msgs::srv::SetEmergency;

constexpr const char* kTopic = "setup/json";
constexpr const char* kRefused = "SETUP_REFUSED";
constexpr const char* kGeometryNode = "/mowbite_geometry";
// setup/json goes out again this often without a change (s), for a bridge that came later.
constexpr double kRepeat = 30.0;
// A write that failed is tried again this often (s).
constexpr double kWriteRetry = 5.0;
// Correction data counts as arriving with a frame this young (s).
constexpr double kFrameAge = 10.0;

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
// setup/json and the events SETUP_STEP and SETUP. It watches and moves nothing. The one thing it asks of the mower is
// the release of an emergency a part of its own brought (Program::OwnLatch), over the checks of the safety filter.
class SetupNode : public rclcpp::Node {
 public:
  explicit SetupNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_setup", mowbite_component::StartupOnly(options)) {
    data_dir_ = declare_parameter("data_dir", std::string());
    const std::string platform = declare_parameter("platform", std::string());
    const std::string steps = declare_parameter("steps", std::string());
    const std::string image_dir = declare_parameter("image_dir", std::string());

    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<RpcError>("bridge/rpc/error", 100);
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_setup";
    if (steps.empty()) {
      RCLCPP_INFO(get_logger(), "The platform has no step list: setup.* is not offered");
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
      hardware.stop = msg->stop;
      hardware.lift = msg->lift;
      hardware.bumper = msg->bumper;
      hardware.bumper_left = msg->bumper_left;
      hardware.bumper_right = msg->bumper_right;
      hardware.dock_contact = msg->dock_contact;
      hardware.battery = msg->battery_source != HardwareStatus::BATTERY_SOURCE_NONE;
      hardware.battery_percentage = msg->battery_percentage;
      if (!msg->blade_motor.ok) hardware.motors_down.push_back("blade");
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
            line.delay_ms = input.delay_ms;
            lines.push_back(std::move(line));
          }
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnInputs(lines);
        });
    release_client_ = create_client<SetEmergency>("safety/release_emergency");
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          Capabilities capabilities;
          for (const auto& entry : msg->firmware) capabilities.firmware.emplace_back(entry.key, entry.value);
          capabilities.safety_measurement_date = msg->safety_measurement_date;
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
    gnss_sub_ = create_subscription<mowbite_msgs::msg::GnssFix>(
        "gps/absolute_pose", 10, [this](mowbite_msgs::msg::GnssFix::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          program_->OnFix(msg->solution, msg->accuracy, MonotonicSeconds());
        });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (world_.state == msg->current_state) return;
          world_.state = msg->current_state;
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
    if (data_dir_.empty()) {
      RCLCPP_WARN(get_logger(), "No data_dir: the setup program forgets its state with every start");
      return;
    }
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(RecordPath(), 4 * 1024 * 1024, &text, &error);
    if (read == mowbite_hold::FileRead::kMissing) return;
    std::string why = error;
    if (read != mowbite_hold::FileRead::kText || !program_->Load(text, WallSeconds(), &why)) {
      RCLCPP_ERROR(get_logger(), "%s is not used, the program starts anew: %s", RecordPath().c_str(), why.c_str());
    }
  }

  void OnParams(const std::string& text) {
    const json parsed = json::parse(text, nullptr, false);
    if (!parsed.is_object() || !parsed.contains("node") || parsed["node"] != kGeometryNode) return;
    if (!parsed.contains("params")) return;
    const json& params = parsed["params"];
    if (!params.is_object() || !params.contains("confirmed") || !params["confirmed"].is_array()) return;
    std::lock_guard<std::mutex> lock(mutex_);
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
    if (data_dir_.empty()) return;
    std::string error;
    const bool written = mowbite_hold::WriteFileSafely(RecordPath(), program_->Text(), &error);
    const bool before = program_->writable();
    if (!written && before) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", RecordPath().c_str(), error.c_str());
    if (written && !before) RCLCPP_INFO(get_logger(), "%s is written again", RecordPath().c_str());
    write_tried_ = now;
    program_->SetWritable(written);
    state_due_ = true;
  }

  // What the program has to tell after a change. Called with the mutex held.
  void Flush(double now) {
    if (program_->TakeChanged()) {
      state_due_ = true;
      Save(now);
    }
    for (const Event& event : program_->TakeEvents()) {
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::BuildEvent(event.type, event.details, WallSeconds())
                        .dump(-1, ' ', false, json::error_handler_t::replace);
      publish_pub_->publish(msg);
    }
  }

  void OnTick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    program_->Tick(now, WallSeconds());
    if (const auto release = program_->TakeRelease()) SendRelease(*release);
    Flush(now);
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
    std::optional<Refusal> refusal;
    json state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = MonotonicSeconds();
      const double wall = WallSeconds();
      if (request.method == "setup.start") {
        refusal = program_->Start(params, now, wall);
      } else if (request.method == "setup.answer") {
        refusal = program_->Answer(params, wall);
      } else if (request.method == "setup.skip") {
        refusal = program_->Skip(params, wall);
      } else if (request.method == "setup.abort") {
        refusal = program_->Abort(params, now, wall);
      } else if (request.method == "setup.reset") {
        refusal = program_->Reset(params);
      } else if (request.method != "setup.get") {
        return;
      }
      if (!refusal) {
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
  std::optional<Program> program_;
  std::map<std::string, Image> images_;
  std::map<std::string, std::pair<std::string, int>> image_infos_;
  World world_;
  double write_tried_ = 0;
  bool state_due_ = true;
  std::string blocked_;
  double state_sent_ = -1;
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
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::NtripStatus>::SharedPtr ntrip_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_setup

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_setup::SetupNode)
