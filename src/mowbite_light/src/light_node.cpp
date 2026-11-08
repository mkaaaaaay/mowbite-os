#include <algorithm>
#include <chrono>
#include <memory>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "mowbite_light/light_service.h"
#include "mowbite_light/output.h"
#include "mowbite_light/ws2812.h"
#include "tron_mcu/serial_port.h"

namespace mowbite_light {

namespace {

constexpr int64_t kMaxBufferLeds = 300;
// An unchanged frame goes out again after this long, in case the chain lost one.
constexpr double kRepeat = 1.0;
constexpr double kReopen = 5.0;

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

// The lights: the effect of the state of robot_state (the emergency over all) as WS2812 frames on spidev or into a file
// for the simulation, and light.get_profile, light.set_profile, light.preview and light.reset through the bridge.
// With output none it offers no method and sends nothing.
class LightNode : public rclcpp::Node {
 public:
  explicit LightNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_light", options), service_(LoadConfig()) {
    if (output_kind_ == "none") {
      RCLCPP_INFO(get_logger(), "output none: no light.* methods, nothing is sent to the lights");
      return;
    }
    const std::string problem = service_.Load();
    if (!problem.empty()) RCLCPP_ERROR(get_logger(), "%s; the default profile applies", problem.c_str());

    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) {
          service_.SetRobotState(msg->current_state, msg->emergency, msg->is_charging, msg->battery_percentage);
        });

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_light";
    methods.methods = LightService::Methods();
    methods_pub_->publish(methods);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frame_rate_), [this] { Tick(); });
  }

  // A dark chain once the OS ends; a WS2812 keeps its last frame.
  ~LightNode() override {
    if (!output_) return;
    std::string error;
    if (!output_->Write(EncodeWs2812({}, buffer_leds_), &error)) {
      RCLCPP_ERROR(get_logger(), "Cannot darken the lights: %s", error.c_str());
    }
  }

 private:
  LightConfig LoadConfig() {
    LightConfig config;
    output_kind_ = declare_parameter("output", std::string("none"));
    device_ = declare_parameter("device", std::string("/dev/spidev3.0"));
    file_ = declare_parameter("file", std::string());
    const int64_t leds = declare_parameter("leds", static_cast<int64_t>(config.leds));
    const int64_t buffer_leds = declare_parameter("buffer_leds", static_cast<int64_t>(60));
    frame_rate_ = declare_parameter("frame_rate", 25.0);
    config.max_brightness = declare_parameter("max_brightness", config.max_brightness);
    config.max_current_ma = declare_parameter("max_current_ma", config.max_current_ma);
    config.low_battery = declare_parameter("low_battery", config.low_battery);
    const std::string data_dir = declare_parameter("data_dir", std::string());
    if (!data_dir.empty()) config.profile_path = data_dir + "/light_profile.json";

    std::string why;
    if (output_kind_ != "none" && output_kind_ != "spidev" && output_kind_ != "file") {
      why = "output is none, spidev or file";
    } else if (output_kind_ == "file" && file_.empty()) {
      why = "output file needs file";
    } else if (buffer_leds < 1 || buffer_leds > kMaxBufferLeds || leds < 1 || leds > buffer_leds) {
      why = "1 <= leds <= buffer_leds <= " + std::to_string(kMaxBufferLeds);
    } else if (!(frame_rate_ >= 1 && frame_rate_ <= 50)) {
      why = "frame_rate is 1 to 50";
    } else if (!(config.max_brightness >= 0 && config.max_brightness <= 1) || !(config.max_current_ma >= 0) ||
               !(config.low_battery >= 0 && config.low_battery <= 1)) {
      why = "max_brightness and low_battery are 0 to 1, max_current_ma at least 0";
    }
    if (!why.empty()) {
      RCLCPP_ERROR(get_logger(), "Invalid parameters (%s): the lights stay off", why.c_str());
      output_kind_ = "none";
    }
    config.leds = static_cast<size_t>(std::max<int64_t>(1, leds));
    buffer_leds_ = static_cast<size_t>(std::max<int64_t>(1, buffer_leds));
    return config;
  }

  void Tick() {
    const double now = MonotonicSeconds();
    if (!output_ && now >= next_open_) Open(now);
    if (!output_) return;
    std::vector<uint8_t> frame = EncodeWs2812(service_.Frame(now), buffer_leds_);
    if (frame == last_frame_ && now - last_write_ < kRepeat) return;
    std::string error;
    if (!output_->Write(frame, &error)) {
      RCLCPP_ERROR(get_logger(), "Cannot send a frame: %s; trying again in %.0f s", error.c_str(), kReopen);
      output_.reset();
      next_open_ = now + kReopen;
      last_frame_.clear();
      return;
    }
    last_frame_ = std::move(frame);
    last_write_ = now;
  }

  void Open(double now) {
    next_open_ = now + kReopen;
    if (output_kind_ == "file") {
      output_ = OpenFile(file_);
      return;
    }
    const auto users = tron::ProcessesUsing(device_);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "%s is open in another process (pid %d), not opening it",
                            device_.c_str(), users.front());
      return;
    }
    std::string error;
    output_ = OpenSpidev(device_, &error);
    if (output_) {
      RCLCPP_INFO(get_logger(), "Opened %s", device_.c_str());
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "%s", error.c_str());
    }
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    const auto& methods = LightService::Methods();
    if (std::find(methods.begin(), methods.end(), request.method) == methods.end()) return;
    Json params = Json::object();
    if (!request.params.empty()) params = Json::parse(request.params, nullptr, false);
    RpcOutcome outcome;
    if (params.is_discarded()) {
      outcome = {false, Json(), kRpcInvalidParams, "The parameters are not valid JSON"};
    } else {
      outcome = *service_.Call(request.method, params, MonotonicSeconds());
    }
    if (outcome.ok) {
      mowbite_msgs::msg::RpcResponse response;
      response.result = outcome.result.dump();
      response.id = request.id;
      response_pub_->publish(response);
    } else {
      RCLCPP_WARN(get_logger(), "%s refused: %s", request.method.c_str(), outcome.message.c_str());
      mowbite_msgs::msg::RpcError error;
      error.code = outcome.code;
      error.message = outcome.message;
      error.id = request.id;
      error_pub_->publish(error);
    }
  }

  // Set by LoadConfig, so declared before service_.
  std::string output_kind_;
  std::string device_;
  std::string file_;
  size_t buffer_leds_ = 60;
  double frame_rate_ = 25.0;
  LightService service_;
  std::unique_ptr<Output> output_;
  double next_open_ = 0;
  std::vector<uint8_t> last_frame_;
  double last_write_ = 0;

  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowbite_light

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_light::LightNode)
