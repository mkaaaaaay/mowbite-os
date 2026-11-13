#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <mowbite_msgs/msg/button_event.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "mowbite_bridge/json_rpc.h"
#include "mowbite_light/light_service.h"
#include "mowbite_light/output.h"
#include "mowbite_light/ws2812.h"
#include "mowbite_watch/attach.h"
#include "tron_mcu/serial_port.h"

namespace mowbite_light {

namespace {

constexpr int64_t kMaxBufferLeds = 300;
// An unchanged frame goes out again after this long, in case the chain lost one.
constexpr double kRepeat = 1.0;
// After a failed open or write the output is tried again this much later.
constexpr double kReopen = 30.0;

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The parameters of the node; output is none when they are not valid.
struct NodeConfig {
  LightConfig light;
  std::string output;
  std::string device;
  std::string file;
  size_t buffer_leds = 0;
  double frame_rate = 0;
};

}  // namespace

// The lights: the effect of the state of robot_state (the emergency over all) as WS2812 frames on spidev or into a file
// for the simulation, and light.get_profile, light.set_profile, light.preview and light.reset through the bridge.
// With output none it offers no method and sends nothing.
class LightNode : public rclcpp::Node {
 public:
  explicit LightNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_light", options), config_(LoadConfig()), service_(config_.light) {
    watch_ = mowbite_watch::Attach(*this);
    if (config_.output == "none") {
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
    publish_sub_ = create_subscription<mowbite_msgs::msg::MqttPublish>(
        "bridge/publish", 100, [this](mowbite_msgs::msg::MqttPublish::ConstSharedPtr msg) {
          if (msg->topic != "events/json") return;
          service_.OnEvent(Json::parse(msg->payload, nullptr, false), MonotonicSeconds());
        });
    button_sub_ = create_subscription<mowbite_msgs::msg::ButtonEvent>(
        "hw/button", rclcpp::QoS(10).reliable(), [this](const mowbite_msgs::msg::ButtonEvent& msg) {
          using mowbite_msgs::msg::ButtonEvent;
          if (msg.button != ButtonEvent::BUTTON_STOP) return;
          if (msg.press == ButtonEvent::PRESS_HELD) service_.OnStopPress(StopPress::kHeld, MonotonicSeconds());
          if (msg.press == ButtonEvent::PRESS_LONG) service_.OnStopPress(StopPress::kLong, MonotonicSeconds());
          if (msg.press == ButtonEvent::PRESS_CANCELED) service_.OnStopPress(StopPress::kCanceled, MonotonicSeconds());
        });

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_light";
    methods.methods = LightService::Methods();
    methods_pub_->publish(methods);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config_.frame_rate), [this] { Tick(); });
  }

  // A dark chain once the OS ends; a WS2812 keeps its last frame.
  ~LightNode() override {
    if (!output_) return;
    std::string error;
    if (!output_->Write(EncodeWs2812({}, config_.buffer_leds), &error)) {
      RCLCPP_ERROR(get_logger(), "Cannot darken the lights: %s", error.c_str());
    }
  }

 private:
  NodeConfig LoadConfig() {
    NodeConfig config;
    LightConfig& light = config.light;
    config.output = declare_parameter("output", std::string("none"));
    config.device = declare_parameter("device", std::string("/dev/spidev3.0"));
    config.file = declare_parameter("file", std::string());
    const int64_t leds = declare_parameter("leds", static_cast<int64_t>(light.leds));
    const auto strip_texts = declare_parameter("strips", std::vector<std::string>{});
    const int64_t buffer_leds = declare_parameter("buffer_leds", static_cast<int64_t>(60));
    config.frame_rate = declare_parameter("frame_rate", 25.0);
    light.max_brightness = declare_parameter("max_brightness", light.max_brightness);
    light.max_current_ma = declare_parameter("max_current_ma", light.max_current_ma);
    light.ma_per_channel = declare_parameter("ma_per_channel", light.ma_per_channel);
    light.low_battery = declare_parameter("low_battery", light.low_battery);
    const std::string data_dir = declare_parameter("data_dir", std::string());
    if (!data_dir.empty()) light.profile_path = data_dir + "/light_profile.json";

    std::string why;
    if (config.output != "none" && config.output != "spidev" && config.output != "file") {
      why = "output is none, spidev or file";
    } else if (config.output == "file" && config.file.empty()) {
      why = "output file needs file";
    } else if (buffer_leds < 1 || buffer_leds > kMaxBufferLeds || leds < 1 || leds > buffer_leds) {
      why = "1 <= leds <= buffer_leds <= " + std::to_string(kMaxBufferLeds);
    } else if (!(config.frame_rate >= 1 && config.frame_rate <= 50)) {
      why = "frame_rate is 1 to 50";
    } else if (!(light.max_brightness >= 0 && light.max_brightness <= 1) || !(light.max_current_ma >= 0) ||
               !(light.ma_per_channel > 0) || !(light.low_battery >= 0 && light.low_battery <= 1)) {
      why = "max_brightness and low_battery are 0 to 1, max_current_ma at least 0, ma_per_channel more than 0";
    }
    if (!why.empty()) {
      RCLCPP_ERROR(get_logger(), "Invalid parameters (%s): the lights stay off", why.c_str());
      config.output = "none";
    }
    light.leds = static_cast<size_t>(std::max<int64_t>(1, leds));
    config.buffer_leds = static_cast<size_t>(std::max<int64_t>(1, buffer_leds));
    const auto strips = ParseStrips(strip_texts, light.leds, &why);
    if (strips) {
      light.strips = *strips;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid strips (%s): the effects run along the whole chain", why.c_str());
    }
    return config;
  }

  void Tick() {
    const double now = MonotonicSeconds();
    if (!output_ && now >= next_open_) Open(now);
    if (!output_) return;
    std::vector<Rgb> leds = service_.Frame(now);
    if (written_ && leds == last_leds_ && now - last_write_ < kRepeat) return;
    std::string error;
    if (!output_->Write(EncodeWs2812(leds, config_.buffer_leds), &error)) {
      RCLCPP_ERROR(get_logger(), "Cannot send a frame: %s; trying again in %.0f s", error.c_str(), kReopen);
      output_.reset();
      next_open_ = now + kReopen;
      written_ = false;
      return;
    }
    last_leds_ = std::move(leds);
    last_write_ = now;
    written_ = true;
  }

  void Open(double now) {
    next_open_ = now + kReopen;
    if (config_.output == "file") {
      output_ = OpenFile(config_.file);
      return;
    }
    const auto users = tron::ProcessesUsing(config_.device);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "%s is open in another process (pid %d), not opening it",
                            config_.device.c_str(), users.front());
      return;
    }
    std::string error;
    output_ = OpenSpidev(config_.device, &error);
    if (output_) {
      RCLCPP_INFO(get_logger(), "Opened %s", config_.device.c_str());
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "%s", error.c_str());
    }
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    Json params = Json::object();
    if (!request.params.empty()) params = Json::parse(request.params, nullptr, false);
    std::optional<RpcOutcome> outcome;
    if (!params.is_discarded()) {
      outcome = service_.Call(request.method, params, MonotonicSeconds());
    } else if (const auto& methods = LightService::Methods();
               std::find(methods.begin(), methods.end(), request.method) != methods.end()) {
      outcome = RpcOutcome{false, Json(), mowbite_bridge::kInvalidParams, "The parameters are not valid JSON"};
    }
    // A method of another node, which answers it.
    if (!outcome) return;
    if (outcome->ok) {
      mowbite_msgs::msg::RpcResponse response;
      response.result = outcome->result.dump();
      response.id = request.id;
      response_pub_->publish(response);
    } else {
      RCLCPP_WARN(get_logger(), "%s refused: %s", request.method.c_str(), outcome->message.c_str());
      mowbite_msgs::msg::RpcError error;
      error.code = outcome->code;
      error.message = outcome->message;
      error.id = request.id;
      error_pub_->publish(error);
    }
  }

  NodeConfig config_;
  LightService service_;
  std::unique_ptr<Output> output_;
  double next_open_ = 0;
  std::vector<Rgb> last_leds_;
  double last_write_ = 0;
  bool written_ = false;

  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_light

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_light::LightNode)
