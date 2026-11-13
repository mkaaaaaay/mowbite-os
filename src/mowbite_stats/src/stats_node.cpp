#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/parts_use.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "mowbite_hold/file_io.h"
#include "mowbite_hold/file_writer.h"
#include "mowbite_stats/counter.h"
#include "mowbite_stats/stats_rpc.h"
#include "mowbite_watch/attach.h"

namespace mowbite_stats {

namespace {

constexpr char kStateFile[] = "stats/state.json";
constexpr char kHoursFile[] = "stats/hours.json";
constexpr char kPartsFile[] = "stats/parts.json";
// hours.json at kMaxHours rows is about 0.5 MB.
constexpr size_t kMaxFileBytes = size_t{2} * 1024 * 1024;
// The state goes to the disk this often (s); at most this much counting is lost when the OS stops hard.
constexpr double kSaveEvery = 60.0;

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

}  // namespace

// Mowing statistics and maintenance: blade time, distance and runs per hour from hw/status and bridge/robot_state,
// the blade hours of the parts since they were last seen to; stats.get, .part_done and .part_due through the bridge,
// stats/parts for the pushes of mowbite_net. Files under <data_dir>/stats. Reads only, moves nothing.
class StatsNode : public rclcpp::Node {
 public:
  explicit StatsNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_stats", mowbite_component::StartupOnly(options)),
        data_dir_(declare_parameter("data_dir", std::string())),
        counter_(Read(kStateFile), Read(kHoursFile), Read(kPartsFile), WallSeconds()),
        writer_(mowbite_hold::WriteFileSafely, [this](const mowbite_hold::FileWriter::Done& done) {
          if (!done.ok) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", done.path.c_str(), done.error.c_str());
        }) {
    parts_pub_ = create_publisher<mowbite_msgs::msg::PartsUse>("stats/parts", LatchedQos(1));
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100,
        [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) {
          // The motor's report where there is one, else what the driver commanded.
          blade_ = msg->blade_motor.ok ? msg->blade_motor.rpm != 0 : msg->blade_on;
        });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) { OnState(*msg); });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_stats";
    methods.methods = StatsMethods();
    methods_pub_->publish(methods);
    PublishParts();
    if (data_dir_.empty()) RCLCPP_WARN(get_logger(), "No data_dir: the statistics are not kept");
    timer_ = create_wall_timer(std::chrono::duration<double>(kSaveEvery), [this] { Save(false); });
    watch_ = mowbite_watch::Attach(*this);
  }

  ~StatsNode() override { Save(false); }

 private:
  json Read(const char* name) {
    if (data_dir_.empty()) return json();
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(PathOf(name), kMaxFileBytes, &text, &error);
    if (read == mowbite_hold::FileRead::kMissing) return json();
    if (read != mowbite_hold::FileRead::kText) {
      RCLCPP_ERROR(get_logger(), "Cannot read %s: %s", name, error.empty() ? "too large" : error.c_str());
      return json();
    }
    json value = json::parse(text, nullptr, false);
    if (value.is_discarded()) RCLCPP_ERROR(get_logger(), "%s is no JSON, counting starts anew", name);
    return value.is_discarded() ? json() : value;
  }

  std::string PathOf(const char* name) const { return (std::filesystem::path(data_dir_) / name).string(); }

  void Write(const char* name, const json& value) {
    if (!data_dir_.empty()) writer_.Submit(PathOf(name), value.dump() + "\n");
  }

  void OnState(const mowbite_msgs::msg::RobotState& state) {
    Sample sample;
    sample.wall = WallSeconds();
    sample.mono = MonotonicSeconds();
    sample.blade = blade_;
    sample.state = state.current_state;
    sample.pose = state.gps_percentage > 0 && std::isfinite(state.pose_x) && std::isfinite(state.pose_y);
    sample.x = state.pose_x;
    sample.y = state.pose_y;
    if (counter_.Add(sample).hour_closed) Save(true);
  }

  void Save(bool hours) {
    if (hours) Write(kHoursFile, counter_.HoursJson());
    const json state = counter_.StateJson();
    if (state != saved_) {
      Write(kStateFile, state);
      saved_ = state;
    }
    PublishParts();
  }

  // On every change of the blade hours to a hundredth, of due or of a part.
  void PublishParts() {
    mowbite_msgs::msg::PartsUse msg;
    for (const Part& part : counter_.parts()) {
      mowbite_msgs::msg::PartUse use;
      use.key = part.key;
      use.used_h = std::round(counter_.UsedHours(part) * 100.0) / 100.0;
      use.due_h = part.due_h;
      use.done_at = part.done_at;
      use.due = use.used_h >= part.due_h;
      msg.parts.push_back(use);
    }
    if (published_ && msg == *published_) return;
    published_ = msg;
    parts_pub_->publish(msg);
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    const auto& methods = StatsMethods();
    if (std::find(methods.begin(), methods.end(), request.method) == methods.end()) return;
    const json params = request.params.empty() ? json() : json::parse(request.params, nullptr, false);
    RpcOutcome out;
    if (params.is_discarded()) {
      out.error = RpcError{kInvalidParams, "The parameters are not valid JSON", nullptr};
    } else {
      out = HandleStats(counter_, request.method, params, WallSeconds());
    }
    if (out.error) {
      mowbite_msgs::msg::RpcError msg;
      msg.id = request.id;
      msg.code = static_cast<int16_t>(out.error->code);
      msg.message = out.error->message;
      msg.data = out.error->data.is_null() ? "" : out.error->data.dump();
      error_pub_->publish(msg);
      return;
    }
    if (out.parts_changed) {
      Write(kPartsFile, counter_.PartsJson());
      RCLCPP_INFO(get_logger(), "%s: %s", request.method.c_str(), params.dump().c_str());
      PublishParts();
    }
    mowbite_msgs::msg::RpcResponse msg;
    msg.id = request.id;
    msg.result = out.result.dump();
    response_pub_->publish(msg);
  }

  const std::string data_dir_;
  Counter counter_;
  mowbite_hold::FileWriter writer_;
  bool blade_ = false;
  json saved_;
  std::optional<mowbite_msgs::msg::PartsUse> published_;

  rclcpp::Publisher<mowbite_msgs::msg::PartsUse>::SharedPtr parts_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_stats

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_stats::StatsNode)
