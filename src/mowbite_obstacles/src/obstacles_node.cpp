#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/obstacle_image.hpp>
#include <mowbite_msgs/msg/obstacle_status.hpp>
#include <mowbite_msgs/msg/obstacles.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <optional>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "mowbite_bridge/event_builder.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_obstacles/obstacle_service.h"
#include "mowbite_obstacles/tracker.h"
#include "mowbite_watch/attach.h"

namespace mowbite_obstacles {

namespace {

constexpr auto kStatusPeriod = std::chrono::milliseconds(100);
constexpr auto kSlowPeriod = std::chrono::seconds(1);
// Events kept while the bridge is not there yet.
constexpr size_t kMaxPendingEvents = 32;

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double Seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string Dump(const Json& json) {
  return json.dump(-1, ' ', false, Json::error_handler_t::replace);
}

Frame ToFrame(const mowbite_msgs::msg::Obstacles& msg) {
  Frame frame;
  frame.source = msg.source;
  frame.stamp = Seconds(msg.header.stamp);
  frame.sight = msg.sight;
  frame.reason = msg.reason;
  frame.view = {msg.view_x, msg.view_y, msg.view_angle_left, msg.view_angle_right, msg.view_range_min,
                msg.view_range_max};
  for (const auto& obstacle : msg.obstacles) {
    Find find;
    if (!ParseKind(obstacle.kind, &find.kind)) find.kind = Kind::kObject;
    find.class_name = obstacle.class_name;
    find.score = obstacle.score;
    find.x = obstacle.x;
    find.y = obstacle.y;
    find.radius = std::max(0.0f, obstacle.radius);
    find.height = obstacle.height;
    find.cov_xx = obstacle.cov_xx;
    find.cov_xy = obstacle.cov_xy;
    find.cov_yy = obstacle.cov_yy;
    find.box_x = obstacle.box_x;
    find.box_y = obstacle.box_y;
    find.box_w = obstacle.box_w;
    find.box_h = obstacle.box_h;
    if (!std::isfinite(find.x) || !std::isfinite(find.y) || !std::isfinite(find.score)) continue;
    frame.finds.push_back(find);
  }
  return frame;
}

}  // namespace

// The obstacles the sensors of the platform report on hw/obstacles: tracks in the map with the sight of each sensor
// and the zones ahead on obstacles/status, and the list with photos for the app (vision.obstacles.*) where the
// platform has such a sensor. Nothing here drives the mower.
class ObstaclesNode : public rclcpp::Node {
 public:
  explicit ObstaclesNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_obstacles", mowbite_component::StartupOnly(options)), tracker_(ReadConfig()) {
    const std::string data_dir = declare_parameter("data_dir", std::string());
    ServiceConfig service;
    if (!data_dir.empty()) service.dir = (std::filesystem::path(data_dir) / "obstacles").string();
    service.limits.entries = static_cast<size_t>(
        std::max<int64_t>(1, declare_parameter("list_entries", static_cast<int64_t>(service.limits.entries))));
    service.limits.bytes = static_cast<int64_t>(
        declare_parameter("list_max_mb", static_cast<double>(service.limits.bytes) / (1024.0 * 1024.0)) * 1024.0 *
        1024.0);
    service.limits.days = declare_parameter("list_days", service.limits.days);
    service_ = std::make_unique<ObstacleService>(
        service, [this](const std::string& text) { RCLCPP_WARN(get_logger(), "%s", text.c_str()); });
    service_->Load(Now());

    status_pub_ = create_publisher<mowbite_msgs::msg::ObstacleStatus>("obstacles/status", 10);
    photos_pub_ = create_publisher<std_msgs::msg::Bool>("obstacles/photos_wanted", LatchedQos(1));
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    FollowImages();
    capabilities_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1),
        [this](const mowbite_msgs::msg::HardwareCapabilities& msg) { OnCapabilities(msg.obstacle_sources); });
    pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
        "localization/pose", 10, [this](const mowbite_msgs::msg::RobotPose& msg) {
          tracker_.OnPose({Seconds(msg.header.stamp), msg.x, msg.y, msg.heading, msg.position_accuracy});
        });
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", LatchedQos(50), [this](const std_msgs::msg::String& msg) { OnParams(msg.data); });
    status_timer_ = create_wall_timer(kStatusPeriod, [this] { OnStatus(); });
    slow_timer_ = create_wall_timer(kSlowPeriod, [this] { OnSlow(); });
    watch_ = mowbite_watch::Attach(*this);
  }

  ~ObstaclesNode() override { service_->Flush(); }

 private:
  TrackerConfig ReadConfig() {
    TrackerConfig c;
    c.min_score_person = declare_parameter("min_score_person", c.min_score_person);
    c.min_score_animal = declare_parameter("min_score_animal", c.min_score_animal);
    c.min_score_object = declare_parameter("min_score_object", c.min_score_object);
    c.confirm_hits = static_cast<int>(declare_parameter("confirm_hits", static_cast<int64_t>(c.confirm_hits)));
    c.confirm_frames = static_cast<int>(declare_parameter("confirm_frames", static_cast<int64_t>(c.confirm_frames)));
    if (c.confirm_hits < 1 || c.confirm_frames < c.confirm_hits) {
      throw std::runtime_error("confirm_hits is 1 to confirm_frames");
    }
    c.match_distance = declare_parameter("match_distance", c.match_distance);
    c.match_max = declare_parameter("match_max", c.match_max);
    c.living_speed = declare_parameter("living_speed", c.living_speed);
    c.object_speed = declare_parameter("object_speed", c.object_speed);
    c.free_seen = declare_parameter("free_seen", c.free_seen);
    c.track_timeout = declare_parameter("track_timeout", c.track_timeout);
    c.candidate_timeout = declare_parameter("candidate_timeout", c.candidate_timeout);
    c.pose_gap = declare_parameter("pose_gap", c.pose_gap);
    c.stale = declare_parameter("stale", c.stale);
    c.stop_living_ahead = declare_parameter("stop_living_ahead", c.stop_living_ahead);
    c.stop_living_margin = declare_parameter("stop_living_margin", c.stop_living_margin);
    c.stop_living_around = declare_parameter("stop_living_around", c.stop_living_around);
    c.stop_object_ahead = declare_parameter("stop_object_ahead", c.stop_object_ahead);
    c.stop_object_margin = declare_parameter("stop_object_margin", c.stop_object_margin);
    c.slow_ahead = declare_parameter("slow_ahead", c.slow_ahead);
    c.zone_sigma = declare_parameter("zone_sigma", c.zone_sigma);
    c.unknown_stops = declare_parameter("unknown_stops", c.unknown_stops);
    return c;
  }

  double Now() { return now().seconds(); }

  void OnCapabilities(const std::vector<std::string>& sources) {
    if (sources == tracker_.sources()) return;
    tracker_.SetSources(sources);
    if (!sources.empty() && !obstacles_sub_) {
      RCLCPP_INFO(get_logger(), "Obstacle sensors: %zu", sources.size());
      obstacles_sub_ = create_subscription<mowbite_msgs::msg::Obstacles>(
          "hw/obstacles", 10, [this](const mowbite_msgs::msg::Obstacles& msg) { OnObstacles(msg); });
      StartRpc();
    }
    FollowImages();
  }

  // The methods are offered from the first obstacle sensor on.
  void StartRpc() {
    if (methods_pub_) return;
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](const mowbite_msgs::msg::RpcRequest& msg) { OnRequest(msg); });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_obstacles";
    methods.methods = ObstacleService::Methods();
    methods_pub_->publish(methods);
  }

  void OnParams(const std::string& text) {
    const Json value = Json::parse(text, nullptr, false);
    if (!value.is_object() || !value.contains("params") || !value["params"].is_object()) return;
    const std::string node = value.value("node", "");
    const Json& params = value["params"];
    const auto number = [&params](const char* name) -> std::optional<double> {
      const auto it = params.find(name);
      if (it == params.end() || !it->is_number()) return std::nullopt;
      return it->get<double>();
    };
    if (node == "/mower_logic") {
      if (const auto mode = number("obstacle_mode")) service_->SetMode(static_cast<int>(*mode));
      if (const auto photos = number("obstacle_photos")) service_->SetPhotos(*photos != 0.0);
      FollowImages();
    } else if (node == "/mowbite_geometry") {
      const auto front = number("robot_front"), rear = number("robot_rear"), width = number("robot_width");
      if (front && rear && width) tracker_.SetBody({*front, *rear, 0.5 * *width});
    }
  }

  // Images are wanted, and taken, only while there is a sensor and the service wants them, so none reach this process
  // otherwise.
  void FollowImages() {
    const bool wanted = service_->PhotosWanted() && !tracker_.sources().empty();
    if (!photos_wanted_ || *photos_wanted_ != wanted) {
      photos_wanted_ = wanted;
      std_msgs::msg::Bool msg;
      msg.data = wanted;
      photos_pub_->publish(msg);
    }
    if (!wanted) {
      images_sub_.reset();
      return;
    }
    if (images_sub_) return;
    images_sub_ = create_subscription<mowbite_msgs::msg::ObstacleImage>(
        "hw/obstacle_images", 2, [this](const mowbite_msgs::msg::ObstacleImage& msg) {
          service_->OnImage(tracker_,
                            {msg.source, Seconds(msg.header.stamp), static_cast<int>(msg.width),
                             static_cast<int>(msg.height), std::string(msg.jpeg.begin(), msg.jpeg.end())});
          Publish();
        });
  }

  void OnObstacles(const mowbite_msgs::msg::Obstacles& msg) {
    const double now = Now();
    const FrameResult result = tracker_.OnFrame(ToFrame(msg), now);
    if (result.no_pose) RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 60000, "Frames of %s without a pose",
                                             msg.source.c_str());
    service_->OnFrame(tracker_, result, now);
    service_->OnEnded(result.ended);
    Publish();
  }

  void OnStatus() {
    const double now = Now();
    service_->OnEnded(tracker_.Tick(now));
    const Status status = tracker_.StatusAt(now);
    service_->SetSight(status.sight, status.reason, status.source, now);
    mowbite_msgs::msg::ObstacleStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.sight = status.sight;
    msg.reason = status.reason;
    msg.source = status.source;
    msg.stop = status.stop;
    msg.stop_id = status.stop_id;
    msg.stop_kind = status.stop ? KindName(status.stop_kind) : "";
    msg.slow = status.slow;
    for (const TrackView& view : status.tracks) {
      const Track& track = *view.track;
      mowbite_msgs::msg::ObstacleTrack out;
      out.id = track.id;
      out.kind = KindName(track.kind);
      out.class_name = track.class_name;
      out.score = static_cast<float>(track.score);
      out.x = track.x;
      out.y = track.y;
      out.radius = static_cast<float>(track.radius);
      out.sigma = static_cast<float>(view.sigma);
      out.confirmed = track.confirmed;
      out.in_view = track.in_view;
      out.age = static_cast<float>(now - track.last_seen);
      out.ahead = static_cast<float>(view.ahead);
      out.aside = static_cast<float>(view.aside);
      msg.tracks.push_back(out);
    }
    status_pub_->publish(msg);
  }

  void OnSlow() {
    service_->Tick(Now());
    Publish();
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    const auto& methods = ObstacleService::Methods();
    if (std::find(methods.begin(), methods.end(), request.method) == methods.end()) return;
    Json params = Json::object();
    if (!request.params.empty()) params = Json::parse(request.params, nullptr, false);
    const Outcome outcome =
        params.is_discarded()
            ? Outcome{false, Json(), mowbite_bridge::kInvalidParams, "The parameters are not valid JSON"}
            : service_->Call(request.method, params);
    if (outcome.ok) {
      mowbite_msgs::msg::RpcResponse response;
      response.result = Dump(outcome.result);
      response.id = request.id;
      response_pub_->publish(response);
    } else {
      RCLCPP_INFO(get_logger(), "%s refused: %s", request.method.c_str(), outcome.message.c_str());
      mowbite_msgs::msg::RpcError refused;
      refused.code = outcome.code;
      refused.message = outcome.message;
      refused.id = request.id;
      error_pub_->publish(refused);
    }
    Publish();
  }

  // Events for events/json, kept while the bridge is not there yet.
  void Publish() {
    for (auto& [type, details] : service_->TakeEvents()) {
      if (pending_events_.size() >= kMaxPendingEvents) pending_events_.erase(pending_events_.begin());
      pending_events_.push_back(mowbite_bridge::BuildEvent(type, details, Now()));
    }
    if (pending_events_.empty() || publish_pub_->get_subscription_count() == 0) return;
    for (const Json& event : pending_events_) {
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = Dump(event);
      publish_pub_->publish(msg);
    }
    pending_events_.clear();
  }

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  Tracker tracker_;
  std::unique_ptr<ObstacleService> service_;
  std::optional<bool> photos_wanted_;
  std::vector<Json> pending_events_;

  rclcpp::Publisher<mowbite_msgs::msg::ObstacleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr photos_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr capabilities_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::Obstacles>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ObstacleImage>::SharedPtr images_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr slow_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_obstacles

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_obstacles::ObstaclesNode)
