#include <algorithm>
#include <chrono>
#include <map>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/srv/add_area.hpp>
#include <mowbite_msgs/srv/replace_map.hpp>
#include <mowbite_msgs/srv/set_dock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "mowbite_bridge/event_builder.h"
#include "mowbite_map/map_service.h"

namespace mowbite_map {

namespace {

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

uint8_t ToMsg(DatumState state) {
  switch (state) {
    case DatumState::kUnknown: return mowbite_msgs::msg::MapStatus::DATUM_UNKNOWN;
    case DatumState::kEmpty: return mowbite_msgs::msg::MapStatus::DATUM_EMPTY;
    case DatumState::kMatch: return mowbite_msgs::msg::MapStatus::DATUM_MATCH;
    case DatumState::kMismatch: return mowbite_msgs::msg::MapStatus::DATUM_MISMATCH;
    case DatumState::kUnbound: return mowbite_msgs::msg::MapStatus::DATUM_UNBOUND;
  }
  return mowbite_msgs::msg::MapStatus::DATUM_UNKNOWN;
}

}  // namespace

// Keeps the map: map/json for the app and the planner, the RPC methods map.replace and map.area_properties through the
// bridge, services for the logic and the planner and the status that says whether the map fits the datum.
class MapNode : public rclcpp::Node {
 public:
  explicit MapNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_map", options), service_(LoadConfig()) {
    map_pub_ = create_publisher<std_msgs::msg::String>("map/json", LatchedQos(1));
    status_pub_ = create_publisher<mowbite_msgs::msg::MapStatus>("map/status", LatchedQos(1));
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);

    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    // The methods other nodes offer: with planner.settings the app is told that the map keeps the planner's settings.
    methods_sub_ = create_subscription<mowbite_msgs::msg::RpcMethods>(
        "bridge/rpc/methods", LatchedQos(50), [this](mowbite_msgs::msg::RpcMethods::ConstSharedPtr msg) {
          if (msg->node_id == "mowbite_map") return;
          offers_planner_[msg->node_id] =
              std::find(msg->methods.begin(), msg->methods.end(), "planner.settings") != msg->methods.end();
          bool any = false;
          for (const auto& [_, offered] : offers_planner_) any = any || offered;
          service_.SetPlannerSettingsOffered(any);
        });
    datum_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/datum", LatchedQos(1), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
          service_.SetDatum({msg->latitude, msg->longitude, msg->altitude});
          Flush();
        });
    add_area_srv_ = create_service<mowbite_msgs::srv::AddArea>(
        "map/add_area", [this](const std::shared_ptr<mowbite_msgs::srv::AddArea::Request> request,
                               std::shared_ptr<mowbite_msgs::srv::AddArea::Response> response) {
          std::vector<Point> outline;
          for (const auto& p : request->outline) outline.push_back({p.x, p.y});
          std::vector<std::vector<Point>> obstacles;
          for (const auto& ring : request->obstacles) {
            obstacles.emplace_back();
            for (const auto& p : ring.points) obstacles.back().push_back({p.x, p.y});
          }
          const EditResult result = service_.AddArea(request->type, request->name, outline, obstacles);
          response->success = result.ok;
          response->id = result.id;
          response->message = result.message;
          response->code = result.code;
          Flush();
        });
    set_dock_srv_ = create_service<mowbite_msgs::srv::SetDock>(
        "map/set_dock", [this](const std::shared_ptr<mowbite_msgs::srv::SetDock::Request> request,
                               std::shared_ptr<mowbite_msgs::srv::SetDock::Response> response) {
          const EditResult result = service_.SetDock({request->x, request->y}, request->heading);
          response->success = result.ok;
          response->id = result.id;
          response->message = result.message;
          response->code = result.code;
          Flush();
        });

    // map.replace for nodes of the stack (the planner), with a version check and an answer of its own.
    replace_srv_ = create_service<mowbite_msgs::srv::ReplaceMap>(
        "map/replace", [this](const std::shared_ptr<mowbite_msgs::srv::ReplaceMap::Request> request,
                              std::shared_ptr<mowbite_msgs::srv::ReplaceMap::Response> response) {
          const RpcOutcome outcome = service_.ReplaceChecked(request->map, request->expected_sha256);
          response->success = outcome.ok;
          response->code = outcome.code;
          response->message = outcome.ok ? outcome.result.get<std::string>() : outcome.message;
          Flush();
        });

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_map";
    methods.methods = MapService::Methods();
    methods_pub_->publish(methods);
    Flush();
    // Events wait until the bridge listens, and go out once a second at the latest.
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { Flush(); });
  }

 private:
  MapService::Config LoadConfig() {
    MapService::Config config;
    config.data_dir = declare_parameter("data_dir", ".");
    config.max_bytes = static_cast<size_t>(declare_parameter("max_map_bytes", static_cast<int64_t>(config.max_bytes)));
    config.datum_tolerance_m = declare_parameter("datum_tolerance", config.datum_tolerance_m);
    return config;
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    json params;
    if (!request.params.empty()) params = json::parse(request.params, nullptr, false);
    std::optional<RpcOutcome> outcome;
    if (params.is_discarded()) {
      if (std::find(MapService::Methods().begin(), MapService::Methods().end(), request.method) !=
          MapService::Methods().end()) {
        outcome = RpcOutcome{false, json(), kRpcInvalidParams, "The parameters are not valid JSON"};
      }
    } else {
      outcome = service_.Call(request.method, params);
    }
    // A method of another node, which answers it.
    if (!outcome) return;
    if (outcome->ok) {
      mowbite_msgs::msg::RpcResponse response;
      response.result = outcome->result.dump();
      response.id = request.id;
      response_pub_->publish(response);
    } else {
      mowbite_msgs::msg::RpcError error;
      error.code = static_cast<int16_t>(outcome->code);
      error.message = outcome->message;
      error.id = request.id;
      error_pub_->publish(error);
    }
    Flush();
  }

  // Sends what the service collected: log lines, the map, the status and, once the bridge listens, the events.
  void Flush() {
    for (const auto& notice : service_.TakeNotices()) {
      switch (notice.level) {
        case Notice::Level::kInfo: RCLCPP_INFO(get_logger(), "%s", notice.text.c_str()); break;
        case Notice::Level::kWarn: RCLCPP_WARN(get_logger(), "%s", notice.text.c_str()); break;
        case Notice::Level::kError: RCLCPP_ERROR(get_logger(), "%s", notice.text.c_str()); break;
      }
    }
    if (service_.TakeChanged()) {
      std_msgs::msg::String msg;
      msg.data = service_.map_text();
      map_pub_->publish(msg);
    }
    if (service_.TakeStatusChanged()) PublishStatus();
    for (auto& event : service_.TakeEvents()) pending_events_.push_back(std::move(event));
    if (!pending_events_.empty() && publish_pub_->get_subscription_count() > 0) {
      for (const auto& event : pending_events_) {
        mowbite_msgs::msg::MqttPublish msg;
        msg.topic = "events/json";
        msg.payload = mowbite_bridge::BuildEvent(event.type, event.details, now().seconds()).dump();
        publish_pub_->publish(msg);
      }
      pending_events_.clear();
    }
  }

  void PublishStatus() {
    const Status& status = service_.status();
    mowbite_msgs::msg::MapStatus msg;
    msg.datum_state = ToMsg(status.datum);
    msg.mowing_allowed = status.mowing_allowed;
    msg.reason = status.reason;
    msg.areas = static_cast<uint32_t>(status.areas);
    msg.docking_stations = static_cast<uint32_t>(status.docking_stations);
    status_pub_->publish(msg);
  }

  MapService service_;
  std::vector<Event> pending_events_;
  std::map<std::string, bool> offers_planner_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MapStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr datum_sub_;
  rclcpp::Service<mowbite_msgs::srv::AddArea>::SharedPtr add_area_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetDock>::SharedPtr set_dock_srv_;
  rclcpp::Service<mowbite_msgs::srv::ReplaceMap>::SharedPtr replace_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowbite_map

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_map::MapNode)
