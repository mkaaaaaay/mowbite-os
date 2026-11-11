#include <ament_index_cpp/get_package_prefix.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mowbite_bridge/event_builder.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_camera/camera_service.h"
#include "mowbite_camera/http_server.h"
#include "mowbite_camera/stream_config.h"
#include "mowbite_camera/stream_server.h"
#include "mowbite_camera/stream_supervisor.h"
#include "mowbite_watch/attach.h"

namespace mowbite_camera {

namespace {

// How long the HTTP thread waits in one round, in ms; it signs its watch after each.
constexpr int kPollMs = 500;
// How often effects are passed on and the picture topics follow the viewers, and how often the slow checks run.
constexpr auto kFastPeriod = std::chrono::milliseconds(100);
constexpr auto kSlowPeriod = std::chrono::seconds(1);
constexpr size_t kMaxNoteBytes = 4096;
constexpr size_t kMaxJpegBytes = 4u << 20;
// Events kept while the bridge is not there yet.
constexpr size_t kMaxPendingEvents = 16;
// The longest body of what the stream server asks: its fields with the query and the user agent of a viewer.
constexpr size_t kMaxAuthBytes = 4096;

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double SteadySeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

double WallSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

std::string Dump(const Json& json) {
  return json.dump(-1, ' ', false, Json::error_handler_t::replace);
}

// The image puts the stream server next to the packages, under <install>/mediamtx.
std::string DefaultMediaServer() {
  try {
    const std::filesystem::path prefix = ament_index_cpp::get_package_prefix("mowbite_camera");
    return (prefix.parent_path() / "mediamtx/mediamtx").string();
  } catch (const std::exception&) {
    return "";
  }
}

uint16_t Port(int64_t value, int64_t lowest, const char* name) {
  if (value < lowest || value > 65535) {
    throw std::runtime_error(std::string(name) + " is " + std::to_string(lowest) + " to 65535");
  }
  return static_cast<uint16_t>(value);
}

CameraEntry ToEntry(const mowbite_msgs::msg::Camera& msg) {
  CameraEntry entry;
  entry.id = msg.id;
  entry.name = msg.name;
  entry.process = msg.process;
  entry.kind = msg.kind;
  entry.position = msg.position;
  entry.width = msg.width;
  entry.height = msg.height;
  entry.detections = msg.detections;
  entry.protocols = msg.protocols;
  entry.source = msg.source;
  entry.available = msg.available;
  entry.reason = msg.reason;
  entry.detail = msg.detail;
  return entry;
}

}  // namespace

// The cameras of the platform for the app, in a process of its own: camera.list and camera.stream through the
// bridge, CAMERA_CHANGED, and the pictures of camera/<id>/jpeg over HTTP to the viewers that hold a place. The
// HTTP server has a thread of its own, so a slow viewer never holds up the node. Cameras with webrtc go through a
// stream server, a child process this node starts, configures and answers when it asks who may watch.
class CameraNode : public rclcpp::Node {
 public:
  explicit CameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_camera", options) {
    data_dir_ = declare_parameter("data_dir", std::string());
    ServiceConfig config;
    config.public_host = declare_parameter("public_host", std::string());
    // 0 for mjpeg_port: any free one.
    const uint16_t mjpeg_port = Port(declare_parameter("mjpeg_port", int64_t{config.mjpeg_port}), 0, "mjpeg_port");
    stream_.webrtc_port = Port(declare_parameter("webrtc_port", int64_t{stream_.webrtc_port}), 1, "webrtc_port");
    stream_.ice_port = Port(declare_parameter("ice_port", int64_t{stream_.ice_port}), 1, "ice_port");
    stream_.api_port = Port(declare_parameter("media_api_port", int64_t{stream_.api_port}), 1, "media_api_port");
    std::string media_server = declare_parameter("media_server", std::string());
    if (media_server.empty()) media_server = DefaultMediaServer();

    server_ = std::make_unique<HttpServer>(
        [this](const HttpRequest& request) {
          return request.listener == auth_listener_ ? service_->OnAuth(request) : service_->OnViewer(request);
        },
        [this](uint64_t connection) { service_->OnViewerGone(connection); }, SteadySeconds);
    ListenerRules viewers;
    viewers.any_origin = true;
    std::string error;
    const int viewer_listener = server_->Listen("0.0.0.0", mjpeg_port, viewers, &error);
    if (viewer_listener < 0) throw std::runtime_error("cannot listen for viewers: " + error);
    // Only the stream server on this host asks here, on a port the configuration tells it.
    ListenerRules asking;
    asking.max_body = kMaxAuthBytes;
    auth_listener_ = server_->Listen("127.0.0.1", 0, asking, &error);
    if (auth_listener_ < 0) throw std::runtime_error("cannot listen for the stream server: " + error);
    stream_.auth_port = server_->Port(auth_listener_);
    config.mjpeg_port = server_->Port(viewer_listener);
    config.webrtc_port = stream_.webrtc_port;
    service_ = std::make_unique<CameraService>(config, Clocks{SteadySeconds, WallSeconds});
    api_ = {stream_.api_port, service_->api_user(), service_->api_password()};
    runtime_dir_ = RuntimeDir(declare_parameter("runtime_dir", std::string()), &runtime_problem_);
    if (runtime_dir_.empty()) RCLCPP_WARN(get_logger(), "No webrtc: %s", runtime_problem_.c_str());
    media_ = std::make_unique<MediaServer>(media_server, runtime_dir_);
    supervisor_ = std::make_unique<StreamSupervisor>(media_.get(), SteadySeconds);
    service_->SetWebrtc(supervisor_->reason(), supervisor_->detail());

    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    cameras_sub_ = create_subscription<mowbite_msgs::msg::Camera>(
        "hw/cameras", LatchedQos(16), [this](mowbite_msgs::msg::Camera::ConstSharedPtr msg) { OnCamera(*msg); });
    fast_timer_ = create_wall_timer(kFastPeriod, [this] { OnFast(); });
    slow_timer_ = create_wall_timer(kSlowPeriod, [this] { OnSlow(); });
    RCLCPP_INFO(get_logger(), "Pictures for viewers on port %d", config.mjpeg_port);
    thread_ = std::thread([this] { Serve(); });
    watch_ = mowbite_watch::Attach(*this);
  }

  ~CameraNode() override {
    media_->Stop();
    running_ = false;
    server_->Post([] {});
    if (thread_.joinable()) thread_.join();
  }

 private:
  void Serve() {
    mowbite_watch::Watch watch("mowbite_camera_http", mowbite_watch::kLimit);
    while (running_) {
      server_->Poll(kPollMs);
      watch.Sign();
    }
  }

  void OnCamera(const mowbite_msgs::msg::Camera& msg) {
    const std::string problem = service_->OnCamera(ToEntry(msg));
    if (!problem.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 60000, "hw/cameras: %s", problem.c_str());
    }
    if (!methods_pub_ && service_->HasCameras()) StartRpc();
    Apply();
  }

  // The methods are offered from the first camera on.
  void StartRpc() {
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_camera";
    methods.methods = CameraService::Methods();
    methods_pub_->publish(methods);
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    // A method of another node, which answers it; its params are not parsed here.
    const auto& methods = CameraService::Methods();
    if (std::find(methods.begin(), methods.end(), request.method) == methods.end()) return;
    Json params = Json::object();
    if (!request.params.empty()) params = Json::parse(request.params, nullptr, false);
    const std::optional<RpcOutcome> outcome =
        params.is_discarded()
            ? RpcOutcome{false, Json(), mowbite_bridge::kInvalidParams, "The parameters are not valid JSON", Json()}
            : service_->Call(request.method, params);
    if (!outcome) return;
    if (outcome->ok) {
      mowbite_msgs::msg::RpcResponse response;
      response.result = Dump(outcome->result);
      response.id = request.id;
      response_pub_->publish(response);
      return;
    }
    RCLCPP_INFO(get_logger(), "%s refused: %s", request.method.c_str(), outcome->message.c_str());
    mowbite_msgs::msg::RpcError refused;
    refused.code = outcome->code;
    refused.message = outcome->message;
    refused.id = request.id;
    if (!outcome->data.is_null()) refused.data = Dump(outcome->data);
    error_pub_->publish(refused);
  }

  // What the service left to do: events for the bridge, connections and sessions to end.
  void Apply() {
    Effects effects = service_->TakeEffects();
    for (Json& event : effects.events) {
      RCLCPP_INFO(get_logger(), "Camera changed: %s", Dump(event).c_str());
      if (pending_events_.size() >= kMaxPendingEvents) pending_events_.erase(pending_events_.begin());
      pending_events_.push_back(std::move(event));
    }
    for (const uint64_t connection : effects.close) {
      server_->Post([this, connection] { server_->Close(connection); });
    }
    for (const std::string& session : effects.kick) {
      if (supervisor_->Up() && !KickStreamSession(api_, session)) {
        RCLCPP_WARN(get_logger(), "The stream server did not end session %s", session.c_str());
      }
    }
  }

  void OnFast() {
    Apply();
    if (!pending_events_.empty() && publish_pub_->get_subscription_count() > 0) {
      for (const Json& details : pending_events_) {
        mowbite_msgs::msg::MqttPublish msg;
        msg.topic = "events/json";
        msg.payload = Dump(mowbite_bridge::BuildEvent("CAMERA_CHANGED", details, now().seconds()));
        publish_pub_->publish(msg);
      }
      pending_events_.clear();
    }
    FollowViewers();
  }

  // A picture topic is subscribed only while a viewer of that camera is connected: the camera makes pictures only
  // for subscribers.
  void FollowViewers() {
    const std::vector<std::string> watched = service_->MjpegCameras();
    for (auto it = picture_subs_.begin(); it != picture_subs_.end();) {
      it = std::find(watched.begin(), watched.end(), it->first) == watched.end() ? picture_subs_.erase(it) : ++it;
    }
    for (const std::string& id : watched) {
      if (picture_subs_.count(id)) continue;
      picture_subs_[id] = create_subscription<sensor_msgs::msg::CompressedImage>(
          "camera/" + id + "/jpeg", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
          [this, id](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
            if (msg->format != "jpeg" || msg->data.empty() || msg->data.size() > kMaxJpegBytes) return;
            server_->Post([this, id, jpeg = std::string(msg->data.begin(), msg->data.end())] {
              server_->Push(id, jpeg);
            });
          });
    }
  }

  void OnSlow() {
    service_->Tick();
    ReadExitNotes();
    Supervise();
    Apply();
  }

  // The stream server follows the cameras with webrtc: its configuration, its start, and what its API tells about
  // its sessions while some hold a place. The calls of the API block this thread for kStreamApiSeconds at most.
  void Supervise() {
    const std::map<std::string, std::string> sources = service_->WebrtcSources();
    std::string problem = runtime_problem_;
    if (problem.empty() && !sources.empty() && sources != stream_.sources) {
      StreamConfig next = stream_;
      next.sources = sources;
      if (ReplaceFile(runtime_dir_, kStreamConfigFile, StreamConfigText(next), &problem)) stream_ = next;
    }
    if (supervisor_->Tick(!sources.empty() && sources == stream_.sources)) service_->OnStreamServerGone();
    std::vector<std::string> sessions;
    if (supervisor_->Starting()) {
      if (StreamSessions(api_, &sessions)) supervisor_->Answered();
    } else if (supervisor_->Up() && !service_->WebrtcSessions().empty() && StreamSessions(api_, &sessions)) {
      service_->OnSessions(sessions);
    }
    const std::string reason = problem.empty() ? supervisor_->reason() : kNotSetUp;
    const std::string& detail = problem.empty() ? supervisor_->detail() : problem;
    service_->SetWebrtc(reason, detail);
    const std::string state = reason.empty() ? "up" : reason + ": " + detail;
    if (!sources.empty() && state != stream_state_) {
      RCLCPP_INFO(get_logger(), "Stream server %s", state.c_str());
      stream_state_ = state;
    }
  }

  // The launch leaves <data_dir>/<process>_exit.json when a process of its own ended.
  void ReadExitNotes() {
    if (data_dir_.empty()) return;
    for (const std::string& process : service_->Processes()) {
      std::ifstream file(std::filesystem::path(data_dir_) / (process + "_exit.json"), std::ios::binary);
      if (!file) continue;
      std::string text(kMaxNoteBytes, '\0');
      file.read(text.data(), static_cast<std::streamsize>(text.size()));
      text.resize(static_cast<size_t>(file.gcount()));
      double mono = 0;
      bool down = false;
      if (ParseExitNote(text, &mono, &down)) service_->OnProcessEnded(process, mono, down);
    }
  }

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::string data_dir_;
  std::unique_ptr<HttpServer> server_;
  int auth_listener_ = -1;
  std::unique_ptr<CameraService> service_;
  // The stream server: what its configuration file holds, how the node calls its API, where its files are (empty
  // with the reason in runtime_problem_), and the last state logged.
  StreamConfig stream_;
  LocalServer api_;
  std::string runtime_dir_;
  std::string runtime_problem_;
  std::unique_ptr<MediaServer> media_;
  std::unique_ptr<StreamSupervisor> supervisor_;
  std::string stream_state_;
  std::atomic<bool> running_{true};
  std::thread thread_;
  std::vector<Json> pending_events_;

  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::Camera>::SharedPtr cameras_sub_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> picture_subs_;
  rclcpp::TimerBase::SharedPtr fast_timer_;
  rclcpp::TimerBase::SharedPtr slow_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_camera

// Ends with 1 when the node cannot start (a port in use, a parameter), which the launch notes before it tries again.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<mowbite_camera::CameraNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("mowbite_camera"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
