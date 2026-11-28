#include <netdb.h>
#include <poll.h>
#include <mowbite_component/node_options.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <mowbite_bridge/event_builder.h>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/ntrip_status.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/rtcm.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "mowbite_ntrip/ntrip_access.h"
#include "mowbite_ntrip/ntrip_client.h"
#include "mowbite_ntrip/ntrip_config.h"
#include "mowbite_ntrip/ntrip_source.h"
#include "mowbite_watch/attach.h"

namespace mowbite_ntrip {

namespace {

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

const char kStateSensor[] = "os_ntrip_state";

// The longest single wait on the socket, so a shutdown is seen within it.
constexpr int kPollStepMs = 100;

// Polls fd for events until timeout_ms is over or running turns false; > 0 when ready.
int PollWhile(int fd, short events, int timeout_ms, const std::atomic<bool>& running) {
  const double end = MonotonicSeconds() + timeout_ms / 1000.0;
  while (running) {
    const int left = static_cast<int>((end - MonotonicSeconds()) * 1000.0);
    if (left <= 0) return 0;
    pollfd p{fd, events, 0};
    const int ready = ::poll(&p, 1, std::min(left, kPollStepMs));
    if (ready != 0) return ready;
  }
  return 0;
}

// Sends all of data within timeout_ms on a non-blocking socket, or gives up once running turns false.
bool SendAll(int fd, const std::string& data, int timeout_ms, const std::atomic<bool>& running) {
  size_t sent = 0;
  const double end = MonotonicSeconds() + timeout_ms / 1000.0;
  while (sent < data.size()) {
    ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
    const int left = static_cast<int>((end - MonotonicSeconds()) * 1000.0);
    if (left <= 0 || PollWhile(fd, POLLOUT, left, running) == 0) return false;
  }
  return true;
}

std::string SourceText(const CorrectionSource& source) {
  return source.host + ":" + std::to_string(source.port) + "/" + source.mountpoint;
}

// getaddrinfo has no time limit: it runs in a thread of its own, which frees what it found once nobody waits.
struct Lookup {
  std::mutex mutex;
  std::condition_variable done_cv;
  bool done = false;
  bool abandoned = false;
  int rc = EAI_AGAIN;
  addrinfo* result = nullptr;
};

// The addresses of host:port within timeout_s, or rc != 0. The caller frees result.
std::pair<int, addrinfo*> Resolve(const std::string& host, int port, double timeout_s,
                                  const std::atomic<bool>& running) {
  auto lookup = std::make_shared<Lookup>();
  std::thread([lookup, host, port] {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const int rc = ::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result);
    std::lock_guard<std::mutex> lock(lookup->mutex);
    if (lookup->abandoned) {
      if (rc == 0) ::freeaddrinfo(result);
      return;
    }
    lookup->rc = rc;
    lookup->result = result;
    lookup->done = true;
    lookup->done_cv.notify_all();
  }).detach();
  std::unique_lock<std::mutex> lock(lookup->mutex);
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (!lookup->done && running && std::chrono::steady_clock::now() < until) {
    lookup->done_cv.wait_for(lock, std::chrono::milliseconds(100));
  }
  if (!lookup->done) {
    lookup->abandoned = true;
    return {EAI_AGAIN, nullptr};
  }
  return {lookup->rc, lookup->result};
}

}  // namespace

// NTRIP client: reads the access from <data_dir>/ntrip.yaml, publishes the caster's RTCM frames on gnss/rtcm for the
// GNSS driver and sends the receiver's GGA (gnss/gga) back. Status on gnss/ntrip_status, the sensor os_ntrip_state and
// CORRECTIONS events; none of them carries user or password, nor do the answers of ntrip.get, ntrip.set and
// ntrip.delete. The source used last stays in <data_dir>/ntrip_source.json, also over restarts and ntrip.delete.
class NtripNode : public rclcpp::Node {
 public:
  explicit NtripNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_ntrip", mowbite_component::StartupOnly(options)), client_(Timing()) {
    const std::string data_dir = declare_parameter("data_dir", std::string());
    file_ = declare_parameter("access_file", "");
    if (file_.empty()) file_ = (std::filesystem::path(data_dir) / "ntrip.yaml").string();
    source_file_ = (std::filesystem::path(data_dir) / "ntrip_source.json").string();
    connect_timeout_s_ = declare_parameter("connect_timeout", 10.0);
    config_check_s_ = declare_parameter("config_check", 5.0);
    access_over_rpc_ = declare_parameter("access_over_rpc", false);

    rtcm_pub_ = create_publisher<mowbite_msgs::msg::Rtcm>("gnss/rtcm", rclcpp::QoS(50).reliable());
    status_pub_ = create_publisher<mowbite_msgs::msg::NtripStatus>("gnss/ntrip_status", 10);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>("bridge/sensor_info", LatchedQos(10));
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
    gga_sub_ = create_subscription<std_msgs::msg::String>(
        "gnss/gga", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          client_.SetGga(msg->data);
        });

    rpc_response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    rpc_error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    rpc_methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    rpc_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRpcRequest(*msg); });
    idle_sub_ = create_subscription<std_msgs::msg::Bool>(
        "logic/idle", LatchedQos(1), [this](std_msgs::msg::Bool::ConstSharedPtr msg) { idle_ = msg->data ? 1 : 0; });
    // The hold of an update, from the logic: the access does not change while it lasts.
    hold_sub_ = create_subscription<std_msgs::msg::Int64>(
        "logic/hold", LatchedQos(1), [this](std_msgs::msg::Int64::ConstSharedPtr msg) { hold_until_ = msg->data; });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_ntrip";
    methods.methods = AccessMethods(access_over_rpc_);
    rpc_methods_pub_->publish(methods);

    const double none = std::numeric_limits<double>::quiet_NaN();
    mowbite_msgs::msg::SensorSpec spec;
    spec.id = kStateSensor;
    spec.name = "NTRIP";
    spec.kind = mowbite_msgs::msg::SensorSpec::KIND_TEXT;
    spec.meaning = mowbite_msgs::msg::SensorSpec::MEANING_UNKNOWN;
    spec.min = none;
    spec.max = none;
    spec.critical_low = none;
    spec.critical_high = none;
    sensor_info_pub_->publish(spec);

    const SourceFile last = ReadSourceFile(source_file_);
    if (last.exists && !last.source) {
      RCLCPP_WARN(get_logger(), "%s: %s; the next corrections replace it without SOURCE_CHANGED", source_file_.c_str(),
                  last.error.c_str());
    }
    client_.SetLastSource(last.source);
    watch_ = mowbite_watch::Attach(*this);
    io_thread_ = std::thread([this] { Loop(); });
  }

  ~NtripNode() override {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
    CloseSocket();
  }

 private:
  ClientTiming Timing() {
    ClientTiming t;
    t.first_retry_s = declare_parameter("first_retry", t.first_retry_s);
    t.max_retry_s = declare_parameter("max_retry", t.max_retry_s);
    t.rejected_retry_s = declare_parameter("rejected_retry", t.rejected_retry_s);
    t.data_timeout_s = declare_parameter("data_timeout", t.data_timeout_s);
    t.gga_interval_s = declare_parameter("gga_interval", t.gga_interval_s);
    t.gga_wait_s = declare_parameter("gga_wait", t.gga_wait_s);
    // The same parameter as the GNSS driver: LOST comes when the fix starts to count as float.
    t.lost_after_s = declare_parameter("max_correction_age", t.lost_after_s);
    return t;
  }

  void Loop() {
    double next_check = 0, next_status = 0;
    std::vector<uint8_t> buffer(4096);
    while (running_ && rclcpp::ok()) {
      double now = MonotonicSeconds();
      const bool changed = reload_now_.exchange(false);
      if (changed || now >= next_check) {
        next_check = now + config_check_s_;
        ReloadConfig(now);
      }
      bool connect = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        connect = fd_ < 0 && client_.ShouldConnect(now);
      }
      if (connect) {
        ReloadConfig(now);
        Connect();
      }
      if (fd_ < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } else {
        pollfd p{fd_, POLLIN, 0};
        int ready = ::poll(&p, 1, 100);
        now = MonotonicSeconds();
        if (ready > 0) {
          ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
          if (n > 0) {
            OnBytes(buffer.data(), static_cast<size_t>(n), now);
          } else if (n == 0) {
            Close("closed by the caster");
          } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            Close(std::string("read: ") + std::strerror(errno));
          }
        }
        if (fd_ >= 0) SendGga(now);
        std::optional<std::string> reason;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          reason = client_.CloseReason(now);
        }
        if (fd_ >= 0 && reason) Close(*reason);
      }
      if (now >= next_status) {
        next_status = now + 1.0;
        PublishStatus(now);
      }
      PublishEvents();
    }
  }

  // Reads the access file; logs once per change what is wrong with it, never a value from it.
  void ReloadConfig(double now) {
    const AccessFile file = ReadAccessFile(file_);
    const std::optional<NtripConfig>& config = file.config;
    std::string problem;
    if (!file.exists) {
      problem = "No NTRIP access file " + file_ + ", no corrections";
    } else if (!config) {
      problem = file_ + ": " + file.error + "; no corrections";
    } else if (file.readable_by_others) {
      // Usable, but others may read the password.
      if (!warned_mode_) RCLCPP_WARN(get_logger(), "%s can be read by others; chmod 600 it", file_.c_str());
      warned_mode_ = true;
    } else {
      warned_mode_ = false;
    }
    if (problem != reported_problem_) {
      reported_problem_ = problem;
      if (!problem.empty()) {
        if (file.exists) {
          RCLCPP_ERROR(get_logger(), "%s", problem.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s", problem.c_str());
        }
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (config != client_.config() && config) {
      RCLCPP_INFO(get_logger(), "NTRIP caster %s:%d, mountpoint %s, NTRIP %d.0%s", config->host.c_str(), config->port,
                  config->mountpoint.c_str(), config->version, config->user.empty() ? ", no login" : "");
    }
    client_.SetConfig(config, now);
  }

  // ntrip.get, ntrip.set and ntrip.delete; the bridge hands every node all methods, the others answer theirs. The
  // params of ntrip.set hold the password: they are never logged, and no answer carries user or password.
  void OnRpcRequest(const mowbite_msgs::msg::RpcRequest& request) {
    using mowbite_msgs::msg::RpcError;
    if (request.method == kGetMethod) {
      Answer(request.id, AccessSummary(ReadAccessFile(file_)));
      return;
    }
    if (request.method != kSetMethod && request.method != kDeleteMethod) return;
    if (!access_over_rpc_) {
      AnswerError(request.id, RpcError::ERROR_METHOD_NOT_FOUND, "Method \"" + request.method + "\" not found");
      return;
    }
    // A rollback of the update would drop the change without a word.
    if (const int64_t until = hold_until_; until > 0) {
      AnswerError(request.id, RpcError::ERROR_HOLD, "An update holds the mower. The NTRIP access changes after it",
                  nlohmann::json{{"reason", "HOLD"}, {"until", until}}.dump());
      return;
    }
    // Like params.set: a new caster in the middle of a job would move or cut off the corrections the mower drives on.
    if (idle_ != 1) {
      AnswerError(request.id, RpcError::ERROR_INVALID_REQUEST,
                  idle_ < 0 ? "The NTRIP access changes only in IDLE, and the state of the mower is not known yet"
                            : "The NTRIP access changes only in IDLE, not while recording, driving or mowing");
      return;
    }
    std::string error;
    if (request.method == kDeleteMethod) {
      if (!RemoveAccessFile(file_, error)) {
        RCLCPP_ERROR(get_logger(), "ntrip.delete: %s", error.c_str());
        AnswerError(request.id, RpcError::ERROR_INTERNAL, "Cannot remove the access: " + error);
        return;
      }
      RCLCPP_INFO(get_logger(), "ntrip.delete: access removed");
    } else {
      nlohmann::ordered_json params;
      if (!request.params.empty()) {
        params = nlohmann::ordered_json::parse(request.params, nullptr, false);
        if (params.is_discarded()) {
          AnswerError(request.id, RpcError::ERROR_INVALID_PARAMS, "The parameters are not valid JSON");
          return;
        }
      }
      const AccessChange change = ChangeAccess(params, ReadAccessFile(file_).config);
      if (!change.config) {
        AnswerError(request.id, RpcError::ERROR_INVALID_PARAMS, change.error);
        return;
      }
      if (!WriteAccessFile(file_, *change.config, error)) {
        RCLCPP_ERROR(get_logger(), "ntrip.set: %s", error.c_str());
        AnswerError(request.id, RpcError::ERROR_INTERNAL, "Cannot save the access: " + error);
        return;
      }
      RCLCPP_INFO(get_logger(), "ntrip.set: access saved to %s", file_.c_str());
    }
    reload_now_ = true;
    // Read back from disk, so the answer shows what the client will use.
    Answer(request.id, AccessSummary(ReadAccessFile(file_)));
  }

  void Answer(const std::string& id, const nlohmann::ordered_json& result) {
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = result.dump();
    msg.id = id;
    rpc_response_pub_->publish(msg);
  }

  void AnswerError(const std::string& id, int16_t code, const std::string& message, const std::string& data = "") {
    mowbite_msgs::msg::RpcError msg;
    msg.code = code;
    msg.message = message;
    msg.id = id;
    msg.data = data;
    rpc_error_pub_->publish(msg);
  }

  void Connect() {
    std::string request;
    NtripConfig config;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!client_.config()) return;
      config = *client_.config();
      request = client_.BeginAttempt(MonotonicSeconds());
    }
    const auto [rc, result] = Resolve(config.host, config.port, connect_timeout_s_, running_);
    if (rc != 0) {
      Failed(std::string("cannot resolve the host: ") + ::gai_strerror(rc));
      return;
    }
    std::string error = "cannot connect";
    for (addrinfo* a = result; a && fd_ < 0 && running_; a = a->ai_next) {
      int fd = ::socket(a->ai_family, a->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC, a->ai_protocol);
      if (fd < 0) continue;
      if (::connect(fd, a->ai_addr, a->ai_addrlen) != 0 && errno != EINPROGRESS) {
        error = std::string("connect: ") + std::strerror(errno);
        ::close(fd);
        continue;
      }
      int soerr = 0;
      socklen_t len = sizeof(soerr);
      if (PollWhile(fd, POLLOUT, static_cast<int>(connect_timeout_s_ * 1000), running_) <= 0) {
        error = "connect: timeout";
        ::close(fd);
        continue;
      }
      ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len);
      if (soerr != 0) {
        error = std::string("connect: ") + std::strerror(soerr);
        ::close(fd);
        continue;
      }
      fd_ = fd;
    }
    ::freeaddrinfo(result);
    if (fd_ < 0) {
      Failed(error);
      return;
    }
    if (!SendAll(fd_, request, 5000, running_)) {
      Close(std::string("sending the request: ") + std::strerror(errno));
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    client_.OnRequestSent(MonotonicSeconds());
  }

  // An attempt failed before a socket was open.
  void Failed(const std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    client_.OnClosed(MonotonicSeconds(), error);
    LogFailure();
  }

  void Close(const std::string& error) {
    CloseSocket();
    std::lock_guard<std::mutex> lock(mutex_);
    const bool was_connected = client_.state() == ClientState::kConnected;
    client_.OnClosed(MonotonicSeconds(), error);
    if (was_connected) RCLCPP_WARN(get_logger(), "NTRIP connection ended: %s", client_.last_error().c_str());
    LogFailure();
  }

  // With the mutex held.
  void LogFailure() {
    const double now = MonotonicSeconds();
    if (client_.state() == ClientState::kRejected) {
      RCLCPP_ERROR(get_logger(), "NTRIP: %s; next attempt in %.0f s or when %s changes", client_.last_error().c_str(),
                   client_.retry_in(now), file_.c_str());
    } else if (client_.state() == ClientState::kWaiting) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "NTRIP: %s; next attempt in %.0f s",
                           client_.last_error().c_str(), client_.retry_in(now));
    }
  }

  void CloseSocket() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }

  void OnBytes(const uint8_t* data, size_t size, double now) {
    std::vector<std::vector<uint8_t>> frames;
    bool connected_now = false;
    std::optional<CorrectionSource> used;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ClientState before = client_.state();
      client_.OnBytes(data, size, now, frames);
      connected_now = before != ClientState::kConnected && client_.state() == ClientState::kConnected;
      used = client_.TakeSourceToSave();
    }
    if (connected_now) RCLCPP_INFO(get_logger(), "NTRIP connected, corrections follow");
    if (!frames.empty()) {
      mowbite_msgs::msg::Rtcm msg;
      msg.header.stamp = this->now();
      for (const auto& frame : frames) msg.data.insert(msg.data.end(), frame.begin(), frame.end());
      rtcm_pub_->publish(msg);
    }
    // After the frames, which need not wait for the disk.
    if (used) SaveSource(*used);
  }

  // The next change, also after a restart, is compared with this source.
  void SaveSource(const CorrectionSource& source) {
    std::string error;
    if (!WriteSourceFile(source_file_, source, error)) {
      RCLCPP_ERROR(get_logger(), "Cannot save the source of the corrections: %s", error.c_str());
    }
  }

  void SendGga(double now) {
    std::optional<std::string> gga;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gga = client_.TakeGga(now);
    }
    if (gga && !SendAll(fd_, *gga, 2000, running_)) Close(std::string("sending GGA: ") + std::strerror(errno));
  }

  void PublishStatus(double now) {
    mowbite_msgs::msg::NtripStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.state = StateName(client_.state());
      status.last_error = client_.last_error();
      status.connects = client_.attempts();
      status.bytes = client_.bytes();
      status.frames = client_.frames();
      status.crc_errors = client_.crc_errors();
      status.last_frame_age = client_.last_frame_age(now).value_or(-1.0);
      status.retry_in = client_.retry_in(now);
      client_.Tick(now);
    }
    status_pub_->publish(status);
    mowbite_msgs::msg::SensorValue value;
    value.sensor_id = kStateSensor;
    value.text = status.state;
    sensor_value_pub_->publish(value);
  }

  void PublishEvents() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& event : client_.TakeEvents()) pending_.push_back(std::move(event));
    }
    if (pending_.empty()) return;
    // Held until the bridge listens (it subscribes without history); a few at most.
    if (publish_pub_->get_subscription_count() == 0) {
      while (pending_.size() > 10) pending_.erase(pending_.begin());
      return;
    }
    for (const auto& event : pending_) {
      const nlohmann::ordered_json details = CorrectionsDetails(event);
      std::string text = event.message;
      if (event.source && event.previous) {
        text += " (" + SourceText(*event.previous) + " before, now " + SourceText(*event.source) + ")";
      }
      if (event.severity == "info") {
        RCLCPP_INFO(get_logger(), "Corrections: %s", text.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Corrections: %s", text.c_str());
      }
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::BuildEvent("CORRECTIONS", details, this->now().seconds()).dump();
      publish_pub_->publish(msg);
    }
    pending_.clear();
  }

  std::string file_;
  std::string source_file_;
  double connect_timeout_s_ = 10.0;
  double config_check_s_ = 5.0;
  // ntrip.set and ntrip.delete; off unless the configuration turns it on (stack.yaml).
  bool access_over_rpc_ = false;
  // 1 while the logic is in IDLE, 0 otherwise, -1 until it said.
  std::atomic<int> idle_{-1};
  // until of logic/hold in unix s, 0 without a hold.
  std::atomic<int64_t> hold_until_{0};
  // An RPC changed the file: the loop reads it at once.
  std::atomic<bool> reload_now_{false};
  std::mutex mutex_;
  NtripClient client_;
  int fd_ = -1;
  std::string reported_problem_ = "-";
  bool warned_mode_ = false;
  std::vector<ClientEvent> pending_;
  rclcpp::Publisher<mowbite_msgs::msg::Rtcm>::SharedPtr rtcm_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::NtripStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr rpc_response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr rpc_error_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr rpc_methods_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gga_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr rpc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr idle_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr hold_sub_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_ntrip

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_ntrip::NtripNode)
