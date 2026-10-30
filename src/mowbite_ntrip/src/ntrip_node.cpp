#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mowbite_bridge/monitor.h>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/ntrip_status.hpp>
#include <mowbite_msgs/msg/rtcm.hpp>
#include <mowbite_msgs/msg/sensor_info.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "mowbite_ntrip/ntrip_client.h"
#include "mowbite_ntrip/ntrip_config.h"

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

// Sends all of data within timeout_ms on a non-blocking socket.
bool SendAll(int fd, const std::string& data, int timeout_ms) {
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
    if (left <= 0) return false;
    pollfd p{fd, POLLOUT, 0};
    ::poll(&p, 1, left);
  }
  return true;
}

}  // namespace

// NTRIP client: reads the access from <data_dir>/ntrip.yaml, publishes the caster's RTCM frames on gnss/rtcm for the
// GNSS driver and sends the receiver's GGA (gnss/gga) back. Status on gnss/ntrip_status, the sensor os_ntrip_state and
// CORRECTIONS events; none of them carries user or password.
class NtripNode : public rclcpp::Node {
 public:
  explicit NtripNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_ntrip", options), client_(Timing()) {
    const std::string data_dir = declare_parameter("data_dir", "/userdata/mowbite");
    file_ = declare_parameter("access_file", "");
    if (file_.empty()) file_ = (std::filesystem::path(data_dir) / "ntrip.yaml").string();
    connect_timeout_s_ = declare_parameter("connect_timeout", 10.0);
    config_check_s_ = declare_parameter("config_check", 5.0);

    rtcm_pub_ = create_publisher<mowbite_msgs::msg::Rtcm>("gnss/rtcm", rclcpp::QoS(50).reliable());
    status_pub_ = create_publisher<mowbite_msgs::msg::NtripStatus>("gnss/ntrip_status", 10);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorInfo>("bridge/sensor_info", LatchedQos(10));
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
    gga_sub_ = create_subscription<std_msgs::msg::String>(
        "gnss/gga", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          client_.SetGga(msg->data);
        });

    mowbite_msgs::msg::SensorInfo info;
    info.sensor_id = kStateSensor;
    info.sensor_name = "NTRIP";
    info.value_type = mowbite_msgs::msg::SensorInfo::TYPE_STRING;
    info.value_description = mowbite_msgs::msg::SensorInfo::VALUE_DESCRIPTION_UNKNOWN;
    sensor_info_pub_->publish(info);

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
    t.lost_after_s = declare_parameter("lost_after", t.lost_after_s);
    return t;
  }

  void Loop() {
    double next_check = 0, next_status = 0;
    std::vector<uint8_t> buffer(4096);
    while (running_ && rclcpp::ok()) {
      double now = MonotonicSeconds();
      if (now >= next_check) {
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
    std::optional<NtripConfig> config;
    std::string problem;
    struct stat st;
    if (::stat(file_.c_str(), &st) != 0) {
      problem = "No NTRIP access file " + file_ + ", no corrections";
    } else {
      std::ifstream in(file_);
      std::stringstream text;
      text << in.rdbuf();
      std::string error;
      if (!in) {
        problem = "Cannot read " + file_ + ", no corrections";
      } else if (!(config = ParseNtripConfig(text.str(), error))) {
        problem = file_ + ": " + error + "; no corrections";
      } else if (st.st_mode & 077) {
        // Usable, but others may read the password.
        if (!warned_mode_) RCLCPP_WARN(get_logger(), "%s can be read by others; chmod 600 it", file_.c_str());
        warned_mode_ = true;
      }
    }
    if (problem != reported_problem_) {
      reported_problem_ = problem;
      if (!problem.empty()) {
        if (::stat(file_.c_str(), &st) != 0) {
          RCLCPP_INFO(get_logger(), "%s", problem.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "%s", problem.c_str());
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

  void Connect() {
    std::string request;
    NtripConfig config;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!client_.config()) return;
      config = *client_.config();
      request = client_.BeginAttempt(MonotonicSeconds());
    }
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const int rc = ::getaddrinfo(config.host.c_str(), std::to_string(config.port).c_str(), &hints, &result);
    if (rc != 0) {
      Failed(std::string("cannot resolve the host: ") + ::gai_strerror(rc));
      return;
    }
    std::string error = "cannot connect";
    for (addrinfo* a = result; a && fd_ < 0; a = a->ai_next) {
      int fd = ::socket(a->ai_family, a->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC, a->ai_protocol);
      if (fd < 0) continue;
      if (::connect(fd, a->ai_addr, a->ai_addrlen) != 0 && errno != EINPROGRESS) {
        error = std::string("connect: ") + std::strerror(errno);
        ::close(fd);
        continue;
      }
      pollfd p{fd, POLLOUT, 0};
      int soerr = 0;
      socklen_t len = sizeof(soerr);
      if (::poll(&p, 1, static_cast<int>(connect_timeout_s_ * 1000)) <= 0) {
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
    if (!SendAll(fd_, request, 5000)) Close(std::string("sending the request: ") + std::strerror(errno));
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ClientState before = client_.state();
      client_.OnBytes(data, size, now, frames);
      connected_now = before != ClientState::kConnected && client_.state() == ClientState::kConnected;
    }
    if (connected_now) RCLCPP_INFO(get_logger(), "NTRIP connected, corrections follow");
    if (frames.empty()) return;
    mowbite_msgs::msg::Rtcm msg;
    msg.header.stamp = this->now();
    for (const auto& frame : frames) msg.data.insert(msg.data.end(), frame.begin(), frame.end());
    rtcm_pub_->publish(msg);
  }

  void SendGga(double now) {
    std::optional<std::string> gga;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gga = client_.TakeGga(now);
    }
    if (gga && !SendAll(fd_, *gga, 2000)) Close(std::string("sending GGA: ") + std::strerror(errno));
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
      mowbite_bridge::json details = {{"reason", event.reason},
                                      {"message", event.message},
                                      {"severity", event.severity},
                                      {"resume", event.resume}};
      if (event.code) details["code"] = event.code;
      if (event.limit > 0) details["limit"] = event.limit;
      if (event.severity == "info") {
        RCLCPP_INFO(get_logger(), "Corrections: %s", event.message.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Corrections: %s", event.message.c_str());
      }
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::EventPayload("CORRECTIONS", details, this->now().seconds()).dump();
      publish_pub_->publish(msg);
    }
    pending_.clear();
  }

  std::string file_;
  double connect_timeout_s_ = 10.0;
  double config_check_s_ = 5.0;
  std::mutex mutex_;
  NtripClient client_;
  int fd_ = -1;
  std::string reported_problem_ = "-";
  bool warned_mode_ = false;
  std::vector<ClientEvent> pending_;
  rclcpp::Publisher<mowbite_msgs::msg::Rtcm>::SharedPtr rtcm_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::NtripStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorInfo>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gga_sub_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;
};

}  // namespace mowbite_ntrip

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_ntrip::NtripNode)
