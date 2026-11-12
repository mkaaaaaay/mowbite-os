#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <mowbite_msgs/msg/clock_status.hpp>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "mowbite_hold/file_io.h"
#include "mowbite_hold/file_writer.h"
#include "mowbite_time/clock_guard.h"
#include "mowbite_watch/attach.h"

namespace mowbite_time {

namespace {

// time/clock goes out again this often without a change, so its readers keep their offset to the wall clock fresh.
constexpr double kRepeat = 60.0;
// clock.json is written at the first synchronization and then at most this often (s).
constexpr double kSaveEvery = 6 * 3600.0;
// A fix of the receiver older than this (s) is not compared.
constexpr double kFixFresh = 2.0;
constexpr char kHctosys[] = "/sys/class/rtc/rtc0/hctosys";

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

// The judgement of the wall clock once a second (ClockGuard) on time/clock, for every node that needs date, weekday or
// sun. Reads adjtimex, the start of the kernel from the real-time clock, the time of day of the GNSS receiver where
// hw/capabilities offers it, and keeps the time of the last synchronization in data_dir/clock.json.
class ClockNode : public rclcpp::Node {
 public:
  explicit ClockNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_clock", options),
        data_dir_(declare_parameter("data_dir", std::string())),
        guard_(LoadStart()),
        writer_(mowbite_hold::WriteFileSafely, [this](const mowbite_hold::FileWriter::Done& done) {
          if (!done.ok) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", done.path.c_str(), done.error.c_str());
        }) {
    status_pub_ = create_publisher<mowbite_msgs::msg::ClockStatus>("time/clock", LatchedQos(1));
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1), [this](const mowbite_msgs::msg::HardwareCapabilities& msg) {
          receiver_time_ = msg.gnss_receiver_time;
        });
    fix_sub_ = create_subscription<mowbite_msgs::msg::GnssFix>(
        "gps/absolute_pose", 10, [this](const mowbite_msgs::msg::GnssFix& msg) {
          using mowbite_msgs::msg::GnssFix;
          if (!receiver_time_ || msg.solution == GnssFix::NONE || msg.solution == GnssFix::DEAD_RECKONING) return;
          fix_day_ = msg.receiver_time_ms / 1000.0;
          fix_mono_ = MonotonicSeconds();
        });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { Tick(); });
    Tick();
    watch_ = mowbite_watch::Attach(*this);
  }

 private:
  ClockStart LoadStart() {
    ClockStart start;
    start.rtc = declare_parameter("rtc", false);
    std::string text;
    std::string error;
    start.hctosys = mowbite_hold::ReadSmallFile(kHctosys, 16, &text, &error) == mowbite_hold::FileRead::kText &&
                    text.rfind('1', 0) == 0;
    if (!data_dir_.empty()) {
      const std::string path = data_dir_ + "/clock.json";
      const mowbite_hold::FileRead read = mowbite_hold::ReadSmallFile(path, 4096, &text, &error);
      if (read == mowbite_hold::FileRead::kText) {
        const nlohmann::json json = nlohmann::json::parse(text, nullptr, false);
        if (json.is_object() && json.contains("synced_at") && json["synced_at"].is_number()) {
          start.synced_at = json["synced_at"].get<double>();
        } else {
          RCLCPP_WARN(get_logger(), "%s holds no synced_at, ignored", path.c_str());
        }
      } else if (read != mowbite_hold::FileRead::kMissing) {
        RCLCPP_WARN(get_logger(), "Cannot read %s: %s", path.c_str(), error.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "rtc %s, hctosys %d, synced_at %s", start.rtc ? "on" : "off", start.hctosys ? 1 : 0,
                start.synced_at ? std::to_string(*start.synced_at).c_str() : "none");
    return start;
  }

  void Tick() {
    ClockProbe probe;
    probe.mono = MonotonicSeconds();
    probe.wall = WallSeconds();
    probe.kernel_synced = SystemClockSynchronized();
    if (fix_mono_ && probe.mono - *fix_mono_ <= kFixFresh) probe.gnss_day = fix_day_ + (probe.mono - *fix_mono_);
    const ClockVerdict verdict = guard_.Probe(probe);
    if (!published_ || verdict != *published_ || probe.mono - published_mono_ >= kRepeat) {
      if (!published_ || verdict != *published_) {
        if (verdict.ok) RCLCPP_INFO(get_logger(), "The clock is safe");
        else RCLCPP_WARN(get_logger(), "The clock is not safe: %s", verdict.reason.c_str());
      }
      mowbite_msgs::msg::ClockStatus status;
      status.ok = verdict.ok;
      status.reason = verdict.reason;
      status_pub_->publish(status);
      published_ = verdict;
      published_mono_ = probe.mono;
    }
    const std::optional<double> synced_at = guard_.SyncedAt();
    if (synced_at && !data_dir_.empty() && (!saved_mono_ || probe.mono - *saved_mono_ >= kSaveEvery)) {
      writer_.Submit(data_dir_ + "/clock.json", nlohmann::json{{"synced_at", *synced_at}}.dump() + "\n");
      saved_mono_ = probe.mono;
    }
  }

  const std::string data_dir_;
  ClockGuard guard_;
  mowbite_hold::FileWriter writer_;
  bool receiver_time_ = false;
  double fix_day_ = 0;
  std::optional<double> fix_mono_;
  std::optional<ClockVerdict> published_;
  double published_mono_ = 0;
  std::optional<double> saved_mono_;
  rclcpp::Publisher<mowbite_msgs::msg::ClockStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::GnssFix>::SharedPtr fix_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_time

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_time::ClockNode)
