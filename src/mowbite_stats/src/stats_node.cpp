#include <sys/statvfs.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/parts_use.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "mowbite_bridge/event_builder.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_hold/file_writer.h"
#include "mowbite_hold/restart.h"
#include "mowbite_journal/journal.h"
#include "mowbite_stats/counter.h"
#include "mowbite_stats/incidents.h"
#include "mowbite_stats/sensor_history.h"
#include "mowbite_stats/stats_rpc.h"
#include "mowbite_stats/stop_book.h"
#include "mowbite_watch/attach.h"

namespace mowbite_stats {

namespace {

constexpr char kStateFile[] = "stats/state.json";
constexpr char kHoursFile[] = "stats/hours.json";
constexpr char kPartsFile[] = "stats/parts.json";
constexpr char kStopsFile[] = "stats/stops.json";
// hours.json at kMaxHours rows is about 0.5 MB.
constexpr size_t kMaxFileBytes = size_t{2} * 1024 * 1024;
// The state goes to the disk this often (s); at most this much counting is lost when the OS stops hard.
constexpr double kSaveEvery = 60.0;
// One file per hour of the sensor history, <hour unix s>.jsonl.
constexpr char kHistoryDir[] = "stats/sensors";
constexpr char kHistoryExtension[] = ".jsonl";
// The sensor history goes to the disk this often (s) and at the end; at most this much is lost when the OS stops hard.
constexpr double kHistorySaveEvery = 900.0;
// Log lines of /rosout kept for incidents are cut to this many bytes.
constexpr size_t kLogLineBytes = 1000;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// Bytes free for this user on the file system of path, -1 when unknown.
int64_t FreeBytes(const std::string& path) {
  struct statvfs fs {};
  if (::statvfs(path.c_str(), &fs) != 0) return -1;
  return static_cast<int64_t>(fs.f_bavail) * static_cast<int64_t>(fs.f_frsize);
}

// An empty text removes the file: hours of the sensor history that are over.
bool WriteOrRemove(const std::string& path, const std::string& text, std::string* error) {
  return text.empty() ? mowbite_hold::RemoveFileSynced(path, error) : mowbite_hold::WriteFileSafely(path, text, error);
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
// stats/parts for the pushes of mowbite_net; min, mean and max of the number sensors of bridge/sensor_value per
// minute for sensors.history. Files under <data_dir>/stats. The incidents of the journal in journal_dir/incidents
// (journal.status, incidents.*, event INCIDENT): Incidents. The stop book in stats/stops.json for stats.stops:
// StopBook, whose unexplained stops open incidents. Reads only, moves nothing.
class StatsNode : public rclcpp::Node {
 public:
  explicit StatsNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_stats", mowbite_component::StartupOnly(options)),
        data_dir_(declare_parameter("data_dir", std::string())),
        counter_(Read(kStateFile), Read(kHoursFile), Read(kPartsFile), WallSeconds()),
        stops_(Read(kStopsFile)),
        reserve_bytes_(std::max(0.0, declare_parameter("journal_reserve_mb", mowbite_journal::Options().reserve_mb)) *
                       1e6),
        writer_(WriteOrRemove, [this](const mowbite_hold::FileWriter::Done& done) {
          if (done.ok) return;
          RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", done.path.c_str(), done.error.c_str());
          if (done.path.rfind(PathOf(kHistoryDir), 0) == 0) history_failed_ = true;
        }) {
    incidents_ = std::make_unique<Incidents>(
        ReadIncidentsConfig(*this, reserve_bytes_ / 1e6, data_dir_), FreeBytes,
        [this](const std::string& why) { RCLCPP_WARN(get_logger(), "%s", why.c_str()); });
    stops_.SetIncidents({[this](const std::string& reason) {
                           std::string id;
                           Publish(incidents_->OnStop(reason, mowbite_journal::MonotonicNs(), WallSeconds(), &id));
                           return id;
                         },
                         [this] { return incidents_->Collecting(); }});
    parts_pub_ = create_publisher<mowbite_msgs::msg::PartsUse>("stats/parts", LatchedQos(1));
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100,
        [this](const mowbite_msgs::msg::RpcRequest& msg) { OnRequest(msg); });
    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](const mowbite_msgs::msg::HardwareStatus& msg) {
          // The motor's report where there is one, else what the driver commanded.
          blade_ = msg.blade_motor.ok ? msg.blade_motor.rpm != 0 : msg.blade_on;
          Publish(incidents_->OnEmergency(msg.emergency_reason, mowbite_journal::MonotonicNs(), WallSeconds()));
        });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](const mowbite_msgs::msg::RobotState& msg) { OnState(msg); });
    LoadHistory();
    // As the bridge takes them for sensor_infos/json and sensors/<id>/data.
    sensor_info_sub_ = create_subscription<mowbite_msgs::msg::SensorSpec>(
        "bridge/sensor_info", LatchedQos(100), [this](const mowbite_msgs::msg::SensorSpec& msg) {
          const bool number = msg.kind == mowbite_msgs::msg::SensorSpec::KIND_NUMBER;
          if (!history_.Describe(msg.id, msg.name, msg.unit, number) && number) {
            RCLCPP_WARN(get_logger(), "Sensor %s has no history: too long a text or more than %zu sensors",
                        msg.id.c_str(), kMaxSensors);
          }
        });
    sensor_value_sub_ = create_subscription<mowbite_msgs::msg::SensorValue>(
        "bridge/sensor_value", rclcpp::QoS(100),
        [this](const mowbite_msgs::msg::SensorValue& msg) { history_.Add(msg.sensor_id, msg.value, WallSeconds()); });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_stats";
    methods.methods = StatsMethods();
    methods_pub_->publish(methods);
    PublishParts();
    if (data_dir_.empty()) RCLCPP_WARN(get_logger(), "No data_dir: the statistics are not kept");
    timer_ = create_wall_timer(std::chrono::duration<double>(kSaveEvery), [this] { Save(false); });
    history_timer_ =
        create_wall_timer(std::chrono::duration<double>(kHistorySaveEvery), [this] { SaveHistory(); });
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    publish_sub_ = create_subscription<mowbite_msgs::msg::MqttPublish>(
        "bridge/publish", 100, [this](const mowbite_msgs::msg::MqttPublish& msg) {
          if (msg.topic != "events/json") return;
          const json event = json::parse(msg.payload, nullptr, false);
          // The statistics never take the container and its safety node down with them.
          try {
            Publish(incidents_->OnEvent(event, mowbite_journal::MonotonicNs(), WallSeconds()));
            stops_.OnEvent(event, MonotonicSeconds(), WallSeconds());
          } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "An event left out of the statistics: %s", e.what());
          }
        });
    log_sub_ = create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", rclcpp::QoS(100), [this](const rcl_interfaces::msg::Log& msg) {
          if (msg.level < rcl_interfaces::msg::Log::WARN) return;
          const double t = static_cast<double>(msg.stamp.sec) + static_cast<double>(msg.stamp.nanosec) / 1e9;
          incidents_->OnLog(mowbite_journal::MonotonicNs(),
                            json{{"t", t}, {"level", msg.level}, {"name", msg.name},
                                 {"msg", msg.msg.substr(0, kLogLineBytes)}});
        });
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local(),
        [this](const std_msgs::msg::String& msg) {
          const json value = json::parse(msg.data, nullptr, false);
          if (!value.is_object() || value.value("node", "") != "/mower_logic") return;
          const auto params = value.find("params");
          if (params == value.end() || !params->is_object()) return;
          const auto enabled = params->find("journal_enabled");
          if (enabled != params->end() && enabled->is_number()) incidents_->SetJournalEnabled(enabled->get<double>() != 0);
        });
    incidents_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      Publish(incidents_->Tick(mowbite_journal::MonotonicNs(), WallSeconds()));
      if (stops_.Tick(MonotonicSeconds())) SaveStops();
    });
    watch_ = mowbite_watch::Attach(*this);
  }

  // The last minute of counting and the sensor history on the way out; a failure to write is logged, never thrown.
  ~StatsNode() override {
    try {
      Save(false);
      SaveHistory();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Statistics not saved: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Statistics not saved");
    }
  }

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

  std::string PathOfHour(int64_t hour) const {
    return PathOf(kHistoryDir) + "/" + std::to_string(hour) + kHistoryExtension;
  }

  // The hour files newest first; a file that cannot be read is removed at the next save.
  void LoadHistory() {
    if (data_dir_.empty()) return;
    std::vector<int64_t> hours;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator(PathOf(kHistoryDir), error)) {
      const std::string stem = entry.path().stem().string();
      if (entry.path().extension() != kHistoryExtension || stem.empty() || stem.size() > 12 ||
          !std::all_of(stem.begin(), stem.end(), [](char c) { return c >= '0' && c <= '9'; })) {
        continue;
      }
      hours.push_back(std::stoll(stem));
    }
    std::sort(hours.begin(), hours.end(), std::greater<>());
    for (const int64_t hour : hours) {
      std::string text, why;
      const auto read = mowbite_hold::ReadSmallFile(PathOfHour(hour), kMaxHistoryBytes, &text, &why);
      if (read != mowbite_hold::FileRead::kText) {
        RCLCPP_ERROR(get_logger(), "Cannot read the sensor history of %lld: %s", static_cast<long long>(hour),
                     why.empty() ? "too large" : why.c_str());
        text.clear();
      }
      history_.Load(hour, text);
    }
    RCLCPP_INFO(get_logger(), "Sensor history: %zu values of minutes in %zu bytes", history_.Buckets(),
                history_.Bytes());
  }

  // Writes the hours changed since the last save; below journal_reserve_mb free only removes.
  void SaveHistory() {
    if (history_failed_.exchange(false)) history_.AllUnwritten();
    std::vector<HourFile> files = history_.TakeFiles(WallSeconds());
    if (data_dir_.empty()) return;
    size_t bytes = 0;
    for (const auto& file : files) bytes += file.text.size();
    const int64_t free = FreeBytes(data_dir_);
    if (bytes > 0 && free >= 0 && static_cast<double>(free) - static_cast<double>(bytes) < reserve_bytes_) {
      RCLCPP_WARN(get_logger(), "Sensor history not written: the journal keeps %.0f MB free", reserve_bytes_ / 1e6);
      std::vector<HourFile> removals;
      std::vector<HourFile> writes;
      for (auto& file : files) (file.text.empty() ? removals : writes).push_back(std::move(file));
      history_.Unwritten(writes);
      files = std::move(removals);
    }
    for (auto& file : files) writer_.Submit(PathOfHour(file.hour), std::move(file.text));
  }

  static IncidentsConfig ReadIncidentsConfig(rclcpp::Node& node, double reserve_mb, const std::string& data_dir) {
    IncidentsConfig config;
    const char* env = std::getenv("MOWBITE_JOURNAL_DIR");
    config.journal_dir = node.declare_parameter("journal_dir", std::string(env ? env : ""));
    config.journal_max_mb = node.declare_parameter("journal_max_mb", config.journal_max_mb);
    config.journal_max_days = node.declare_parameter("journal_max_days", config.journal_max_days);
    config.reserve_mb = reserve_mb;
    IncidentLimits& limits = config.limits;
    limits.max_mb = node.declare_parameter("incidents_max_mb", limits.max_mb);
    limits.max_days = node.declare_parameter("incidents_max_days", limits.max_days);
    limits.max_days_safety = node.declare_parameter("incidents_max_days_safety", limits.max_days_safety);
    config.boot_id = BootId();
    if (!data_dir.empty()) {
      config.driver_note = (std::filesystem::path(data_dir) / mowbite_hold::kDriverExitNoteFile).string();
    }
    return config;
  }

  // INCIDENT events from Incidents.
  void Publish(const Incidents::Notices& notices) {
    for (const json& details : notices) {
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::BuildEvent("INCIDENT", nlohmann::ordered_json(details), WallSeconds()).dump();
      publish_pub_->publish(msg);
    }
  }

  void OnState(const mowbite_msgs::msg::RobotState& state) {
    incidents_->OnRobot(json{{"state", state.current_state},
                             {"sub_state", state.current_sub_state},
                             {"area", state.current_area},
                             {"path", state.current_path},
                             {"path_index", state.current_path_index},
                             {"task_index", state.task_index},
                             {"task_count", state.task_count},
                             {"battery", state.battery_percentage},
                             {"gps", state.gps_percentage},
                             {"emergency", state.emergency},
                             {"emergency_reason", state.emergency_reason},
                             {"charging", state.is_charging}});
    Sample sample;
    sample.wall = WallSeconds();
    sample.mono = MonotonicSeconds();
    sample.blade = blade_;
    sample.state = state.current_state;
    sample.pose = state.gps_percentage > 0 && std::isfinite(state.pose_x) && std::isfinite(state.pose_y);
    sample.x = state.pose_x;
    sample.y = state.pose_y;
    if (counter_.Add(sample).hour_closed) Save(true);
    if (stops_.OnPose(!state.pause_reason.empty(), sample.pose, sample.x, sample.y, sample.mono, sample.wall)) {
      SaveStops();
    }
  }

  void SaveStops() { Write(kStopsFile, stops_.Json(WallSeconds())); }

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
    } else if (request.method == kHistoryMethod) {
      out = HandleHistory(history_, params, WallSeconds());
    } else if (request.method == kStopsMethod) {
      out = HandleStops(stops_, counter_, params, WallSeconds());
    } else if (std::find(Incidents::Methods().begin(), Incidents::Methods().end(), request.method) !=
               Incidents::Methods().end()) {
      Incidents::Notices notices;
      out = incidents_->Handle(request.method, params, mowbite_journal::MonotonicNs(), WallSeconds(), &notices);
      Publish(notices);
      // A mark without a stop before it is a stop the book missed.
      const bool marked = request.method == "incidents.mark" && !out.error;
      if (marked && stops_.OnMark(out.result.value("id", ""), WallSeconds())) SaveStops();
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
  StopBook stops_;
  SensorHistory history_;
  const double reserve_bytes_;
  // A write of the sensor history failed: the next save writes every hour.
  std::atomic<bool> history_failed_{false};
  mowbite_hold::FileWriter writer_;
  bool blade_ = false;
  json saved_;
  std::optional<mowbite_msgs::msg::PartsUse> published_;
  std::unique_ptr<Incidents> incidents_;

  rclcpp::Publisher<mowbite_msgs::msg::PartsUse>::SharedPtr parts_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr history_timer_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::TimerBase::SharedPtr incidents_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_stats

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_stats::StatsNode)
