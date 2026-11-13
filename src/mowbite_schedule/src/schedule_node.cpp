#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <mowbite_journal/node_options.h>
#include <mowbite_msgs/msg/clock_status.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/run_status.hpp>
#include <mowbite_msgs/msg/weather.hpp>
#include <mowbite_msgs/msg/weather_request.hpp>
#include <mowbite_msgs/srv/drop_tasks.hpp>
#include <mowbite_msgs/srv/start_run.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowbite_hold/file_io.h"
#include "mowbite_hold/file_writer.h"
#include "mowbite_logic/logic_params.h"
#include "mowbite_logic/mowing_plan.h"
#include "mowbite_schedule/schedule_journal.h"
#include "mowbite_schedule/schedule_rpc.h"
#include "mowbite_watch/attach.h"

namespace mowbite_schedule {

namespace {

// robot_state older than this (s) counts as a silent logic.
constexpr double kLogicFresh = 5.0;
// The files of the schedule are small; larger ones are not read.
constexpr size_t kMaxFileBytes = 64 * 1024;
// How long a start waits for its mark on the disk (S4) before it fails with storage.
constexpr auto kMarkWait = std::chrono::seconds(2);
// mowbite/schedule/ready again after this long (s) while a start is to come.
constexpr double kReadyRepeat = 60.0;
constexpr char kFile[] = "schedule.json";
constexpr char kStateFile[] = "schedule-state.json";
constexpr char kLogFile[] = "schedule-log.json";

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

int64_t WallSeconds() {
  using namespace std::chrono;
  return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
}

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

Phase PhaseOf(uint8_t phase) {
  return phase <= static_cast<uint8_t>(Phase::kEnded) ? static_cast<Phase>(phase) : Phase::kNone;
}

// The value of a refusal for the log: the first field of the reason, null without one.
json FirstValue(const std::string& data) {
  const nlohmann::ordered_json fields = nlohmann::ordered_json::parse(data, nullptr, false);
  if (!fields.is_object() || fields.empty()) return nullptr;
  return json(fields.begin().value());
}

}  // namespace

// The schedule (FP22): once a second the plans against the state of the mower, starts only through logic/start_run
// from the station, pauses of a running plan through logic/drop_tasks, schedule/next_start for the update hold, and
// schedule.get, .set, .log and .leftovers through the bridge, mowbite/schedule/ready for App and HA. Files in data_dir: schedule.json, schedule-state.json
// (the mark of each start, written before the start goes out) and schedule-log.json. Nothing under hw/ or safety/,
// no cmd_vel, no follow_path.
class ScheduleNode : public rclcpp::Node {
 public:
  explicit ScheduleNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_schedule", options),
        journal_(mowbite_journal::OpenFromParameters(*this)),
        data_dir_(declare_parameter("data_dir", std::string())),
        core_(journal_, Load()),
        writer_(mowbite_hold::WriteFileSafely, [this](const mowbite_hold::FileWriter::Done& done) {
          if (!done.ok) RCLCPP_ERROR(get_logger(), "Cannot write %s: %s", done.path.c_str(), done.error.c_str());
        }) {
    next_start_pub_ = create_publisher<std_msgs::msg::Int64>("schedule/next_start", LatchedQos(1));
    weather_pub_ = create_publisher<mowbite_msgs::msg::WeatherRequest>("net/weather/request", 10);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    start_client_ = create_client<mowbite_msgs::srv::StartRun>("logic/start_run");
    drop_client_ = create_client<mowbite_msgs::srv::DropTasks>("logic/drop_tasks");

    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) {
          robot_state_ = *msg;
          robot_state_mono_ = MonotonicSeconds();
        });
    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) { hw_ = *msg; });
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1),
        [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) { rain_sensor_ = msg->rain_sensor; });
    hold_sub_ = create_subscription<std_msgs::msg::Int64>(
        "logic/hold", LatchedQos(1), [this](std_msgs::msg::Int64::ConstSharedPtr msg) { hold_until_ = msg->data; });
    clock_sub_ = create_subscription<mowbite_msgs::msg::ClockStatus>(
        "time/clock", LatchedQos(1), [this](mowbite_msgs::msg::ClockStatus::ConstSharedPtr msg) {
          clock_ = ClockState{msg->ok, msg->reason};
        });
    datum_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/datum", LatchedQos(1), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
          latitude_ = msg->latitude;
          longitude_ = msg->longitude;
        });
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "map/json", LatchedQos(1), [this](std_msgs::msg::String::ConstSharedPtr msg) { map_.SetMap(msg->data); });
    // rain_mode of the logic, from the settings it publishes for the app.
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", LatchedQos(50), [this](std_msgs::msg::String::ConstSharedPtr msg) {
          const json params = json::parse(msg->data, nullptr, false);
          if (!params.is_object() || params.value("node", "") != mowbite_logic::kLogicParamsNode) return;
          const json value = params.value("params", json::object()).value("rain_mode", json());
          if (value.is_number()) rain_mode_ = static_cast<int>(std::lround(value.get<double>()));
        });
    run_sub_ = create_subscription<mowbite_msgs::msg::RunStatus>(
        "logic/run", LatchedQos(1), [this](mowbite_msgs::msg::RunStatus::ConstSharedPtr msg) {
          core_.Call<&Scheduler::OnRunStatus>(RunStatus{msg->run_id, PhaseOf(msg->phase), msg->end_code, msg->detail});
          Carry();
        });
    weather_sub_ = create_subscription<mowbite_msgs::msg::Weather>(
        "net/weather", 10, [this](mowbite_msgs::msg::Weather::ConstSharedPtr msg) {
          core_.Call<&Scheduler::OnWeather>(Weather{msg->id, msg->ok, msg->age_s, msg->has_rain, msg->rain_now_mm,
                                                    msg->rain_hour_mm, msg->has_temp, msg->temp_c});
          Carry();
        });

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_schedule";
    methods.methods = {"schedule.get", "schedule.set", "schedule.log", "schedule.leftovers"};
    methods_pub_->publish(methods);
    Carry();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { Tick(); });
    watch_ = mowbite_watch::Attach(*this);
  }

 private:
  std::string PathOf(const char* name) const {
    return data_dir_ + "/" + name;
  }

  // A file of data_dir as JSON; null when missing or not readable (with a warning).
  json ReadJson(const char* name) {
    if (data_dir_.empty()) return nullptr;
    std::string text;
    std::string error;
    const std::string path = PathOf(name);
    switch (mowbite_hold::ReadSmallFile(path, kMaxFileBytes, &text, &error)) {
      case mowbite_hold::FileRead::kMissing: return nullptr;
      case mowbite_hold::FileRead::kTooLarge: error = "larger than 64 kB"; break;
      case mowbite_hold::FileRead::kFailed: break;
      case mowbite_hold::FileRead::kText: {
        json value = json::parse(text, nullptr, false);
        if (!value.is_discarded()) return value;
        error = "not JSON";
        break;
      }
    }
    RCLCPP_WARN(get_logger(), "Cannot read %s: %s", path.c_str(), error.c_str());
    return nullptr;
  }

  SchedulerStart Load() {
    SchedulerStart start;
    // S8: the platform lets the schedule start the mower; off in every platform file until its gates are passed.
    start.available = declare_parameter("allowed", false);
    start.now = WallSeconds();
    if (data_dir_.empty()) {
      RCLCPP_WARN(get_logger(), "No data_dir: the schedule cannot keep its marks and starts nothing");
    }
    const json file = ReadJson(kFile);
    if (!file.is_null()) {
      std::string error;
      if (const std::optional<ScheduleFile> read = FileFromJson(file, &error)) {
        start.file = *read;
      } else {
        // Off, the file stays as it is until the next schedule.set.
        RCLCPP_WARN(get_logger(), "%s is not a schedule (%s): the schedule is off", PathOf(kFile).c_str(),
                    error.c_str());
      }
    }
    start.state = ReadJson(kStateFile);
    start.log = ReadJson(kLogFile);
    RCLCPP_INFO(get_logger(), "Schedule %s, %s on this platform, version %lld",
                start.file.schedule.enabled ? "on" : "off", start.available ? "allowed" : "not allowed",
                static_cast<long long>(start.file.version));
    return start;
  }

  Facts Gather() const {
    Facts facts;
    facts.wall = WallSeconds();
    facts.mono = MonotonicSeconds();
    facts.clock_ok = clock_.ok;
    facts.clock_reason = clock_.reason;
    facts.logic_fresh = robot_state_mono_ && facts.mono - *robot_state_mono_ <= kLogicFresh;
    facts.state = robot_state_.current_state;
    facts.docked = hw_ && hw_->dock_contact;
    facts.emergency = robot_state_.emergency || (hw_ && (hw_->emergency || hw_->emergency_latched));
    facts.battery = static_cast<int>(std::lround(robot_state_.battery_percentage * 100.0));
    facts.rain_sensor = rain_sensor_;
    facts.rain_wet = hw_ && hw_->rain;
    facts.rain_mode = rain_mode_;
    facts.hold = hold_until_ > facts.wall || (hw_ && hw_->hold);
    facts.latitude = latitude_;
    facts.longitude = longitude_;
    for (const auto& area : map_.MowAreas()) facts.areas.push_back(Area{area.id, area.mowable});
    return facts;
  }

  void Tick() {
    core_.Call<&Scheduler::Tick>(Gather());
    Carry();
  }

  // Carries out what the scheduler asks for until it asks for nothing more, then publishes the next start.
  void Carry() {
    for (int round = 0; round < 4; ++round) {
      const Output out = core_.Call<&Scheduler::TakeOutput>();
      if (out.weather) {
        mowbite_msgs::msg::WeatherRequest request;
        request.id = out.weather->id;
        request.latitude = out.weather->latitude;
        request.longitude = out.weather->longitude;
        request.hours = out.weather->hours;
        weather_pub_->publish(request);
      }
      if (out.file_changed) Write(kFile, FileToJson(core_.View().file()));
      if (out.log_changed) Write(kLogFile, core_.View().LogJson());
      if (out.state_changed || out.start) Write(kStateFile, core_.View().StateJson());
      if (out.start) Start(*out.start);
      if (out.drop) Drop(*out.drop);
      if (!out.start) break;
    }
    PublishNextStart();
    MirrorReady();
  }

  void Write(const char* name, const json& value) {
    if (data_dir_.empty()) return;
    writer_.Submit(PathOf(name), value.dump(2) + "\n");
  }

  // S4: the mark first; a start whose mark is not on the disk does not go out.
  void Start(const RunRequest& run) {
    const bool marked = !data_dir_.empty() &&
                        writer_.WaitFor(PathOf(kStateFile), kMarkWait) == mowbite_hold::FileWriter::Wait::kWritten;
    if (!marked) {
      RCLCPP_ERROR(get_logger(), "Schedule: %s not started, its mark is not on the disk", run.run_id.c_str());
      core_.Call<&Scheduler::StartFailed>(run.run_id, std::string("storage"));
      return;
    }
    if (!start_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Schedule: %s not started, logic/start_run is not there", run.run_id.c_str());
      core_.Call<&Scheduler::StartFailed>(run.run_id, std::string("service"));
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::StartRun::Request>();
    request->run_id = run.run_id;
    request->area_ids = run.area_ids;
    request->pass_over = run.pass_over;
    request->restart = run.restart;
    request->has_angle = run.angle_offset_deg.has_value();
    request->angle_offset_deg = run.angle_offset_deg.value_or(0.0);
    request->dark_allowed = run.dark_allowed;
    request->until_sunset = run.until_sunset;
    request->end_in_s = run.end_in_s;
    RCLCPP_INFO(get_logger(), "Schedule: starting %s", run.run_id.c_str());
    const std::string run_id = run.run_id;
    start_client_->async_send_request(
        request, [this, run_id](rclcpp::Client<mowbite_msgs::srv::StartRun>::SharedFuture future) {
          const auto answer = future.get();
          if (!answer->accepted) {
            RCLCPP_WARN(get_logger(), "Schedule: %s refused: %s %s", run_id.c_str(), answer->code.c_str(),
                        answer->message.c_str());
          }
          core_.Call<&Scheduler::OnStartAnswer>(run_id, static_cast<bool>(answer->accepted), answer->code,
                                                answer->accepted ? json() : FirstValue(answer->data));
          Carry();
        });
  }

  void Drop(const DropRequest& drop) {
    if (!drop_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Schedule: cannot pause areas of %s, logic/drop_tasks is not there",
                   drop.run_id.c_str());
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::DropTasks::Request>();
    request->run_id = drop.run_id;
    request->area_ids = drop.area_ids;
    drop_client_->async_send_request(
        request, [this](rclcpp::Client<mowbite_msgs::srv::DropTasks>::SharedFuture future) {
          const auto answer = future.get();
          if (!answer->accepted) RCLCPP_WARN(get_logger(), "Schedule: pause refused: %s", answer->message.c_str());
        });
  }

  void PublishNextStart() {
    const int64_t next = core_.View().NextStart().value_or(0);
    if (published_next_ == next) return;
    published_next_ = next;
    std_msgs::msg::Int64 msg;
    msg.data = next;
    next_start_pub_->publish(msg);
  }

  // On every change, again every kReadyRepeat while a start is to come, empty once after the start and once none is.
  // It waits for the bridge.
  void MirrorReady() {
    if (publish_pub_->get_subscription_count() == 0) return;
    const std::optional<Ready> ready = core_.View().ReadyNow();
    const double now = MonotonicSeconds();
    if (ready_mirrored_ && ready == ready_sent_ && (!ready || now - ready_mirrored_mono_ < kReadyRepeat)) return;
    ready_mirrored_ = true;
    ready_sent_ = ready;
    ready_mirrored_mono_ = now;
    mowbite_msgs::msg::MqttPublish msg;
    msg.topic = kReadyTopic;
    msg.payload = ready ? ReadyToJson(*ready).dump() : "";
    msg.retain = true;
    publish_pub_->publish(msg);
  }

  json Answer() const {
    const Scheduler& scheduler = core_.View();
    return GetAnswer(scheduler, clock_, SunOfDay(scheduler.file().schedule.zone, WallSeconds(), latitude_, longitude_));
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    const std::string& method = request.method;
    if (method != "schedule.get" && method != "schedule.set" && method != "schedule.log" &&
        method != "schedule.leftovers") {
      return;
    }
    const json params = request.params.empty() ? json() : json::parse(request.params, nullptr, false);
    if (params.is_discarded()) {
      Refuse(request.id, RpcError{kInvalidParams, "The parameters are not valid JSON", nullptr});
      return;
    }
    std::optional<RpcError> error;
    json result;
    if (method == "schedule.get") {
      result = Answer();
    } else if (method == "schedule.set") {
      json change;
      error = CheckSet(params, core_.View().file().version, &change);
      if (!error) {
        if (const std::optional<Invalid> invalid = core_.Call<&Scheduler::SetSchedule>(change, WallSeconds())) {
          error = InvalidError(*invalid);
        }
      }
      if (!error) {
        Carry();
        const ScheduleFile& file = core_.View().file();
        RCLCPP_INFO(get_logger(), "schedule.set: version %lld, %s, %zu plan(s)", static_cast<long long>(file.version),
                    file.schedule.enabled ? "on" : "off", file.schedule.plans.size());
        result = Answer();
      }
    } else if (method == "schedule.log") {
      size_t limit = 0;
      error = CheckLog(params, &limit);
      if (!error) result = {{"entries", core_.View().LogEntries(limit)}};
    } else {
      json change;
      error = CheckLeftovers(params, &change);
      if (!error && !change.empty()) {
        if (const std::optional<Invalid> invalid = core_.Call<&Scheduler::SetLeftovers>(change)) {
          error = InvalidError(*invalid);
        }
        Carry();
      }
      if (!error) result = LeftoversToJson(core_.View().file().leftovers);
    }
    if (error) {
      Refuse(request.id, *error);
      return;
    }
    mowbite_msgs::msg::RpcResponse response;
    response.id = request.id;
    response.result = result.dump();
    response_pub_->publish(response);
  }

  void Refuse(const std::string& id, const RpcError& error) {
    RCLCPP_WARN(get_logger(), "schedule refused: %s", error.message.c_str());
    mowbite_msgs::msg::RpcError msg;
    msg.id = id;
    msg.code = error.code;
    msg.message = error.message;
    msg.data = error.data.is_null() ? "" : error.data.dump();
    error_pub_->publish(msg);
  }

  mowbite_journal::Journal* journal_;
  const std::string data_dir_;
  mowbite_journal::Journaled<Scheduler> core_;
  mowbite_hold::FileWriter writer_;
  // What the facts are made of; until time/clock comes the clock is not safe.
  mowbite_msgs::msg::RobotState robot_state_;
  std::optional<double> robot_state_mono_;
  std::optional<mowbite_msgs::msg::HardwareStatus> hw_;
  bool rain_sensor_ = false;
  int rain_mode_ = 0;
  int64_t hold_until_ = 0;
  ClockState clock_{false, "never_synced"};
  std::optional<double> latitude_;
  std::optional<double> longitude_;
  mowbite_logic::MowingPlanner map_;
  std::optional<int64_t> published_next_;
  bool ready_mirrored_ = false;
  std::optional<Ready> ready_sent_;
  double ready_mirrored_mono_ = 0;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr next_start_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::WeatherRequest>::SharedPtr weather_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Client<mowbite_msgs::srv::StartRun>::SharedPtr start_client_;
  rclcpp::Client<mowbite_msgs::srv::DropTasks>::SharedPtr drop_client_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr hold_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ClockStatus>::SharedPtr clock_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr datum_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RunStatus>::SharedPtr run_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::Weather>::SharedPtr weather_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_schedule

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_schedule::ScheduleNode)
