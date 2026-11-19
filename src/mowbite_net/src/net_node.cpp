#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/parts_use.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/schedule_notice.hpp>
#include <mowbite_msgs/msg/weather.hpp>
#include <mowbite_msgs/msg/weather_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "mowbite_hold/file_io.h"
#include "mowbite_hold/file_writer.h"
#include "mowbite_net/http.h"
#include "mowbite_net/notifier.h"
#include "mowbite_net/notify_config.h"
#include "mowbite_net/outbox.h"
#include "mowbite_net/texts.h"
#include "mowbite_net/weather.h"
#include "mowbite_watch/attach.h"

namespace mowbite_net {

namespace {

constexpr char kConfigFile[] = "notify.json";
constexpr char kStateFile[] = "notify-state.json";
constexpr char kLogFile[] = "notify-log.json";
constexpr size_t kMaxFileBytes = size_t{64} * 1024;
constexpr size_t kLogSize = 50;
constexpr size_t kWeatherQueue = 8;
// notify.test calls that may wait; more are refused, so that they cannot hold up real pushes.
constexpr size_t kMaxTests = 4;
constexpr int kInvalidParams = -32602;

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

const std::vector<std::string>& NotifyMethods() {
  static const std::vector<std::string> methods = {"notify.get",    "notify.set",         "notify.test",
                                                   "notify.push_add", "notify.push_remove", "notify.snooze",
                                                   "notify.log"};
  return methods;
}

// notify.json holds the token, so only the owner may read it.
bool WriteByName(const std::string& path, const std::string& text, std::string* error) {
  return std::filesystem::path(path).filename() == kConfigFile ? mowbite_hold::WriteFileOwnerOnly(path, text, error)
                                                               : mowbite_hold::WriteFileSafely(path, text, error);
}

}  // namespace

// The only node of the OS that talks to the internet, in a process of its own: the weather of open-meteo for the
// schedule (net/weather/request, net/weather), and the pushes of notify.* to an ntfy topic and the phones signed up in
// MowBite. Files in data_dir: notify.json (0600, holds the token), notify-state.json and notify-log.json. Neither the
// token nor an address of a phone goes into a log line, an answer or another file.
class NetNode : public rclcpp::Node {
 public:
  explicit NetNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_net", mowbite_component::StartupOnly(options)),
        data_dir_(declare_parameter("data_dir", std::string())),
        // Another server for the weather only in tests.
        weather_server_(declare_parameter("weather_server", std::string(kWeatherServer))),
        writer_(WriteByName, [this](const mowbite_hold::FileWriter::Done& done) {
          if (!done.ok) RCLCPP_ERROR(get_logger(), "Cannot write %s", done.path.c_str());
        }) {
    InitCurl();
    Load();
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    weather_pub_ = create_publisher<mowbite_msgs::msg::Weather>("net/weather", 10);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](const mowbite_msgs::msg::RpcRequest& msg) { OnRequest(msg); });
    publish_sub_ = create_subscription<mowbite_msgs::msg::MqttPublish>(
        "bridge/publish", 100, [this](const mowbite_msgs::msg::MqttPublish& msg) {
          if (msg.topic != "events/json") return;
          notifier_.OnEvent(json::parse(msg.payload, nullptr, false), MonotonicSeconds());
          Flush();
        });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](const mowbite_msgs::msg::RobotState& msg) { OnState(msg); });
    parts_sub_ = create_subscription<mowbite_msgs::msg::PartsUse>(
        "stats/parts", LatchedQos(1), [this](const mowbite_msgs::msg::PartsUse& msg) { OnParts(msg); });
    notice_sub_ = create_subscription<mowbite_msgs::msg::ScheduleNotice>(
        "schedule/notice", 20, [this](const mowbite_msgs::msg::ScheduleNotice& msg) {
          if (msg.kind == mowbite_msgs::msg::ScheduleNotice::KIND_CHANGED) {
            notifier_.OnScheduleChanged();
          } else {
            notifier_.OnScheduleLogged(msg.what, msg.at);
          }
          Flush();
        });
    weather_sub_ = create_subscription<mowbite_msgs::msg::WeatherRequest>(
        "net/weather/request", 10, [this](const mowbite_msgs::msg::WeatherRequest& msg) {
          std::lock_guard<std::mutex> lock(weather_mutex_);
          if (weather_queue_.size() >= kWeatherQueue) weather_queue_.pop_front();
          weather_queue_.push_back(msg);
          weather_wake_.notify_one();
        });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_net";
    methods.methods = NotifyMethods();
    methods_pub_->publish(methods);
    weather_thread_ = std::thread([this] { WeatherLoop(); });
    push_thread_ = std::thread([this] { PushLoop(); });
    watch_ = mowbite_watch::Attach(*this);
  }

  ~NetNode() override {
    // Under the mutexes of the waits, so that no thread checks stop_ and blocks right after the wake.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::lock_guard<std::mutex> weather_lock(weather_mutex_);
      stop_ = true;
    }
    weather_wake_.notify_all();
    push_wake_.notify_all();
    if (weather_thread_.joinable()) weather_thread_.join();
    if (push_thread_.joinable()) push_thread_.join();
  }

 private:
  struct TestJob {
    std::string id;
    NotifyConfig config;
    std::optional<std::string> address;
  };

  std::string PathOf(const char* name) const { return (std::filesystem::path(data_dir_) / name).string(); }

  // The file, or null for none; one that cannot be read is logged and kept as <name>.bad for the user, since the next
  // change writes the defaults over it.
  json Read(const char* name) {
    if (data_dir_.empty()) return json();
    std::string text, error;
    const auto read = mowbite_hold::ReadSmallFile(PathOf(name), kMaxFileBytes, &text, &error);
    if (read == mowbite_hold::FileRead::kMissing) return json();
    if (read == mowbite_hold::FileRead::kText) {
      json value = json::parse(text, nullptr, false);
      if (!value.is_discarded()) return value;
    }
    RCLCPP_ERROR(get_logger(), "%s cannot be read (%s): kept as %s.bad, the defaults hold", name,
                 read == mowbite_hold::FileRead::kText ? "not valid JSON" : error.c_str(), name);
    std::error_code ec;
    const std::string bad = PathOf(name) + ".bad";
    std::filesystem::copy_file(PathOf(name), bad, std::filesystem::copy_options::overwrite_existing, ec);
    if (!ec) {
      std::filesystem::permissions(bad, std::filesystem::perms::owner_read | std::filesystem::perms::owner_write, ec);
    }
    return json();
  }

  void Write(const char* name, const json& value) {
    if (!data_dir_.empty()) writer_.Submit(PathOf(name), value.dump() + "\n");
  }

  void Load() {
    std::string error;
    config_ = ConfigFromJson(Read(kConfigFile), &error);
    if (!error.empty()) RCLCPP_ERROR(get_logger(), "%s: %s; the defaults hold", kConfigFile, error.c_str());
    const json state = Read(kStateFile);
    if (state.is_object()) {
      const auto snooze = state.find("snooze");
      snooze_ = snooze != state.end() && snooze->is_number_integer() ? snooze->get<int64_t>() : 0;
      std::map<std::string, double> told;
      const auto found = state.find("maintenance");
      const json maintenance = found != state.end() && found->is_object() ? *found : json::object();
      for (const auto& item : maintenance.items()) {
        if (item.value().is_number()) told[item.key()] = item.value().get<double>();
      }
      notifier_.RestoreMaintenance(told);
    }
    const json log = Read(kLogFile);
    if (log.is_array()) {
      for (const json& entry : log) {
        if (entry.is_object()) log_.push_back(entry);
      }
      while (log_.size() > kLogSize) log_.pop_front();
    }
    Rules();
    RCLCPP_INFO(get_logger(), "Pushes %s: %s, %zu phone(s), %zu event(s)", config_.enabled ? "on" : "off",
                config_.topic.empty() ? "no topic" : "a topic", config_.pushes.size(), config_.events.size());
  }

  // The notifier's view of config_ and snooze_; mutex_ held or the threads not yet started.
  void Rules() {
    NotifyRules rules;
    rules.lang = config_.lang;
    rules.name = config_.name;
    rules.events = std::set<std::string>(config_.events.begin(), config_.events.end());
    rules.remind_s = config_.remind * 60.0;
    rules.emergency_wait_s = config_.emergency_wait;
    rules.snooze_until = static_cast<double>(snooze_);
    notifier_.SetRules(rules);
  }

  void SaveState() {
    json told = json::object();
    for (const auto& [key, due] : notifier_.maintenance_told()) told[key] = due;
    Write(kStateFile, {{"schema", 1}, {"snooze", snooze_}, {"maintenance", told}});
  }

  void OnState(const mowbite_msgs::msg::RobotState& msg) {
    StateView state;
    state.wall = WallSeconds();
    state.mono = MonotonicSeconds();
    state.state = msg.current_state;
    state.emergency = msg.emergency;
    state.charging = msg.is_charging;
    state.pose = msg.gps_percentage > 0;
    state.x = msg.pose_x;
    state.y = msg.pose_y;
    state.pause_reason = msg.pause_reason;
    state.pause_kind = msg.pause_kind;
    notifier_.OnState(state);
    Flush();
  }

  void OnParts(const mowbite_msgs::msg::PartsUse& msg) {
    std::vector<PartView> parts;
    for (const auto& part : msg.parts) parts.push_back({part.key, part.used_h, part.due_h, part.due});
    const auto before = notifier_.maintenance_told();
    notifier_.OnParts(parts);
    if (notifier_.maintenance_told() != before) {
      std::lock_guard<std::mutex> lock(mutex_);
      SaveState();
    }
    Flush();
  }

  // The pushes of the notifier into the outbox, with the settings now (or before, for a change of them).
  void Flush(const std::optional<NotifyConfig>& before = std::nullopt) {
    const std::vector<Push> pushes = notifier_.Take();
    if (pushes.empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const NotifyConfig& config = before ? *before : config_;
    if (!config.enabled || !config.stored || (config.topic.empty() && config.pushes.empty())) return;
    const double t = WallSeconds(), mono = MonotonicSeconds();
    for (const Push& push : pushes) outbox_.Add(push, config, t, mono);
    push_wake_.notify_one();
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    const auto& methods = NotifyMethods();
    const std::string& method = request.method;
    if (std::find(methods.begin(), methods.end(), method) == methods.end()) return;
    const json params = request.params.empty() ? json() : json::parse(request.params, nullptr, false);
    if (params.is_discarded()) {
      Refuse(request.id, {"params", "type", "not valid JSON"});
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    const NotifyConfig before = config_;
    NotifyConfig next = config_;
    std::optional<Rejection> bad;
    if (method == "notify.get") {
      // nothing to change
    } else if (method == "notify.set") {
      bad = ApplySet(params, &next);
    } else if (method == "notify.push_add") {
      bad = AddPush(params, &next);
    } else if (method == "notify.push_remove") {
      bad = RemovePush(params, &next);
    } else if (method == "notify.snooze") {
      int64_t seconds = 0;
      bad = CheckSnooze(params, &seconds);
      if (!bad) {
        snooze_ = seconds > 0 ? static_cast<int64_t>(WallSeconds()) + seconds : 0;
        Rules();
        SaveState();
      }
    } else if (method == "notify.log") {
      size_t limit = 0;
      bad = CheckLog(params, &limit);
      if (!bad) {
        json entries = json::array();
        for (auto it = log_.rbegin(); it != log_.rend() && entries.size() < limit; ++it) entries.push_back(*it);
        lock.unlock();
        Answer(request.id, entries);
        return;
      }
    } else {
      std::optional<std::string> address;
      bad = CheckTest(params, config_, &address);
      if (!bad && tests_.size() >= kMaxTests) {
        bad = Rejection{"params", "range", "at most " + std::to_string(kMaxTests) + " tests wait at a time"};
      }
      if (!bad) {
        tests_.push_back({request.id, config_, address});
        push_wake_.notify_one();
        return;
      }
    }
    if (bad) {
      lock.unlock();
      Refuse(request.id, *bad);
      return;
    }
    const bool changed = next != before;
    if (changed) {
      config_ = next;
      Write(kConfigFile, ConfigToJson(config_));
      Rules();
      RCLCPP_INFO(get_logger(), "%s: pushes %s, %s, %zu phone(s), %zu event(s)", method.c_str(),
                  config_.enabled ? "on" : "off", config_.topic.empty() ? "no topic" : "a topic",
                  config_.pushes.size(), config_.events.size());
    }
    const json answer = ConfigAnswer(config_, snooze_);
    lock.unlock();
    // G13: whoever changes where the pushes go, the targets of before hear of it.
    if (changed && before.stored) {
      notifier_.OnNotifyChanged();
      Flush(before);
    }
    Answer(request.id, answer);
  }

  void Answer(const std::string& id, const json& result) {
    mowbite_msgs::msg::RpcResponse msg;
    msg.id = id;
    msg.result = result.dump();
    response_pub_->publish(msg);
  }

  void Refuse(const std::string& id, const Rejection& bad) {
    mowbite_msgs::msg::RpcError msg;
    msg.id = id;
    msg.code = kInvalidParams;
    msg.message = bad.field + ": " + bad.message;
    msg.data = json{{"field", bad.field}, {"reason", bad.reason}}.dump();
    error_pub_->publish(msg);
  }

  void WeatherLoop() {
    std::unique_ptr<Http> http = MakeCurlHttp();
    WeatherCache cache;
    while (!stop_) {
      mowbite_msgs::msg::WeatherRequest request;
      {
        std::unique_lock<std::mutex> lock(weather_mutex_);
        weather_wake_.wait(lock, [this] { return stop_ || !weather_queue_.empty(); });
        if (stop_) break;
        request = weather_queue_.front();
        weather_queue_.pop_front();
      }
      const int hours = std::clamp(request.hours, 1, kWeatherHours);
      double age = 0;
      std::optional<WeatherData> data = cache.Get(request.latitude, request.longitude, hours, MonotonicSeconds(), &age);
      if (!data && cache.MayFetch(MonotonicSeconds())) {
        const std::string url = WeatherUrl(weather_server_, request.latitude, request.longitude, kWeatherHours);
        const HttpResult result = http->Send({url, {}, ""});
        WeatherData fetched = result.status == 200 ? ParseWeather(result.body, kWeatherHours) : WeatherData();
        if (!fetched.ok) {
          RCLCPP_WARN(get_logger(), "Weather: no usable answer (%s)",
                      result.status == 0 ? result.error.c_str() : ("HTTP " + std::to_string(result.status)).c_str());
        }
        cache.Fetched(request.latitude, request.longitude, kWeatherHours, MonotonicSeconds(), fetched);
        data = cache.Get(request.latitude, request.longitude, hours, MonotonicSeconds(), &age);
      }
      mowbite_msgs::msg::Weather answer;
      answer.id = request.id;
      if (data) {
        answer.ok = true;
        answer.age_s = age;
        answer.has_rain = data->has_rain;
        answer.rain_now_mm = data->rain_now_mm;
        answer.rain_hour_mm = data->rain_hour_mm;
        answer.has_temp = data->has_temp;
        answer.temp_c = data->temp_c;
      }
      weather_pub_->publish(answer);
    }
  }

  void PushLoop() {
    std::unique_ptr<Http> http = MakeCurlHttp();
    while (!stop_) {
      std::optional<TestJob> test;
      std::optional<Outbox::Item> item;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto ready = [this] { return stop_ || !tests_.empty() || outbox_.Due(MonotonicSeconds()); };
        // Asleep until a push, a test or the next try of one that did not go through.
        if (const std::optional<double> next = outbox_.NextDue()) {
          push_wake_.wait_for(lock, std::chrono::duration<double>(std::max(0.0, *next - MonotonicSeconds())), ready);
        } else {
          push_wake_.wait(lock, ready);
        }
        if (stop_) break;
        // A push that is due goes before a test.
        if (outbox_.Due(MonotonicSeconds())) {
          item = outbox_.Take(MonotonicSeconds());
        } else if (!tests_.empty()) {
          test = tests_.front();
          tests_.pop_front();
        }
      }
      if (test) {
        RunTest(*http, *test);
      } else if (item) {
        const Delivery delivery = Deliver(*http, item->push, item->config, item->t, std::nullopt, Stopping());
        std::lock_guard<std::mutex> lock(mutex_);
        // A push cut short by the stop is lost with the outbox, like the ones still waiting.
        if (!delivery.stopped) Logged(item->push, delivery.sent);
        Gone(delivery.gone);
        if (delivery.again) outbox_.Again(*item, MonotonicSeconds());
      }
    }
  }

  void RunTest(Http& http, const TestJob& job) {
    Push push;
    push.kind = "test";
    push.tag = "seedling";
    push.path = "/settings/";
    push.title = Say(job.config.lang, "test.title");
    const std::string name = job.config.name.empty() ? Say(job.config.lang, "mower") : job.config.name;
    push.message = Say(job.config.lang, "test", {{"n", name}});
    const Delivery delivery = Deliver(http, push, job.config, WallSeconds(), job.address, Stopping());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (delivery.targets && !delivery.stopped) Logged(push, delivery.sent);
      Gone(delivery.gone);
    }
    if (delivery.stopped) return;
    Answer(job.id, {{"result", !delivery.targets ? "no_target" : delivery.sent ? "sent" : "failed"}});
  }

  // For Deliver: whether the node is being stopped.
  std::function<bool()> Stopping() const {
    return [this] { return stop_.load(); };
  }

  // mutex_ held.
  void Logged(const Push& push, bool sent) {
    log_.push_back({{"t", static_cast<int64_t>(WallSeconds())}, {"sent", sent}, {"kind", push.kind},
                    {"title", push.title}});
    while (log_.size() > kLogSize) log_.pop_front();
    Write(kLogFile, json(log_));
  }

  // Phones that are no longer signed up; mutex_ held.
  void Gone(const std::vector<std::string>& gone) {
    if (gone.empty()) return;
    for (const std::string& address : gone) {
      config_.pushes.erase(std::remove(config_.pushes.begin(), config_.pushes.end(), address), config_.pushes.end());
    }
    Write(kConfigFile, ConfigToJson(config_));
    RCLCPP_INFO(get_logger(), "%zu phone(s) no longer signed up, %zu left", gone.size(), config_.pushes.size());
  }

  const std::string data_dir_;
  const std::string weather_server_;
  mowbite_hold::FileWriter writer_;
  Notifier notifier_;

  std::mutex mutex_;  // config_, snooze_, log_, outbox_, tests_
  NotifyConfig config_;
  int64_t snooze_ = 0;
  std::deque<json> log_;
  Outbox outbox_;
  std::deque<TestJob> tests_;
  std::condition_variable push_wake_;

  std::mutex weather_mutex_;
  std::deque<mowbite_msgs::msg::WeatherRequest> weather_queue_;
  std::condition_variable weather_wake_;

  std::atomic<bool> stop_{false};
  std::thread weather_thread_;
  std::thread push_thread_;

  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::Weather>::SharedPtr weather_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::PartsUse>::SharedPtr parts_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ScheduleNotice>::SharedPtr notice_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::WeatherRequest>::SharedPtr weather_sub_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_net

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_net::NetNode)
