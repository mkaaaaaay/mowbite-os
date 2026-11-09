#include <algorithm>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int64.hpp>
#include <string>
#include <vector>

#include "mowbite_bridge/json_rpc.h"
#include "mowbite_journal/journal.h"
#include "mowbite_sound/convert.h"
#include "mowbite_sound/player.h"
#include "mowbite_sound/sound_service.h"

namespace mowbite_sound {

namespace {

// Longest run of the converter for one file in s.
constexpr double kConvertTimeout = 30.0;
// How often the executor looks whether the import or the check of an upload is done, in s.
constexpr double kImportPoll = 0.5;
constexpr double kUploadPoll = 0.1;
constexpr int kLogEveryMs = 10000;

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The parameters of the node; output is none when they are not valid.
struct NodeConfig {
  SoundConfig sound;
  std::string output;
  std::string device;
  std::string file;
  std::string converter;
  std::vector<std::string> vendor_dirs;
  // Situation -> the vendor's code of its announcement.
  std::map<std::string, std::string> vendor_codes;
};

// What the import found, or why sound.* stays off.
struct Imported {
  std::map<std::string, SoundFile> files;
  std::vector<std::string> notes;
  std::string error;
};

}  // namespace

// The sounds: announcements for the events and states of the mower, the emergency over all of them (G33: a sound ends
// within 0.1 s of hw/status telling it, then only EMERGENCY or LIFT_TRIGGERED), and sound.* through the bridge. The
// vendor's MP3 are converted by mowbite_sound_convert, a process of its own, once at the start, an upload after its
// last part in a thread of its own. With output none, or without a sound card or a file, it offers no method and plays
// nothing.
class SoundNode : public rclcpp::Node {
 public:
  explicit SoundNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_sound", options), config_(LoadConfig()), player_(MakePlayer()),
        service_(config_.sound, player_.get()) {
    if (!player_) {
      RCLCPP_INFO(get_logger(), "output none: no sound.* methods, no sound");
      return;
    }
    std::error_code error;
    std::filesystem::create_directories(config_.sound.dir, error);
    const std::string problem = service_.Load();
    if (!problem.empty()) RCLCPP_ERROR(get_logger(), "%s; the vendor's sounds are taken again", problem.c_str());

    imported_ = std::async(std::launch::async, [this, have = service_.files()] { return Import(have); });
    import_timer_ = create_wall_timer(std::chrono::duration<double>(kImportPoll), [this] { OnImportPoll(); });
  }

  // The import and the check of an upload end after the file they convert; their futures wait for them.
  ~SoundNode() override { stopping_ = true; }

 private:
  NodeConfig LoadConfig() {
    NodeConfig config;
    SoundConfig& sound = config.sound;
    config.output = declare_parameter("output", std::string("none"));
    config.device = declare_parameter("device", std::string("plughw:0,0"));
    config.file = declare_parameter("file", std::string());
    config.converter = declare_parameter("converter", std::string());
    sound.max_volume = declare_parameter("max_volume", sound.max_volume);
    const int64_t volume = declare_parameter("volume", static_cast<int64_t>(sound.volume));
    sound.min_gap = declare_parameter("min_gap", sound.min_gap);
    config.vendor_dirs = declare_parameter("vendor_dirs", std::vector<std::string>{});
    const auto codes = declare_parameter("vendor_codes", std::vector<std::string>{});
    const std::string data_dir = declare_parameter("data_dir", std::string());
    sound.dir = data_dir + "/sounds";
    const double reserve_mb = declare_parameter("journal_reserve_mb", mowbite_journal::Options().reserve_mb);
    sound.reserve_bytes = static_cast<int64_t>(std::max(0.0, reserve_mb) * 1e6);
    if (config.converter.empty()) {
      try {
        config.converter =
            ament_index_cpp::get_package_prefix("mowbite_sound") + "/lib/mowbite_sound/mowbite_sound_convert";
      } catch (const std::exception&) {
      }
    }

    std::string why;
    for (const auto& entry : codes) {
      const size_t equals = entry.find('=');
      const std::string situation = entry.substr(0, equals);
      if (equals == std::string::npos || !IsSituation(situation) || equals + 1 == entry.size()) {
        why = "vendor_codes are SITUATION=code with a situation of sound.list, not " + entry;
        break;
      }
      config.vendor_codes[situation] = entry.substr(equals + 1);
    }
    if (config.output != "none" && config.output != "alsa" && config.output != "file") {
      why = "output is none, alsa or file";
    } else if (config.output == "file" && config.file.empty()) {
      why = "output file needs file";
    } else if (config.output != "none" && data_dir.empty()) {
      why = "the sounds need data_dir";
    } else if (!(sound.max_volume >= 0 && sound.max_volume <= 1) || volume < 0 || volume > 100 ||
               !(sound.min_gap >= 0)) {
      why = "max_volume is 0 to 1, volume 0 to 100, min_gap at least 0";
    }
    if (!why.empty()) {
      RCLCPP_ERROR(get_logger(), "Invalid parameters (%s): no sound", why.c_str());
      config.output = "none";
    }
    sound.volume = static_cast<int>(std::clamp<int64_t>(volume, 0, 100));
    return config;
  }

  std::unique_ptr<Player> MakePlayer() {
    if (config_.output == "none") return nullptr;
    auto output = config_.output == "alsa" ? MakeAlsaOutput(config_.device) : MakeFileOutput(config_.file);
    return std::make_unique<Player>(std::move(output), [this](const std::string& problem) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kLogEveryMs, "%s", problem.c_str());
    });
  }

  // In a thread of its own: the card, then the vendor's announcements that are not converted yet.
  Imported Import(const std::map<std::string, SoundFile>& have) {
    Imported out;
    if (config_.output == "alsa" && !ProbeAlsa(config_.device, &out.error)) return out;
    out.files = ImportVendor(config_.sound.dir, have, config_.vendor_codes, VendorDirs(config_.vendor_dirs),
                             Converter(), &out.notes);
    return out;
  }

  Convert Converter() {
    return [this](const std::string& mp3, const std::string& wav, std::string* error) {
      if (stopping_) {
        *error = "the node stops";
        return Converted::kFailed;
      }
      int code = -1;
      if (RunProgram({config_.converter, mp3, wav}, kConvertTimeout, error, &code)) return Converted::kOk;
      if (code == kExitNotMp3) return Converted::kNotMp3;
      if (code == kExitTooLong) return Converted::kTooLong;
      return Converted::kFailed;
    };
  }

  void OnImportPoll() {
    if (imported_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    import_timer_->cancel();
    Imported imported = imported_.get();
    if (!imported.error.empty()) {
      RCLCPP_ERROR(get_logger(), "%s: no sound.* methods, no sound", imported.error.c_str());
      return;
    }
    for (const auto& note : imported.notes) RCLCPP_WARN(get_logger(), "%s", note.c_str());
    const std::string problem = service_.SetFiles(std::move(imported.files));
    if (!problem.empty()) RCLCPP_ERROR(get_logger(), "Cannot keep sounds.json: %s", problem.c_str());
    if (service_.files().empty() && service_.uploads().empty()) {
      RCLCPP_ERROR(get_logger(), "No sound file: no sound.* methods, no sound");
      return;
    }
    RCLCPP_INFO(get_logger(), "%zu sounds and %zu uploads at volume %d", service_.files().size(),
                service_.uploads().size(), service_.volume());
    Start();
  }

  void Start() {
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    status_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) {
          service_.OnDriverEmergency(msg->emergency, msg->emergency_reason, MonotonicSeconds());
        });
    state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) {
          service_.OnRobotState(msg->current_state, msg->emergency, msg->emergency_reason, msg->is_charging,
                                msg->battery_percentage, MonotonicSeconds());
        });
    publish_sub_ = create_subscription<mowbite_msgs::msg::MqttPublish>(
        "bridge/publish", 100, [this](mowbite_msgs::msg::MqttPublish::ConstSharedPtr msg) {
          if (msg->topic != "events/json") return;
          service_.OnEvent(Json::parse(msg->payload, nullptr, false), MonotonicSeconds());
        });
    hold_sub_ = create_subscription<std_msgs::msg::Int64>(
        "logic/hold", LatchedQos(1), [this](std_msgs::msg::Int64::ConstSharedPtr msg) { service_.SetHold(msg->data); });

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_sound";
    methods.methods = SoundService::Methods();
    methods_pub_->publish(methods);
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    // A method of another node, which answers it; its params (a map, say) are not parsed here.
    const auto& methods = SoundService::Methods();
    if (std::find(methods.begin(), methods.end(), request.method) == methods.end()) return;
    Json params = Json::object();
    if (!request.params.empty()) params = Json::parse(request.params, nullptr, false);
    std::optional<RpcOutcome> outcome =
        params.is_discarded()
            ? RpcOutcome{false, Json(), mowbite_bridge::kInvalidParams, "The parameters are not valid JSON", Json(),
                         std::nullopt}
            : service_.Call(request.method, params, MonotonicSeconds());
    if (!outcome) return;
    if (outcome->job) {
      upload_id_ = request.id;
      checked_ = std::async(std::launch::async, [job = std::move(*outcome->job), convert = Converter()] {
        return FinishUpload(job, convert);
      });
      upload_timer_ = create_wall_timer(std::chrono::duration<double>(kUploadPoll), [this] { OnUploadPoll(); });
      return;
    }
    Answer(request.method, request.id, *outcome);
  }

  void OnUploadPoll() {
    if (checked_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    upload_timer_->cancel();
    const RpcOutcome outcome = service_.CompleteUpload(checked_.get());
    if (outcome.ok) RCLCPP_INFO(get_logger(), "Upload kept: %s", outcome.result.dump().c_str());
    Answer("sound.upload", upload_id_, outcome);
  }

  void Answer(const std::string& method, const std::string& id, const RpcOutcome& outcome) {
    if (outcome.ok) {
      mowbite_msgs::msg::RpcResponse response;
      response.result = outcome.result.dump();
      response.id = id;
      response_pub_->publish(response);
    } else {
      RCLCPP_WARN(get_logger(), "%s refused: %s", method.c_str(), outcome.message.c_str());
      mowbite_msgs::msg::RpcError error;
      error.code = outcome.code;
      error.message = outcome.message;
      error.id = id;
      if (!outcome.data.is_null()) error.data = outcome.data.dump();
      error_pub_->publish(error);
    }
  }

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  NodeConfig config_;
  std::unique_ptr<Player> player_;
  SoundService service_;
  std::atomic<bool> stopping_{false};
  std::future<Imported> imported_;
  // The check of the upload whose last part waits for its answer.
  std::future<UploadResult> checked_;
  std::string upload_id_;

  rclcpp::TimerBase::SharedPtr import_timer_;
  rclcpp::TimerBase::SharedPtr upload_timer_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr error_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr hold_sub_;
};

}  // namespace mowbite_sound

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_sound::SoundNode)
