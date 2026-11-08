#include "mowbite_light/light_service.h"

#include <algorithm>

#include "mowbite_bridge/json_rpc.h"
#include "mowbite_hold/file_io.h"

namespace mowbite_light {

namespace {

constexpr int kProfileSchema = 1;
constexpr size_t kProfileMaxBytes = 64 * 1024;

RpcOutcome Invalid(std::string message) {
  return {false, Json(), mowbite_bridge::kInvalidParams, std::move(message)};
}

}  // namespace

LightService::LightService(LightConfig config) : config_(std::move(config)), profile_(DefaultProfile()) {}

std::string LightService::Load() {
  profile_ = DefaultProfile();
  if (config_.profile_path.empty()) return "";
  std::string text;
  std::string error;
  switch (mowbite_hold::ReadSmallFile(config_.profile_path, kProfileMaxBytes, &text, &error)) {
    case mowbite_hold::FileRead::kMissing: return "";
    case mowbite_hold::FileRead::kFailed: return error;
    case mowbite_hold::FileRead::kTooLarge: return config_.profile_path + " is too large";
    case mowbite_hold::FileRead::kText: break;
  }
  const Json parsed = Json::parse(text, nullptr, false);
  if (parsed.is_discarded() || !parsed.is_object()) return config_.profile_path + " is not valid JSON";
  const auto schema = parsed.find("schema");
  if (schema == parsed.end() || !schema->is_number_integer() || schema->get<int>() != kProfileSchema) {
    return config_.profile_path + " has no schema " + std::to_string(kProfileSchema);
  }
  const auto profile = ProfileFromJson(parsed.value("profile", Json()), &error);
  if (!profile) return config_.profile_path + ": " + error;
  profile_ = *profile;
  return "";
}

const std::vector<std::string>& LightService::Methods() {
  static const std::vector<std::string> methods = {"light.get_profile", "light.set_profile", "light.preview",
                                                   "light.reset"};
  return methods;
}

std::optional<RpcOutcome> LightService::Call(const std::string& method, const Json& params, double now) {
  if (method == "light.get_profile") return GetProfile();
  if (method == "light.set_profile") return SetProfile(params);
  if (method == "light.preview") return Preview(params, now);
  if (method == "light.reset") return Reset();
  return std::nullopt;
}

void LightService::SetRobotState(const std::string& current_state, bool emergency, bool is_charging,
                                 double battery) {
  state_ = current_state;
  emergency_ = emergency;
  charging_ = is_charging;
  battery_ = battery;
}

std::vector<Rgb> LightService::Frame(double now) {
  if (preview_ && now >= preview_until_) preview_.reset();
  std::string key;
  Effect effect;
  if (emergency_) {
    key = "EMERGENCY";
    effect = EmergencyEffect();
  } else if (preview_) {
    key = "PREVIEW " + std::to_string(previews_);
    effect = *preview_;
  } else {
    key = ShownState(state_, charging_, battery_, config_.low_battery);
    effect = profile_.at(key);
  }
  if (key != shown_) {
    shown_ = key;
    shown_since_ = now;
  }
  effect.brightness *= config_.max_brightness;
  auto leds = Render(effect, config_.leds, now - shown_since_);
  LimitCurrent(&leds, config_.max_current_ma, config_.ma_per_channel);
  return leds;
}

RpcOutcome LightService::GetProfile() const {
  return {true,
          Json{{"states", ProfileStates()},
               {"effects", EffectNames()},
               {"profile", ToJson(profile_)},
               {"defaults", ToJson(DefaultProfile())},
               {"emergency", ToJson(EmergencyEffect())},
               {"limits",
                {{"colors", kMaxColors},
                 {"min_period", kMinPeriod},
                 {"max_period", kMaxPeriod},
                 {"max_preview", kMaxPreview}}},
               {"leds", config_.leds}},
          0,
          ""};
}

RpcOutcome LightService::SetProfile(const Json& params) {
  if (!params.is_object() || !params.contains("profile")) return Invalid("params are {\"profile\": {state: effect}}");
  std::string error;
  const auto profile = ProfileFromJson(params["profile"], &error);
  if (!profile) return Invalid(error);
  if (!config_.profile_path.empty()) {
    const Json file = {{"schema", kProfileSchema}, {"profile", ToJson(*profile)}};
    if (!mowbite_hold::WriteFileAtomic(config_.profile_path, file.dump(2) + "\n", &error)) {
      return {false, Json(), mowbite_bridge::kInternalError, error};
    }
  }
  profile_ = *profile;
  return {true, Json{{"profile", ToJson(profile_)}}, 0, ""};
}

RpcOutcome LightService::Preview(const Json& params, double now) {
  if (!params.is_object() || !params.contains("effect")) {
    return Invalid("params are {\"effect\": effect, \"duration\": s}");
  }
  std::string error;
  const auto effect = EffectFromJson(params["effect"], &error);
  if (!effect) return Invalid(error);
  const Json duration = params.value("duration", Json(kMaxPreview));
  if (!duration.is_number() || !(duration.get<double>() > 0 && duration.get<double>() <= kMaxPreview)) {
    return Invalid("duration is more than 0 and at most " + Json(kMaxPreview).dump() + " s");
  }
  preview_ = *effect;
  preview_until_ = now + duration.get<double>();
  ++previews_;
  return {true, Json{{"duration", duration.get<double>()}, {"emergency", emergency_}}, 0, ""};
}

RpcOutcome LightService::Reset() {
  std::string error;
  if (!config_.profile_path.empty() && !mowbite_hold::RemoveFileSynced(config_.profile_path, &error)) {
    return {false, Json(), mowbite_bridge::kInternalError, error};
  }
  profile_ = DefaultProfile();
  preview_.reset();
  return {true, Json{{"profile", ToJson(profile_)}}, 0, ""};
}

}  // namespace mowbite_light
