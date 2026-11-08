#include "mowbite_sound/sound_service.h"

#include <glob.h>

#include <algorithm>
#include <cstring>
#include <filesystem>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_map/sha256.h"
#include "mowbite_sound/wav.h"

namespace mowbite_sound {

namespace fs = std::filesystem;

namespace {

constexpr int kStoreSchema = 1;
constexpr size_t kStoreMaxBytes = 64 * 1024;
constexpr char kStoreFile[] = "sounds.json";
// The group of x-reasons whose reasons the refusals of sound.* carry.
constexpr char kRefused[] = "SOUND_REFUSED";

RpcOutcome Ok(Json result) {
  return {true, std::move(result), 0, "", Json()};
}

RpcOutcome Invalid(std::string message) {
  return {false, Json(), mowbite_bridge::kInvalidParams, std::move(message), Json()};
}

std::string Mb(int64_t bytes) {
  return std::to_string(bytes >> 20) + " MB";
}

Json ToJson(const SoundFile& file) {
  Json out = {{"source", file.source}};
  if (!file.code.empty()) out["code"] = file.code;
  out["size"] = file.size;
  out["sha256"] = file.sha256;
  out["seconds"] = file.seconds;
  return out;
}

std::optional<SoundFile> FileFromJson(const Json& json) {
  if (!json.is_object()) return std::nullopt;
  const Json source = json.value("source", Json());
  const Json size = json.value("size", Json());
  const Json sha256 = json.value("sha256", Json());
  const Json seconds = json.value("seconds", Json());
  const Json code = json.value("code", Json(""));
  if (!source.is_string() || (source != "vendor" && source != "upload") || !size.is_number_integer() ||
      !sha256.is_string() || !mowbite_map::IsSha256Hex(sha256.get<std::string>()) || !seconds.is_number() ||
      !code.is_string()) {
    return std::nullopt;
  }
  return SoundFile{source.get<std::string>(), code.get<std::string>(), size.get<int64_t>(),
                   sha256.get<std::string>(), seconds.get<double>()};
}

int64_t SizeOr0(const fs::path& path) {
  std::error_code error;
  const auto size = fs::file_size(path, error);
  return error ? 0 : static_cast<int64_t>(size);
}

void RemoveQuietly(const fs::path& path) {
  std::error_code error;
  fs::remove(path, error);
}

}  // namespace

bool IsSituation(const std::string& name) {
  const auto& all = mowbite_api::SoundSituations();
  return std::any_of(all.begin(), all.end(), [&](const auto& situation) { return situation.name == name; });
}

std::string SoundPath(const std::string& dir, const std::string& situation, const char* extension) {
  return dir + "/" + situation + extension;
}

SoundService::SoundService(SoundConfig config, Sink* sink)
    : config_(std::move(config)), sink_(sink), volume_(config_.volume) {}

std::string SoundService::Load() {
  files_.clear();
  volume_ = config_.volume;
  const std::string path = StorePath();
  std::string text;
  std::string error;
  switch (mowbite_hold::ReadSmallFile(path, kStoreMaxBytes, &text, &error)) {
    case mowbite_hold::FileRead::kMissing: return "";
    case mowbite_hold::FileRead::kFailed: return error;
    case mowbite_hold::FileRead::kTooLarge: return path + " is too large";
    case mowbite_hold::FileRead::kText: break;
  }
  const Json parsed = Json::parse(text, nullptr, false);
  if (parsed.is_discarded() || !parsed.is_object() || parsed.value("schema", 0) != kStoreSchema) {
    return path + " is not JSON of schema " + std::to_string(kStoreSchema);
  }
  const Json volume = parsed.value("volume", Json());
  const Json situations = parsed.value("situations", Json::object());
  if (!volume.is_number_integer() || volume.get<int>() < 0 || volume.get<int>() > 100 || !situations.is_object()) {
    return path + " has no volume 0 to 100 or no situations";
  }
  std::map<std::string, SoundFile> files;
  for (const auto& [name, entry] : situations.items()) {
    const auto file = FileFromJson(entry);
    if (!IsSituation(name) || !file) return path + ": " + name + " is not valid";
    files[name] = *file;
  }
  files_ = std::move(files);
  volume_ = volume.get<int>();
  return "";
}

std::string SoundService::SetFiles(std::map<std::string, SoundFile> files) {
  files_ = std::move(files);
  return Save();
}

std::string SoundService::Save() {
  Json situations = Json::object();
  for (const auto& [name, file] : files_) situations[name] = ToJson(file);
  const Json store = {{"schema", kStoreSchema}, {"volume", volume_}, {"situations", situations}};
  std::string error;
  if (!mowbite_hold::WriteFileAtomic(StorePath(), store.dump(2) + "\n", &error)) return error;
  return "";
}

const std::vector<std::string>& SoundService::Methods() {
  static const std::vector<std::string> methods = {"sound.list", "sound.play", "sound.stop", "sound.volume"};
  return methods;
}

std::optional<RpcOutcome> SoundService::Call(const std::string& method, const Json& params) {
  if (method == "sound.list") return List();
  if (method == "sound.play") return Play(params);
  if (method == "sound.stop") return Stop();
  if (method == "sound.volume") return Volume(params);
  return std::nullopt;
}

void SoundService::OnEvent(const Json& event, double now) {
  if (!event.is_object()) return;
  const std::string type = event.value("type", "");
  const Json reason = event.value("reason", Json());
  const auto reason_is = [&](std::initializer_list<const char*> codes) {
    return reason.is_string() && std::any_of(codes.begin(), codes.end(), [&](const char* c) { return reason == c; });
  };
  if (type == "STATE") {
    const Json state = event.value("state", Json());
    if (state.is_string()) OnState(state.get<std::string>(), now);
  } else if (type == "JOB_COMPLETE") {
    Announce("MOWING_COMPLETE", now);
  } else if (type == "BATTERY_LOW") {
    Announce("LOW_BATTERY", now);
  } else if (type == "DOCKING" && reason_is({"BATTERY_LOW", "BATTERY_CRITICAL"})) {
    Announce("LOW_BATTERY", now);
  } else if (type == "DOCKING" && reason_is({"RAIN"})) {
    Announce("RAIN_DETECTED", now);
  } else if (type == "DOCKING_FAILED") {
    Announce("DOCKING_FAILED", now);
  }
}

void SoundService::OnState(const std::string& state, double now) {
  const std::string before = state_;
  state_ = state;
  state_from_events_ = true;
  if (state == before) return;
  if (state == "MOWING" && before != "PAUSED") Announce("MOWING_START", now);
  if (state == "PAUSED") Announce("MOWING_PAUSED", now);
  if (state == "DOCKING") Announce("DOCKING_START", now);
  if (state == "AREA_RECORDING") Announce("AREA_RECORDING_MODE", now);
}

void SoundService::OnDriverEmergency(bool emergency, uint32_t bits, double now) {
  UpdateEmergency(&driver_, emergency, bits, now);
}

void SoundService::OnRobotState(const std::string& state, bool emergency, uint32_t bits, bool charging,
                                double battery, double now) {
  UpdateEmergency(&robot_, emergency, bits, now);
  if (!state_from_events_) state_ = state;
  // The first value after the start is no change: nothing on booting at the dock.
  if (charging_) {
    if (charging && !*charging_) Announce("CHARGING_START", now);
    if (charging && *charging_ && battery_ < kCharged && battery >= kCharged) Announce("CHARGING_DONE", now);
  }
  charging_ = charging;
  battery_ = battery;
}

void SoundService::UpdateEmergency(Emergency* source, bool emergency, uint32_t bits, double now) {
  const bool first = !source->seen;
  const bool was = InEmergency();
  const uint32_t was_bits = EmergencyBits();
  *source = {true, emergency, emergency ? bits : 0};
  if (!InEmergency()) return;
  const bool lift_joined = (EmergencyBits() & kLiftBits) && !(was_bits & kLiftBits);
  if (was && !lift_joined) return;
  // Every sound ends at the emergency; an emergency held since before the start is not announced.
  sink_->Stop();
  if (first) return;
  Announce((EmergencyBits() & kLiftBits) ? "LIFT_TRIGGERED" : "EMERGENCY", now);
}

void SoundService::Announce(const std::string& situation, double now) {
  const bool alarm = situation == "EMERGENCY" || situation == "LIFT_TRIGGERED";
  if (InEmergency() && !alarm) return;
  if (!files_.count(situation) || volume_ == 0) return;
  const auto last = last_played_.find(situation);
  if (last != last_played_.end() && now - last->second < config_.min_gap) return;
  last_played_[situation] = now;
  sink_->Play({situation, SoundPath(config_.dir, situation, ".wav"), Gain()});
}

double SoundService::Gain() const {
  const double share = volume_ / 100.0;
  return config_.max_volume * share * share;
}

std::string SoundService::StorePath() const {
  return config_.dir + "/" + kStoreFile;
}

RpcOutcome SoundService::List() const {
  Json situations = Json::array();
  for (const auto& situation : mowbite_api::SoundSituations()) {
    const auto file = files_.find(situation.name);
    Json json = file == files_.end() ? Json() : ToJson(file->second);
    if (json.is_object()) json.erase("code");
    situations.push_back({{"name", situation.name}, {"event", situation.event}, {"file", json}});
  }
  return Ok(Json{{"situations", situations}, {"volume", volume_}});
}

RpcOutcome SoundService::Play(const Json& params) {
  const Json situation = params.is_object() ? params.value("situation", Json()) : Json();
  if (!situation.is_string() || !IsSituation(situation.get<std::string>())) {
    return Invalid("params are {\"situation\": a name of sound.list}");
  }
  const std::string name = situation.get<std::string>();
  if (InEmergency()) return Refuse("EMERGENCY", "No sound during an emergency");
  const auto file = files_.find(name);
  if (file == files_.end()) return Refuse("NO_FILE", name + " has no sound");
  if (volume_ > 0) sink_->Play({name, SoundPath(config_.dir, name, ".wav"), Gain()});
  return Ok(Json{{"situation", name}, {"seconds", file->second.seconds}});
}

RpcOutcome SoundService::Stop() {
  if (InEmergency()) return Refuse("EMERGENCY", "No sound.stop during an emergency");
  sink_->Stop();
  return Ok(Json::object());
}

RpcOutcome SoundService::Volume(const Json& params) {
  if (!params.is_object()) return Invalid("params are {\"volume\": 0 to 100} or none");
  const Json volume = params.value("volume", Json());
  if (volume.is_null()) return Ok(Json{{"volume", volume_}});
  if (!volume.is_number_integer() || volume.get<int64_t>() < 0 || volume.get<int64_t>() > 100) {
    return Invalid("volume is a whole number from 0 to 100");
  }
  if (hold_until_ > 0) {
    return {false, Json(), mowbite_bridge::kHold, "An update holds the mower. The volume changes after it",
            Json{{"reason", "HOLD"}, {"until", hold_until_}}};
  }
  const int before = volume_;
  volume_ = volume.get<int>();
  const std::string error = Save();
  if (!error.empty()) {
    volume_ = before;
    return {false, Json(), mowbite_bridge::kInternalError, error, Json()};
  }
  return Ok(Json{{"volume", volume_}});
}

RpcOutcome SoundService::Refuse(const std::string& reason, const std::string& message) const {
  const mowbite_api::Rating rating = mowbite_api::RateReason(kRefused, reason);
  return {false, Json(), mowbite_bridge::kInvalidRequest, message,
          Json{{"event", kRefused},
               {"reason", reason},
               {"message", message},
               {"severity", rating.severity},
               {"resume", rating.resume}}};
}

std::vector<std::string> VendorDirs(const std::vector<std::string>& patterns) {
  std::vector<std::string> dirs;
  for (const auto& pattern : patterns) {
    glob_t found{};
    if (::glob(pattern.c_str(), GLOB_NOSORT, nullptr, &found) == 0) {
      std::vector<std::string> matches(found.gl_pathv, found.gl_pathv + found.gl_pathc);
      std::sort(matches.begin(), matches.end(),
                [](const std::string& a, const std::string& b) { return ::strverscmp(a.c_str(), b.c_str()) > 0; });
      for (auto& match : matches) {
        std::error_code error;
        if (fs::is_directory(match, error)) dirs.push_back(std::move(match));
      }
    }
    ::globfree(&found);
  }
  return dirs;
}

std::map<std::string, SoundFile> ImportVendor(const std::string& dir, const std::map<std::string, SoundFile>& have,
                                              const std::map<std::string, std::string>& codes,
                                              const std::vector<std::string>& dirs, const Convert& convert,
                                              std::vector<std::string>* notes) {
  const auto mp3_of = [&](const std::string& situation) { return SoundPath(dir, situation, ".mp3"); };
  const auto wav_of = [&](const std::string& situation) { return SoundPath(dir, situation, ".wav"); };
  std::map<std::string, SoundFile> out;
  int64_t stored = 0;
  const auto keep = [&](const std::string& situation, const SoundFile& file) {
    out[situation] = file;
    stored += SizeOr0(mp3_of(situation)) + SizeOr0(wav_of(situation));
  };
  for (const auto& [situation, file] : have) {
    std::error_code error;
    const bool same = file.source == "vendor" && codes.count(situation) && codes.at(situation) == file.code;
    if ((file.source == "upload" || same) && fs::is_regular_file(wav_of(situation), error)) keep(situation, file);
  }

  for (const auto& [situation, code] : codes) {
    if (out.count(situation)) continue;
    if (!IsSituation(situation)) {
      notes->push_back(situation + " is no situation");
      continue;
    }
    fs::path source;
    for (const auto& folder : dirs) {
      std::error_code error;
      if (fs::is_regular_file(fs::path(folder) / (code + ".mp3"), error)) {
        source = fs::path(folder) / (code + ".mp3");
        break;
      }
    }
    if (source.empty()) {
      notes->push_back(situation + ": no " + code + ".mp3 in the vendor's folders");
      continue;
    }
    std::string mp3;
    std::string error;
    if (mowbite_hold::ReadSmallFile(source, kMaxVendorBytes, &mp3, &error) != mowbite_hold::FileRead::kText) {
      notes->push_back(situation + ": " + source.string() + " cannot be read or is larger than " +
                       Mb(kMaxVendorBytes) + " " + error);
      continue;
    }
    const std::string wav_new = SoundPath(dir, situation, ".wav.new");
    std::string wav_bytes;
    std::optional<Wav> wav;
    if (!mowbite_hold::WriteFileAtomic(mp3_of(situation), mp3, &error) ||
        !convert(mp3_of(situation), wav_new, &error) ||
        mowbite_hold::ReadSmallFile(wav_new, kMaxWavBytes, &wav_bytes, &error) != mowbite_hold::FileRead::kText ||
        !(wav = DecodeWav(wav_bytes, &error))) {
      notes->push_back(situation + ": " + code + ".mp3 not converted: " + error);
    } else if (wav->Seconds() < kMinSeconds || wav->Seconds() > kMaxSeconds) {
      notes->push_back(situation + ": " + code + ".mp3 lasts " + std::to_string(wav->Seconds()) + " s, not " +
                       Json(kMinSeconds).dump() + " to " + Json(kMaxSeconds).dump() + " s");
    } else if (stored + static_cast<int64_t>(mp3.size() + wav_bytes.size()) > kMaxStoreBytes) {
      notes->push_back(situation + ": no room, the sounds may take " + Mb(kMaxStoreBytes));
    } else {
      std::error_code renamed;
      fs::rename(wav_new, wav_of(situation), renamed);
      if (!renamed) {
        keep(situation, {"vendor", code, static_cast<int64_t>(mp3.size()), mowbite_map::Sha256Hex(mp3),
                         wav->Seconds()});
        continue;
      }
      notes->push_back(situation + ": " + renamed.message());
    }
    RemoveQuietly(wav_new);
    RemoveQuietly(mp3_of(situation));
  }

  // The files of situations no longer kept.
  for (const auto& situation : mowbite_api::SoundSituations()) {
    if (out.count(situation.name)) continue;
    RemoveQuietly(mp3_of(situation.name));
    RemoveQuietly(wav_of(situation.name));
  }
  return out;
}

}  // namespace mowbite_sound
