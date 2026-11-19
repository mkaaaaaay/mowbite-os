#include "mowbite_sound/sound_service.h"

#include <glob.h>
#include <sys/statvfs.h>

#include <algorithm>
#include <cstring>
#include <filesystem>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_map/sha256.h"
#include "mowbite_sound/convert.h"
#include "mowbite_sound/wav.h"

namespace mowbite_sound {

namespace fs = std::filesystem;

namespace {

constexpr int kStoreSchema = 1;
constexpr size_t kStoreMaxBytes = 64 * 1024;
constexpr char kStoreFile[] = "sounds.json";
// The group of x-reasons whose reasons the refusals of sound.* carry.
constexpr char kRefused[] = "SOUND_REFUSED";
constexpr char kUploadFolder[] = "upload";
// Parts of kMaxPartBytes that hold the largest MP3.
constexpr int64_t kMaxParts = (static_cast<int64_t>(kMaxMp3Bytes) + kMaxPartBytes - 1) / kMaxPartBytes;
// Base64 of a part of kMaxPartBytes.
constexpr size_t kMaxPartText = (static_cast<size_t>(kMaxPartBytes) + 2) / 3 * 4;

RpcOutcome Ok(Json result) {
  return {true, std::move(result), 0, "", Json(), std::nullopt};
}

RpcOutcome Invalid(std::string message) {
  return {false, Json(), mowbite_bridge::kInvalidParams, std::move(message), Json(), std::nullopt};
}

RpcOutcome Internal(std::string message) {
  return {false, Json(), mowbite_bridge::kInternalError, std::move(message), Json(), std::nullopt};
}

RpcOutcome Refuse(const std::string& reason, const std::string& message, Json details = Json::object()) {
  const mowbite_api::Rating rating = mowbite_api::RateReason(kRefused, reason);
  details["event"] = kRefused;
  details["reason"] = reason;
  details["message"] = message;
  details["severity"] = rating.severity;
  details["resume"] = rating.resume;
  return {false, Json(), mowbite_bridge::kInvalidRequest, message, details, std::nullopt};
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

// The file of a situation as sound.list shows it, null for none.
Json ListedFile(const SoundFile* file) {
  if (!file) return Json();
  Json json = ToJson(*file);
  json.erase("code");
  return json;
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

// Standard base64 with its padding; nullopt for anything else.
std::optional<std::string> DecodeBase64(const std::string& text) {
  if (text.size() % 4 != 0) return std::nullopt;
  std::string out;
  out.reserve(text.size() / 4 * 3);
  uint32_t buffer = 0;
  int bits = 0;
  size_t padding = 0;
  for (size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    uint32_t value = 0;
    if (c >= 'A' && c <= 'Z') {
      value = static_cast<uint32_t>(c - 'A');
    } else if (c >= 'a' && c <= 'z') {
      value = static_cast<uint32_t>(c - 'a' + 26);
    } else if (c >= '0' && c <= '9') {
      value = static_cast<uint32_t>(c - '0' + 52);
    } else if (c == '+' || c == '/') {
      value = c == '+' ? 62 : 63;
    } else if (c == '=' && i + 2 >= text.size()) {
      ++padding;
      continue;
    } else {
      return std::nullopt;
    }
    if (padding > 0) return std::nullopt;
    buffer = (buffer << 6) | value;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((buffer >> bits) & 0xff));
    }
  }
  return out;
}

// Why adding bytes does not fit, "" when it does: all sounds together at most kMaxStoreBytes, the reserve stays free.
std::string NoSpace(int64_t stored, int64_t adding, int64_t free, int64_t reserve) {
  if (stored + adding > kMaxStoreBytes) return "No room: all sounds together may take " + Mb(kMaxStoreBytes);
  if (free >= 0 && free - adding < reserve) return "No room: the journal keeps " + Mb(reserve) + " free";
  return "";
}

}  // namespace

bool IsSituation(const std::string& name) {
  const auto& all = mowbite_api::SoundSituations();
  return std::any_of(all.begin(), all.end(), [&](const auto& situation) { return situation.name == name; });
}

std::string SoundPath(const std::string& dir, const std::string& situation, const char* extension) {
  return dir + "/" + situation + extension;
}

std::string UploadPath(const std::string& dir, const std::string& situation, const std::string& sha256,
                       const char* extension) {
  return dir + "/" + situation + "-" + sha256.substr(0, 16) + extension;
}

int64_t FreeBytesOf(const std::string& path) {
  struct statvfs fs {};
  if (::statvfs(path.c_str(), &fs) != 0) return -1;
  return static_cast<int64_t>(fs.f_bavail) * static_cast<int64_t>(fs.f_frsize);
}

namespace {

// The checks of FinishUpload; the files go under their names only when all pass.
UploadResult CheckAndKeep(const UploadJob& job, const Convert& convert, const std::string& mp3_new,
                          const std::string& wav_new) {
  const auto refused = [](const std::string& reason, const std::string& message, Json details = Json::object()) {
    return UploadResult{std::nullopt, Refuse(reason, message, std::move(details))};
  };
  const auto failed = [](const std::string& error) { return UploadResult{std::nullopt, Internal(error)}; };
  if (mowbite_map::Sha256Hex(job.bytes) != job.sha256) {
    return refused("CHECKSUM", "The file that came has another sha256 than the one given");
  }
  std::string error;
  std::error_code made;
  fs::create_directories(job.dir, made);
  if (!mowbite_hold::WriteFileAtomic(mp3_new, job.bytes, &error)) return failed("cannot keep the upload: " + error);
  switch (convert(mp3_new, wav_new, &error)) {
    case Converted::kOk: break;
    case Converted::kNotMp3: return refused("NOT_MP3", "The file is no MP3 the mower can play: " + error);
    case Converted::kTooLong:
      return refused("TOO_LONG", "The sound is longer than " + Json(kMaxSeconds).dump() + " s",
                     {{"limit", kMaxSeconds}});
    case Converted::kFailed: return failed("cannot convert the upload: " + error);
  }
  std::string wav;
  if (mowbite_hold::ReadSmallFile(wav_new, kMaxWavBytes, &wav, &error) != mowbite_hold::FileRead::kText) {
    return failed("cannot read the converted upload: " + error);
  }
  const std::optional<Wav> decoded = DecodeWav(wav, &error);
  if (!decoded) return failed("the converted upload is no WAV: " + error);
  const int64_t size = static_cast<int64_t>(job.bytes.size() + wav.size());
  const std::string no_space =
      NoSpace(job.stored, size, job.free_bytes ? job.free_bytes(job.dir) : -1, job.reserve_bytes);
  if (!no_space.empty()) return refused("NO_SPACE", no_space);
  std::error_code renamed;
  fs::rename(mp3_new, UploadPath(job.dir, job.situation, job.sha256, ".mp3"), renamed);
  // The WAV is written synced with its folder, so the renamed original lasts as well.
  if (renamed || !mowbite_hold::WriteFileAtomic(UploadPath(job.dir, job.situation, job.sha256, ".wav"), wav, &error)) {
    return failed("cannot keep the upload: " + (renamed ? renamed.message() : error));
  }
  return {SoundFile{"upload", "", static_cast<int64_t>(job.bytes.size()), job.sha256, decoded->Seconds()}, {}};
}

}  // namespace

UploadResult FinishUpload(const UploadJob& job, const Convert& convert) {
  const std::string mp3_new = UploadPath(job.dir, job.situation, job.sha256, ".mp3.new");
  const std::string wav_new = UploadPath(job.dir, job.situation, job.sha256, ".wav.new");
  UploadResult result = CheckAndKeep(job, convert, mp3_new, wav_new);
  RemoveQuietly(mp3_new);
  RemoveQuietly(wav_new);
  return result;
}

SoundService::SoundService(SoundConfig config, Sink* sink)
    : config_(std::move(config)), sink_(sink), volume_(config_.volume) {}

std::string SoundService::Load() {
  files_.clear();
  uploads_.clear();
  upload_.reset();
  volume_ = config_.volume;
  const std::string problem = LoadStore();
  // Files of the upload folder sounds.json does not name: leftovers of an upload or a crash.
  std::error_code error;
  for (const auto& entry : fs::directory_iterator(UploadDir(), error)) {
    const std::string path = entry.path().string();
    const bool named = std::any_of(uploads_.begin(), uploads_.end(), [&](const auto& upload) {
      return path == UploadPath(UploadDir(), upload.first, upload.second.sha256, ".mp3") ||
             path == UploadPath(UploadDir(), upload.first, upload.second.sha256, ".wav");
    });
    if (!named) RemoveQuietly(entry.path());
  }
  return problem;
}

std::string SoundService::LoadStore() {
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
  const Json uploads = parsed.value("uploads", Json::object());
  if (!volume.is_number_integer() || volume.get<int>() < 0 || volume.get<int>() > 100 || !situations.is_object() ||
      !uploads.is_object()) {
    return path + " has no volume 0 to 100, no situations or uploads that are no object";
  }
  std::map<std::string, SoundFile> files;
  for (const auto& [name, entry] : situations.items()) {
    const auto file = FileFromJson(entry);
    if (!IsSituation(name) || !file) return path + ": " + name + " is not valid";
    files[name] = *file;
  }
  std::map<std::string, SoundFile> kept;
  for (const auto& [name, entry] : uploads.items()) {
    const auto file = FileFromJson(entry);
    if (!IsSituation(name) || !file || file->source != "upload") return path + ": upload " + name + " is not valid";
    std::error_code error;
    if (fs::is_regular_file(UploadPath(UploadDir(), name, file->sha256, ".wav"), error)) kept[name] = *file;
  }
  files_ = std::move(files);
  uploads_ = std::move(kept);
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
  Json uploads = Json::object();
  for (const auto& [name, file] : uploads_) uploads[name] = ToJson(file);
  const Json store = {{"schema", kStoreSchema}, {"volume", volume_}, {"situations", situations}, {"uploads", uploads}};
  std::string error;
  if (!mowbite_hold::WriteFileAtomic(StorePath(), store.dump(2) + "\n", &error)) return error;
  return "";
}

const std::vector<std::string>& SoundService::Methods() {
  static const std::vector<std::string> methods = {"sound.list",   "sound.play",   "sound.stop",
                                                   "sound.volume", "sound.upload", "sound.delete"};
  return methods;
}

std::optional<RpcOutcome> SoundService::Call(const std::string& method, const Json& params, double now) {
  if (method == "sound.list") return List();
  if (method == "sound.play") return Play(params);
  if (method == "sound.stop") return Stop();
  if (method == "sound.volume") return Volume(params);
  if (method == "sound.upload") return ReceivePart(params, now);
  if (method == "sound.delete") return Delete(params);
  return std::nullopt;
}

RpcOutcome SoundService::CompleteUpload(const UploadResult& result) {
  const std::string name = upload_ ? upload_->situation : "";
  upload_.reset();
  if (!result.file) return result.failure;
  const auto saved = uploads_;
  uploads_[name] = *result.file;
  const auto before = saved.find(name);
  const bool same = before != saved.end() && before->second.sha256 == result.file->sha256;
  // sounds.json decides which files count: the old ones go once it names the new ones.
  const std::string error = Save();
  if (!error.empty()) {
    uploads_ = saved;
    if (!same) RemoveUpload(name, result.file->sha256);
    return Internal(error);
  }
  if (before != saved.end() && !same) RemoveUpload(name, before->second.sha256);
  return Ok(Json{{"situation", name},
                 {"source", "upload"},
                 {"duration_s", result.file->seconds},
                 {"sha256", result.file->sha256}});
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
  } else if (type == "SCHEDULE_START") {
    Announce("SCHEDULE_START", now);
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
  if (volume_ == 0) return;
  if (alarm) {
    const std::string played = Active(situation) || !Active("EMERGENCY") ? situation : "EMERGENCY";
    sink_->Play({situation, Active(played) ? WavOf(played) : "", Gain(), true});
    return;
  }
  if (!Active(situation)) return;
  const auto last = last_played_.find(situation);
  if (last != last_played_.end() && now - last->second < config_.min_gap) return;
  last_played_[situation] = now;
  sink_->Play({situation, WavOf(situation), Gain()});
}

double SoundService::Gain() const {
  const double share = volume_ / 100.0;
  return config_.max_volume * share * share;
}

const SoundFile* SoundService::Active(const std::string& situation) const {
  const auto upload = uploads_.find(situation);
  if (upload != uploads_.end()) return &upload->second;
  const auto vendor = files_.find(situation);
  return vendor == files_.end() ? nullptr : &vendor->second;
}

std::string SoundService::WavOf(const std::string& situation) const {
  const auto upload = uploads_.find(situation);
  if (upload == uploads_.end()) return SoundPath(config_.dir, situation, ".wav");
  return UploadPath(UploadDir(), situation, upload->second.sha256, ".wav");
}

std::string SoundService::UploadDir() const {
  return config_.dir + "/" + kUploadFolder;
}

void SoundService::RemoveUpload(const std::string& situation, const std::string& sha256) const {
  RemoveQuietly(UploadPath(UploadDir(), situation, sha256, ".mp3"));
  RemoveQuietly(UploadPath(UploadDir(), situation, sha256, ".wav"));
}

int64_t SoundService::StoredBytes(const std::string& situation) const {
  int64_t bytes = 0;
  for (const auto& [name, file] : files_) {
    bytes += SizeOr0(SoundPath(config_.dir, name, ".mp3")) + SizeOr0(SoundPath(config_.dir, name, ".wav"));
  }
  for (const auto& [name, file] : uploads_) {
    if (name == situation) continue;
    bytes += SizeOr0(UploadPath(UploadDir(), name, file.sha256, ".mp3")) +
             SizeOr0(UploadPath(UploadDir(), name, file.sha256, ".wav"));
  }
  return bytes;
}

std::string SoundService::StorePath() const {
  return config_.dir + "/" + kStoreFile;
}

RpcOutcome SoundService::List() const {
  Json situations = Json::array();
  for (const auto& situation : mowbite_api::SoundSituations()) {
    situations.push_back(
        {{"name", situation.name}, {"event", situation.event}, {"file", ListedFile(Active(situation.name))}});
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
  const SoundFile* file = Active(name);
  if (!file) return Refuse("NO_FILE", name + " has no sound");
  if (volume_ > 0) sink_->Play({name, WavOf(name), Gain()});
  return Ok(Json{{"situation", name}, {"seconds", file->seconds}});
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
  if (hold_until_ > 0) return RefuseInHold("The volume changes");
  const int before = volume_;
  volume_ = volume.get<int>();
  const std::string error = Save();
  if (!error.empty()) {
    volume_ = before;
    return Internal(error);
  }
  return Ok(Json{{"volume", volume_}});
}

RpcOutcome SoundService::ReceivePart(const Json& params, double now) {
  static const Json kNone;
  const auto field = [&](const char* name) -> const Json& {
    if (!params.is_object()) return kNone;
    const auto found = params.find(name);
    return found == params.end() ? kNone : *found;
  };
  const Json& situation = field("situation");
  const Json& part = field("part");
  const Json& parts = field("parts");
  const Json& data = field("data");
  const Json& sha256 = field("sha256");
  if (!situation.is_string() || !IsSituation(situation.get<std::string>()) || !part.is_number_integer() ||
      !parts.is_number_integer() || part.get<int64_t>() < 0 || part.get<int64_t>() >= parts.get<int64_t>() ||
      !data.is_string() || !sha256.is_string() || !mowbite_map::IsSha256Hex(sha256.get<std::string>())) {
    return Invalid(
        "params are {\"situation\": a name of sound.list, \"part\": from 0, \"parts\", \"data\": base64, "
        "\"sha256\": of the whole file, lower-case hex}");
  }
  if (hold_until_ > 0) return RefuseInHold("Uploads are taken");

  const std::string name = situation.get<std::string>();
  const std::string hash = sha256.get<std::string>();
  const int64_t index = part.get<int64_t>();
  const int64_t count = parts.get<int64_t>();
  if (upload_ && !upload_->Checked() && now - upload_->last > kUploadExpiry) upload_.reset();
  if (upload_ && upload_->Checked()) {
    return Refuse("BUSY", "The upload for " + upload_->situation + " is being checked");
  }
  if (upload_ && (upload_->situation != name || upload_->sha256 != hash)) {
    return Refuse("BUSY", "The upload for " + upload_->situation + " is not finished");
  }
  if (index == 0) {
    // The same file from its start again.
    upload_.reset();
  } else if (!upload_ || upload_->next != index || upload_->parts != count) {
    const int64_t expected = upload_ ? upload_->next : 0;
    return Refuse("PART_ORDER", "Part " + std::to_string(expected) + " of the upload comes next",
                  {{"expected", expected}});
  }
  // Every refusal from here on drops the upload.
  const auto drop = [&](RpcOutcome refusal) {
    upload_.reset();
    return refusal;
  };
  const auto too_large = [&](int64_t limit, const std::string& what) {
    return drop(Refuse("TOO_LARGE", what + " may have " + std::to_string(limit) + " bytes", {{"limit", limit}}));
  };
  if (count > kMaxParts) return too_large(static_cast<int64_t>(kMaxMp3Bytes), "The file");
  const std::string& text = data.get_ref<const std::string&>();
  if (text.size() > kMaxPartText) return too_large(kMaxPartBytes, "A part");
  const std::optional<std::string> bytes = DecodeBase64(text);
  if (!bytes) return drop(Invalid("data is no base64 with padding"));
  if (static_cast<int64_t>(bytes->size()) > kMaxPartBytes) return too_large(kMaxPartBytes, "A part");
  const size_t total = (upload_ ? upload_->bytes.size() : 0) + bytes->size();
  if (total > kMaxMp3Bytes) return too_large(static_cast<int64_t>(kMaxMp3Bytes), "The file");
  const std::string no_space =
      NoSpace(StoredBytes(name), static_cast<int64_t>(total),
              config_.free_bytes ? config_.free_bytes(config_.dir) : -1, config_.reserve_bytes);
  if (!no_space.empty()) return drop(Refuse("NO_SPACE", no_space));

  if (!upload_) upload_ = Upload{name, hash, count, 0, "", now};
  upload_->bytes += *bytes;
  upload_->next = index + 1;
  upload_->last = now;
  if (!upload_->Checked()) return Ok(Json{{"received", index}});
  RpcOutcome later = Ok(Json());
  later.job = UploadJob{name,        hash,          std::move(upload_->bytes), UploadDir(),
                        StoredBytes(name), config_.reserve_bytes, config_.free_bytes};
  return later;
}

RpcOutcome SoundService::Delete(const Json& params) {
  const Json situation = params.is_object() ? params.value("situation", Json()) : Json();
  if (!situation.is_string() || !IsSituation(situation.get<std::string>())) {
    return Invalid("params are {\"situation\": a name of sound.list}");
  }
  if (hold_until_ > 0) return RefuseInHold("Sounds are deleted");
  const std::string name = situation.get<std::string>();
  if (upload_ && upload_->situation == name && upload_->Checked()) {
    return Refuse("BUSY", "The upload for " + name + " is being checked");
  }
  const auto found = uploads_.find(name);
  if (found != uploads_.end()) {
    const auto saved = uploads_;
    const std::string sha256 = found->second.sha256;
    uploads_.erase(found);
    const std::string error = Save();
    if (!error.empty()) {
      uploads_ = saved;
      return Internal(error);
    }
    RemoveUpload(name, sha256);
  }
  const SoundFile* file = Active(name);
  return Ok(Json{{"situation", name}, {"source", file ? Json(file->source) : Json()}, {"file", ListedFile(file)}});
}

RpcOutcome SoundService::RefuseInHold(const std::string& what) const {
  return {false, Json(), mowbite_bridge::kHold, "An update holds the mower. " + what + " after it",
          Json{{"reason", "HOLD"}, {"until", hold_until_}}, std::nullopt};
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
    const bool same = codes.count(situation) && codes.at(situation) == file.code;
    if (same && fs::is_regular_file(wav_of(situation), error)) keep(situation, file);
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
        convert(mp3_of(situation), wav_new, &error) != Converted::kOk ||
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
