#include <gtest/gtest.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_map/sha256.h"
#include "mowbite_sound/sound_service.h"
#include "mowbite_sound/wav.h"
#include "temp_dir.h"

using namespace mowbite_sound;
namespace fs = std::filesystem;

namespace {

struct FakeSink : Sink {
  void Play(const Cue& cue) override {
    calls.push_back("play " + cue.situation);
    cues.push_back(cue);
  }
  void Stop() override { calls.push_back("stop"); }
  std::vector<std::string> calls;
  std::vector<Cue> cues;
};

constexpr char kSha[] = "0000000000000000000000000000000000000000000000000000000000000000";

SoundFile Vendor(const std::string& code) {
  return {"vendor", code, 1000, kSha, 1.5};
}

// Every situation with a file.
std::map<std::string, SoundFile> AllFiles() {
  std::map<std::string, SoundFile> files;
  for (const auto& situation : mowbite_api::SoundSituations()) files[situation.name] = Vendor("9");
  return files;
}

struct Fixture {
  explicit Fixture(int volume = 60) {
    config.dir = dir.path;
    config.max_volume = 0.8;
    config.volume = volume;
    service = std::make_unique<SoundService>(config, &sink);
    service->SetFiles(AllFiles());
    // The first values after the start only set the scene.
    service->OnRobotState("IDLE", false, 0, false, 0.5, 0);
    service->OnDriverEmergency(false, 0, 0);
  }
  Json Event(const std::string& type, Json details = Json::object()) {
    details["type"] = type;
    return details;
  }
  TempDir dir;
  SoundConfig config;
  FakeSink sink;
  std::unique_ptr<SoundService> service;
};

}  // namespace

TEST(SoundService, MethodsAndLiftBitsAreThoseOfTheCatalog) {
  for (const auto& method : SoundService::Methods()) EXPECT_EQ(mowbite_api::NodeOf(method), "mowbite_sound") << method;
  uint32_t lift = 0;
  for (const auto& bit : mowbite_api::EmergencyBits()) {
    if (bit.name == "LIFT" || bit.name == "LIFT_MULTIPLE") lift |= 1u << bit.bit;
  }
  EXPECT_EQ(kLiftBits, lift);
}

TEST(SoundService, RefusalsCarryTheRatingOfTheCatalog) {
  Fixture f;
  f.service->SetFiles({});
  const auto no_file = f.service->Call("sound.play", {{"situation", "MOWING_START"}}, 0);
  ASSERT_TRUE(no_file && !no_file->ok);
  EXPECT_EQ(no_file->code, mowbite_bridge::kInvalidRequest);
  EXPECT_EQ(no_file->data["event"], "SOUND_REFUSED");
  EXPECT_EQ(no_file->data["reason"], "NO_FILE");
  for (const char* code : {"NO_FILE", "EMERGENCY"}) {
    const auto rating = mowbite_api::RateReason("SOUND_REFUSED", code);
    EXPECT_STREQ(rating.severity, "info") << code;
    EXPECT_STREQ(rating.resume, "user") << code;
  }
  const auto* group = mowbite_api::FindReasonGroup("SOUND_REFUSED");
  ASSERT_NE(group, nullptr);
  for (const auto& field : group->fields) {
    if (field.required) {
      EXPECT_TRUE(no_file->data.contains(field.name)) << field.name;
    }
  }
}

TEST(SoundService, EventsAndStatesBecomeSituations) {
  Fixture f;
  f.service->OnEvent(f.Event("STATE", {{"state", "UNDOCKING"}}), 1);
  f.service->OnEvent(f.Event("STATE", {{"state", "MOWING"}}), 2);
  f.service->OnEvent(f.Event("STATE", {{"state", "PAUSED"}}), 3);
  // Back from a pause is no start.
  f.service->OnEvent(f.Event("STATE", {{"state", "MOWING"}}), 4);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 5);
  f.service->OnEvent(f.Event("STATE", {{"state", "DOCKING"}}), 6);
  f.service->OnEvent(f.Event("DOCKING", {{"reason", "RAIN"}}), 7);
  f.service->OnEvent(f.Event("DOCKING", {{"reason", "BATTERY_CRITICAL"}}), 8);
  f.service->OnEvent(f.Event("DOCKING", {{"reason", "MANUAL"}}), 9);
  f.service->OnEvent(f.Event("DOCKING_FAILED", {{"reason", "dock_failed"}}), 10);
  f.service->OnEvent(f.Event("STATE", {{"state", "AREA_RECORDING"}}), 11);
  f.service->OnEvent(f.Event("BLADES", {{"enabled", true}}), 12);
  f.service->OnEvent(Json("not an object"), 13);
  f.service->OnEvent(f.Event("SCHEDULE_START", {{"seconds", 10}}), 14);
  f.service->OnEvent(f.Event("SCHEDULE_START_CANCELED", {{"source", "rpc"}}), 15);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"play MOWING_START", "play MOWING_PAUSED", "play MOWING_COMPLETE",
                                                     "play DOCKING_START", "play RAIN_DETECTED", "play LOW_BATTERY",
                                                     "play DOCKING_FAILED", "play AREA_RECORDING_MODE",
                                                     "play SCHEDULE_START"}));
}

TEST(SoundService, ASituationPlaysAtMostOnceInMinGap) {
  Fixture f;
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 100);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 109);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 110.5);
  f.service->OnEvent(f.Event("BATTERY_LOW"), 111);
  EXPECT_EQ(f.sink.calls,
            (std::vector<std::string>{"play MOWING_COMPLETE", "play MOWING_COMPLETE", "play LOW_BATTERY"}));
}

TEST(SoundService, ChargingStartsAndIsDoneOnChangesOnly) {
  Fixture f;
  f.service->OnRobotState("IDLE", false, 0, true, 0.5, 1);
  f.service->OnRobotState("IDLE", false, 0, true, 0.98, 2);
  f.service->OnRobotState("IDLE", false, 0, true, 0.99, 3);
  f.service->OnRobotState("IDLE", false, 0, true, 1.0, 4);
  // Off the contacts at full charge and back on: a start, no second done.
  f.service->OnRobotState("IDLE", false, 0, false, 1.0, 20);
  f.service->OnRobotState("IDLE", false, 0, true, 1.0, 30);
  EXPECT_EQ(f.sink.calls,
            (std::vector<std::string>{"play CHARGING_START", "play CHARGING_DONE", "play CHARGING_START"}));
}

TEST(SoundService, NothingForTheFirstValuesAfterTheStart) {
  TempDir dir;
  FakeSink sink;
  SoundService service({dir.path, 1.0, 60, 10.0}, &sink);
  service.SetFiles(AllFiles());
  // Booted on the dock, in an emergency held from before.
  service.OnRobotState("IDLE", true, 1, true, 1.0, 0);
  service.OnDriverEmergency(true, 1, 0);
  EXPECT_EQ(sink.calls, (std::vector<std::string>{"stop"}));
}

TEST(SoundService, TheEmergencyStopsEverySoundAndOnlyItSpeaks) {
  Fixture f;
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 1);
  f.service->OnDriverEmergency(true, 1, 2);
  // robot_state follows a second later: the same emergency.
  f.service->OnRobotState("MOWING", true, 1, false, 0.5, 3);
  f.service->OnEvent(f.Event("STATE", {{"state", "DOCKING"}}), 4);
  f.service->OnRobotState("MOWING", true, 1, true, 0.5, 5);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"play MOWING_COMPLETE", "stop", "play EMERGENCY"}));

  const auto play = f.service->Call("sound.play", {{"situation", "MOWING_START"}}, 0);
  ASSERT_TRUE(play && !play->ok);
  EXPECT_EQ(play->data["reason"], "EMERGENCY");
  const auto stop = f.service->Call("sound.stop", Json::object(), 0);
  ASSERT_TRUE(stop && !stop->ok);
  EXPECT_EQ(stop->data["reason"], "EMERGENCY");

  // Over once both say so; then sounds play again.
  f.service->OnDriverEmergency(false, 0, 6);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 20);
  EXPECT_EQ(f.sink.calls.back(), "play EMERGENCY");
  f.service->OnRobotState("IDLE", false, 0, true, 0.5, 21);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 22);
  EXPECT_EQ(f.sink.calls.back(), "play MOWING_COMPLETE");
}

TEST(SoundService, ALiftIsAnnouncedAlsoWhenItJoinsAnEmergency) {
  Fixture f;
  f.service->OnDriverEmergency(true, 1u << 3, 1);
  f.service->OnDriverEmergency(false, 0, 30);
  f.service->OnDriverEmergency(true, 1, 31);
  f.service->OnDriverEmergency(true, 1 | (1u << 4), 32);
  f.service->OnDriverEmergency(true, 1 | (1u << 4), 33);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"stop", "play LIFT_TRIGGERED", "stop", "play EMERGENCY", "stop",
                                                     "play LIFT_TRIGGERED"}));
}

TEST(SoundService, AnAlarmPlaysEveryTimeAndAlwaysWithASound) {
  Fixture f;
  f.service->OnDriverEmergency(true, 1, 1);
  f.service->OnDriverEmergency(false, 0, 2);
  // Within min_gap of the last one.
  f.service->OnDriverEmergency(true, 1, 8);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"stop", "play EMERGENCY", "stop", "play EMERGENCY"}));
  ASSERT_EQ(f.sink.cues.size(), 2u);
  EXPECT_TRUE(f.sink.cues[1].alarm);
  // Without a file of the lift it sounds as the emergency, without either as the tone of the player.
  auto files = AllFiles();
  files.erase("LIFT_TRIGGERED");
  f.service->SetFiles(files);
  f.service->OnDriverEmergency(false, 0, 9);
  f.service->OnDriverEmergency(true, 1u << 3, 10);
  EXPECT_EQ(f.sink.cues.back().situation, "LIFT_TRIGGERED");
  EXPECT_EQ(f.sink.cues.back().path, f.dir.path + "/EMERGENCY.wav");
  f.service->SetFiles({});
  f.service->OnDriverEmergency(false, 0, 11);
  f.service->OnDriverEmergency(true, 1, 12);
  EXPECT_EQ(f.sink.cues.back().situation, "EMERGENCY");
  EXPECT_EQ(f.sink.cues.back().path, "");
  EXPECT_TRUE(f.sink.cues.back().alarm);
}

TEST(SoundService, VolumeZeroIsSilentAlsoInAnEmergency) {
  Fixture f(0);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 1);
  f.service->OnDriverEmergency(true, 1, 2);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"stop"}));
}

TEST(SoundService, GainGrowsWithTheSquareOfTheVolumeUpToMaxVolume) {
  Fixture f(50);
  f.service->OnEvent(f.Event("JOB_COMPLETE"), 1);
  ASSERT_EQ(f.sink.cues.size(), 1u);
  EXPECT_DOUBLE_EQ(f.sink.cues[0].gain, 0.8 * 0.25);
  EXPECT_EQ(f.sink.cues[0].path, f.dir.path + "/MOWING_COMPLETE.wav");
  ASSERT_TRUE(f.service->Call("sound.volume", {{"volume", 100}}, 0)->ok);
  f.service->Call("sound.play", {{"situation", "MOWING_COMPLETE"}}, 0);
  EXPECT_DOUBLE_EQ(f.sink.cues.back().gain, 0.8);
}

TEST(SoundService, VolumeIsKeptAndRefusedOnlyToChangeInAHold) {
  Fixture f;
  ASSERT_TRUE(f.service->Call("sound.volume", {{"volume", 35}}, 0)->ok);
  SoundService again(f.config, &f.sink);
  EXPECT_EQ(again.Load(), "");
  EXPECT_EQ(again.volume(), 35);
  EXPECT_EQ(again.files().size(), mowbite_api::SoundSituations().size());

  f.service->SetHold(1800000000);
  const auto refused = f.service->Call("sound.volume", {{"volume", 50}}, 0);
  ASSERT_TRUE(refused && !refused->ok);
  EXPECT_EQ(refused->code, mowbite_bridge::kHold);
  EXPECT_EQ(refused->data, (Json{{"reason", "HOLD"}, {"until", 1800000000}}));
  const auto read = f.service->Call("sound.volume", Json::object(), 0);
  ASSERT_TRUE(read && read->ok);
  EXPECT_EQ(read->result["volume"], 35);
  for (const Json& bad : {Json{{"volume", 101}}, Json{{"volume", 2.5}}, Json{{"volume", "50"}}, Json("x")}) {
    EXPECT_EQ(f.service->Call("sound.volume", bad, 0)->code, mowbite_bridge::kInvalidParams) << bad;
  }
}

TEST(SoundService, ListShowsEverySituationAndPlayAndStopReachTheSink) {
  Fixture f;
  f.service->SetFiles({{"MOWING_START", Vendor("900001")}});
  const auto list = f.service->Call("sound.list", Json::object(), 0);
  ASSERT_TRUE(list && list->ok);
  const Json& situations = list->result["situations"];
  ASSERT_EQ(situations.size(), 13u);
  EXPECT_EQ(situations[0]["name"], "MOWING_START");
  EXPECT_EQ(situations[0]["file"],
            (Json{{"source", "vendor"}, {"size", 1000}, {"sha256", kSha}, {"seconds", 1.5}}));
  EXPECT_TRUE(situations[1]["file"].is_null());
  EXPECT_EQ(list->result["volume"], 60);

  const auto play = f.service->Call("sound.play", {{"situation", "MOWING_START"}}, 0);
  ASSERT_TRUE(play && play->ok);
  EXPECT_EQ(play->result, (Json{{"situation", "MOWING_START"}, {"seconds", 1.5}}));
  EXPECT_TRUE(f.service->Call("sound.stop", Json::object(), 0)->ok);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"play MOWING_START", "stop"}));
  EXPECT_EQ(f.service->Call("sound.play", {{"situation", "NOPE"}}, 0)->code, mowbite_bridge::kInvalidParams);
  EXPECT_FALSE(f.service->Call("light.preview", Json::object(), 0));
}

TEST(SoundService, LoadRefusesABrokenStore) {
  TempDir dir;
  FakeSink sink;
  SoundService service({dir.path, 1.0, 60, 10.0}, &sink);
  std::ofstream(dir.path + "/sounds.json") << R"({"schema":1,"volume":60,"situations":{"NOPE":{}}})";
  EXPECT_NE(service.Load(), "");
  EXPECT_TRUE(service.files().empty());
  EXPECT_EQ(service.volume(), 60);
}

namespace {

// A vendor folder with <code>.mp3 files; the fake converter writes a WAV of seconds[mp3 content].
struct ImportFixture {
  ImportFixture() {
    fs::create_directories(vendor + "/upgrade/v1.3.9/sound_files");
    fs::create_directories(vendor + "/upgrade/v1.3.32/sound_files");
    fs::create_directories(store);
  }
  void Mp3(const std::string& folder, const std::string& code, const std::string& content) {
    std::ofstream(vendor + "/upgrade/" + folder + "/sound_files/" + code + ".mp3") << content;
  }
  Convert Converter() {
    return [this](const std::string& mp3, const std::string& wav, std::string* error) {
      ++conversions;
      const std::string content = ReadAll(mp3);
      if (content == "broken") {
        *error = "no MP3 frames";
        return Converted::kNotMp3;
      }
      const double seconds = content == "long" ? 31 : content == "short" ? 0.1 : 1;
      std::ofstream(wav, std::ios::binary) << EncodeWav({8000, std::vector<int16_t>(seconds * 8000, 1)});
      return Converted::kOk;
    };
  }
  std::vector<std::string> Dirs() { return VendorDirs({vendor + "/upgrade/*/sound_files", vendor + "/none"}); }
  TempDir dir;
  std::string vendor = dir.path + "/vendor";
  std::string store = dir.path + "/sounds";
  int conversions = 0;
  std::vector<std::string> notes;
};

}  // namespace

TEST(ImportVendor, TheNewestVersionComesFirst) {
  ImportFixture f;
  EXPECT_EQ(f.Dirs(), (std::vector<std::string>{f.vendor + "/upgrade/v1.3.32/sound_files",
                                                 f.vendor + "/upgrade/v1.3.9/sound_files"}));
}

TEST(ImportVendor, ConvertsWhatIsMissingAndKeepsWhatIsThere) {
  ImportFixture f;
  f.Mp3("v1.3.9", "900001", "old");
  f.Mp3("v1.3.32", "900001", "new");
  f.Mp3("v1.3.9", "900102", "only old");
  f.Mp3("v1.3.32", "900010", "broken");
  f.Mp3("v1.3.32", "900017", "long");
  f.Mp3("v1.3.32", "900018", "short");
  const std::map<std::string, std::string> codes = {
      {"MOWING_START", "900001"},   {"MOWING_COMPLETE", "900102"}, {"MOWING_PAUSED", "900010"},
      {"EMERGENCY", "900017"},      {"LIFT_TRIGGERED", "900018"},  {"DOCKING_FAILED", "900025"},
      {"NOT_A_SITUATION", "900003"}};
  auto files = ImportVendor(f.store, {}, codes, f.Dirs(), f.Converter(), &f.notes);
  ASSERT_EQ(files.size(), 2u);
  EXPECT_EQ(files["MOWING_START"].code, "900001");
  EXPECT_EQ(files["MOWING_START"].size, 3);
  EXPECT_EQ(ReadAll(f.store + "/MOWING_START.mp3"), "new");
  EXPECT_EQ(files["MOWING_START"].sha256, "11507a0e2f5e69d5dfa40a62a1bd7b6ee57e6bcd85c67c9b8431b36fff21c437");
  EXPECT_DOUBLE_EQ(files["MOWING_START"].seconds, 1.0);
  EXPECT_EQ(files["MOWING_COMPLETE"].source, "vendor");
  EXPECT_EQ(f.notes.size(), 5u);
  EXPECT_EQ(f.conversions, 5);
  for (const char* gone : {"MOWING_PAUSED", "EMERGENCY", "LIFT_TRIGGERED"}) {
    EXPECT_FALSE(fs::exists(f.store + "/" + gone + ".mp3")) << gone;
    EXPECT_FALSE(fs::exists(f.store + "/" + gone + ".wav")) << gone;
    EXPECT_FALSE(fs::exists(f.store + "/" + gone + ".wav.new")) << gone;
  }

  // Again: nothing to convert; a new code for one situation, a situation left out goes, uploads are not touched.
  fs::create_directories(f.store + "/upload");
  std::ofstream(f.store + "/upload/MOWING_COMPLETE-0000000000000000.wav") << "x";
  f.conversions = 0;
  files = ImportVendor(f.store, files, {{"MOWING_START", "900102"}}, f.Dirs(), f.Converter(), &f.notes);
  EXPECT_EQ(f.conversions, 1);
  EXPECT_EQ(files["MOWING_START"].code, "900102");
  EXPECT_FALSE(files.count("MOWING_COMPLETE"));
  EXPECT_FALSE(fs::exists(f.store + "/MOWING_COMPLETE.wav"));
  EXPECT_TRUE(fs::exists(f.store + "/upload/MOWING_COMPLETE-0000000000000000.wav"));
}

namespace {

std::string Base64(const std::string& bytes) {
  static const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < bytes.size(); i += 3) {
    const uint32_t n = (static_cast<uint8_t>(bytes[i]) << 16) | (static_cast<uint8_t>(bytes[i + 1]) << 8) |
                       static_cast<uint8_t>(bytes[i + 2]);
    for (const int shift : {18, 12, 6, 0}) out.push_back(kAlphabet[(n >> shift) & 63]);
  }
  if (i + 1 == bytes.size()) {
    const uint32_t n = static_cast<uint8_t>(bytes[i]) << 16;
    out += std::string{kAlphabet[(n >> 18) & 63], kAlphabet[(n >> 12) & 63]} + "==";
  } else if (i + 2 == bytes.size()) {
    const uint32_t n = (static_cast<uint8_t>(bytes[i]) << 16) | (static_cast<uint8_t>(bytes[i + 1]) << 8);
    out += std::string{kAlphabet[(n >> 18) & 63], kAlphabet[(n >> 12) & 63], kAlphabet[(n >> 6) & 63]} + "=";
  }
  return out;
}

// "sound:<seconds>" and any filler: an MP3 to the fake converter; "noise", "long" and "fail" are not.
std::string Sound(double seconds, size_t size = 100) {
  std::string file = "sound:" + std::to_string(seconds) + ";";
  file.resize(std::max(size, file.size()), 'x');
  return file;
}

std::string Sha(const std::string& bytes) {
  return mowbite_map::Sha256Hex(bytes);
}

struct UploadFixture : Fixture {
  UploadFixture() {
    config.reserve_bytes = 1000;
    config.free_bytes = [this](const std::string&) { return free; };
    service = std::make_unique<SoundService>(config, &sink);
    service->SetFiles({{"MOWING_START", Vendor("900001")}});
    std::ofstream(dir.path + "/MOWING_START.wav") << "vendor";
    service->OnRobotState("IDLE", false, 0, false, 0.5, 0);
    service->OnDriverEmergency(false, 0, 0);
  }
  std::optional<RpcOutcome> Part(const std::string& situation, int part, int parts, const std::string& bytes,
                                 const std::string& sha256, double now = 0) {
    return service->Call("sound.upload",
                         {{"situation", situation},
                          {"part", part},
                          {"parts", parts},
                          {"data", Base64(bytes)},
                          {"sha256", sha256}},
                         now);
  }
  // All parts of file, then the answer to the last one; the first refusal of a part instead.
  RpcOutcome Upload(const std::string& situation, const std::string& file, const std::string& sha256 = "") {
    const size_t size = static_cast<size_t>(kMaxPartBytes);
    const int parts = static_cast<int>(std::max<size_t>(1, (file.size() + size - 1) / size));
    const std::string sha = sha256.empty() ? Sha(file) : sha256;
    for (int part = 0; part < parts; ++part) {
      const auto outcome = Part(situation, part, parts, file.substr(part * size, size), sha);
      if (!outcome->ok) return *outcome;
      if (outcome->job) return Finish(*outcome->job);
      EXPECT_EQ(outcome->result, (Json{{"received", part}}));
    }
    ADD_FAILURE() << "the last part brought no job";
    return {};
  }
  RpcOutcome Finish(const UploadJob& job) { return service->CompleteUpload(FinishUpload(job, Converter())); }
  Convert Converter() {
    return [this](const std::string& mp3, const std::string& wav, std::string* error) {
      ++conversions;
      const std::string content = ReadAll(mp3);
      if (content.rfind("noise", 0) == 0) return Converted::kNotMp3;
      if (content.rfind("long", 0) == 0) return Converted::kTooLong;
      if (content.rfind("fail", 0) == 0) {
        *error = "it broke";
        return Converted::kFailed;
      }
      // As the converter: longer than kMaxSeconds is refused.
      const double seconds = std::stod(content.substr(6));
      if (seconds > kMaxSeconds) return Converted::kTooLong;
      std::ofstream(wav, std::ios::binary) << EncodeWav({8000, std::vector<int16_t>(seconds * 8000, 1)});
      return Converted::kOk;
    };
  }
  Json List(const std::string& situation) {
    const auto list = service->Call("sound.list", Json::object(), 0);
    for (const auto& row : list->result["situations"]) {
      if (row["name"] == situation) return row["file"];
    }
    return "missing";
  }
  std::string UploadFile(const std::string& situation, const std::string& file, const char* extension) {
    return UploadPath(dir.path + "/upload", situation, Sha(file), extension);
  }
  // Files in the upload folder.
  size_t Uploaded() {
    size_t count = 0;
    std::error_code error;
    for (auto it = fs::directory_iterator(dir.path + "/upload", error); it != fs::directory_iterator(); ++it) ++count;
    return count;
  }
  int64_t free = int64_t{1} << 40;
  int conversions = 0;
};

}  // namespace

TEST(SoundUpload, PlaysInsteadOfTheVendorsSoundUntilDeleted) {
  UploadFixture f;
  const std::string file = Sound(2, 1200 * 1024);
  const auto first = f.Part("MOWING_START", 0, 3, file.substr(0, kMaxPartBytes), Sha(file));
  ASSERT_TRUE(first && first->ok);
  // Nothing changes before the last part is checked.
  EXPECT_EQ(f.List("MOWING_START")["source"], "vendor");
  ASSERT_TRUE(f.Part("MOWING_START", 1, 3, file.substr(kMaxPartBytes, kMaxPartBytes), Sha(file))->ok);
  const auto last = f.Part("MOWING_START", 2, 3, file.substr(2 * kMaxPartBytes), Sha(file));
  ASSERT_TRUE(last && last->ok && last->job);
  EXPECT_EQ(f.List("MOWING_START")["source"], "vendor");
  const auto done = f.Finish(*last->job);
  ASSERT_TRUE(done.ok) << done.message;
  EXPECT_EQ(done.result, (Json{{"situation", "MOWING_START"}, {"source", "upload"}, {"duration_s", 2.0},
                               {"sha256", Sha(file)}}));
  EXPECT_EQ(f.List("MOWING_START"),
            (Json{{"source", "upload"}, {"size", file.size()}, {"sha256", Sha(file)}, {"seconds", 2.0}}));
  EXPECT_EQ(ReadAll(f.UploadFile("MOWING_START", file, ".mp3")), file);
  EXPECT_EQ(f.Uploaded(), 2u);

  ASSERT_TRUE(f.service->Call("sound.play", {{"situation", "MOWING_START"}}, 0)->ok);
  EXPECT_EQ(f.sink.cues.back().path, f.UploadFile("MOWING_START", file, ".wav"));
  f.service->OnEvent(f.Event("STATE", {{"state", "MOWING"}}), 1);
  EXPECT_EQ(f.sink.cues.back().path, f.UploadFile("MOWING_START", file, ".wav"));

  // Kept over a restart; what sounds.json does not name goes.
  std::ofstream(f.dir.path + "/upload/MOWING_START-0123456789abcdef.mp3.new") << "half";
  std::ofstream(f.dir.path + "/upload/MOWING_START-ffffffffffffffff.wav") << "old";
  SoundService again(f.config, &f.sink);
  EXPECT_EQ(again.Load(), "");
  EXPECT_EQ(again.uploads().at("MOWING_START").sha256, Sha(file));
  EXPECT_EQ(again.files().count("MOWING_START"), 1u);
  EXPECT_EQ(f.Uploaded(), 2u);

  const auto deleted = f.service->Call("sound.delete", {{"situation", "MOWING_START"}}, 0);
  ASSERT_TRUE(deleted && deleted->ok);
  const Json vendor = {{"source", "vendor"}, {"size", 1000}, {"sha256", kSha}, {"seconds", 1.5}};
  EXPECT_EQ(deleted->result, (Json{{"situation", "MOWING_START"}, {"source", "vendor"}, {"file", vendor}}));
  EXPECT_EQ(f.Uploaded(), 0u);
  f.service->Call("sound.play", {{"situation", "MOWING_START"}}, 0);
  EXPECT_EQ(f.sink.cues.back().path, f.dir.path + "/MOWING_START.wav");
  // Nothing to delete is no error.
  EXPECT_EQ(f.service->Call("sound.delete", {{"situation", "MOWING_START"}}, 0)->result, deleted->result);
}

TEST(SoundUpload, WithoutAVendorsSoundDeleteLeavesNone) {
  UploadFixture f;
  ASSERT_TRUE(f.Upload("RAIN_DETECTED", Sound(1)).ok);
  EXPECT_EQ(f.List("RAIN_DETECTED")["source"], "upload");
  const auto deleted = f.service->Call("sound.delete", {{"situation", "RAIN_DETECTED"}}, 0);
  ASSERT_TRUE(deleted && deleted->ok);
  EXPECT_EQ(deleted->result, (Json{{"situation", "RAIN_DETECTED"}, {"source", nullptr}, {"file", nullptr}}));
  EXPECT_EQ(f.service->Call("sound.play", {{"situation", "RAIN_DETECTED"}}, 0)->data["reason"], "NO_FILE");
}

TEST(SoundUpload, ARefusedFileLeavesTheSoundBeforeAlone) {
  UploadFixture f;
  const std::string good = Sound(3);
  ASSERT_TRUE(f.Upload("MOWING_START", good).ok);
  const auto expect_refused = [&](const RpcOutcome& outcome, const char* reason) {
    EXPECT_FALSE(outcome.ok) << reason;
    EXPECT_EQ(outcome.code, mowbite_bridge::kInvalidRequest) << reason;
    EXPECT_EQ(outcome.data["event"], "SOUND_REFUSED") << reason;
    EXPECT_EQ(outcome.data["reason"], reason);
    EXPECT_EQ(f.List("MOWING_START")["sha256"], Sha(good)) << reason;
    EXPECT_EQ(f.Uploaded(), 2u) << reason;
  };
  expect_refused(f.Upload("MOWING_START", "noise"), "NOT_MP3");
  const auto long_one = f.Upload("MOWING_START", "long");
  expect_refused(long_one, "TOO_LONG");
  EXPECT_EQ(long_one.data["limit"], kMaxSeconds);
  expect_refused(f.Upload("MOWING_START", Sound(31)), "TOO_LONG");
  expect_refused(f.Upload("MOWING_START", Sound(1), Sha("another file")), "CHECKSUM");
  const auto failed = f.Upload("MOWING_START", "fail");
  EXPECT_EQ(failed.code, mowbite_bridge::kInternalError);
  EXPECT_NE(failed.message.find("it broke"), std::string::npos) << failed.message;
  EXPECT_EQ(f.Uploaded(), 2u);
  // After each one the next upload starts.
  ASSERT_TRUE(f.Upload("MOWING_START", Sound(4)).ok);
  EXPECT_EQ(f.List("MOWING_START")["seconds"], 4.0);
  EXPECT_EQ(f.Uploaded(), 2u);
}

TEST(SoundUpload, PartsComeInOrderAndOneUploadAtATime) {
  UploadFixture f;
  const std::string file = Sound(1, 1100 * 1024);
  const std::string sha = Sha(file);
  const auto piece = [&](int part) { return file.substr(part * kMaxPartBytes, kMaxPartBytes); };
  const auto reason = [](const std::optional<RpcOutcome>& outcome) {
    return outcome->data.is_object() ? outcome->data.value("reason", "") : "";
  };

  EXPECT_EQ(reason(f.Part("MOWING_START", 1, 3, piece(1), sha)), "PART_ORDER");
  ASSERT_TRUE(f.Part("MOWING_START", 0, 3, piece(0), sha)->ok);
  const auto skipped = f.Part("MOWING_START", 2, 3, piece(2), sha);
  EXPECT_EQ(reason(skipped), "PART_ORDER");
  EXPECT_EQ(skipped->data["expected"], 1);
  EXPECT_EQ(reason(f.Part("MOWING_START", 1, 4, piece(1), sha)), "PART_ORDER");
  EXPECT_EQ(reason(f.Part("MOWING_START", 0, 3, piece(0), sha)).size(), 0u);

  // Another situation or another file waits.
  EXPECT_EQ(reason(f.Part("RAIN_DETECTED", 0, 1, Sound(1), Sha(Sound(1)))), "BUSY");
  EXPECT_EQ(reason(f.Part("MOWING_START", 0, 1, Sound(1), Sha(Sound(1)))), "BUSY");
  EXPECT_FALSE(f.Part("RAIN_DETECTED", 0, 1, Sound(1), Sha(Sound(1)))->data.contains("state"));

  ASSERT_TRUE(f.Part("MOWING_START", 1, 3, piece(1), sha)->ok);
  const auto last = f.Part("MOWING_START", 2, 3, piece(2), sha);
  ASSERT_TRUE(last->job);
  // Being checked: every upload waits, and so does deleting this situation.
  EXPECT_EQ(reason(f.Part("MOWING_START", 0, 3, piece(0), sha)), "BUSY");
  EXPECT_EQ(reason(f.service->Call("sound.delete", {{"situation", "MOWING_START"}}, 0)), "BUSY");
  EXPECT_TRUE(f.service->Call("sound.delete", {{"situation", "RAIN_DETECTED"}}, 0)->ok);
  ASSERT_TRUE(f.Finish(*last->job).ok);
  EXPECT_EQ(ReadAll(f.UploadFile("MOWING_START", file, ".mp3")), file);
  EXPECT_TRUE(f.Upload("RAIN_DETECTED", Sound(1)).ok);
}

TEST(SoundUpload, AnUnfinishedUploadIsDropped60sAfterItsLastPart) {
  UploadFixture f;
  const std::string file = Sound(1, 1100 * 1024);
  ASSERT_TRUE(f.Part("MOWING_START", 0, 3, file.substr(0, kMaxPartBytes), Sha(file), 0)->ok);
  ASSERT_TRUE(f.Part("MOWING_START", 1, 3, file.substr(kMaxPartBytes, kMaxPartBytes), Sha(file), 50)->ok);
  EXPECT_EQ(f.Part("RAIN_DETECTED", 0, 1, Sound(1), Sha(Sound(1)), 50 + kUploadExpiry)->data["reason"], "BUSY");
  const auto rain = f.Part("RAIN_DETECTED", 0, 1, Sound(1), Sha(Sound(1)), 50.1 + kUploadExpiry);
  ASSERT_TRUE(rain->job);
  ASSERT_TRUE(f.Finish(*rain->job).ok);
  const auto late = f.Part("MOWING_START", 2, 3, file.substr(2 * kMaxPartBytes), Sha(file), 120);
  EXPECT_EQ(late->data["reason"], "PART_ORDER");
  EXPECT_EQ(late->data["expected"], 0);
}

TEST(SoundUpload, SizeAndSpaceHaveLimits) {
  UploadFixture f;
  const std::string sha = Sha(Sound(1));
  const auto too_many = f.Part("MOWING_START", 0, 5, Sound(1), sha);
  EXPECT_EQ(too_many->data["reason"], "TOO_LARGE");
  EXPECT_EQ(too_many->data["limit"], 2 << 20);
  const auto too_big = f.Part("MOWING_START", 0, 2, Sound(1, kMaxPartBytes + 1), sha);
  EXPECT_EQ(too_big->data["reason"], "TOO_LARGE");
  EXPECT_EQ(too_big->data["limit"], kMaxPartBytes);

  // The journal's reserve stays free; a refused upload is dropped.
  f.free = f.config.reserve_bytes + 50;
  const auto no_room = f.Part("MOWING_START", 0, 2, Sound(1), sha);
  EXPECT_EQ(no_room->data["reason"], "NO_SPACE");
  EXPECT_EQ(no_room->data["severity"], "warning");
  EXPECT_EQ(f.Part("MOWING_START", 1, 2, Sound(1), sha)->data["reason"], "PART_ORDER");
  f.free = int64_t{1} << 40;
  // All sounds together at most kMaxStoreBytes (a sparse file takes no disk).
  fs::resize_file(f.dir.path + "/MOWING_START.wav", kMaxStoreBytes - 50);
  EXPECT_EQ(f.Upload("RAIN_DETECTED", Sound(1)).data["reason"], "NO_SPACE");
  fs::resize_file(f.dir.path + "/MOWING_START.wav", kMaxStoreBytes - 1000);
  EXPECT_EQ(f.Upload("RAIN_DETECTED", Sound(1)).data["reason"], "NO_SPACE");
  EXPECT_EQ(f.Uploaded(), 0u);
}

TEST(SoundUpload, AHoldRefusesUploadAndDeleteAnEmergencyDoesNot) {
  UploadFixture f;
  f.service->OnDriverEmergency(true, 1, 1);
  ASSERT_TRUE(f.Upload("MOWING_START", Sound(1)).ok);
  EXPECT_EQ(f.service->Call("sound.play", {{"situation", "MOWING_START"}}, 0)->data["reason"], "EMERGENCY");
  f.service->SetHold(1800000000);
  for (const auto& outcome : {f.Part("MOWING_START", 0, 1, Sound(2), Sha(Sound(2))),
                              f.service->Call("sound.delete", {{"situation", "MOWING_START"}}, 0)}) {
    EXPECT_EQ(outcome->code, mowbite_bridge::kHold);
    EXPECT_EQ(outcome->data, (Json{{"reason", "HOLD"}, {"until", 1800000000}}));
  }
  EXPECT_EQ(f.List("MOWING_START")["seconds"], 1.0);
}

TEST(SoundUpload, InvalidParamsAreRefusedWithInvalidParams) {
  UploadFixture f;
  const Json good = {{"situation", "MOWING_START"}, {"part", 0}, {"parts", 1}, {"data", "AAAA"}, {"sha256", kSha}};
  const auto taken = f.service->Call("sound.upload", good, 0);
  ASSERT_TRUE(taken->job);
  f.Finish(*taken->job);
  const auto with = [&](const char* key, Json value) {
    Json params = good;
    if (value.is_null()) {
      params.erase(key);
    } else {
      params[key] = value;
    }
    return params;
  };
  for (const Json& bad : {with("situation", "NOPE"), with("situation", nullptr), with("part", -1), with("part", 1),
                          with("part", 0.5), with("parts", 0), with("data", "AAA"), with("data", "A=AA"),
                          with("data", "AA=A"), with("data", "AA*A"), with("data", 5),
                          with("sha256", std::string(kSha).replace(0, 1, "A")), with("sha256", "00"), Json("x")}) {
    EXPECT_EQ(f.service->Call("sound.upload", bad, 0)->code, mowbite_bridge::kInvalidParams) << bad;
  }
  EXPECT_EQ(f.service->Call("sound.delete", {{"situation", "NOPE"}}, 0)->code, mowbite_bridge::kInvalidParams);
  EXPECT_EQ(f.service->Call("sound.delete", Json::object(), 0)->code, mowbite_bridge::kInvalidParams);
}

TEST(SoundUpload, EveryReasonIsInTheCatalogWithTexts) {
  for (const char* code : {"NOT_MP3", "TOO_LARGE", "TOO_LONG", "CHECKSUM", "PART_ORDER", "BUSY", "NO_SPACE"}) {
    const auto* group = mowbite_api::FindReasonGroup("SOUND_REFUSED");
    ASSERT_NE(group, nullptr);
    const auto reason = std::find_if(group->reasons.begin(), group->reasons.end(),
                                     [&](const auto& entry) { return entry.code == code; });
    ASSERT_NE(reason, group->reasons.end()) << code;
    EXPECT_EQ(reason->texts.count("de") + reason->texts.count("en"), 2u) << code;
  }
}
