#include <gtest/gtest.h>
#include <stdlib.h>

#include <filesystem>
#include <fstream>
#include <iterator>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_sound/sound_service.h"
#include "mowbite_sound/wav.h"

using namespace mowbite_sound;
namespace fs = std::filesystem;

namespace {

struct TempDir {
  TempDir() {
    char pattern[] = "/tmp/mowbite_sound_XXXXXX";
    path = ::mkdtemp(pattern);
  }
  ~TempDir() { fs::remove_all(path); }
  std::string path;
};

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
  for (const auto& situation : Situations()) files[situation.name] = Vendor("9");
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

std::string ReadAll(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), {});
}

}  // namespace

TEST(SoundService, MethodsAndSituationsAreThoseOfTheCatalog) {
  for (const auto& method : SoundService::Methods()) EXPECT_EQ(mowbite_api::NodeOf(method), "mowbite_sound") << method;
  const auto& methods = mowbite_api::Methods();
  const auto play = std::find_if(methods.begin(), methods.end(), [](const auto& m) { return m.name == "sound.play"; });
  ASSERT_NE(play, methods.end());
  std::vector<std::string> names;
  for (const auto& situation : Situations()) names.push_back(situation.name);
  EXPECT_EQ(play->params.fields.at(0).values, names);
  EXPECT_EQ(names.size(), 12u);
}

TEST(SoundService, RefusalsCarryTheRatingOfTheCatalog) {
  Fixture f;
  f.service->SetFiles({});
  const auto no_file = f.service->Call("sound.play", {{"situation", "MOWING_START"}});
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
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"play MOWING_START", "play MOWING_PAUSED", "play MOWING_COMPLETE",
                                                     "play DOCKING_START", "play RAIN_DETECTED", "play LOW_BATTERY",
                                                     "play DOCKING_FAILED", "play AREA_RECORDING_MODE"}));
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

  const auto play = f.service->Call("sound.play", {{"situation", "MOWING_START"}});
  ASSERT_TRUE(play && !play->ok);
  EXPECT_EQ(play->data["reason"], "EMERGENCY");
  const auto stop = f.service->Call("sound.stop", Json::object());
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
  ASSERT_TRUE(f.service->Call("sound.volume", {{"volume", 100}})->ok);
  f.service->Call("sound.play", {{"situation", "MOWING_COMPLETE"}});
  EXPECT_DOUBLE_EQ(f.sink.cues.back().gain, 0.8);
}

TEST(SoundService, VolumeIsKeptAndRefusedOnlyToChangeInAHold) {
  Fixture f;
  ASSERT_TRUE(f.service->Call("sound.volume", {{"volume", 35}})->ok);
  SoundService again(f.config, &f.sink);
  EXPECT_EQ(again.Load(), "");
  EXPECT_EQ(again.volume(), 35);
  EXPECT_EQ(again.files().size(), 12u);

  f.service->SetHold(1800000000);
  const auto refused = f.service->Call("sound.volume", {{"volume", 50}});
  ASSERT_TRUE(refused && !refused->ok);
  EXPECT_EQ(refused->code, kRpcHold);
  EXPECT_EQ(refused->data, (Json{{"reason", "HOLD"}, {"until", 1800000000}}));
  const auto read = f.service->Call("sound.volume", Json::object());
  ASSERT_TRUE(read && read->ok);
  EXPECT_EQ(read->result["volume"], 35);
  for (const Json& bad : {Json{{"volume", 101}}, Json{{"volume", 2.5}}, Json{{"volume", "50"}}, Json("x")}) {
    EXPECT_EQ(f.service->Call("sound.volume", bad)->code, mowbite_bridge::kInvalidParams) << bad;
  }
}

TEST(SoundService, ListShowsEverySituationAndPlayAndStopReachTheSink) {
  Fixture f;
  f.service->SetFiles({{"MOWING_START", Vendor("900001")}});
  const auto list = f.service->Call("sound.list", Json::object());
  ASSERT_TRUE(list && list->ok);
  const Json& situations = list->result["situations"];
  ASSERT_EQ(situations.size(), 12u);
  EXPECT_EQ(situations[0]["name"], "MOWING_START");
  EXPECT_EQ(situations[0]["file"],
            (Json{{"source", "vendor"}, {"size", 1000}, {"sha256", kSha}, {"seconds", 1.5}}));
  EXPECT_TRUE(situations[1]["file"].is_null());
  EXPECT_EQ(list->result["volume"], 60);

  const auto play = f.service->Call("sound.play", {{"situation", "MOWING_START"}});
  ASSERT_TRUE(play && play->ok);
  EXPECT_EQ(play->result, (Json{{"situation", "MOWING_START"}, {"seconds", 1.5}}));
  EXPECT_TRUE(f.service->Call("sound.stop", Json::object())->ok);
  EXPECT_EQ(f.sink.calls, (std::vector<std::string>{"play MOWING_START", "stop"}));
  EXPECT_EQ(f.service->Call("sound.play", {{"situation", "NOPE"}})->code, mowbite_bridge::kInvalidParams);
  EXPECT_FALSE(f.service->Call("light.preview", Json::object()));
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
        return false;
      }
      const double seconds = content == "long" ? 31 : content == "short" ? 0.1 : 1;
      std::ofstream(wav, std::ios::binary) << EncodeWav({8000, std::vector<int16_t>(seconds * 8000, 1)});
      return true;
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

  // Again: nothing to convert; a new code for one situation, an upload stays, a situation left out goes.
  auto have = files;
  have["RAIN_DETECTED"] = {"upload", "", 10, kSha, 2};
  std::ofstream(f.store + "/RAIN_DETECTED.wav") << "x";
  f.conversions = 0;
  files = ImportVendor(f.store, have, {{"MOWING_START", "900102"}}, f.Dirs(), f.Converter(), &f.notes);
  EXPECT_EQ(f.conversions, 1);
  EXPECT_EQ(files["MOWING_START"].code, "900102");
  EXPECT_EQ(files["RAIN_DETECTED"].source, "upload");
  EXPECT_FALSE(files.count("MOWING_COMPLETE"));
  EXPECT_FALSE(fs::exists(f.store + "/MOWING_COMPLETE.wav"));
}
