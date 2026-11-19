#include <gtest/gtest.h>
#include <stdlib.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_light/light_service.h"
#include "mowbite_light/output.h"
#include "decode_ws2812.h"

using namespace mowbite_light;
namespace fs = std::filesystem;

namespace {

struct TempDir {
  TempDir() {
    char pattern[] = "/tmp/mowbite_light_XXXXXX";
    path = ::mkdtemp(pattern);
  }
  ~TempDir() { fs::remove_all(path); }
  std::string path;
};

LightConfig Config(const std::string& dir = "") {
  LightConfig config;
  config.leds = 10;
  config.max_brightness = 1.0;
  config.max_current_ma = 1e9;
  if (!dir.empty()) config.profile_path = dir + "/light_profile.json";
  return config;
}

const Json kOrange = Json::parse(R"({"effect":"solid","colors":["#ff8800"]})");
const Rgb kRed{255, 0, 0};

}  // namespace

TEST(LightService, EveryMethodIsInTheCatalogWithThisNode) {
  for (const auto& method : LightService::Methods()) EXPECT_EQ(mowbite_api::NodeOf(method), "mowbite_light") << method;
  LightService service(Config());
  EXPECT_FALSE(service.Call("map.replace", Json(), 0));
}

TEST(LightService, GetProfileTellsStatesEffectsLimitsAndTheEmergency) {
  LightService service(Config());
  const auto answer = service.Call("light.get_profile", Json::object(), 0);
  ASSERT_TRUE(answer && answer->ok);
  const Json& r = answer->result;
  EXPECT_EQ(r["states"], Json(ProfileStates()));
  EXPECT_EQ(r["effects"], Json(EffectNames()));
  EXPECT_EQ(r["profile"], r["defaults"]);
  EXPECT_EQ(r["emergency"]["effect"], "blink");
  EXPECT_EQ(r["limits"]["colors"], 4);
  EXPECT_EQ(r["leds"], 10);
}

TEST(LightService, SetProfileStoresAllOrNothingAndLoadsItAgain) {
  TempDir dir;
  {
    LightService service(Config(dir.path));
    EXPECT_EQ(service.Load(), "");
    auto answer = service.Call("light.set_profile", Json{{"profile", {{"MOWING", kOrange}}}}, 0);
    ASSERT_TRUE(answer && answer->ok) << answer->message;
    EXPECT_EQ(answer->result["profile"]["MOWING"]["colors"][0], "#ff8800");
    answer = service.Call("light.set_profile", Json{{"profile", {{"MOWING", kOrange}, {"NOPE", kOrange}}}}, 0);
    ASSERT_TRUE(answer);
    EXPECT_FALSE(answer->ok);
    EXPECT_EQ(answer->code, mowbite_bridge::kInvalidParams);
    EXPECT_FALSE(service.Call("light.set_profile", Json{{"MOWING", kOrange}}, 0)->ok);
    EXPECT_EQ(service.profile().at("MOWING").colors[0], (Rgb{255, 136, 0}));
  }
  LightService again(Config(dir.path));
  EXPECT_EQ(again.Load(), "");
  EXPECT_EQ(again.profile().at("MOWING").colors[0], (Rgb{255, 136, 0}));

  EXPECT_TRUE(again.Call("light.reset", Json::object(), 0)->ok);
  EXPECT_FALSE(fs::exists(dir.path + "/light_profile.json"));
  EXPECT_EQ(again.profile().at("MOWING").colors[0], DefaultProfile().at("MOWING").colors[0]);
}

TEST(LightService, ABrokenFileGivesTheDefaultsAndWhy) {
  TempDir dir;
  std::ofstream(dir.path + "/light_profile.json") << R"({"schema":1,"profile":{"MOWING":{"effect":"x"}}})";
  LightService service(Config(dir.path));
  EXPECT_NE(service.Load(), "");
  EXPECT_EQ(service.profile().at("MOWING").kind, DefaultProfile().at("MOWING").kind);
  std::ofstream(dir.path + "/light_profile.json") << "{";
  EXPECT_NE(service.Load(), "");
}

TEST(LightService, TheFrameRunsAlongTheStrips) {
  LightConfig config = Config();
  config.strips = {{0, 4}, {9, 5}};
  LightService service(config);
  const Json chase = Json::parse(R"({"effect":"chase","colors":["#ff0000"],"period":1})");
  ASSERT_TRUE(service.Call("light.preview", Json{{"effect", chase}}, 0)->ok);
  for (double t : {0.0, 0.25, 0.5, 0.75}) {
    const auto leds = service.Frame(t);
    for (size_t k = 0; k < 5; ++k) EXPECT_EQ(leds[k], leds[9 - k]) << t << " " << k;
  }
  EXPECT_EQ(service.Frame(0.0)[0], kRed);
  EXPECT_EQ(service.Frame(0.0)[9], kRed);
}

TEST(LightService, TheFrameFollowsTheStateAndTheEmergencyGoesOverAll) {
  LightService service(Config());
  service.Call("light.set_profile", Json{{"profile", {{"MOWING", kOrange}}}}, 0);
  service.SetRobotState("MOWING", false, false, 0.8);
  EXPECT_EQ(service.Frame(1.0)[9], (Rgb{255, 136, 0}));
  // A preview for 2 s, then the state again.
  const auto answer = service.Call(
      "light.preview", Json{{"effect", {{"effect", "solid"}, {"colors", {"#0000ff"}}}}, {"duration", 2}}, 1.0);
  ASSERT_TRUE(answer && answer->ok);
  EXPECT_EQ(service.Frame(2.9)[0], (Rgb{0, 0, 255}));
  service.SetRobotState("MOWING", true, false, 0.8);
  EXPECT_EQ(service.Frame(2.95)[0], kRed);
  service.SetRobotState("MOWING", false, false, 0.8);
  EXPECT_EQ(service.Frame(3.1)[0], (Rgb{255, 136, 0}));
  // The emergency blinks from its start: on for 0.25 s, then dark.
  service.SetRobotState("IDLE", true, false, 0.8);
  EXPECT_EQ(service.Frame(10.0)[0], kRed);
  EXPECT_EQ(service.Frame(10.3)[0], Rgb{});
  service.SetRobotState("IDLE", false, true, 0.5);
  EXPECT_EQ(service.Frame(11.0)[0], Render(DefaultProfile().at("CHARGING"), 10, 0)[0]);
}

// The driver's emergency and a robot_state that stopped coming show as the emergency, before the logic tells it.
TEST(LightService, TheDriverAndALostStateShowTheEmergency) {
  LightService service(Config());
  service.Call("light.set_profile", Json{{"profile", {{"MOWING", kOrange}}}}, 0);
  service.SetRobotState("MOWING", false, false, 0.8);
  EXPECT_EQ(service.Frame(1.0)[9], (Rgb{255, 136, 0}));
  service.SetDriverEmergency(true);
  EXPECT_EQ(service.Frame(2.0)[0], kRed);
  service.SetDriverEmergency(false);
  EXPECT_EQ(service.Frame(3.0)[9], (Rgb{255, 136, 0}));
  service.SetStateLost(true);
  EXPECT_EQ(service.Frame(4.0)[0], kRed);
  service.SetStateLost(false);
  EXPECT_EQ(service.Frame(5.0)[9], (Rgb{255, 136, 0}));
}

TEST(LightService, AnUpdateHoldKeepsTheProfile) {
  LightService service(Config());
  service.SetHold(2000000000);
  const auto set = service.Call("light.set_profile", Json{{"profile", {{"MOWING", kOrange}}}}, 0);
  ASSERT_TRUE(set && !set->ok);
  EXPECT_EQ(set->code, mowbite_bridge::kHold);
  EXPECT_EQ(set->data["reason"], "HOLD");
  const auto reset = service.Call("light.reset", Json::object(), 0);
  ASSERT_TRUE(reset && !reset->ok);
  EXPECT_EQ(reset->code, mowbite_bridge::kHold);
  // The preview writes nothing and goes on.
  const auto preview =
      service.Call("light.preview", Json{{"effect", {{"effect", "solid"}, {"colors", {"#0000ff"}}}}}, 0);
  EXPECT_TRUE(preview && preview->ok);
  service.SetHold(0);
  EXPECT_TRUE(service.Call("light.reset", Json::object(), 0)->ok);
}

TEST(LightService, TheAnnouncementOfAStartBlinksOverChargingUntilItEnds) {
  LightService service(Config());
  const Rgb yellow{255, 160, 0};
  const Json start = {{"type", "SCHEDULE_START"}, {"seconds", 10}, {"until", 1794484810}};
  service.SetRobotState("IDLE", false, true, 0.9);
  const Rgb charging = service.Frame(0.0)[0];
  // Twice a second for its seconds.
  service.OnEvent(start, 1.0);
  EXPECT_EQ(service.Frame(1.0)[0], yellow);
  EXPECT_EQ(service.Frame(1.3)[0], Rgb{});
  EXPECT_EQ(service.Frame(1.5)[0], yellow);
  EXPECT_EQ(service.Frame(10.7)[0], yellow);
  EXPECT_EQ(service.Frame(11.0)[0], charging);
  // Canceled.
  service.OnEvent(start, 20.0);
  EXPECT_EQ(service.Frame(20.0)[0], yellow);
  service.OnEvent(Json{{"type", "SCHEDULE_START_CANCELED"}, {"source", "rpc"}}, 20.4);
  EXPECT_EQ(service.Frame(20.5)[0], charging);
  // The state changed.
  service.OnEvent(start, 30.0);
  service.SetRobotState("UNDOCKING", false, false, 0.9);
  EXPECT_EQ(service.Frame(30.0)[0], Render(DefaultProfile().at("UNDOCKING"), 10, 0)[0]);
  service.SetRobotState("IDLE", false, true, 0.9);
  EXPECT_NE(service.Frame(31.0)[0], yellow);
  // An emergency ends it for good.
  service.OnEvent(start, 40.0);
  service.SetRobotState("IDLE", true, true, 0.9);
  EXPECT_EQ(service.Frame(40.0)[0], kRed);
  service.SetRobotState("IDLE", false, true, 0.9);
  EXPECT_NE(service.Frame(41.0)[0], yellow);
  // Without seconds, or other events: nothing.
  service.OnEvent(Json{{"type", "SCHEDULE_START"}}, 50.0);
  service.OnEvent(Json{{"type", "JOB_COMPLETE"}}, 50.0);
  service.OnEvent(Json("no event"), 50.0);
  EXPECT_NE(service.Frame(50.0)[0], yellow);
}

TEST(LightService, ALevelShowsTheBattery) {
  const Rgb green{0, 255, 0};
  LightService service(Config());
  auto lit = [&service, green](double now) {
    const auto leds = service.Frame(now);
    return std::count(leds.begin(), leds.end(), green);
  };
  const Json level = {{"effect", "level"}, {"colors", {"#00ff00"}}, {"period", 0}};
  ASSERT_TRUE(service.Call("light.set_profile", Json{{"profile", {{"MOWING", level}}}}, 0)->ok);
  service.SetRobotState("MOWING", false, false, 0.5);
  EXPECT_EQ(lit(1.0), 5);
  service.SetRobotState("MOWING", false, false, 0.8);
  EXPECT_EQ(lit(1.1), 8);
  service.SetRobotState("MOWING", false, false, 1.4);
  EXPECT_EQ(lit(1.2), 10);
  service.SetRobotState("MOWING", false, false, 0.0);
  EXPECT_EQ(lit(1.3), 0);
  // A running preview follows the battery as well.
  ASSERT_TRUE(service.Call("light.preview", Json{{"effect", level}}, 2.0)->ok);
  service.SetRobotState("IDLE", false, false, 0.3);
  EXPECT_EQ(lit(2.1), 3);
  service.SetRobotState("IDLE", false, false, 0.6);
  EXPECT_EQ(lit(2.2), 6);
}

// STOP held in an emergency: green over the red until canceled, the end of the emergency, or the time is up.
TEST(LightService, TheReadySignalOfStop) {
  const Rgb green{0, 255, 0};
  LightService service(Config());
  service.SetRobotState("IDLE", false, false, 0.8);
  // Outside an emergency STOP shows nothing.
  service.OnStopPress(StopPress::kHeld, 1.0);
  EXPECT_NE(service.Frame(1.1)[0], green);
  service.SetRobotState("IDLE", true, false, 0.8);
  service.OnStopPress(StopPress::kHeld, 2.0);
  EXPECT_EQ(service.Frame(2.1)[0], green);
  EXPECT_EQ(service.Frame(2.4)[0], green);
  service.OnStopPress(StopPress::kCanceled, 3.0);
  EXPECT_EQ(service.Frame(3.0)[0], kRed);
  // Held, then let go: green while the release runs, red again after 2 s without its end.
  service.OnStopPress(StopPress::kHeld, 4.0);
  service.OnStopPress(StopPress::kLong, 5.0);
  EXPECT_EQ(service.Frame(6.9)[0], green);
  EXPECT_EQ(service.Frame(7.0)[0], kRed);
  // Released: the state shows.
  service.OnStopPress(StopPress::kHeld, 8.0);
  service.OnStopPress(StopPress::kLong, 9.0);
  service.SetRobotState("IDLE", false, false, 0.8);
  EXPECT_EQ(service.Frame(9.5)[0], Render(DefaultProfile().at("IDLE"), 10, 0)[0]);
  // A held press without its end shows at most 10 s.
  service.SetRobotState("IDLE", true, false, 0.8);
  service.OnStopPress(StopPress::kHeld, 20.0);
  EXPECT_EQ(service.Frame(29.9)[0], green);
  EXPECT_EQ(service.Frame(30.0)[0], kRed);
}

TEST(LightService, APreviewHasItsLimits) {
  LightService service(Config());
  const Json blue = {{"effect", "solid"}, {"colors", {"#0000ff"}}};
  EXPECT_FALSE(service.Call("light.preview", Json{{"effect", blue}, {"duration", 10.5}}, 0)->ok);
  EXPECT_FALSE(service.Call("light.preview", Json{{"effect", blue}, {"duration", 0}}, 0)->ok);
  EXPECT_FALSE(service.Call("light.preview", Json{{"duration", 1}}, 0)->ok);
  const auto answer = service.Call("light.preview", Json{{"effect", blue}}, 0);
  ASSERT_TRUE(answer && answer->ok);
  EXPECT_DOUBLE_EQ(answer->result["duration"].get<double>(), 10.0);
  EXPECT_EQ(service.Frame(9.9)[0], (Rgb{0, 0, 255}));
  EXPECT_NE(service.Frame(10.0)[0], (Rgb{0, 0, 255}));
  service.Call("light.preview", Json{{"effect", blue}}, 20);
  service.Call("light.reset", Json::object(), 21);
  EXPECT_NE(service.Frame(21.5)[0], (Rgb{0, 0, 255}));
}

TEST(LightService, BrightnessAndCurrentStayWithinTheLimits) {
  LightConfig config = Config();
  config.max_brightness = 0.47;
  LightService service(config);
  service.Call("light.set_profile",
               Json{{"profile", {{"MOWING", {{"effect", "solid"}, {"colors", {"#ffffff"}}}}}}}, 0);
  service.SetRobotState("MOWING", false, false, 0.8);
  EXPECT_EQ(service.Frame(0)[0], (Rgb{120, 120, 120}));
  config.max_current_ma = 100;
  LightService limited(config);
  limited.Call("light.set_profile",
               Json{{"profile", {{"MOWING", {{"effect", "solid"}, {"colors", {"#ffffff"}}}}}}}, 0);
  limited.SetRobotState("MOWING", false, false, 0.8);
  double ma = 0;
  for (const auto& c : limited.Frame(0)) ma += (c.r + c.g + c.b) / 255.0 * 20.0;
  EXPECT_LE(ma, 100.0);
  // The emergency as well.
  limited.SetRobotState("IDLE", true, false, 0.8);
  EXPECT_LE(limited.Frame(0)[0].r, 120);
}

TEST(Output, AFileHoldsTheLastFrameWhole) {
  TempDir dir;
  const std::string path = dir.path + "/frame";
  auto output = OpenFile(path);
  std::string error;
  const auto frame = EncodeWs2812({kRed}, 3);
  ASSERT_TRUE(output->Write(frame, &error)) << error;
  std::ifstream in(path, std::ios::binary);
  const std::vector<uint8_t> back((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_EQ(DecodeWs2812(back), (std::vector<Rgb>{kRed, {}, {}}));
  EXPECT_FALSE(OpenSpidev(dir.path + "/no_spidev", &error));
  EXPECT_FALSE(error.empty());
}
