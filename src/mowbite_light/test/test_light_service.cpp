#include <gtest/gtest.h>
#include <stdlib.h>

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
