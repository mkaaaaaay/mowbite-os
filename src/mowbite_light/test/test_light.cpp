#include <gtest/gtest.h>

#include "mowbite_light/effect.h"
#include "mowbite_light/ws2812.h"
#include "decode_ws2812.h"

using namespace mowbite_light;

TEST(Ws2812, AFrameOf60LedsIs1640Bytes) {
  const auto bytes = EncodeWs2812({{255, 0, 0}}, 60);
  ASSERT_EQ(bytes.size(), 1640u);
  for (size_t i = 0; i < kResetBytes; ++i) {
    EXPECT_EQ(bytes[i], 0) << i;
    EXPECT_EQ(bytes[bytes.size() - 1 - i], 0) << i;
  }
  // Green first: 8 zeros, then red: 8 ones, then blue: 8 zeros.
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(bytes[kResetBytes + i], 0xC0) << i;
    EXPECT_EQ(bytes[kResetBytes + 8 + i], 0xFC) << i;
    EXPECT_EQ(bytes[kResetBytes + 16 + i], 0xC0) << i;
  }
  // The LEDs beyond the given ones are dark.
  EXPECT_EQ(bytes[kResetBytes + 24], 0xC0);
}

TEST(Ws2812, DecodingGivesTheColoursBack) {
  std::vector<Rgb> leds = {{1, 2, 3}, {255, 128, 0}, {0, 0, 255}};
  const auto back = DecodeWs2812(EncodeWs2812(leds, 5));
  ASSERT_EQ(back.size(), 5u);
  for (size_t i = 0; i < leds.size(); ++i) EXPECT_EQ(back[i], leds[i]) << i;
  EXPECT_EQ(back[4], Rgb{});
  auto broken = EncodeWs2812(leds, 5);
  broken[kResetBytes + 3] = 0x80;
  EXPECT_TRUE(DecodeWs2812(broken).empty());
  broken = EncodeWs2812(leds, 5);
  broken.pop_back();
  EXPECT_TRUE(DecodeWs2812(broken).empty());
}

TEST(Ws2812, TheCurrentStaysWithinTheLimit) {
  std::vector<Rgb> white(60, Rgb{255, 255, 255});
  LimitCurrent(&white, 1200.0, 20.0);
  double sum = 0;
  for (const auto& c : white) sum += c.r + c.g + c.b;
  EXPECT_LE(sum / 255.0 * 20.0, 1200.0);
  EXPECT_GT(white[0].r, 0);
  std::vector<Rgb> dim(10, Rgb{10, 0, 0});
  LimitCurrent(&dim, 1200.0, 20.0);
  EXPECT_EQ(dim[0], (Rgb{10, 0, 0}));
}

TEST(Effect, EachKindAtItsPhase) {
  const Rgb red{255, 0, 0};
  const Rgb blue{0, 0, 255};
  EXPECT_EQ(Render({EffectKind::kSolid, {red}, 1.0, 1.0}, 3, 0.7)[2], red);
  EXPECT_EQ(Render({EffectKind::kBlink, {red}, 1.0, 1.0}, 3, 0.2)[0], red);
  EXPECT_EQ(Render({EffectKind::kBlink, {red}, 1.0, 1.0}, 3, 0.7)[0], Rgb{});
  EXPECT_EQ(Render({EffectKind::kBlink, {red, blue}, 1.0, 1.0}, 3, 1.7)[0], blue);
  const Effect pulse{EffectKind::kPulse, {red}, 2.0, 1.0};
  EXPECT_EQ(Render(pulse, 1, 1.0)[0], red);
  EXPECT_LT(Render(pulse, 1, 0.0)[0].r, 20);
  // A chase of 10 LEDs lights 2 and moves them along.
  const Effect chase{EffectKind::kChase, {red}, 1.0, 1.0};
  auto at = Render(chase, 10, 0.0);
  EXPECT_EQ(at[0], red);
  EXPECT_EQ(at[1], red);
  EXPECT_EQ(at[2], Rgb{});
  at = Render(chase, 10, 0.5);
  EXPECT_EQ(at[5], red);
  EXPECT_EQ(at[0], Rgb{});
  // A still gradient from red to blue and back.
  const auto gradient = Render({EffectKind::kGradient, {red, blue}, 0.0, 1.0}, 4, 3.3);
  EXPECT_EQ(gradient[0], red);
  EXPECT_EQ(gradient[2], blue);
  EXPECT_EQ(Render({EffectKind::kSolid, {red}, 1.0, 0.5}, 1, 0)[0], (Rgb{128, 0, 0}));
  EXPECT_EQ(Render({EffectKind::kOff, {red}, 1.0, 1.0}, 1, 0)[0], Rgb{});
}

TEST(Strips, TheTextsAndTheirLimits) {
  std::string error;
  const auto strips = ParseStrips({"0-13", "14-27", "28-41", "54-42"}, 55, &error);
  ASSERT_TRUE(strips) << error;
  ASSERT_EQ(strips->size(), 4u);
  EXPECT_EQ((*strips)[3].first, 54u);
  EXPECT_EQ((*strips)[3].last, 42u);
  EXPECT_TRUE(ParseStrips({}, 55, &error)->empty());
  EXPECT_TRUE(ParseStrips({"7-7"}, 55, &error));
  for (const auto& bad : std::vector<std::vector<std::string>>{
           {"0-55"}, {"55-0"}, {"0-13", "13-20"}, {"20-10", "5-10"}, {"0-"}, {"-1-5"}, {"1-2-3"}, {" 1-2"},
           {"1- 2"}, {"a-b"}, {""}, {"3"}}) {
    EXPECT_FALSE(ParseStrips(bad, 55, &error)) << bad[0];
    EXPECT_FALSE(error.empty());
  }
}

TEST(Strips, EveryStripShowsTheSameFrameInItsDirection) {
  const Rgb red{255, 0, 0};
  const Rgb blue{0, 0, 255};
  std::string error;
  const auto strips = *ParseStrips({"0-13", "27-14", "30-35"}, 40, &error);
  for (const Effect& effect : {Effect{EffectKind::kChase, {red}, 1.0, 1.0},
                               Effect{EffectKind::kGradient, {red, blue}, 2.0, 1.0}}) {
    for (double t : {0.0, 0.3, 0.55, 0.9, 1.7}) {
      const auto leds = Render(effect, strips, 40, t);
      const auto along = Render(effect, 14, t);
      for (size_t k = 0; k < 14; ++k) {
        EXPECT_EQ(leds[k], along[k]) << t << " " << k;
        EXPECT_EQ(leds[27 - k], along[k]) << t << " " << k;
      }
      const auto short_strip = Render(effect, 6, t);
      for (size_t k = 0; k < 6; ++k) EXPECT_EQ(leds[30 + k], short_strip[k]) << t << " " << k;
      for (size_t i : {28u, 29u, 36u, 39u}) EXPECT_EQ(leds[i], Rgb{}) << t << " " << i;
    }
  }
  // A chase starts at the first LED of each strip.
  const auto start = Render({EffectKind::kChase, {red}, 1.0, 1.0}, strips, 40, 0.0);
  EXPECT_EQ(start[0], red);
  EXPECT_EQ(start[27], red);
  EXPECT_EQ(start[13], Rgb{});
  EXPECT_EQ(start[14], Rgb{});
  // Outside every strip it stays dark even for a solid effect.
  EXPECT_EQ(Render({EffectKind::kSolid, {red}, 1.0, 1.0}, strips, 40, 0.0)[38], Rgb{});
  EXPECT_EQ(Render({EffectKind::kSolid, {red}, 1.0, 1.0}, strips, 40, 0.0)[20], red);
}

TEST(Strips, WithoutStripsTheWholeChainIsOne) {
  const Effect chase{EffectKind::kChase, {Rgb{0, 255, 0}}, 2.0, 0.5};
  for (double t : {0.0, 0.7, 1.4}) EXPECT_EQ(Render(chase, {}, 55, t), Render(chase, 55, t)) << t;
}

TEST(Effect, JsonBothWaysAndItsLimits) {
  std::string error;
  const auto effect = EffectFromJson(Json::parse(R"({"effect":"chase","colors":["#00ff00","#0000FF"],"period":2,
                                                     "brightness":0.4})"),
                                     &error);
  ASSERT_TRUE(effect) << error;
  EXPECT_EQ(effect->kind, EffectKind::kChase);
  EXPECT_EQ(effect->colors[1], (Rgb{0, 0, 255}));
  EXPECT_EQ(ToJson(*effect)["colors"][1], "#0000ff");
  EXPECT_DOUBLE_EQ(ToJson(*effect)["brightness"].get<double>(), 0.4);
  for (const char* bad : {R"({"effect":"strobe","colors":["#ffffff"]})", R"({"effect":"solid","colors":[]})",
                          R"({"effect":"solid","colors":["red"]})", R"({"effect":"solid","colors":["#fffff"]})",
                          R"({"effect":"solid","colors":["#ffffff","#ffffff","#ffffff","#ffffff","#ffffff"]})",
                          R"({"effect":"blink","colors":["#ffffff"],"period":0.1})",
                          R"({"effect":"blink","colors":["#ffffff"],"period":0})",
                          R"({"effect":"blink","colors":["#ffffff"],"period":21})",
                          R"({"effect":"solid","colors":["#ffffff"],"brightness":1.5})", R"(["solid"])",
                          R"({"effect":1,"colors":["#ffffff"]})"}) {
    EXPECT_FALSE(EffectFromJson(Json::parse(bad), &error)) << bad;
    EXPECT_FALSE(error.empty());
  }
  EXPECT_TRUE(EffectFromJson(Json::parse(R"({"effect":"gradient","colors":["#ff0000","#0000ff"],"period":0})"),
                             &error));
  EXPECT_TRUE(EffectFromJson(Json::parse(R"({"effect":"off"})"), &error));
}

TEST(Profile, EveryStateHasADefaultAndUnknownOnesAreRefused) {
  const Profile defaults = DefaultProfile();
  for (const auto& state : ProfileStates()) EXPECT_TRUE(defaults.count(state)) << state;
  EXPECT_EQ(defaults.size(), ProfileStates().size());
  std::string error;
  const auto profile = ProfileFromJson(Json::parse(R"({"MOWING":{"effect":"solid","colors":["#ff8800"]}})"), &error);
  ASSERT_TRUE(profile) << error;
  EXPECT_EQ(profile->at("MOWING").colors[0], (Rgb{255, 136, 0}));
  EXPECT_EQ(profile->at("IDLE").kind, defaults.at("IDLE").kind);
  EXPECT_FALSE(ProfileFromJson(Json::parse(R"({"EMERGENCY":{"effect":"off"}})"), &error));
  EXPECT_EQ(error, "unknown state EMERGENCY");
  EXPECT_FALSE(ProfileFromJson(Json::parse(R"({"IDLE":{"effect":"solid"}})"), &error));
  EXPECT_EQ(ToJson(*profile).size(), ProfileStates().size());
}

TEST(Profile, TheShownState) {
  EXPECT_EQ(ShownState("IDLE", true, 0.5, 0.2), "CHARGING");
  EXPECT_EQ(ShownState("IDLE", true, 1.0, 0.2), "CHARGED");
  EXPECT_EQ(ShownState("IDLE", false, 0.1, 0.2), "BATTERY_LOW");
  EXPECT_EQ(ShownState("MOWING", false, 0.1, 0.2), "MOWING");
  EXPECT_EQ(ShownState("IDLE", false, 0.0, 0.2), "IDLE");
  EXPECT_EQ(ShownState("PAUSED", false, 0.5, 0.2), "PAUSED");
  EXPECT_EQ(ShownState("SOMETHING_NEW", false, 0.5, 0.2), "IDLE");
  EXPECT_EQ(EmergencyEffect().colors[0], (Rgb{255, 0, 0}));
}
