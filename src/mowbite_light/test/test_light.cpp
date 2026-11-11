#include <gtest/gtest.h>

#include <algorithm>

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

TEST(Effect, EveryKindHasItsName) {
  EXPECT_EQ(EffectNames().size(), static_cast<size_t>(EffectKind::kLevel) + 1);
  EXPECT_EQ(EffectNames()[static_cast<size_t>(EffectKind::kGradient)], "gradient");
  EXPECT_EQ(EffectNames()[static_cast<size_t>(EffectKind::kLevel)], "level");
}

TEST(Effect, AFadeGoesToTheSecondColourAndBack) {
  const Rgb red{255, 0, 0};
  const Rgb blue{0, 0, 255};
  const Effect fade{EffectKind::kFade, {red, blue}, 2.0, 1.0};
  EXPECT_EQ(Render(fade, 3, 0.0), (std::vector<Rgb>(3, red)));
  EXPECT_EQ(Render(fade, 3, 1.0), (std::vector<Rgb>(3, blue)));
  EXPECT_EQ(Render(fade, 3, 2.0), (std::vector<Rgb>(3, red)));
  const auto between = Render(fade, 3, 0.5);
  EXPECT_EQ(between[0], between[2]);
  EXPECT_GT(between[0].r, 0);
  EXPECT_GT(between[0].b, 0);
  EXPECT_LT(between[0].r, 255);
  // Without a second colour it fades to dark.
  EXPECT_EQ(Render({EffectKind::kFade, {red}, 2.0, 1.0}, 3, 1.0)[1], Rgb{});
}

TEST(Effect, ACometHasAHeadAndAFadingTail) {
  const Rgb red{255, 0, 0};
  const Effect comet{EffectKind::kComet, {red}, 1.0, 1.0};
  // 10 LEDs: a tail of 3 behind the head, which starts at the first LED and wraps round.
  auto at = Render(comet, 10, 0.0);
  EXPECT_EQ(at[0], red);
  EXPECT_GT(at[9].r, at[8].r);
  EXPECT_GT(at[8].r, at[7].r);
  EXPECT_GT(at[7].r, 0);
  EXPECT_EQ(at[6], Rgb{});
  EXPECT_EQ(at[1], Rgb{});
  at = Render(comet, 10, 0.5);
  EXPECT_EQ(at[5], red);
  for (size_t i = 2; i < 5; ++i) EXPECT_LT(at[i].r, at[i + 1].r) << i;
  EXPECT_GT(at[2].r, 0);
  EXPECT_EQ(at[1], Rgb{});
  EXPECT_EQ(at[6], Rgb{});
  // On a second colour the tail fades to it.
  const Rgb blue{0, 0, 255};
  at = Render({EffectKind::kComet, {red, blue}, 1.0, 1.0}, 10, 0.5);
  EXPECT_EQ(at[0], blue);
  EXPECT_GT(at[4].b, 0);
  EXPECT_LT(at[4].b, at[3].b);
  // Even a short strip has a tail.
  at = Render(comet, 3, 0.0);
  EXPECT_EQ(at[0], red);
  EXPECT_GT(at[2].r, at[1].r);
  EXPECT_GT(at[1].r, 0);
}

TEST(Effect, AScannerGoesThereAndBack) {
  const Rgb red{255, 0, 0};
  const Effect scanner{EffectKind::kScanner, {red}, 1.0, 1.0};
  auto at = Render(scanner, 10, 0.0);
  EXPECT_EQ(at[0], red);
  EXPECT_EQ(at[1], Rgb{});
  at = Render(scanner, 10, 0.5);
  EXPECT_EQ(at[9], red);
  EXPECT_EQ(at[8], Rgb{});
  // Between two LEDs both show their share.
  at = Render(scanner, 10, 0.25);
  EXPECT_EQ(at[4], (Rgb{128, 0, 0}));
  EXPECT_EQ(at[5], (Rgb{128, 0, 0}));
  EXPECT_EQ(at[3], Rgb{});
  EXPECT_EQ(at[6], Rgb{});
  at = Render(scanner, 10, 0.125);
  EXPECT_EQ(at[2], (Rgb{191, 0, 0}));
  EXPECT_EQ(at[3], (Rgb{64, 0, 0}));
  for (double x : {0.125, 0.25, 0.375}) EXPECT_EQ(Render(scanner, 10, x), Render(scanner, 10, 1 - x)) << x;
  const Rgb blue{0, 0, 255};
  EXPECT_EQ(Render({EffectKind::kScanner, {red, blue}, 1.0, 1.0}, 10, 0.0)[5], blue);
  EXPECT_EQ(Render(scanner, 1, 0.3)[0], red);
}

TEST(Effect, ARainbowIsTheColourCircleAlongTheChain) {
  const Effect rainbow{EffectKind::kRainbow, {}, 1.0, 1.0};
  auto at = Render(rainbow, 6, 0.0);
  EXPECT_EQ(at[0], (Rgb{255, 0, 0}));
  EXPECT_EQ(at[1], (Rgb{255, 255, 0}));
  EXPECT_EQ(at[2], (Rgb{0, 255, 0}));
  EXPECT_EQ(at[3], (Rgb{0, 255, 255}));
  EXPECT_EQ(at[4], (Rgb{0, 0, 255}));
  EXPECT_EQ(at[5], (Rgb{255, 0, 255}));
  // It moves from the first LED to the last once per period.
  const auto later = Render(rainbow, 6, 1.0 / 6);
  for (size_t k = 0; k < 5; ++k) EXPECT_EQ(later[k + 1], at[k]) << k;
  EXPECT_EQ(later[0], at[5]);
  // Colours do not matter, period 0 stands still.
  EXPECT_EQ(Render({EffectKind::kRainbow, {Rgb{0, 0, 255}}, 1.0, 1.0}, 6, 0.4), Render(rainbow, 6, 0.4));
  const Effect still{EffectKind::kRainbow, {}, 0.0, 1.0};
  EXPECT_EQ(Render(still, 6, 3.7), at);
  EXPECT_EQ(Render({EffectKind::kRainbow, {}, 1.0, 0.5}, 6, 0.0)[0], (Rgb{128, 0, 0}));
}

TEST(Effect, ASparkleIsTheSameForTheSameTimeAndChangesEveryPeriod) {
  const Rgb red{255, 0, 0};
  const Effect sparkle{EffectKind::kSparkle, {red}, 1.0, 1.0};
  constexpr size_t kLeds = 50;
  auto lit = [](const std::vector<Rgb>& leds) {
    std::vector<size_t> on;
    for (size_t i = 0; i < leds.size(); ++i) {
      if (leds[i] != Rgb{}) on.push_back(i);
    }
    return on;
  };
  EXPECT_EQ(Render(sparkle, kLeds, 0.5), Render(sparkle, kLeds, 0.5));
  EXPECT_TRUE(lit(Render(sparkle, kLeds, 0.0)).empty());
  // Within a period the same LEDs glow up and fade, brightest in its middle.
  const auto middle = Render(sparkle, kLeds, 0.5);
  const auto chosen = lit(middle);
  ASSERT_FALSE(chosen.empty());
  EXPECT_LT(chosen.size(), kLeds / 2);
  for (size_t i : chosen) EXPECT_EQ(middle[i], red) << i;
  EXPECT_EQ(lit(Render(sparkle, kLeds, 0.3)), chosen);
  EXPECT_EQ(lit(Render(sparkle, kLeds, 0.8)), chosen);
  EXPECT_LT(Render(sparkle, kLeds, 0.2)[chosen[0]].r, 255);
  // About every fifth LED, a different choice in every period.
  size_t sum = 0;
  size_t changes = 0;
  for (int slot = 1; slot <= 20; ++slot) {
    const auto now = lit(Render(sparkle, kLeds, slot + 0.5));
    const auto before = lit(Render(sparkle, kLeds, slot - 0.5));
    sum += now.size();
    if (now != before) ++changes;
  }
  EXPECT_GT(sum, 20 * kLeds / 10);
  EXPECT_LT(sum, 20 * kLeds * 3 / 10);
  EXPECT_EQ(changes, 20u);
  // The others show the second colour.
  const Rgb blue{0, 0, 255};
  const auto on_blue = Render({EffectKind::kSparkle, {red, blue}, 1.0, 1.0}, kLeds, 0.5);
  for (size_t i = 0; i < kLeds; ++i) {
    if (std::find(chosen.begin(), chosen.end(), i) == chosen.end()) {
      EXPECT_EQ(on_blue[i], blue) << i;
    }
  }
}

TEST(Effect, AFlashIsTwoShortFlashesAtTheStart) {
  const Rgb red{255, 0, 0};
  const Rgb blue{0, 0, 255};
  const Effect flash{EffectKind::kFlash, {red, blue}, 1.0, 1.0};
  for (double t : {0.0, 0.07, 0.17, 0.23, 1.02}) EXPECT_EQ(Render(flash, 2, t)[1], red) << t;
  for (double t : {0.08, 0.09, 0.15, 0.25, 0.5, 0.99}) EXPECT_EQ(Render(flash, 2, t)[1], blue) << t;
  EXPECT_EQ(Render({EffectKind::kFlash, {red}, 1.0, 1.0}, 2, 0.5)[0], Rgb{});
}

TEST(Effect, ALevelFillsTheChain) {
  const Rgb green{0, 255, 0};
  const Rgb blue{0, 0, 255};
  const Effect still{EffectKind::kLevel, {green, blue}, 0.0, 1.0};
  auto filled = [&](double level) {
    const auto leds = Render(still, {}, 10, 4.2, level);
    size_t n = 0;
    while (n < leds.size() && leds[n] == green) ++n;
    for (size_t i = n; i < leds.size(); ++i) EXPECT_EQ(leds[i], blue) << level << " " << i;
    return n;
  };
  EXPECT_EQ(filled(0.0), 0u);
  EXPECT_EQ(filled(0.5), 5u);
  EXPECT_EQ(filled(1.0), 10u);
  EXPECT_EQ(filled(0.01), 1u);
  EXPECT_EQ(filled(1.7), 10u);
  EXPECT_EQ(filled(-0.3), 0u);
  EXPECT_EQ(Render(still, {}, 10, 0.0, 0.5), Render(still, {}, 10, 4.2, 0.5));
  // Rendered without a level it is full.
  EXPECT_EQ(Render(still, 10, 0.0), (std::vector<Rgb>(10, green)));
  // With a period its top breathes like a pulse, the rest stays.
  const Effect breathing{EffectKind::kLevel, {green}, 2.0, 1.0};
  auto at = Render(breathing, {}, 10, 0.0, 0.5);
  EXPECT_EQ(at[3], green);
  EXPECT_GT(at[4].g, 0);
  EXPECT_LT(at[4].g, 20);
  EXPECT_EQ(at[5], Rgb{});
  at = Render(breathing, {}, 10, 1.0, 0.5);
  EXPECT_EQ(at[4], green);
  EXPECT_EQ(at[5], Rgb{});
  EXPECT_EQ(Render(breathing, {}, 10, 1.0, 0.0), (std::vector<Rgb>(10)));
}

TEST(Strips, TheNewEffectsRunAlongEveryStripInItsDirection) {
  const Rgb red{255, 0, 0};
  const Rgb blue{0, 0, 255};
  std::string error;
  const auto strips = *ParseStrips({"0-13", "27-14", "30-35"}, 40, &error);
  for (EffectKind kind : {EffectKind::kFade, EffectKind::kComet, EffectKind::kScanner, EffectKind::kRainbow,
                          EffectKind::kSparkle, EffectKind::kFlash, EffectKind::kLevel}) {
    const Effect effect{kind, {red, blue}, 2.0, 1.0};
    for (double t : {0.0, 0.3, 0.55, 0.9, 1.7, 2.5}) {
      const auto leds = Render(effect, strips, 40, t, 0.4);
      const auto along = Render(effect, {}, 14, t, 0.4);
      for (size_t k = 0; k < 14; ++k) {
        EXPECT_EQ(leds[k], along[k]) << static_cast<int>(kind) << " " << t << " " << k;
        EXPECT_EQ(leds[27 - k], along[k]) << static_cast<int>(kind) << " " << t << " " << k;
      }
      const auto short_strip = Render(effect, {}, 6, t, 0.4);
      for (size_t k = 0; k < 6; ++k) EXPECT_EQ(leds[30 + k], short_strip[k]) << static_cast<int>(kind) << " " << k;
      for (size_t i : {28u, 29u, 36u, 39u}) EXPECT_EQ(leds[i], Rgb{}) << static_cast<int>(kind) << " " << i;
    }
  }
  // A level fills each strip from its first LED.
  const auto level = Render({EffectKind::kLevel, {red}, 0.0, 1.0}, strips, 40, 0.0, 0.5);
  EXPECT_EQ(level[0], red);
  EXPECT_EQ(level[7], Rgb{});
  EXPECT_EQ(level[27], red);
  EXPECT_EQ(level[14], Rgb{});
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

TEST(Effect, TheNewEffectsInJsonBothWaysAndTheirLimits) {
  std::string error;
  for (const char* name : {"fade", "comet", "scanner", "rainbow", "sparkle", "flash", "level"}) {
    const Json value = {{"effect", name}, {"colors", {"#ff0000", "#0000ff"}}, {"period", 2.5}, {"brightness", 0.3}};
    const auto effect = EffectFromJson(value, &error);
    ASSERT_TRUE(effect) << name << " " << error;
    EXPECT_EQ(ToJson(*effect), value) << name;
    const auto again = EffectFromJson(ToJson(*effect), &error);
    ASSERT_TRUE(again) << name << " " << error;
    EXPECT_EQ(again->kind, effect->kind) << name;
  }
  EXPECT_FALSE(EffectFromJson(Json::parse(R"({"effect":"strobe","colors":["#ffffff"]})"), &error));
  EXPECT_EQ(error.rfind("effect is one of off, solid, blink, pulse, chase, gradient, fade", 0), 0u) << error;
  EXPECT_NE(error.find("level"), std::string::npos) << error;
  for (const char* bad : {R"({"effect":"fade","colors":[]})", R"({"effect":"comet"})",
                          R"({"effect":"level","colors":[]})", R"({"effect":"fade","colors":["#ffffff"],"period":0})",
                          R"({"effect":"comet","colors":["#ffffff"],"period":0})",
                          R"({"effect":"scanner","colors":["#ffffff"],"period":0})",
                          R"({"effect":"sparkle","colors":["#ffffff"],"period":0})",
                          R"({"effect":"flash","colors":["#ffffff"],"period":0})",
                          R"({"effect":"rainbow","period":0.1})", R"({"effect":"rainbow","period":21})",
                          R"({"effect":"rainbow","colors":["red"]})",
                          R"({"effect":"sparkle","colors":["#ffffff","#ffffff","#ffffff","#ffffff","#ffffff"]})",
                          R"({"effect":"level","colors":["#ffffff"],"brightness":-0.1})"}) {
    EXPECT_FALSE(EffectFromJson(Json::parse(bad), &error)) << bad;
    EXPECT_FALSE(error.empty());
  }
  for (const char* good : {R"({"effect":"rainbow"})", R"({"effect":"rainbow","period":0})",
                           R"({"effect":"rainbow","colors":["#ffffff"]})",
                           R"({"effect":"level","colors":["#00ff00"],"period":0})"}) {
    EXPECT_TRUE(EffectFromJson(Json::parse(good), &error)) << good << " " << error;
  }
  EXPECT_TRUE(ToJson(*EffectFromJson(Json::parse(R"({"effect":"rainbow"})"), &error))["colors"].empty());
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
