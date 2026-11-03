#include <gtest/gtest.h>

#include "xcore_driver/input_config.h"

using namespace xcore;

namespace {

InputConfig Nx60i() {
  InputConfig c;
  c.gpio = {{"GPIO1", true, "stop", 10, "stop"},          {"GPIO5", true, "stop", 10, "stop"},
            {"GPIO10", true, "lift", 2500, "lift"},       {"GPIO11", true, "lift", 2500, "lift"},
            {"GPIO12", true, "collision", 500, "bumper_right"}, {"GPIO13", true, "collision", 500, "bumper_left"}};
  c.cover = {{"hall_mux", "om"}};
  return c;
}

}  // namespace

TEST(InputConfig, Json) {
  InputConfig c;
  c.gpio = {{"GPIO1", true, "stop", 10, "stop"}, {"GPIO7", false, "", 0, ""}};
  c.cover = {{"hall_mux", "om"}};
  EXPECT_EQ(InputConfigJson(c),
            "{\"gpio\":[{\"line\":\"GPIO1\",\"active\":\"high\",\"emergency\":{\"reason\":\"stop\",\"delay\":10}},"
            "{\"line\":\"GPIO7\",\"active\":\"low\"}],\"yf_cover_ui\":[{\"id\":\"hall_mux\",\"value\":\"om\"}]}");
  EXPECT_EQ(InputConfigJson(InputConfig{}), "{}");
}

TEST(InputConfig, IndicesFollowTheOrder) {
  auto map = InputMapOf(Nx60i());
  EXPECT_EQ(map.stop, (std::vector<int>{0, 1}));
  EXPECT_EQ(map.lift, (std::vector<int>{2, 3}));
  EXPECT_EQ(map.bumper_right, (std::vector<int>{4}));
  EXPECT_EQ(map.bumper_left, (std::vector<int>{5}));
}

TEST(InputConfig, Check) {
  EXPECT_FALSE(CheckInputConfig(Nx60i()));
  InputConfig c = Nx60i();
  c.gpio[0].reason = "rain";
  EXPECT_TRUE(CheckInputConfig(c));
  c = Nx60i();
  c.gpio[0].line = "GPIO\"1";
  EXPECT_TRUE(CheckInputConfig(c));
  c = Nx60i();
  c.gpio[0].role = "button";
  EXPECT_TRUE(CheckInputConfig(c));
  c = Nx60i();
  c.gpio.resize(30);
  EXPECT_TRUE(CheckInputConfig(c));
}

TEST(Heatshrink, LiteralBits) {
  // 'A' = 0x41: 1 0100 0001, then 'B' = 0x42: 1 0100 0010, padded with zeros.
  EXPECT_EQ(HeatshrinkLiterals("AB"), (Bytes{0xA0, 0xD0, 0x80}));
  EXPECT_TRUE(HeatshrinkLiterals("").empty());
  std::string json = InputConfigJson(Nx60i());
  Bytes stream = HeatshrinkLiterals(json);
  EXPECT_EQ(stream.size(), (json.size() * 9 + 7) / 8);
  EXPECT_LE(stream.size(), kMaxPayload - 8);
  EXPECT_EQ(HeatshrinkDecode(stream), json);
}

TEST(Heatshrink, DecodesBackReferences) {
  // "ab" as literals, then a reference 2 back with length 4: "ababab".
  Bytes stream{0xB0, 0xD8, 0x80, 0x11, 0x80};
  // 1 01100001 1 01100010 0 000000001 00011 and zero padding.
  EXPECT_EQ(HeatshrinkDecode(stream), "ababab");
  // A reference before the start.
  EXPECT_FALSE(HeatshrinkDecode(Bytes{0x00, 0x80, 0x00}));
}
