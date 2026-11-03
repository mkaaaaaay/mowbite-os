#include <gtest/gtest.h>

#include <stdexcept>

#include "mowbite_logic/geometry.h"
#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {
namespace {

json TronValues() {
  return json{{"robot_width", 0.47}, {"robot_front", 0.60},    {"robot_rear", 0.12},     {"mower_width", 0.22},
              {"blade_ahead", 0.23}, {"blade_offset", 0.0},    {"antenna_x", 0.0},       {"antenna_y", 0.0},
              {"track", 0.395},      {"wheel_radius", 0.1125}, {"dock_contact_x", -0.10}, {"ticks_per_m", 1000.0},
              {"imu_roll", 0.0},     {"imu_pitch", 0.0}};
}

Geometry Tron() {
  return Geometry("tron", TronValues());
}

json Request(const Geometry& geometry, json settings, json reset = json::array()) {
  json request = {{"version", geometry.version()}};
  if (!settings.empty()) request["settings"] = std::move(settings);
  if (!reset.empty()) request["reset"] = std::move(reset);
  return request;
}

// The refusal leaves everything as it was.
void ExpectRefused(Geometry& geometry, const json& request, const std::string& reason, const std::string& key) {
  const json before = geometry.Get();
  const std::string text = geometry.Text();
  const GeometryChange change = geometry.Set(request);
  EXPECT_FALSE(change.ok) << request.dump();
  EXPECT_FALSE(change.stored);
  EXPECT_EQ(change.problem.reason, reason) << change.problem.message;
  EXPECT_EQ(change.problem.key, key);
  EXPECT_FALSE(change.problem.message.empty());
  EXPECT_EQ(geometry.Get(), before);
  EXPECT_EQ(geometry.Text(), text);
}

const json kRectangle = {{0.6, -0.235}, {0.6, 0.235}, {-0.12, 0.235}, {-0.12, -0.235}};

TEST(Geometry, GetListsEveryKeyInOrderWithItsRules) {
  const json answer = Tron().Get();
  EXPECT_EQ(answer["platform"], "tron");
  EXPECT_EQ(answer["version"], 0);
  std::vector<std::string> names;
  for (const auto& item : answer["settings"].items()) names.push_back(item.key());
  EXPECT_EQ(names, (std::vector<std::string>{"robot_width", "robot_front", "robot_rear", "robot_outline", "mower_width",
                                             "blade_ahead", "blade_offset", "antenna_x", "antenna_y", "track",
                                             "wheel_radius", "dock_contact_x", "ticks_per_m", "imu_roll",
                                             "imu_pitch"}));
  const json& width = answer["settings"]["robot_width"];
  EXPECT_DOUBLE_EQ(width["value"].get<double>(), 0.47);
  EXPECT_DOUBLE_EQ(width["default"].get<double>(), 0.47);
  EXPECT_FALSE(width["stored"].get<bool>());
  EXPECT_EQ(width["type"], "number");
  EXPECT_EQ(width["unit"], "m");
  EXPECT_DOUBLE_EQ(width["min"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(width["max"].get<double>(), 1.5);
  EXPECT_TRUE(width["settable"].get<bool>());
  const json& outline = answer["settings"]["robot_outline"];
  EXPECT_EQ(outline["type"], "points");
  EXPECT_EQ(outline["value"], json::array());
  EXPECT_FALSE(outline["stored"].get<bool>());
  for (const char* name : {"wheel_radius", "dock_contact_x"}) {
    EXPECT_FALSE(answer["settings"][name]["settable"].get<bool>()) << name;
  }
  for (const char* name : {"antenna_x", "track", "ticks_per_m", "imu_roll", "imu_pitch"}) {
    EXPECT_TRUE(answer["settings"][name]["settable"].get<bool>()) << name;
  }
  EXPECT_EQ(answer["settings"]["ticks_per_m"]["unit"], "1/m");
  EXPECT_EQ(answer["settings"]["imu_pitch"]["unit"], "rad");
}

TEST(Geometry, ParamsCarryEveryKeyAndTheVersion) {
  const json params = Tron().Params();
  EXPECT_DOUBLE_EQ(params["blade_ahead"].get<double>(), 0.23);
  EXPECT_DOUBLE_EQ(params["dock_contact_x"].get<double>(), -0.10);
  EXPECT_EQ(params["robot_outline"], json::array());
  EXPECT_EQ(params["version"], 0);
  EXPECT_EQ(params["stored"], json::array());
  EXPECT_EQ(params.size(), 17u);
}

TEST(Geometry, ThePlatformsFileHasToBeWholeAndKeepTheRules) {
  json missing = TronValues();
  missing.erase("track");
  EXPECT_THROW(Geometry("tron", missing), std::invalid_argument);
  json text = TronValues();
  text["track"] = "0.395";
  EXPECT_THROW(Geometry("tron", text), std::invalid_argument);
  json behind = TronValues();
  behind["blade_ahead"] = 0.9;
  EXPECT_THROW(Geometry("tron", behind), std::invalid_argument);
  EXPECT_THROW(Geometry("tron", json::array()), std::invalid_argument);
  json outline = TronValues();
  outline["robot_outline"] = kRectangle;
  const Geometry with_outline("tron", outline);
  EXPECT_EQ(with_outline.Params()["robot_outline"].size(), 4u);
}

TEST(Geometry, SetChangesCountsUpAndKeepsTheOthers) {
  Geometry geometry = Tron();
  const GeometryChange change = geometry.Set(Request(geometry, {{"blade_ahead", 0.25}, {"antenna_x", 0.1}}));
  ASSERT_TRUE(change.ok) << change.problem.message;
  EXPECT_TRUE(change.stored);
  EXPECT_EQ(change.changed, (std::vector<std::string>{"blade_ahead", "antenna_x"}));
  EXPECT_TRUE(change.below_default.empty());
  EXPECT_EQ(geometry.version(), 1);
  EXPECT_DOUBLE_EQ(geometry.Value("blade_ahead"), 0.25);
  EXPECT_DOUBLE_EQ(geometry.Value("antenna_x"), 0.1);
  EXPECT_DOUBLE_EQ(geometry.Value("robot_width"), 0.47);
  const json answer = geometry.Get();
  EXPECT_TRUE(answer["settings"]["blade_ahead"]["stored"].get<bool>());
  EXPECT_DOUBLE_EQ(answer["settings"]["blade_ahead"]["default"].get<double>(), 0.23);
  EXPECT_FALSE(answer["settings"]["robot_width"]["stored"].get<bool>());
  EXPECT_EQ(geometry.Params()["version"], 1);
}

TEST(Geometry, ASetThatChangesNothingKeepsTheVersion) {
  Geometry geometry = Tron();
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"blade_ahead", 0.25}})).ok);
  const std::string text = geometry.Text();
  const GeometryChange again = geometry.Set(Request(geometry, {{"blade_ahead", 0.25}}));
  EXPECT_TRUE(again.ok);
  EXPECT_FALSE(again.stored);
  EXPECT_TRUE(again.changed.empty());
  EXPECT_EQ(geometry.version(), 1);
  EXPECT_EQ(geometry.Text(), text);
  const GeometryChange nothing = geometry.Set(Request(geometry, json::object(), {"robot_width"}));
  EXPECT_TRUE(nothing.ok);
  EXPECT_FALSE(nothing.stored);
}

TEST(Geometry, TheValueOfThePlatformEnteredByHandIsStoredButChangesNothing) {
  Geometry geometry = Tron();
  const GeometryChange change = geometry.Set(Request(geometry, {{"blade_ahead", 0.23}}));
  ASSERT_TRUE(change.ok);
  EXPECT_TRUE(change.stored);
  EXPECT_TRUE(change.changed.empty());
  EXPECT_TRUE(geometry.Get()["settings"]["blade_ahead"]["stored"].get<bool>());
  // No value changed, so the version stays; the entry is kept all the same.
  EXPECT_EQ(geometry.version(), 0);
  EXPECT_EQ(json::parse(geometry.Text())["entries"].size(), 1u);
}

TEST(Geometry, ResettingAValueThatEqualsThePlatformsKeepsTheVersion) {
  Geometry geometry = Tron();
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"blade_ahead", 0.23}})).ok);
  const GeometryChange change = geometry.Set(Request(geometry, json::object(), {"blade_ahead"}));
  ASSERT_TRUE(change.ok);
  EXPECT_TRUE(change.stored);
  EXPECT_TRUE(change.changed.empty());
  EXPECT_FALSE(geometry.Get()["settings"]["blade_ahead"]["stored"].get<bool>());
  EXPECT_EQ(geometry.version(), 0);
  EXPECT_TRUE(json::parse(geometry.Text())["entries"].empty());
}

TEST(Geometry, ResetGoesBackToThePlatformsValue) {
  Geometry geometry = Tron();
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"blade_ahead", 0.25}, {"robot_width", 0.5}})).ok);
  const GeometryChange change = geometry.Set(Request(geometry, json::object(), {"blade_ahead"}));
  ASSERT_TRUE(change.ok) << change.problem.message;
  EXPECT_EQ(change.changed, (std::vector<std::string>{"blade_ahead"}));
  EXPECT_DOUBLE_EQ(geometry.Value("blade_ahead"), 0.23);
  EXPECT_DOUBLE_EQ(geometry.Value("robot_width"), 0.5);
  EXPECT_FALSE(geometry.Get()["settings"]["blade_ahead"]["stored"].get<bool>());
  EXPECT_EQ(geometry.version(), 2);
}

TEST(Geometry, RequestsThatAreWrongChangeNothing) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, json::object(), "INVALID_REQUEST", "version");
  ExpectRefused(geometry, json::array(), "INVALID_REQUEST", "version");
  ExpectRefused(geometry, json{{"version", "0"}, {"settings", {{"blade_ahead", 0.2}}}}, "INVALID_REQUEST", "version");
  ExpectRefused(geometry, json{{"version", 0}}, "INVALID_REQUEST", "");
  ExpectRefused(geometry, json{{"version", 0}, {"settings", json::object()}, {"reset", json::array()}},
                "INVALID_REQUEST", "");
  ExpectRefused(geometry, json{{"version", 0}, {"settings", json::array()}}, "INVALID_REQUEST", "");
  ExpectRefused(geometry, json{{"version", 0}, {"reset", "blade_ahead"}}, "INVALID_REQUEST", "");
  ExpectRefused(geometry, json{{"version", 0}, {"reset", json::array({1})}}, "INVALID_REQUEST", "reset");
  ExpectRefused(geometry, json{{"version", 0}, {"setting", {{"blade_ahead", 0.2}}}}, "INVALID_REQUEST", "setting");
  ExpectRefused(geometry, json{{"version", 0}, {"settings", {{"blade_ahead", 0.2}}}, {"reset", {"blade_ahead"}}},
                "INVALID_REQUEST", "blade_ahead");
}

TEST(Geometry, AStaleVersionIsRefusedWithTheCurrentOne) {
  Geometry geometry = Tron();
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"blade_ahead", 0.25}})).ok);
  const json before = geometry.Get();
  const GeometryChange change = geometry.Set(json{{"version", 0}, {"settings", {{"blade_ahead", 0.3}}}});
  EXPECT_FALSE(change.ok);
  EXPECT_EQ(change.problem.reason, "VERSION");
  EXPECT_EQ(change.problem.data["version"], 1);
  EXPECT_EQ(geometry.Get(), before);
}

TEST(Geometry, KeysAndValuesAreChecked) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, Request(geometry, {{"antenna_z", 0.1}}), "UNKNOWN_KEY", "antenna_z");
  ExpectRefused(geometry, Request(geometry, {{"wheel_radius", 0.1}}), "NOT_SETTABLE", "wheel_radius");
  ExpectRefused(geometry, Request(geometry, {{"dock_contact_x", -0.2}}), "NOT_SETTABLE", "dock_contact_x");
  ExpectRefused(geometry, Request(geometry, json::object(), {"wheel_radius"}), "NOT_SETTABLE", "wheel_radius");
  ExpectRefused(geometry, Request(geometry, json::object(), {"nope"}), "UNKNOWN_KEY", "nope");
  ExpectRefused(geometry, Request(geometry, {{"blade_ahead", "0.2"}}), "INVALID_VALUE", "blade_ahead");
  ExpectRefused(geometry, Request(geometry, {{"blade_ahead", nullptr}}), "INVALID_VALUE", "blade_ahead");
  ExpectRefused(geometry, Request(geometry, {{"blade_ahead", true}}), "INVALID_VALUE", "blade_ahead");
}

TEST(Geometry, ValuesOutsideTheRangeAreRefusedNotCut) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, Request(geometry, {{"robot_width", 47}}), "OUT_OF_RANGE", "robot_width");
  ExpectRefused(geometry, Request(geometry, {{"robot_width", 0.047}}), "OUT_OF_RANGE", "robot_width");
  ExpectRefused(geometry, Request(geometry, {{"robot_rear", -0.1}}), "OUT_OF_RANGE", "robot_rear");
  ExpectRefused(geometry, Request(geometry, {{"mower_width", 22}}), "OUT_OF_RANGE", "mower_width");
  const GeometryChange change = geometry.Set(Request(geometry, {{"robot_width", 47}}));
  EXPECT_DOUBLE_EQ(change.problem.data["min"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(change.problem.data["max"].get<double>(), 1.5);
  EXPECT_NE(change.problem.message.find("robot_width"), std::string::npos);
  // The boundaries themselves are fine.
  EXPECT_TRUE(geometry
                  .Set(Request(geometry, {{"robot_width", 0.1},
                                          {"robot_front", 0.5},
                                          {"robot_rear", 0.0},
                                          {"mower_width", 0.1},
                                          {"blade_ahead", 0.1},
                                          {"antenna_x", 0.0}}))
                  .ok);
}

TEST(Geometry, TheCalibratedValuesHaveTheirOwnRanges) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, Request(geometry, {{"ticks_per_m", 99}}), "OUT_OF_RANGE", "ticks_per_m");
  ExpectRefused(geometry, Request(geometry, {{"ticks_per_m", 10001}}), "OUT_OF_RANGE", "ticks_per_m");
  ExpectRefused(geometry, Request(geometry, {{"track", 0.14}}), "OUT_OF_RANGE", "track");
  ExpectRefused(geometry, Request(geometry, {{"track", 0.81}}), "OUT_OF_RANGE", "track");
  ExpectRefused(geometry, Request(geometry, {{"antenna_x", 0.61}}), "OUT_OF_RANGE", "antenna_x");
  ExpectRefused(geometry, Request(geometry, {{"antenna_y", 0.31}}), "OUT_OF_RANGE", "antenna_y");
  ExpectRefused(geometry, Request(geometry, {{"imu_roll", 0.09}}), "OUT_OF_RANGE", "imu_roll");
  ExpectRefused(geometry, Request(geometry, {{"imu_pitch", -0.09}}), "OUT_OF_RANGE", "imu_pitch");
  const GeometryChange change = geometry.Set(Request(geometry, {{"ticks_per_m", 50}}));
  EXPECT_DOUBLE_EQ(change.problem.data["min"].get<double>(), 100.0);
  EXPECT_NE(change.problem.message.find("1/m"), std::string::npos);
  const GeometryChange done = geometry.Set(
      Request(geometry, {{"ticks_per_m", 940}, {"track", 0.4}, {"imu_roll", 0.03}, {"imu_pitch", -0.087}}));
  ASSERT_TRUE(done.ok) << done.problem.message;
  EXPECT_DOUBLE_EQ(geometry.Value("track"), 0.4);
  EXPECT_DOUBLE_EQ(geometry.Value("imu_pitch"), -0.087);
  EXPECT_EQ(geometry.Params()["stored"], json::array({"track", "ticks_per_m", "imu_roll", "imu_pitch"}));
  EXPECT_TRUE(geometry.Get()["settings"]["imu_roll"]["stored"].get<bool>());
}

TEST(Geometry, TheMountingOfTheImuMayDifferFromThePlatformsByFiveDegrees) {
  // The IMU of the NX60i sits about 4.5 degrees nose up: its range lies around that.
  json values = TronValues();
  values["imu_pitch"] = 0.079;
  Geometry geometry("yardforce_nx60i", values);
  const json pitch = geometry.Get()["settings"]["imu_pitch"];
  EXPECT_DOUBLE_EQ(pitch["default"].get<double>(), 0.079);
  EXPECT_NEAR(pitch["min"].get<double>(), -0.008, 1e-12);
  EXPECT_NEAR(pitch["max"].get<double>(), 0.166, 1e-12);
  EXPECT_DOUBLE_EQ(geometry.Get()["settings"]["imu_roll"]["max"].get<double>(), 0.087);
  EXPECT_TRUE(geometry.Set(Request(geometry, {{"imu_pitch", 0.16}})).ok);
  ExpectRefused(geometry, Request(geometry, {{"imu_pitch", -0.01}}), "OUT_OF_RANGE", "imu_pitch");
  // A file entry outside of it is not taken.
  std::string why;
  Geometry other("yardforce_nx60i", values);
  EXPECT_FALSE(other.Load(R"({"entries": {"imu_pitch": 0.17}})", &why));
  EXPECT_TRUE(other.Load(R"({"entries": {"imu_pitch": 0.0}})", &why)) << why;
}

TEST(Geometry, TheBodyIsNotLongerOrWiderThanPossible) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, Request(geometry, {{"robot_front", 1.5}}), "OUT_OF_RANGE", "robot_front");
  ExpectRefused(geometry, Request(geometry, {{"robot_front", 0.05}, {"robot_rear", 0.02}, {"blade_ahead", 0.0}}),
                "OUT_OF_RANGE", "robot_front");
  EXPECT_TRUE(geometry.Set(Request(geometry, {{"robot_front", 1.38}})).ok);
}

TEST(Geometry, BladeAndAntennaSitOnTheBodyAndTheCutIsNotWiderThanIt) {
  Geometry geometry = Tron();
  // A sign slip: the blade 0.23 m behind base_link is outside of the body, which reaches back 0.12 m.
  ExpectRefused(geometry, Request(geometry, {{"blade_ahead", -0.23}}), "OUTSIDE_BODY", "blade_ahead");
  ExpectRefused(geometry, Request(geometry, {{"blade_offset", 0.3}}), "OUTSIDE_BODY", "blade_offset");
  ExpectRefused(geometry, Request(geometry, {{"antenna_x", -0.2}}), "OUTSIDE_BODY", "antenna_x");
  ExpectRefused(geometry, Request(geometry, {{"antenna_y", -0.3}}), "OUTSIDE_BODY", "antenna_y");
  ExpectRefused(geometry, Request(geometry, {{"mower_width", 0.5}}), "OUTSIDE_BODY", "mower_width");
  // The body is checked together with the rest: a smaller body that leaves the blade out is refused ...
  ExpectRefused(geometry, Request(geometry, {{"robot_front", 0.2}}), "OUTSIDE_BODY", "blade_ahead");
  // ... unless the blade moves with it.
  EXPECT_TRUE(geometry.Set(Request(geometry, {{"robot_front", 0.2}, {"blade_ahead", 0.1}})).ok);
  const GeometryChange change = geometry.Set(Request(geometry, {{"antenna_x", 0.5}}));
  EXPECT_EQ(change.problem.reason, "OUTSIDE_BODY");
  EXPECT_DOUBLE_EQ(change.problem.data["max"].get<double>(), 0.2);
  EXPECT_DOUBLE_EQ(change.problem.data["min"].get<double>(), -0.12);
}

TEST(Geometry, ARefusedPartLeavesTheValidPartsOut) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, Request(geometry, {{"blade_ahead", 0.25}, {"antenna_x", 9}}), "OUT_OF_RANGE", "antenna_x");
  EXPECT_DOUBLE_EQ(geometry.Value("blade_ahead"), 0.23);
  EXPECT_EQ(geometry.version(), 0);
}

TEST(Geometry, AnOutlineTakesTheBodyFromTheRectangle) {
  Geometry geometry = Tron();
  const GeometryChange change = geometry.Set(Request(geometry, {{"robot_outline", kRectangle}}));
  ASSERT_TRUE(change.ok) << change.problem.message;
  EXPECT_EQ(change.changed, (std::vector<std::string>{"robot_outline"}));
  EXPECT_TRUE(change.below_default.empty());
  EXPECT_EQ(geometry.Params()["robot_outline"].size(), 4u);
  EXPECT_TRUE(geometry.Get()["settings"]["robot_outline"]["stored"].get<bool>());
  // The blade has to sit on the outline's box now.
  ExpectRefused(geometry, Request(geometry, {{"blade_ahead", 0.7}}), "OUTSIDE_BODY", "blade_ahead");
  // A shorter outline leaves the blade out.
  const json short_outline = {{0.2, -0.235}, {0.2, 0.235}, {-0.12, 0.235}, {-0.12, -0.235}};
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", short_outline}}), "OUTSIDE_BODY", "blade_ahead");
  const GeometryChange reset = geometry.Set(Request(geometry, json::object(), {"robot_outline"}));
  ASSERT_TRUE(reset.ok);
  EXPECT_EQ(geometry.Params()["robot_outline"], json::array());
  EXPECT_FALSE(geometry.Get()["settings"]["robot_outline"]["stored"].get<bool>());
}

TEST(Geometry, AnOutlineHasToBeAListOfPairsWithThreePointsOrMore) {
  Geometry geometry = Tron();
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", 3}}), "INVALID_VALUE", "robot_outline");
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", json::array({0.6, 0.2, 0.1})}}), "INVALID_VALUE",
                "robot_outline");
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", json::array({json::array({0.6, 0.2, 0.0})})}}),
                "INVALID_VALUE", "robot_outline");
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", json::array({json::array({0.6, "a"})})}}),
                "INVALID_VALUE", "robot_outline");
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", json{{0.6, -0.2}, {0.6, 0.2}}}}), "INVALID_VALUE",
                "robot_outline");
  ExpectRefused(geometry, Request(geometry, {{"robot_outline", json{{2.0, -0.2}, {0.6, 0.2}, {0.0, 0.2}}}}),
                "OUT_OF_RANGE", "robot_outline");
  // An empty list is the rectangle again.
  EXPECT_TRUE(geometry.Set(Request(geometry, {{"robot_outline", json::array()}})).ok);
  EXPECT_EQ(geometry.Params()["robot_outline"], json::array());
}

TEST(Geometry, BodyKeysBelowThePlatformsValueAreNamed) {
  Geometry geometry = Tron();
  EXPECT_TRUE(geometry.BelowDefault().empty());
  GeometryChange change = geometry.Set(Request(geometry, {{"robot_width", 0.40}, {"robot_rear", 0.10}}));
  ASSERT_TRUE(change.ok) << change.problem.message;
  EXPECT_EQ(change.below_default, (std::vector<std::string>{"robot_width", "robot_rear"}));
  EXPECT_EQ(geometry.BelowDefault(), change.below_default);
  // Larger is fine, and it does not take the others off the list.
  change = geometry.Set(Request(geometry, {{"robot_front", 0.65}}));
  ASSERT_TRUE(change.ok);
  EXPECT_EQ(change.below_default, (std::vector<std::string>{"robot_width", "robot_rear"}));
  change = geometry.Set(Request(geometry, json::object(), {"robot_width", "robot_rear"}));
  ASSERT_TRUE(change.ok);
  EXPECT_TRUE(change.below_default.empty());
}

TEST(Geometry, AnOutlineBelowThePlatformsBodyIsNamed) {
  Geometry geometry = Tron();
  const json narrow = {{0.6, -0.2}, {0.6, 0.2}, {-0.12, 0.2}, {-0.12, -0.2}};
  GeometryChange change = geometry.Set(Request(geometry, {{"robot_outline", narrow}}));
  ASSERT_TRUE(change.ok) << change.problem.message;
  EXPECT_EQ(change.below_default, (std::vector<std::string>{"robot_outline"}));
  const json wide = {{0.62, -0.25}, {0.62, 0.25}, {-0.15, 0.25}, {-0.15, -0.25}};
  change = geometry.Set(Request(geometry, {{"robot_outline", wide}}));
  ASSERT_TRUE(change.ok);
  EXPECT_TRUE(change.below_default.empty());
}

TEST(Geometry, TheFileGivesTheEntriesBack) {
  Geometry geometry = Tron();
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"blade_ahead", 0.25}, {"robot_outline", kRectangle}})).ok);
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"antenna_y", -0.05}})).ok);
  const json file = json::parse(geometry.Text());
  EXPECT_EQ(file["version"], 2);
  EXPECT_EQ(file["entries"].size(), 3u);
  EXPECT_DOUBLE_EQ(file["entries"]["blade_ahead"].get<double>(), 0.25);

  Geometry again = Tron();
  std::string why;
  ASSERT_TRUE(again.Load(geometry.Text(), &why)) << why;
  EXPECT_EQ(again.Get(), geometry.Get());
  EXPECT_EQ(again.Params(), geometry.Params());
  EXPECT_EQ(again.version(), 2);
  EXPECT_EQ(again.Text(), geometry.Text());
}

TEST(Geometry, AFileWithoutVersionOrEntriesIsTaken) {
  Geometry geometry = Tron();
  std::string why;
  ASSERT_TRUE(geometry.Load(R"({"entries": {"antenna_x": 0.12}})", &why)) << why;
  EXPECT_DOUBLE_EQ(geometry.Value("antenna_x"), 0.12);
  EXPECT_EQ(geometry.version(), 0);
  ASSERT_TRUE(geometry.Load("{}", &why));
  EXPECT_DOUBLE_EQ(geometry.Value("antenna_x"), 0.0);
  EXPECT_FALSE(geometry.Get()["settings"]["antenna_x"]["stored"].get<bool>());
}

TEST(Geometry, AFileThatIsWrongChangesNothing) {
  Geometry geometry = Tron();
  ASSERT_TRUE(geometry.Set(Request(geometry, {{"blade_ahead", 0.25}})).ok);
  const json before = geometry.Get();
  const std::string text = geometry.Text();
  const std::vector<std::string> wrong = {
      "",
      "not json",
      "[]",
      R"({"version": -1})",
      R"({"version": 1.5})",
      R"({"version": "1"})",
      R"({"entries": []})",
      R"({"entries": {"antenna_z": 1}})",
      R"({"entries": {"wheel_radius": 0.1}})",
      R"({"entries": {"imu_pitch": 0.1}})",
      R"({"entries": {"blade_ahead": "x"}})",
      R"({"entries": {"robot_width": 47}})",
      R"({"entries": {"blade_ahead": -0.5}})",
      R"({"entries": {"robot_outline": [[1, 2]]}})",
      R"({"entries": {"robot_outline": 5}})",
  };
  for (const auto& text_in : wrong) {
    std::string why;
    EXPECT_FALSE(geometry.Load(text_in, &why)) << text_in;
    EXPECT_FALSE(why.empty()) << text_in;
    EXPECT_EQ(geometry.Get(), before) << text_in;
    EXPECT_EQ(geometry.Text(), text) << text_in;
  }
  EXPECT_FALSE(geometry.Load("not json", nullptr));
}

TEST(GeometryEvent, NamesWhatChangedAndStaysInfoWithoutASmallerBody) {
  LogicCore core{LogicConfig{}};
  core.GeometryChanged({"blade_ahead", "antenna_x"}, {}, 3);
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "GEOMETRY_CHANGED");
  const json& details = events[0].details;
  EXPECT_EQ(details["changed"], json::array({"blade_ahead", "antenna_x"}));
  EXPECT_EQ(details["below_default"], json::array());
  EXPECT_EQ(details["version"], 3);
  EXPECT_EQ(details["severity"], "info");
  EXPECT_FALSE(details.contains("reason"));
  EXPECT_FALSE(details.contains("resume"));
  EXPECT_EQ(details["message"], "The dimensions of the mower changed: blade_ahead, antenna_x");
  EXPECT_TRUE(details.contains("x"));
}

TEST(GeometryEvent, WarnsWithTheReasonWhenTheBodyIsBelowThePlatformsValue) {
  LogicCore core{LogicConfig{}};
  core.GeometryChanged({"robot_width", "robot_rear"}, {"robot_width"}, 4);
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  const json& details = events[0].details;
  EXPECT_EQ(details["reason"], "BELOW_DEFAULT");
  EXPECT_EQ(details["severity"], "warning");
  EXPECT_EQ(details["below_default"], json::array({"robot_width"}));
  EXPECT_FALSE(details.contains("resume"));
  EXPECT_EQ(details["message"],
            "The dimensions of the mower changed: robot_width, robot_rear. robot_width below the platform's value: "
            "check the measurement");
}

}  // namespace
}  // namespace mowbite_logic
