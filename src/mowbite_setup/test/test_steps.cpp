#include <gtest/gtest.h>

#include <string>

#include "mowbite_api/setup.h"
#include "mowbite_setup/steps.h"

using mowbite_setup::json;
using mowbite_setup::ParseSteps;
using mowbite_setup::Step;
using mowbite_setup::StepList;

namespace {

const char* const kList = R"({
  "catalog": 3,
  "steps": [
    {"id": "variant", "stage": "device", "check": "choice",
     "params": {"name": "variant", "values": ["standard", "rebuilt"]},
     "value_texts": {"standard": {"de": {"name": "Standard"}, "en": {"name": "Standard"}},
                     "rebuilt": {"de": {"name": "Umgebaut"}, "en": {"name": "Rebuilt"}}},
     "texts": {"de": {"name": "Bauart"}, "en": {"name": "Build"}}},
    {"id": "board", "stage": "device", "check": "board", "params": {"time": 2, "battery": 20},
     "image": "board.svg", "accent": "board-accent.svg", "symbol": "board"},
    {"id": "stop", "stage": "inputs", "check": "input", "params": {"role": "stop"}, "needs": ["board"],
     "parts": [{"id": "top", "texts": {"de": {"name": "STOP oben"}, "en": {"name": "STOP on top"}}},
               {"id": "rear", "texts": {"de": {"name": "STOP hinten"}, "en": {"name": "STOP at the rear"}}}],
     "mark": ["stop_top"], "note": {"de": "Zwei Tasten.", "en": "Two buttons."}},
    {"id": "lift", "stage": "inputs", "check": "input", "params": {"role": "lift", "within": 3, "short_max": 1.5},
     "parts": [{"id": "hold"}, {"id": "bump", "mode": "short"}], "skip_reasons": ["not_fitted"]},
    {"id": "imu", "stage": "inputs", "check": "imu"},
    {"id": "raised", "stage": "inputs", "check": "confirm",
     "confirm": ["raised", {"id": "switch", "texts": {"de": {"name": "Schalter an"}, "en": {"name": "Switch on"}}}],
     "texts": {"de": {"name": "Aufbocken"}, "en": {"name": "Jack up"}}, "skippable": false},
    {"id": "ntrip", "stage": "position", "check": "done_when", "params": {"condition": "ntrip"}},
    {"id": "fix", "stage": "position", "check": "position"},
    {"id": "length", "stage": "dimensions", "check": "number",
     "params": {"name": "length", "unit": "m", "min": 0.1, "max": 2},
     "texts": {"de": {"name": "Länge"}, "en": {"name": "Length"}}},
    {"id": "dimensions", "stage": "dimensions", "check": "geometry", "params": {"keys": ["track", "antenna_x"]}}
  ]})";

json List() {
  return json::parse(kList);
}

// The list with one change, and what ParseSteps says about it.
std::string Problem(const std::string& pointer, const json& value) {
  json list = List();
  list[json::json_pointer(pointer)] = value;
  std::string why;
  EXPECT_FALSE(ParseSteps(list, &why).has_value()) << pointer;
  return why;
}

}  // namespace

TEST(Steps, AListOfEveryCheckIsRead) {
  std::string why;
  const auto list = ParseSteps(List(), &why);
  ASSERT_TRUE(list.has_value()) << why;
  EXPECT_EQ(list->catalog, 3);
  ASSERT_EQ(list->steps.size(), 10u);
  EXPECT_EQ(list->Images(), (std::vector<std::string>{"board.svg", "board-accent.svg"}));

  const Step& board = *list->Find("board");
  EXPECT_EQ(board.kind, "measure");
  EXPECT_TRUE(board.start);
  EXPECT_DOUBLE_EQ(board.Number("time"), 2.0);
  EXPECT_DOUBLE_EQ(board.Number("status_rate"), 8.0);
  EXPECT_EQ(board.Word("imu"), "yes");
  EXPECT_EQ(board.texts["de"]["name"], "Platine und Sensoren");
  EXPECT_EQ(board.skip_reasons.size(), mowbite_api::SetupSkipReasons().size());

  const Step& stop = *list->Find("stop");
  EXPECT_EQ(stop.Word("emergency"), "STOP");
  EXPECT_EQ(stop.needs, (std::vector<std::string>{"board"}));
  ASSERT_EQ(stop.parts.size(), 2u);
  EXPECT_EQ(stop.parts[1].mode, "press");
  EXPECT_EQ(stop.parts[1].role, "stop");
  EXPECT_EQ(stop.parts[1].texts["en"]["name"], "STOP at the rear");
  EXPECT_EQ(stop.texts["en"]["name"], "STOP button");
  const std::string help = stop.texts["de"]["help"];
  EXPECT_EQ(help.substr(help.size() - 12), "Zwei Tasten.");

  const Step& lift = *list->Find("lift");
  EXPECT_EQ(lift.Word("emergency"), "LIFT");
  EXPECT_EQ(lift.parts[0].texts["de"]["name"], "Anheben");
  EXPECT_EQ(lift.parts[1].texts["de"]["name"], "Kurz auslösen");
  ASSERT_EQ(lift.skip_reasons.size(), 1u);
  EXPECT_EQ(lift.skip_reasons[0].id, "not_fitted");

  const Step& imu = *list->Find("imu");
  ASSERT_EQ(imu.parts.size(), 3u);
  EXPECT_EQ(imu.parts[2].move, "turn_left");

  const Step& raised = *list->Find("raised");
  EXPECT_FALSE(raised.skippable);
  EXPECT_TRUE(raised.skip_reasons.empty());
  ASSERT_EQ(raised.confirm.size(), 2u);
  EXPECT_EQ(raised.confirm[0].texts["en"]["name"], "Mower jacked up, wheels free");
  EXPECT_EQ(raised.confirm[1].texts["de"]["name"], "Schalter an");

  const Step& ntrip = *list->Find("ntrip");
  EXPECT_FALSE(ntrip.start);
  EXPECT_TRUE(ntrip.automatic);
  EXPECT_EQ(ntrip.open, "ntrip");
  EXPECT_EQ(ntrip.texts["en"]["name"], "Correction data");
  EXPECT_EQ(list->Find("fix")->Word("solution"), "fixed");
  EXPECT_EQ(list->Find("dimensions")->open, "geometry");
  EXPECT_EQ(list->Find("variant")->choices[1].texts["de"]["name"], "Umgebaut");
  EXPECT_EQ(list->Find("nothing"), nullptr);
}

// The screen of a step is the one of its condition or check, unless the list names another.
TEST(Steps, AStepNamesItsOwnScreen) {
  json list = List();
  list["steps"][6]["open"] = "map";
  list["steps"][9]["open"] = "map";
  const auto parsed = ParseSteps(list, nullptr);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->Find("ntrip")->open, "map");
  EXPECT_EQ(parsed->Find("dimensions")->open, "map");
  EXPECT_EQ(parsed->Find("fix")->open, "");
}

TEST(Steps, AGeometryStepWithoutKeysTakesAllTheUserSets) {
  json list = List();
  list["steps"][9].erase("params");
  const auto parsed = ParseSteps(list, nullptr);
  ASSERT_TRUE(parsed.has_value());
  const auto& keys = parsed->Find("dimensions")->lists.at("keys");
  EXPECT_NE(std::find(keys.begin(), keys.end(), "robot_outline"), keys.end());
  EXPECT_EQ(std::find(keys.begin(), keys.end(), "wheel_radius"), keys.end());
  EXPECT_EQ(std::find(keys.begin(), keys.end(), "robot_shape"), keys.end());
}

TEST(Steps, WhatBreaksTheRulesIsNamed) {
  EXPECT_EQ(Problem("/catalog", 0), "catalog is no whole number from 1");
  EXPECT_EQ(Problem("/extra", 1), "the step list: extra is no field of it");
  EXPECT_EQ(Problem("/steps/1/id", "variant"), "variant comes twice");
  EXPECT_EQ(Problem("/steps/1/id", "Board"), "Board is no id of a step");
  EXPECT_EQ(Problem("/steps/1/speed", 1), "board: speed is no field of it");
  EXPECT_EQ(Problem("/steps/1/check", "drive"), "board: drive is no check");
  EXPECT_EQ(Problem("/steps/1/stage", "garden"), "board: garden is no stage");
  EXPECT_EQ(Problem("/steps/6/stage", "device"), "ntrip: its stage comes before the one of the step ahead");
  EXPECT_NE(Problem("/steps/1/params/time", 31).find("board.time is outside of"), std::string::npos);
  EXPECT_EQ(Problem("/steps/1/params/fast", true), "board: fast is no param of board");
  EXPECT_EQ(Problem("/steps/2/params/role", "rain"), "stop.role: rain is none of its values");
  EXPECT_EQ(Problem("/steps/2/params", json::object()), "stop.role is missing");
  EXPECT_EQ(Problem("/steps/1/needs", json::array({"stop"})), "board needs stop, which is no step ahead of it");
  EXPECT_EQ(Problem("/steps/0/value_texts/rebuilt", json::object()), "variant.value_texts.rebuilt: no name in de");
  EXPECT_EQ(Problem("/steps/5/texts/en", json::object()), "raised: no name in en");
  EXPECT_EQ(Problem("/steps/5/texts/fr", json::object()), "raised: texts in fr, only de and en");
  EXPECT_EQ(Problem("/steps/5/confirm/0", "sober"), "raised.confirm: sober is not in the catalog");
  EXPECT_EQ(Problem("/steps/6/confirm", json::array({"raised"})),
            "ntrip: confirm goes with a step setup.start begins");
  EXPECT_EQ(Problem("/steps/1/parts", json::array()), "board: parts go with input, inputs_assign, imu, stop_moving and wheels only");
  EXPECT_EQ(Problem("/steps/2/parts/1/id", "top"), "stop: the part top comes twice");
  EXPECT_EQ(Problem("/steps/2/parts/1/mode", "long"), "stop: long is no mode");
  EXPECT_EQ(Problem("/steps/4/parts", json::array({"front_up", "jump"})), "imu: jump is no move");
  EXPECT_EQ(Problem("/steps/1/image", "../board.svg"), "board: ../board.svg is no image name");
  EXPECT_EQ(Problem("/steps/2/accent", "stop-accent.svg"), "stop: accent needs an image and an image name");
  EXPECT_EQ(Problem("/steps/1/symbol", "star"), "board: star is no symbol");
  EXPECT_EQ(Problem("/steps/6/open", "settings"), "ntrip: open is no screen");
  EXPECT_EQ(Problem("/steps/3/skip_reasons/0", "bored"), "lift.skip_reasons: bored is not in the catalog");
  EXPECT_EQ(Problem("/steps/5/skip_reasons", json::array({"later"})), "raised: skip_reasons without skippable");
  EXPECT_EQ(Problem("/steps/8/params/max", 0.05), "length: min is not below max");
  EXPECT_EQ(Problem("/steps/9/params/keys/0", "wheel_radius"), "dimensions: wheel_radius is no key the user sets");
  EXPECT_EQ(Problem("/steps", json::array()), "steps is no list of steps");
}

TEST(Steps, AnImageIsOneColourAndNothingElse) {
  using mowbite_setup::CheckImage;
  const std::string head = R"(<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">)";
  EXPECT_EQ(CheckImage(head + R"(<path fill="#000" d="M0 0h10v10z"/><circle fill="none" stroke="#000" r="3"/></svg>)"),
            "");
  EXPECT_EQ(CheckImage(head + R"(<path fill="currentColor" d="M0 0h10v10z"/></svg>)"), "");
  EXPECT_EQ(CheckImage(head + R"(<path fill="#000"/><path fill="#f00"/></svg>)"), "more than one colour");
  EXPECT_EQ(CheckImage(head + "<text>STOP</text></svg>"), "it has <text");
  EXPECT_EQ(CheckImage(head + R"(<image href="a.png"/></svg>)"), "it has <image");
  EXPECT_EQ(CheckImage(head + "<SCRIPT>x()</SCRIPT></svg>"), "it has <script");
  EXPECT_EQ(CheckImage(head + R"x(<path fill="url(#g)"/></svg>)x"), "it has url(");
  EXPECT_EQ(CheckImage(head + R"(<path style="fill:#f00"/></svg>)"), "it has style=");
  EXPECT_EQ(CheckImage("<html></html>"), "no SVG");
  EXPECT_EQ(CheckImage(head + std::string(mowbite_api::kSetupImageBytes, ' ') + "</svg>"),
            "larger than " + std::to_string(mowbite_api::kSetupImageBytes) + " bytes");
}

TEST(Steps, EmergenciesByTheBitsOfTheInterface) {
  EXPECT_EQ(mowbite_setup::EmergencyBit("STOP"), 4u);
  EXPECT_EQ(mowbite_setup::EmergencyBit("LIFT"), 8u);
  EXPECT_EQ(mowbite_setup::EmergencyBit("COLLISION"), 32u);
  EXPECT_EQ(mowbite_setup::EmergencyBit("NONE"), 0u);
}
