#include <gtest/gtest.h>

#include <map>
#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_setup/program.h"

using namespace mowbite_setup;

namespace {

const char* const kList = R"({
  "catalog": 1,
  "steps": [
    {"id": "variant", "stage": "device", "check": "choice", "params": {"name": "variant", "values": ["a", "b"]},
     "value_texts": {"a": {"de": {"name": "A"}, "en": {"name": "A"}}, "b": {"de": {"name": "B"}, "en": {"name": "B"}}},
     "texts": {"de": {"name": "Bauart"}, "en": {"name": "Build"}}},
    {"id": "board", "stage": "device", "check": "board", "params": {"time": 2, "battery": 20}},
    {"id": "stop", "stage": "inputs", "check": "input", "params": {"role": "stop", "time": 10},
     "parts": [{"id": "top"}, {"id": "rear"}], "needs": ["board"]},
    {"id": "lift", "stage": "inputs", "check": "input",
     "params": {"role": "lift", "within": 3, "short_max": 1, "time": 20},
     "parts": [{"id": "bump", "mode": "short"}, {"id": "hold"}]},
    {"id": "left", "stage": "inputs", "check": "input", "params": {"role": "bumper_left", "time": 10}},
    {"id": "imu", "stage": "inputs", "check": "imu", "params": {"time": 10},
     "parts": ["front_up", "left_up", "turn_left", "tilt_stop"]},
    {"id": "ready", "stage": "inputs", "check": "confirm", "confirm": ["raised", "clear"], "skippable": false,
     "texts": {"de": {"name": "Bereit"}, "en": {"name": "Ready"}}},
    {"id": "ntrip", "stage": "position", "check": "done_when", "params": {"condition": "ntrip"}},
    {"id": "fix", "stage": "position", "check": "position", "params": {"time": 2, "timeout": 20}},
    {"id": "length", "stage": "dimensions", "check": "number", "params": {"name": "length", "min": 0.1, "max": 2},
     "texts": {"de": {"name": "Länge"}, "en": {"name": "Length"}}},
    {"id": "dimensions", "stage": "dimensions", "check": "geometry", "params": {"keys": ["track"]}},
    {"id": "station", "stage": "station", "check": "done_when", "params": {"condition": "station"}},
    {"id": "area", "stage": "map", "check": "done_when", "params": {"condition": "area"}},
    {"id": "docking", "stage": "trial", "check": "done_when", "params": {"condition": "dock_test"}},
    {"id": "mow", "stage": "trial", "check": "done_when", "params": {"condition": "mow"}}
  ]})";

// The list with its first two steps alone.
json FirstTwo() {
  json list = json::parse(kList);
  list["steps"] = json::array({list["steps"][0], list["steps"][1]});
  return list;
}

constexpr double kWall = 1700000000.0;

// The program with a clock: every Run() ticks in steps of 0.1 s and feeds hw/status, IMU and position as set.
struct Bench {
  Bench() : program(*ParseSteps(json::parse(kList), nullptr), "tron") {
    capabilities.firmware = {{"chassis", "0.6.36"}, {"cutter", "0.6.34"}};
    program.OnCapabilities(capabilities);
    hardware.battery = true;
    hardware.battery_percentage = 80;
    Run(0.3);
  }

  void Run(double seconds) {
    for (double end = now + seconds - 1e-9; now < end;) {
      now += 0.1;
      if (driver) program.OnHardware(hardware, now);
      if (imu_on) {
        for (int i = 0; i < 5; ++i) program.OnImu(imu, now - 0.1 + 0.02 * (i + 1));
      }
      if (solution >= 0) {
        program.OnFix(solution, accuracy, now - 0.05);
        program.OnFix(solution, accuracy, now);
      }
      program.Tick(now, kWall + now);
    }
  }

  json State() {
    return program.State(now, {});
  }
  json StepState(const std::string& id) {
    const json state = State();
    for (const auto& step : state["steps"]) {
      if (step["id"] == id) return step;
    }
    return nullptr;
  }
  std::optional<Refusal> Start(const std::string& id, json confirm = nullptr) {
    json params = {{"step", id}, {"version", program.version()}};
    if (!confirm.is_null()) params["confirm"] = confirm;
    return program.Start(params, now, kWall + now);
  }
  std::string Refused(const std::optional<Refusal>& refusal) {
    return refusal ? (refusal->field.empty() ? refusal->reason : "param " + refusal->field) : "";
  }
  // The events since the last call, each checked against the catalog.
  std::vector<json> Events() {
    std::vector<json> out;
    for (const Event& event : program.TakeEvents()) {
      json details = event.details;
      for (const char* key : {"id", "t", "type"}) EXPECT_FALSE(details.contains(key)) << event.type << " " << key;
      details["id"] = "x";
      details["t"] = kWall;
      details["type"] = event.type;
      const auto problems = mowbite_api::CheckEvent(event.type, mowbite_api::Json(details));
      EXPECT_TRUE(problems.empty()) << event.type << ": " << (problems.empty() ? "" : problems[0]);
      details.erase("id");
      details.erase("t");
      out.push_back(details);
    }
    return out;
  }
  void PassBoard() {
    ASSERT_EQ(Refused(Start("board")), "");
    Run(2.2);
    ASSERT_EQ(StepState("board")["state"], "ok");
  }
  // Presses the input for a while, with the emergency bit that follows after delay, then lets go.
  void Press(bool Hardware::*input, uint32_t bit, double delay, double hold) {
    hardware.*input = true;
    Run(delay);
    hardware.emergency_reason |= bit;
    Run(hold - delay);
    hardware.*input = false;
    Run(0.2);
  }

  Program program;
  Hardware hardware;
  Capabilities capabilities;
  ImuSample imu{0, 0, 9.81, 0};
  bool driver = true;
  bool imu_on = true;
  int solution = -1;
  double accuracy = 0.02;
  double now = 100;
};

}  // namespace

TEST(Program, AFreshProgramShowsItsSteps) {
  Bench bench;
  const json state = bench.State();
  EXPECT_EQ(state["platform"], "tron");
  EXPECT_EQ(state["catalog"], 1);
  EXPECT_EQ(state["stage"], "device");
  EXPECT_EQ(state["current"], "variant");
  EXPECT_FALSE(state["complete"].get<bool>());
  EXPECT_EQ(state["safety"], (json{{"open", false}, {"date", nullptr}, {"firmware", nullptr},
                                   {"why", "NO_SAFETY_MEASUREMENT"}}));
  ASSERT_EQ(state["stages"].size(), 7u);
  EXPECT_EQ(state["stages"][1]["texts"]["de"]["name"], "Eingänge");
  EXPECT_EQ(state["stages"][0]["state"], "open");
  ASSERT_EQ(state["steps"].size(), 15u);
  const json stop = bench.StepState("stop");
  EXPECT_EQ(stop["state"], "locked");
  EXPECT_EQ(stop["kind"], "measure");
  EXPECT_TRUE(stop["start"].get<bool>());
  EXPECT_TRUE(stop["auto"].get<bool>());
  EXPECT_EQ(stop["parts"].size(), 2u);
  EXPECT_EQ(stop["parts"][0]["state"], "open");
  EXPECT_TRUE(stop["result"].is_null());
  EXPECT_TRUE(stop["motion"].is_null());
  EXPECT_TRUE(stop["image"].is_null());
  const json variant = bench.StepState("variant");
  EXPECT_EQ(variant["inputs"][0]["type"], "choice");
  EXPECT_EQ(variant["inputs"][0]["values"][1]["id"], "b");
  EXPECT_TRUE(variant["inputs"][0]["value"].is_null());
  bench.capabilities.safety_measurement_date = "2026-11-08";
  bench.program.OnCapabilities(bench.capabilities);
  EXPECT_EQ(bench.State()["safety"]["open"], true);
  EXPECT_EQ(bench.State()["safety"]["date"], "2026-11-08");
}

TEST(Program, ImagesComeWithTheirHash) {
  json list = json::parse(kList);
  list["steps"][1]["image"] = "board.svg";
  list["steps"][1]["accent"] = "board-accent.svg";
  Program program(*ParseSteps(list, nullptr), "tron");
  const json with = program.State(0, {{"board.svg", {"abc", 120}}, {"board-accent.svg", {"def", 80}}});
  EXPECT_EQ(with["steps"][1]["image"],
            (json{{"name", "board.svg"}, {"sha256", "abc"}, {"bytes", 120}, {"type", "image/svg+xml"},
                  {"accent", {{"name", "board-accent.svg"}, {"sha256", "def"}, {"bytes", 80}}}}));
  EXPECT_TRUE(program.State(0, {})["steps"][1]["image"].is_null());
  EXPECT_TRUE(program.State(0, {{"board.svg", {"abc", 120}}})["steps"][1]["image"]["accent"].is_null());
}

TEST(Program, TheBoardCheckPassesWithAQuietBoard) {
  Bench bench;
  ASSERT_EQ(bench.Refused(bench.Start("board")), "");
  bench.Run(1.0);
  json step = bench.StepState("board");
  EXPECT_EQ(step["state"], "running");
  EXPECT_EQ(bench.State()["current"], "board");
  EXPECT_TRUE(step["values"][0]["ok"].is_null());
  EXPECT_TRUE(step["progress"].is_null());
  EXPECT_DOUBLE_EQ(step["ends_at"].get<double>(), std::floor(kWall + 100.3 + 2));
  bench.Run(1.2);
  step = bench.StepState("board");
  EXPECT_EQ(step["state"], "ok");
  EXPECT_EQ(step["attempt"], 1);
  EXPECT_EQ(step["result"]["ok"], true);
  EXPECT_EQ(step["result"]["firmware"], "chassis 0.6.36, cutter 0.6.34");
  std::map<std::string, json> values;
  for (const auto& value : step["values"]) values[value["id"]] = value;
  EXPECT_NEAR(values["status_rate"]["value"].get<double>(), 10.0, 0.6);
  EXPECT_EQ(values["firmware"]["value"], "chassis 0.6.36, cutter 0.6.34");
  EXPECT_EQ(values["battery"]["value"], 80.0);
  EXPECT_NEAR(values["gravity"]["value"].get<double>(), 9.81, 0.01);
  for (const auto& [id, value] : values) EXPECT_EQ(value["ok"], true) << id;
  const auto events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0], (json{{"step", "board"}, {"state", "ok"}, {"attempt", 1}, {"type", "SETUP_STEP"}}));
  EXPECT_TRUE(bench.program.TakeChanged());
  EXPECT_EQ(bench.StepState("stop")["state"], "open");
}

TEST(Program, TheBoardCheckNamesWhatIsWrong) {
  struct Case {
    const char* name;
    void (*change)(Bench&);
    const char* reason;
    const char* field;
    const char* value;
  };
  const Case cases[] = {
      {"a pressed button", [](Bench& b) { b.hardware.stop = true; }, "LIMIT", "value_id", "inputs"},
      {"a motor controller down", [](Bench& b) { b.hardware.motors_down = {"left"}; }, "LIMIT", "value_id", "motors"},
      {"a low battery", [](Bench& b) { b.hardware.battery_percentage = 10; }, "LIMIT", "value_id", "battery"},
      {"no reading of the battery", [](Bench& b) { b.hardware.battery = false; }, "LIMIT", "value_id", "battery"},
      {"errors on the link", [](Bench& b) { b.hardware.link_errors = 3; }, "LIMIT", "value_id", "link_errors"},
      {"firmware unknown", [](Bench& b) {
         b.capabilities.firmware[1].second = "";
         b.program.OnCapabilities(b.capabilities);
       }, "LIMIT", "value_id", "firmware"},
      {"firmware incompatible", [](Bench& b) { b.hardware.emergency_reason = 4096; }, "LIMIT", "value_id", "firmware"},
      {"a tilted or wrong IMU", [](Bench& b) { b.imu.az = 5.0; }, "LIMIT", "value_id", "gravity"},
      {"no IMU", [](Bench& b) { b.imu_on = false; }, "NO_DATA", "source", "imu/data"},
      {"the driver gone", [](Bench& b) { b.driver = false; }, "NO_DATA", "source", "hw/status"},
  };
  for (const Case& c : cases) {
    Bench bench;
    ASSERT_EQ(bench.Refused(bench.Start("board")), "") << c.name;
    c.change(bench);
    bench.Run(2.2);
    const json step = bench.StepState("board");
    EXPECT_EQ(step["state"], "failed") << c.name;
    EXPECT_EQ(step["result"]["reason"], c.reason) << c.name;
    const auto events = bench.Events();
    ASSERT_EQ(events.size(), 1u) << c.name;
    EXPECT_EQ(events[0]["reason"], c.reason) << c.name;
    EXPECT_EQ(events[0][c.field], c.value) << c.name;
    EXPECT_EQ(events[0]["severity"], "warning") << c.name;
    EXPECT_EQ(events[0]["state"], "failed") << c.name;
  }
}

TEST(Program, EveryStopButtonHasToBringTheEmergency) {
  Bench bench;
  bench.PassBoard();
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  json step = bench.StepState("stop");
  EXPECT_EQ(step["progress"], (json{{"n", 1}, {"of", 2}}));
  EXPECT_EQ(step["parts"][0]["state"], "waiting");
  EXPECT_EQ(step["parts"][1]["state"], "open");
  bench.Press(&Hardware::stop, 4, 0.2, 0.6);
  step = bench.StepState("stop");
  EXPECT_EQ(step["parts"][0]["state"], "ok");
  EXPECT_EQ(step["parts"][1]["state"], "waiting");
  EXPECT_EQ(step["progress"]["n"], 2);
  // The emergency of the first button is still on: the second press counts only after its release.
  EXPECT_EQ(step["blocked"][0]["reason"], "EMERGENCY");
  bench.Press(&Hardware::stop, 4, 0.0, 0.5);
  EXPECT_EQ(bench.StepState("stop")["parts"][1]["state"], "waiting");
  bench.hardware.emergency_reason = 0;
  bench.Run(0.3);
  EXPECT_TRUE(bench.StepState("stop")["blocked"].empty());
  bench.Press(&Hardware::stop, 4, 0.1, 0.5);
  step = bench.StepState("stop");
  EXPECT_EQ(step["state"], "ok");
  ASSERT_EQ(step["values"].size(), 2u);
  EXPECT_NEAR(step["values"][0]["value"].get<double>(), 0.2, 0.11);
  EXPECT_EQ(step["values"][0]["max"], 1.0);
  EXPECT_EQ(step["values"][0]["ok"], true);
  EXPECT_EQ(step["values"][0]["id"], "top");
  EXPECT_EQ(step["parts"][1]["state"], "ok");
  EXPECT_TRUE(step["progress"].is_null());
}

TEST(Program, AnInputWithoutEmergencyFails) {
  Bench bench;
  bench.PassBoard();
  bench.Events();
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.hardware.stop = true;
  bench.Run(1.3);
  const json step = bench.StepState("stop");
  EXPECT_EQ(step["state"], "failed");
  EXPECT_EQ(step["result"]["reason"], "NO_EMERGENCY");
  EXPECT_EQ(step["result"]["part"], "top");
  EXPECT_EQ(step["parts"][0]["state"], "failed");
  const auto events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["part"], "top");
  EXPECT_EQ(events[0]["limit"], 1.0);
}

TEST(Program, NothingSeenOrAnotherInputEndsThePartInTime) {
  Bench bench;
  bench.PassBoard();
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(10.3);
  EXPECT_EQ(bench.StepState("stop")["result"]["reason"], "NOT_SEEN");
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.hardware.lift = true;
  bench.Run(10.3);
  const json step = bench.StepState("stop");
  EXPECT_EQ(step["result"]["reason"], "WRONG_INPUT");
  EXPECT_EQ(step["attempt"], 2);
  bench.hardware.lift = false;
  // The other side of a split bumper counts as another input.
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.hardware.bumper = bench.hardware.bumper_right = true;
  bench.Run(10.3);
  EXPECT_EQ(bench.StepState("left")["result"]["reason"], "WRONG_INPUT");
  EXPECT_EQ(bench.StepState("left")["result"]["message"], "The input bumper_right switched, not bumper_left");
}

TEST(Program, AnEmergencyOfAnotherInputDoesNotCountForThisOne) {
  Bench bench;
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  // The right side brings the emergency, which stays latched after its release.
  bench.hardware.bumper = bench.hardware.bumper_right = true;
  bench.hardware.emergency_reason = EmergencyBit("COLLISION");
  bench.Run(0.3);
  bench.hardware.bumper = bench.hardware.bumper_right = false;
  bench.Run(0.5);
  EXPECT_EQ(bench.StepState("left")["blocked"][0]["reason"], "EMERGENCY");
  // A press of the left side shows nothing now: the part waits for the release of the emergency.
  bench.hardware.bumper = bench.hardware.bumper_left = true;
  bench.Run(0.3);
  bench.hardware.bumper = bench.hardware.bumper_left = false;
  bench.Run(0.3);
  EXPECT_EQ(bench.StepState("left")["state"], "running");
  EXPECT_EQ(bench.StepState("left")["parts"][0]["state"], "waiting");
  // Released, the left side has to bring the emergency itself.
  bench.hardware.emergency_reason = 0;
  bench.Run(0.3);
  bench.hardware.bumper = bench.hardware.bumper_left = true;
  bench.hardware.emergency_reason = EmergencyBit("COLLISION");
  bench.Run(0.3);
  bench.hardware.bumper = bench.hardware.bumper_left = false;
  bench.Run(0.3);
  EXPECT_EQ(bench.StepState("left")["state"], "ok");
}

TEST(Program, AnInputThatIsHeldAlreadyWaitsForItsRelease) {
  Bench bench;
  bench.PassBoard();
  bench.hardware.stop = true;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.5);
  EXPECT_EQ(bench.StepState("stop")["blocked"][0]["reason"], "INPUT_ACTIVE");
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "waiting");
  bench.hardware.stop = false;
  bench.Run(0.3);
  EXPECT_TRUE(bench.StepState("stop")["blocked"].empty());
}

TEST(Program, APressedInputTheBoardHoldsShowsItsEmergency) {
  Bench bench;
  bench.PassBoard();
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.hardware.stop = true;
  bench.Run(0.1);
  bench.hardware.emergency_reason = EmergencyBit("STOP");
  bench.Run(0.3);
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "waiting");
  EXPECT_EQ(bench.StepState("stop")["blocked"][0]["reason"], "EMERGENCY");
  // The release takes the input back with the emergency.
  bench.hardware.stop = false;
  bench.hardware.emergency_reason = 0;
  bench.Run(0.3);
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "ok");
  EXPECT_TRUE(bench.StepState("stop")["blocked"].empty());
}

TEST(Program, AShortLiftMustNotStop) {
  Bench bench;
  ASSERT_EQ(bench.Refused(bench.Start("lift")), "");
  bench.Run(0.2);
  // Held longer than short_max: it does not count, and the emergency it brought has to go first.
  bench.Press(&Hardware::lift, 8, 1.2, 1.5);
  EXPECT_EQ(bench.StepState("lift")["parts"][0]["state"], "waiting");
  EXPECT_EQ(bench.StepState("lift")["blocked"][0]["reason"], "EMERGENCY");
  bench.hardware.emergency_reason = 0;
  bench.Run(0.3);
  bench.hardware.lift = true;
  bench.Run(0.4);
  bench.hardware.lift = false;
  bench.Run(2.5);
  EXPECT_EQ(bench.StepState("lift")["parts"][0]["state"], "waiting");
  bench.Run(1.0);
  EXPECT_EQ(bench.StepState("lift")["parts"][0]["state"], "ok");
  bench.Press(&Hardware::lift, 8, 2.5, 3.0);
  const json step = bench.StepState("lift");
  EXPECT_EQ(step["state"], "ok");
  ASSERT_EQ(step["values"].size(), 1u);
  EXPECT_EQ(step["values"][0]["id"], "hold");
  EXPECT_NEAR(step["values"][0]["value"].get<double>(), 2.5, 0.11);
  EXPECT_EQ(step["values"][0]["max"], 3.0);
}

TEST(Program, AShortLiftThatStopsFails) {
  Bench bench;
  ASSERT_EQ(bench.Refused(bench.Start("lift")), "");
  bench.Run(0.2);
  bench.hardware.lift = true;
  bench.Run(0.3);
  bench.hardware.lift = false;
  bench.Run(0.5);
  bench.hardware.emergency_reason = 8;
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("lift")["result"]["reason"], "STOPPED_EARLY");
  EXPECT_EQ(bench.StepState("lift")["result"]["part"], "bump");
}

TEST(Program, TheImuShowsEveryMoveWithItsSign) {
  Bench bench;
  ASSERT_EQ(bench.Refused(bench.Start("imu")), "");
  bench.Run(0.8);
  const double g = 9.81;
  const auto tilt = [&](double pitch_deg, double roll_deg) {
    bench.imu.ax = g * std::sin(pitch_deg * M_PI / 180);
    bench.imu.ay = g * std::sin(roll_deg * M_PI / 180);
    bench.imu.az = g * std::cos(std::max(std::abs(pitch_deg), std::abs(roll_deg)) * M_PI / 180);
  };
  tilt(15, 0);
  bench.Run(1.5);
  json step = bench.StepState("imu");
  EXPECT_EQ(step["parts"][0]["state"], "ok");
  EXPECT_NEAR(step["values"][0]["value"].get<double>(), 10.0, 3.0);
  EXPECT_EQ(step["values"][0]["min"], 10.0);
  // Still up: the next part waits until the mower stands level again.
  EXPECT_EQ(step["parts"][1]["state"], "waiting");
  EXPECT_EQ(step["blocked"][0]["reason"], "INPUT_ACTIVE");
  tilt(0, 0);
  bench.Run(1.5);
  tilt(0, 15);
  bench.Run(1.5);
  EXPECT_EQ(bench.StepState("imu")["parts"][1]["state"], "ok");
  tilt(0, 0);
  bench.Run(1.5);
  bench.imu.gz = 0.5;
  bench.Run(1.3);
  bench.imu.gz = 0;
  step = bench.StepState("imu");
  EXPECT_EQ(step["parts"][2]["state"], "ok");
  EXPECT_NEAR(step["values"][2]["value"].get<double>(), 30.0, 3.0);
  EXPECT_EQ(step["parts"][3]["state"], "waiting");
  bench.hardware.emergency_reason = 8192;
  bench.Run(0.3);
  EXPECT_EQ(bench.StepState("imu")["state"], "ok");
}

TEST(Program, AnImuTheWrongWayRoundFails) {
  {
    Bench bench;
    ASSERT_EQ(bench.Refused(bench.Start("imu")), "");
    bench.Run(0.8);
    bench.imu.ax = -9.81 * std::sin(15 * M_PI / 180);
    bench.Run(1.5);
    const json step = bench.StepState("imu");
    EXPECT_EQ(step["state"], "failed");
    EXPECT_EQ(step["result"]["reason"], "WRONG_SIGN");
    EXPECT_EQ(step["result"]["part"], "front_up");
    EXPECT_LT(step["values"][0]["value"].get<double>(), -9.0);
  }
  {
    Bench bench;
    bench.imu_on = false;
    ASSERT_EQ(bench.Refused(bench.Start("imu")), "");
    bench.Run(3.3);
    EXPECT_EQ(bench.StepState("imu")["result"]["reason"], "NO_DATA");
  }
  {
    Bench bench;
    ASSERT_EQ(bench.Refused(bench.Start("imu")), "");
    bench.Run(11.0);
    EXPECT_EQ(bench.StepState("imu")["result"]["reason"], "NOT_SEEN");
  }
}

TEST(Program, ThePositionHasToHold) {
  Bench bench;
  EXPECT_EQ(bench.StepState("fix")["blocked"][0]["reason"], "NO_DATA");
  bench.solution = 3;
  bench.Run(6.0);
  EXPECT_TRUE(bench.StepState("fix")["blocked"].empty());
  ASSERT_EQ(bench.Refused(bench.Start("fix")), "");
  bench.Run(3.0);
  json step = bench.StepState("fix");
  EXPECT_EQ(step["state"], "running");
  EXPECT_EQ(step["values"][0]["value"], "float");
  EXPECT_EQ(step["values"][0]["ok"], false);
  EXPECT_EQ(step["values"][2]["ok"], true);
  bench.solution = 4;
  bench.Run(1.5);
  EXPECT_EQ(bench.StepState("fix")["state"], "running");
  bench.Run(0.8);
  step = bench.StepState("fix");
  EXPECT_EQ(step["state"], "ok");
  EXPECT_EQ(step["values"][0]["value"], "fixed");
  EXPECT_NEAR(step["values"][2]["value"].get<double>(), 20.0, 1.0);

  ASSERT_EQ(bench.Refused(bench.Start("fix")), "");
  bench.solution = 1;
  bench.Run(20.3);
  EXPECT_EQ(bench.StepState("fix")["result"]["reason"], "NO_RTK");
  bench.solution = 4;
  bench.accuracy = 0.2;
  ASSERT_EQ(bench.Refused(bench.Start("fix")), "");
  bench.Run(20.3);
  EXPECT_EQ(bench.StepState("fix")["result"]["reason"], "LIMIT");
  bench.solution = -1;
  ASSERT_EQ(bench.Refused(bench.Start("fix")), "");
  bench.Run(6.0);
  EXPECT_EQ(bench.StepState("fix")["result"]["reason"], "NO_DATA");
}

TEST(Program, StepsTheOsSeesEndByThemselves) {
  Bench bench;
  World world;
  world.state = "IDLE";
  world.ntrip = true;
  bench.program.OnWorld(world);
  bench.Run(0.1);
  EXPECT_EQ(bench.StepState("ntrip")["state"], "ok");
  EXPECT_EQ(bench.StepState("ntrip")["attempt"], 1);
  world.docking_stations = 1;
  bench.program.OnWorld(world);
  bench.Run(0.1);
  EXPECT_EQ(bench.StepState("station")["state"], "open");
  bench.hardware.dock_contact = true;
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("station")["state"], "ok");
  world.areas = 1;
  bench.program.OnWorld(world);
  bench.Run(0.1);
  EXPECT_EQ(bench.StepState("area")["state"], "open");
  world.mowing_allowed = true;
  bench.program.OnWorld(world);
  bench.Run(0.1);
  EXPECT_EQ(bench.StepState("area")["state"], "ok");
  world.confirmed = {"track", "antenna_x"};
  bench.program.OnWorld(world);
  bench.Run(0.1);
  EXPECT_EQ(bench.StepState("dimensions")["state"], "ok");
  // Pushed onto the station by hand: no docking test.
  bench.Run(0.1);
  EXPECT_EQ(bench.StepState("docking")["state"], "open");
  const auto drive = [&](std::initializer_list<const char*> states) {
    for (const char* state : states) {
      world.state = state;
      bench.hardware.dock_contact = false;
      bench.program.OnWorld(world);
      bench.Run(0.5);
    }
    bench.hardware.dock_contact = true;
    bench.Run(0.2);
  };
  drive({"UNDOCKING", "DOCKING", "IDLE"});
  EXPECT_EQ(bench.StepState("docking")["state"], "ok");
  EXPECT_EQ(bench.StepState("mow")["state"], "open");
  drive({"UNDOCKING", "MOWING", "DOCKING", "IDLE"});
  EXPECT_EQ(bench.StepState("mow")["state"], "ok");
}

TEST(Program, AMowerThatDoesNotComeBackByItselfDidNotDock) {
  Bench bench;
  World world;
  for (const char* state : {"UNDOCKING", "MOWING", "DOCKING", "IDLE"}) {
    world.state = state;
    bench.program.OnWorld(world);
    bench.Run(0.5);
  }
  bench.Run(4.0);
  bench.hardware.dock_contact = true;
  bench.Run(0.5);
  EXPECT_EQ(bench.StepState("mow")["state"], "open");
  EXPECT_EQ(bench.StepState("docking")["state"], "open");
}

TEST(Program, AnswersAndConfirmations) {
  Bench bench;
  const auto answer = [&](const std::string& step, json values) {
    return bench.Refused(
        bench.program.Answer({{"step", step}, {"version", bench.program.version()}, {"values", values}}, kWall));
  };
  EXPECT_EQ(answer("variant", {{"variant", "c"}}), "INVALID_VALUE");
  EXPECT_EQ(answer("variant", {{"other", "a"}}), "INVALID_VALUE");
  EXPECT_EQ(answer("variant", {{"variant", "a"}, {"more", 1}}), "INVALID_VALUE");
  EXPECT_EQ(answer("board", {{"x", 1}}), "WRONG_METHOD");
  EXPECT_EQ(answer("variant", {{"variant", "b"}}), "");
  json step = bench.StepState("variant");
  EXPECT_EQ(step["state"], "ok");
  EXPECT_EQ(step["inputs"][0]["value"], "b");
  EXPECT_EQ(answer("length", {{"length", 3}}), "INVALID_VALUE");
  EXPECT_EQ(answer("length", {{"length", "1"}}), "INVALID_VALUE");
  EXPECT_EQ(answer("length", {{"length", 0.7}}), "");
  EXPECT_EQ(bench.StepState("length")["inputs"][0]["value"], 0.7);
  EXPECT_EQ(bench.StepState("length")["inputs"][0]["max"], 2.0);

  EXPECT_EQ(bench.Refused(bench.Start("ready")), "CONFIRM_MISSING");
  const auto refusal = bench.Start("ready", json::array({"raised"}));
  ASSERT_TRUE(refusal.has_value());
  EXPECT_EQ(refusal->data["missing"], json::array({"clear"}));
  EXPECT_EQ(bench.Refused(bench.Start("ready", json::array({"clear", "raised"}))), "");
  EXPECT_EQ(bench.StepState("ready")["state"], "ok");
  EXPECT_EQ(bench.StepState("ready")["confirm"][1]["id"], "clear");
}

TEST(Program, WhatIsRefused) {
  Bench bench;
  const int version = bench.program.version();
  const auto start = [&](json params) { return bench.Refused(bench.program.Start(params, bench.now, kWall)); };
  EXPECT_EQ(start({{"step", "nothing"}, {"version", version}}), "UNKNOWN_STEP");
  EXPECT_EQ(start({{"step", "board"}, {"version", version - 1}}), "VERSION");
  EXPECT_EQ(start({{"step", "board"}}), "param version");
  EXPECT_EQ(start({{"version", version}}), "param step");
  EXPECT_EQ(start(json::array()), "param params");
  EXPECT_EQ(start({{"step", "board"}, {"version", version}, {"fast", true}}), "param fast");
  EXPECT_EQ(start({{"step", "board"}, {"version", version}, {"confirm", "raised"}}), "param confirm");
  EXPECT_EQ(start({{"step", "variant"}, {"version", version}}), "WRONG_METHOD");
  EXPECT_EQ(start({{"step", "ntrip"}, {"version", version}}), "WRONG_METHOD");
  const auto locked = bench.program.Start({{"step", "stop"}, {"version", version}}, bench.now, kWall);
  ASSERT_TRUE(locked.has_value());
  EXPECT_EQ(locked->reason, "LOCKED");
  EXPECT_EQ(locked->data["needs"], json::array({"board"}));

  World world;
  world.state = "MOWING";
  bench.program.OnWorld(world);
  EXPECT_EQ(bench.Refused(bench.Start("board")), "BUSY");
  EXPECT_EQ(bench.StepState("board")["blocked"][0]["reason"], "BUSY");
  // What only asks the user goes on while the mower works.
  EXPECT_EQ(bench.Refused(bench.Start("ready", json::array({"raised", "clear"}))), "");
  world.state = "IDLE";
  bench.program.OnWorld(world);

  bench.driver = false;
  bench.Run(1.5);
  EXPECT_EQ(bench.Refused(bench.Start("board")), "DRIVER_MISSING");
  EXPECT_EQ(bench.StepState("board")["blocked"][0]["reason"], "DRIVER_MISSING");
  // What blocks changed without a change of a step: the text tells the node to send the state again.
  EXPECT_EQ(bench.program.BlockedNow(bench.now),
            "board:DRIVER_MISSING;stop:DRIVER_MISSING;lift:DRIVER_MISSING;left:DRIVER_MISSING;fix:NO_DATA;");
  bench.driver = true;
  bench.Run(0.2);
  EXPECT_EQ(bench.program.BlockedNow(bench.now), "fix:NO_DATA;");

  ASSERT_EQ(bench.Refused(bench.Start("board")), "");
  const auto busy = bench.Start("lift");
  ASSERT_TRUE(busy.has_value());
  EXPECT_EQ(busy->reason, "BUSY");
  EXPECT_EQ(busy->data["step"], "board");
  EXPECT_EQ(bench.StepState("lift")["blocked"][0]["reason"], "BUSY");

  bench.program.SetWritable(false);
  EXPECT_EQ(bench.Refused(bench.Start("lift")), "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(bench.StepState("lift")["blocked"][0]["reason"], "STORAGE_NOT_WRITABLE");
  for (const auto& group : {"SETUP_REFUSED", "SETUP_BLOCKED"}) {
    ASSERT_NE(mowbite_api::FindReasonGroup(group), nullptr);
  }
  for (const char* reason : {"UNKNOWN_STEP", "VERSION", "LOCKED", "BUSY", "CONFIRM_MISSING", "NOT_SKIPPABLE",
                             "INVALID_VALUE", "WRONG_METHOD", "DRIVER_MISSING", "STORAGE_NOT_WRITABLE"}) {
    bool found = false;
    for (const auto& entry : mowbite_api::FindReasonGroup("SETUP_REFUSED")->reasons) found |= entry.code == reason;
    EXPECT_TRUE(found) << reason;
  }
}

TEST(Program, SkippingAbortingAndResetting) {
  Bench bench;
  const auto skip = [&](const std::string& step, const std::string& reason, json note = nullptr) {
    json params = {{"step", step}, {"version", bench.program.version()}, {"reason", reason}};
    if (!note.is_null()) params["note"] = note;
    return bench.Refused(bench.program.Skip(params, kWall));
  };
  EXPECT_EQ(skip("ready", "later"), "NOT_SKIPPABLE");
  EXPECT_EQ(skip("board", "bored"), "INVALID_VALUE");
  EXPECT_EQ(skip("board", "other"), "INVALID_VALUE");
  EXPECT_EQ(skip("board", "later", std::string(201, 'x')), "INVALID_VALUE");
  EXPECT_EQ(skip("board", "other", "the board is new"), "");
  json step = bench.StepState("board");
  EXPECT_EQ(step["state"], "skipped");
  EXPECT_EQ(step["result"]["skip"], "other");
  EXPECT_EQ(step["result"]["note"], "the board is new");
  EXPECT_EQ(step["result"]["ok"], false);
  auto events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0], (json{{"step", "board"}, {"state", "skipped"}, {"attempt", 0}, {"skip", "other"},
                             {"note", "the board is new"}, {"type", "SETUP_STEP"}}));
  // A skipped step does not open what needs it.
  EXPECT_EQ(bench.StepState("stop")["state"], "locked");
  EXPECT_EQ(skip("variant", "later"), "");
  events = bench.Events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1], (json{{"what", "stage_done"}, {"stage", "device"}, {"type", "SETUP"}}));
  EXPECT_EQ(bench.State()["stages"][0]["state"], "skipped");
  EXPECT_EQ(bench.State()["stage"], "inputs");

  // A skipped step can still be done.
  ASSERT_EQ(bench.Refused(bench.Start("board")), "");
  EXPECT_EQ(skip("board", "later"), "BUSY");
  bench.Run(0.5);
  EXPECT_EQ(bench.Refused(bench.program.Abort({{"step", "lift"}}, bench.now, kWall)), "");
  EXPECT_EQ(bench.StepState("board")["state"], "running");
  EXPECT_EQ(bench.Refused(bench.program.Abort(nullptr, bench.now, kWall)), "");
  step = bench.StepState("board");
  EXPECT_EQ(step["state"], "open");
  EXPECT_EQ(step["result"]["reason"], "STOPPED");
  events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["reason"], "STOPPED");
  EXPECT_EQ(events[0]["severity"], "info");
  EXPECT_EQ(bench.Refused(bench.program.Abort(json::object(), bench.now, kWall)), "");
  EXPECT_TRUE(bench.Events().empty());

  bench.PassBoard();
  EXPECT_EQ(bench.Refused(bench.program.Reset({{"version", bench.program.version() + 1}})), "VERSION");
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  EXPECT_EQ(bench.Refused(bench.program.Reset({{"version", bench.program.version()}})), "");
  EXPECT_FALSE(bench.program.running());
  const json after = bench.State();
  for (const auto& entry : after["steps"]) {
    EXPECT_TRUE(entry["state"] == "open" || entry["state"] == "locked") << entry["id"];
    EXPECT_TRUE(entry["result"].is_null());
    EXPECT_EQ(entry["attempt"], 0);
  }
  events = bench.Events();
  EXPECT_EQ(events.back(), (json{{"what", "reset"}, {"type", "SETUP"}}));
}

TEST(Program, EveryStepDoneCompletesTheProgram) {
  Program program(*ParseSteps(FirstTwo(), nullptr), "tron");
  ASSERT_FALSE(program.Answer({{"step", "variant"}, {"version", program.version()}, {"values", {{"variant", "a"}}}},
                              kWall));
  ASSERT_FALSE(program.Skip({{"step", "board"}, {"version", program.version()}, {"reason", "later"}}, kWall));
  const auto events = program.TakeEvents();
  ASSERT_EQ(events.size(), 4u);
  EXPECT_EQ(events[2].details, (json{{"what", "stage_done"}, {"stage", "device"}}));
  EXPECT_EQ(events[3].details, (json{{"what", "complete"}}));
  const json state = program.State(0, {});
  EXPECT_TRUE(state["complete"].get<bool>());
  EXPECT_TRUE(state["stage"].is_null());
  EXPECT_TRUE(state["current"].is_null());
}

TEST(Program, ARecordWithBytesThatAreNoTextIsWritten) {
  Bench bench;
  bench.capabilities.firmware = {{"main", "1.\xff"}};
  bench.program.OnCapabilities(bench.capabilities);
  const json params = {{"step", "variant"}, {"version", bench.program.version()}, {"values", {{"variant", "a"}}}};
  ASSERT_FALSE(bench.program.Answer(params, kWall));
  std::string text;
  ASSERT_NO_THROW(text = bench.program.Text());
  std::string why;
  Program next(*ParseSteps(json::parse(kList), nullptr), "tron");
  EXPECT_TRUE(next.Load(text, kWall, &why)) << why;
}

TEST(Program, TheRecordOutlivesARestart) {
  Bench bench;
  bench.PassBoard();
  ASSERT_FALSE(
      bench.program.Skip({{"step", "lift"}, {"version", bench.program.version()}, {"reason", "later"}}, kWall));
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.3);
  const std::string text = bench.program.Text();
  const int version = bench.program.version();
  const json file = json::parse(text);
  EXPECT_EQ(file["schema"], 1);
  EXPECT_EQ(file["platform"], "tron");
  EXPECT_EQ(file["steps"]["stop"]["state"], "running");

  Program next(*ParseSteps(json::parse(kList), nullptr), "tron");
  std::string why;
  ASSERT_TRUE(next.Load(text, kWall + 500, &why)) << why;
  EXPECT_FALSE(next.running());
  EXPECT_GT(next.version(), version);
  EXPECT_TRUE(next.TakeChanged());
  const json state = next.State(0, {});
  EXPECT_EQ(state["steps"][1]["state"], "ok");
  EXPECT_EQ(state["steps"][1]["values"], bench.StepState("board")["values"]);
  EXPECT_EQ(state["steps"][3]["state"], "skipped");
  // The step that ran is open again and never goes on by itself.
  EXPECT_EQ(state["steps"][2]["state"], "open");
  EXPECT_EQ(state["steps"][2]["result"]["reason"], "INTERRUPTED");
  const auto events = next.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "INTERRUPTED");
  EXPECT_EQ(events[0].details["state"], "failed");

  Program other(*ParseSteps(json::parse(kList), nullptr), "nx60i");
  EXPECT_FALSE(other.Load(text, kWall, &why));
  EXPECT_EQ(why, "it is of another platform");
  EXPECT_FALSE(other.Load("{", kWall, &why));
  EXPECT_FALSE(other.Load(R"({"schema": 2, "platform": "nx60i", "steps": {}})", kWall, &why));
  EXPECT_EQ(why, "it is of another schema");
  // Values of another type are a file that is not used, nothing throws.
  EXPECT_FALSE(other.Load(R"({"schema": "1", "platform": "nx60i", "steps": {}})", kWall, &why));
  EXPECT_EQ(why, "it is of another schema");
  EXPECT_FALSE(other.Load(R"({"schema": 1, "platform": "nx60i", "steps": {"board": {"state": 3}}})", kWall, &why));
  EXPECT_EQ(why, "board has no state of a step");
  ASSERT_TRUE(other.Load(R"({"schema": 1, "platform": "nx60i", "steps": {"board": {"attempt": 99999999999}}})", kWall,
                         &why));
  EXPECT_EQ(other.State(0, {})["steps"][1]["attempt"], 1000000000);
  // A list with other steps keeps what it still knows.
  Program less(*ParseSteps(FirstTwo(), nullptr), "tron");
  ASSERT_TRUE(less.Load(text, kWall, &why)) << why;
  EXPECT_EQ(less.State(0, {})["steps"][1]["state"], "ok");
}
