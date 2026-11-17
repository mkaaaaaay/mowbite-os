#include <gtest/gtest.h>

#include <map>
#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_hold/lock.h"
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
  explicit Bench(const char* list = kList) : program(*ParseSteps(json::parse(list), nullptr), "tron") {
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
      if (const auto release = program.TakeRelease()) releases.emplace_back(now, *release);
      if (!node) continue;
      // What the node does after a tick: the record, the parameter file, then a restart.
      if (program.TakeChanged()) saved = program.Text();
      if (const auto layer = program.TakeLayer()) {
        if (layer_ok) layer_text = *layer;
        program.LayerWritten(layer_ok, now, kWall + now);
      }
      if (program.TakeRestart(now)) ++restarts;
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

  // hw/inputs with this line switched or free.
  void SetLine(const std::string& name, bool active) {
    for (Line& line : lines) {
      if (line.name == name) line.active = active;
    }
    program.OnInputs(lines, learning);
  }

  Program program;
  Hardware hardware;
  std::vector<Line> lines;
  bool learning = false;
  // The releases the program asked for, with the time of each.
  std::vector<std::pair<double, ReleaseRequest>> releases;
  // With node: the record and the parameter file as written last, and the restarts asked for.
  bool node = false;
  bool layer_ok = true;
  std::string saved;
  std::string layer_text;
  int restarts = 0;
  Capabilities capabilities;
  ImuSample imu{0, 0, 9.81, 0};
  bool driver = true;
  bool imu_on = true;
  int solution = -1;
  double accuracy = 0.02;
  double now = 100;
};

Line MakeLine(const std::string& name, const std::string& role, const std::string& reason, bool held = false) {
  Line line;
  line.name = name;
  line.role = role;
  line.reason = reason;
  line.assigned = true;
  line.held = held;
  return line;
}

// What the program's own release of an emergency needs: the mower in IDLE and a driver that counts its latches.
struct ReleaseBench : Bench {
  ReleaseBench() {
    World world;
    world.state = "IDLE";
    program.OnWorld(world);
    PassBoard();
  }

  // The driver latches these bits and counts it.
  void Latch(uint32_t bits) {
    hardware.emergency_reason = bits | EmergencyBit("LATCH");
    ++hardware.latch_count;
  }
  // The input goes on, its emergency follows 0.1 s later. The time the program saw the input.
  double Switch(bool Hardware::*input, uint32_t bit) {
    hardware.*input = true;
    Run(0.1);
    const double edge = now;
    Latch(bit);
    Run(0.1);
    return edge;
  }

  const uint32_t stop = EmergencyBit("STOP");
};

const char* const kAssignList = R"({
  "catalog": 1,
  "restart": "self",
  "steps": [
    {"id": "stop", "stage": "inputs", "check": "input", "params": {"role": "stop", "time": 10}},
    {"id": "assign", "stage": "inputs", "check": "inputs_assign",
     "params": {"node": "driver", "quiet": 3, "time": 10, "restart_time": 20},
     "parts": [{"id": "stop_top", "role": "stop", "delay_ms": 10},
               {"id": "lift_left", "role": "lift", "delay_ms": 2500}]}
  ]})";

// A mower whose driver learns its inputs, with what the node does around the program and the restarts of the OS.
struct AssignBench : Bench {
  AssignBench() : Bench(kAssignList) {
    node = true;
    world.state = "IDLE";
    program.OnWorld(world);
    program.SetLayer(Layer(), {});
    lines = {MakeLine("GPIO1", "stop", "stop"), MakeLine("GPIO10", "lift", "lift")};
    program.OnInputs(lines);
    Run(0.2);
  }

  // The OS started again: a new program over the record and the parameter file, the driver with these lines.
  void Restart(bool learns, std::vector<Line> with) {
    program = Program(*ParseSteps(json::parse(kAssignList), nullptr), "tron");
    std::string why;
    if (!saved.empty()) {
      ASSERT_TRUE(program.Load(saved, kWall + now, &why)) << why;
    }
    Layer layer;
    ASSERT_TRUE(layer.Parse(layer_text, &why)) << why;
    program.SetLayer(layer, shadowed);
    program.OnCapabilities(capabilities);
    program.OnWorld(world);
    learning = learns;
    hardware.emergency_reason = learns ? EmergencyBit("INPUTS_LEARNING") | EmergencyBit("LATCH") : 0;
    lines = std::move(with);
    program.OnInputs(lines, learning);
    Run(0.2);
  }
  // The candidate lines of a driver that learns: none assigned, GPIO12 high at rest.
  static std::vector<Line> Candidates() {
    std::vector<Line> lines;
    for (const char* name : {"GPIO1", "GPIO5", "GPIO10", "GPIO12"}) {
      Line line;
      line.name = name;
      line.active = line.name == "GPIO12";
      lines.push_back(line);
    }
    return lines;
  }
  // The table the parts of TakeBothParts find.
  static std::vector<Line> Table() {
    Line stop = MakeLine("GPIO5", "stop", "stop");
    stop.delay_ms = 10;
    Line lift = MakeLine("GPIO12", "lift", "lift");
    lift.delay_ms = 2500;
    lift.active_high = false;
    return {stop, lift};
  }
  std::optional<Refusal> Accept(const std::string& value) {
    return program.Answer({{"step", "assign"}, {"version", program.version()}, {"values", {{"accept", value}}}}, now,
                          kWall + now);
  }
  // Up to the first part: started, restarted into learning, the lines untouched for the quiet time.
  void ToTheParts() {
    ASSERT_EQ(Refused(Start("assign")), "");
    Run(0.2);
    ASSERT_EQ(restarts, 1);
    Restart(true, Candidates());
    Run(3.2);
    ASSERT_EQ(StepState("assign")["parts"][0]["state"], "waiting");
  }
  // One line goes to its other level and back, and the question in between is answered.
  void Take(const std::string& line, bool level, const std::string& answer) {
    SetLine(line, level);
    Run(0.5);
    ASSERT_EQ(StepState("assign")["ask"]["line"], line);
    ASSERT_EQ(Refused(Accept(answer)), "");
    SetLine(line, !level);
    Run(0.2);
  }
  void TakeBothParts() {
    Take("GPIO5", true, "yes");
    Take("GPIO12", false, "yes");
  }

  World world;
  std::set<std::string> shadowed;
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
                                   {"why", "NO_SAFETY_MEASUREMENT"}, {"source", nullptr},
                                   {"closed", json::array()}}));
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
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0]["part"], "top");
  EXPECT_EQ(events[0]["limit"], 1.0);
  // A safety step: the failure closes the lock.
  EXPECT_EQ(events[1],
            (json{{"what", "safety_closed"}, {"step", "stop"}, {"cause", "NO_EMERGENCY"}, {"type", "SETUP"}}));
}

TEST(Program, NothingSeenOrAnotherInputEndsThePartInTime) {
  Bench bench;
  bench.PassBoard();
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(10.3);
  // Nothing seen: the step asks whether the user did it, and only yes fails it.
  EXPECT_EQ(bench.StepState("stop")["state"], "running");
  ASSERT_EQ(bench.Refused(bench.program.Answer(
                {{"step", "stop"}, {"version", bench.program.version()}, {"values", {{"pressed", "yes"}}}}, bench.now,
                kWall + bench.now)),
            "");
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

// Rule of the own release: at the earliest 1.5 s after the input switched, only once it is free, with the count of the
// last hw/status, again every 1.5 s until hw/status shows the emergency gone.
TEST(Program, TheEmergencyOfACheckedInputIsReleasedOnceItIsFree) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  const double edge = bench.Switch(&Hardware::stop, bench.stop);
  bench.Run(1.7);
  // Still pressed: no try, and the step tells to let go instead of an emergency to release.
  EXPECT_TRUE(bench.releases.empty());
  EXPECT_EQ(bench.StepState("stop")["blocked"][0]["reason"], "INPUT_ACTIVE");
  bench.hardware.stop = false;
  bench.Run(0.1);
  ASSERT_EQ(bench.releases.size(), 1u);
  EXPECT_GE(bench.releases[0].first - edge, 1.5);
  EXPECT_EQ(bench.releases[0].second.latch_count, 1u);
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "ok");
  EXPECT_TRUE(bench.StepState("stop")["blocked"].empty());
  // The driver did not take it yet: the next try comes 1.5 s later, not before.
  bench.Run(1.3);
  EXPECT_EQ(bench.releases.size(), 1u);
  bench.Run(0.3);
  ASSERT_EQ(bench.releases.size(), 2u);
  EXPECT_NEAR(bench.releases[1].first - bench.releases[0].first, 1.5, 0.11);
  // Gone in hw/status: no further try, and the second button counts.
  bench.hardware.emergency_reason = 0;
  bench.Run(4);
  EXPECT_EQ(bench.releases.size(), 2u);
  bench.Switch(&Hardware::stop, bench.stop);
  bench.hardware.stop = false;
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("stop")["state"], "ok");
  bench.Run(1.5);
  ASSERT_EQ(bench.releases.size(), 3u);
  EXPECT_EQ(bench.releases[2].second.latch_count, 2u);
}

TEST(Program, AnInputLetGoEarlyWaitsItsTimeBeforeTheRelease) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  const double edge = bench.Switch(&Hardware::stop, bench.stop);
  bench.hardware.stop = false;
  bench.Run(1.2);
  EXPECT_TRUE(bench.releases.empty());
  bench.Run(0.3);
  ASSERT_EQ(bench.releases.size(), 1u);
  EXPECT_NEAR(bench.releases[0].first - edge, 1.5, 0.11);
}

// Where the board holds the input until the release the try itself is the probe: it comes while the input shows.
TEST(Program, AnInputTheBoardHoldsIsReleasedByTrying) {
  ReleaseBench bench;
  bench.lines = {MakeLine("stop", "stop", "stop", true)};
  bench.program.OnInputs(bench.lines);
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.SetLine("stop", true);
  const double edge = bench.Switch(&Hardware::stop, bench.stop);
  bench.Run(1.2);
  EXPECT_TRUE(bench.releases.empty());
  bench.Run(0.3);
  ASSERT_EQ(bench.releases.size(), 1u);
  EXPECT_NEAR(bench.releases[0].first - edge, 1.5, 0.11);
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "waiting");
  EXPECT_EQ(bench.StepState("stop")["blocked"][0]["reason"], "INPUT_ACTIVE");
  // The button is still down, the board keeps its bit: the same again 1.5 s later.
  bench.Run(1.7);
  ASSERT_EQ(bench.releases.size(), 2u);
  EXPECT_NEAR(bench.releases[1].first - bench.releases[0].first, 1.5, 0.11);
  // Let go, the next try clears the board.
  bench.hardware.stop = false;
  bench.hardware.emergency_reason = 0;
  bench.SetLine("stop", false);
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "ok");
  bench.Run(3);
  EXPECT_EQ(bench.releases.size(), 2u);
}

// Any other bit since the input switched, in one hw/status between two ticks: the program releases nothing.
TEST(Program, AnotherReasonEndsTheOwnReleaseForGood) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.Switch(&Hardware::stop, bench.stop);
  Hardware tilted = bench.hardware;
  tilted.emergency_reason |= EmergencyBit("TILT");
  bench.program.OnHardware(tilted, bench.now + 0.01);
  bench.program.OnHardware(bench.hardware, bench.now + 0.02);
  bench.hardware.stop = false;
  bench.Run(1);
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "ok");
  EXPECT_EQ(bench.StepState("stop")["blocked"][0]["reason"], "EMERGENCY");
  bench.Run(25);
  EXPECT_TRUE(bench.releases.empty());
}

TEST(Program, AnotherReasonBeforeTheEmergencyOfTheInputEndsItAsWell) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  // HIGH_LEVEL stands while the button goes down.
  bench.Latch(EmergencyBit("HIGH_LEVEL"));
  bench.Run(0.5);
  bench.hardware.stop = true;
  bench.Run(0.1);
  bench.Latch(bench.stop);
  bench.Run(0.2);
  bench.hardware.stop = false;
  bench.Run(15);
  EXPECT_TRUE(bench.releases.empty());
}

TEST(Program, ADriverThatDoesNotCountGetsNoOwnRelease) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.hardware.stop = true;
  bench.Run(0.1);
  bench.hardware.emergency_reason = bench.stop | EmergencyBit("LATCH");
  bench.Run(0.2);
  bench.hardware.stop = false;
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("stop")["blocked"][0]["reason"], "EMERGENCY");
  // A count that shows up later changes nothing.
  bench.hardware.latch_count = 3;
  bench.Run(15);
  EXPECT_TRUE(bench.releases.empty());
}

TEST(Program, TheOwnReleaseWaitsForIdle) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.Switch(&Hardware::stop, bench.stop);
  bench.hardware.stop = false;
  World world;
  world.state = "MOWING";
  bench.program.OnWorld(world);
  bench.Run(5);
  EXPECT_TRUE(bench.releases.empty());
  world.state = "IDLE";
  bench.program.OnWorld(world);
  bench.Run(0.1);
  EXPECT_EQ(bench.releases.size(), 1u);
}

TEST(Program, TheOwnReleaseNeedsAFreshStatus) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.Switch(&Hardware::stop, bench.stop);
  bench.hardware.stop = false;
  bench.Run(0.1);
  bench.driver = false;
  bench.Run(5);
  EXPECT_EQ(bench.StepState("stop")["result"]["reason"], "NO_DATA");
  EXPECT_TRUE(bench.releases.empty());
  bench.driver = true;
  bench.Run(0.2);
  EXPECT_EQ(bench.releases.size(), 1u);
}

// The tries end 10 s after the step: an emergency that stays is left to emergency.reset.
TEST(Program, TheOwnReleaseEndsAfterTheStep) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.hardware.bumper = true;
  bench.Switch(&Hardware::bumper_left, EmergencyBit("COLLISION"));
  bench.hardware.bumper = bench.hardware.bumper_left = false;
  bench.Run(0.1);
  ASSERT_EQ(bench.StepState("left")["state"], "ok");
  const double ended = bench.now;
  bench.Run(30);
  ASSERT_FALSE(bench.releases.empty());
  EXPECT_LE(bench.releases.back().first - ended, 10.0);
  EXPECT_GE(bench.releases.back().first - ended, 8.4);
}

// An emergency that stood before the input showed is not the one of the part, and a part of mode short has none.
TEST(Program, OnlyTheEmergencyOfAPressIsReleased) {
  ReleaseBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.Latch(bench.stop);
  bench.Run(0.5);
  bench.hardware.stop = true;
  bench.Run(0.5);
  bench.hardware.stop = false;
  bench.Run(5);
  EXPECT_TRUE(bench.releases.empty());
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "waiting");
  ASSERT_EQ(bench.Refused(bench.program.Abort(nullptr, bench.now, kWall)), "");
  bench.hardware.emergency_reason = 0;

  ASSERT_EQ(bench.Refused(bench.Start("lift")), "");
  bench.Run(0.2);
  bench.hardware.lift = true;
  bench.Run(0.3);
  bench.Latch(EmergencyBit("LIFT"));
  bench.Run(0.2);
  bench.hardware.lift = false;
  bench.Run(15);
  EXPECT_EQ(bench.StepState("lift")["result"]["reason"], "STOPPED_EARLY");
  EXPECT_TRUE(bench.releases.empty());
}

// With hw/inputs every part of a role has to bring a line of its own, and the step shows the lines live.
TEST(Program, PartsOfOneRoleNeedDifferentLines) {
  Bench bench;
  bench.PassBoard();
  bench.lines = {MakeLine("GPIO1", "stop", "stop"), MakeLine("GPIO5", "stop", "stop"),
                 MakeLine("GPIO10", "lift", "lift")};
  bench.program.OnInputs(bench.lines);
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  const auto press = [&bench](const std::string& line) {
    bench.SetLine(line, true);
    bench.Press(&Hardware::stop, 4, 0.1, 0.4);
    bench.SetLine(line, false);
    bench.hardware.emergency_reason = 0;
    bench.Run(0.3);
  };
  bench.SetLine("GPIO1", true);
  bench.hardware.stop = true;
  bench.Run(0.2);
  json step = bench.StepState("stop");
  ASSERT_EQ(step["lines"].size(), 3u);
  EXPECT_EQ(step["lines"][0], (json{{"name", "GPIO1"}, {"role", "stop"}, {"reason", "stop"}, {"active", true},
                                    {"assigned", true}, {"held", false}, {"delay_ms", 0}}));
  EXPECT_EQ(step["lines"][1]["active"], false);
  EXPECT_TRUE(bench.StepState("board")["lines"].empty());
  press("GPIO1");
  EXPECT_EQ(bench.StepState("stop")["parts"][0]["state"], "ok");
  // The same button again does not count for the second one.
  press("GPIO1");
  EXPECT_EQ(bench.StepState("stop")["parts"][1]["state"], "waiting");
  press("GPIO5");
  EXPECT_EQ(bench.StepState("stop")["state"], "ok");
}

TEST(Program, ALineThatNeverSwitchesFailsTheStep) {
  Bench bench;
  bench.PassBoard();
  bench.lines = {MakeLine("GPIO1", "stop", "stop"), MakeLine("GPIO5", "stop", "stop")};
  bench.program.OnInputs(bench.lines);
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.Events();
  for (int i = 0; i < 2; ++i) {
    bench.SetLine("GPIO1", true);
    bench.Press(&Hardware::stop, 4, 0.1, 0.4);
    bench.SetLine("GPIO1", false);
    bench.hardware.emergency_reason = 0;
    bench.Run(0.3);
  }
  bench.Run(10);
  json step = bench.StepState("stop");
  EXPECT_EQ(step["state"], "failed");
  EXPECT_EQ(step["result"]["reason"], "LINE_SILENT");
  EXPECT_EQ(step["result"]["part"], "rear");
  const auto events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["lines"], json::array({"GPIO5"}));

  // One part for a pair of lines: the one that stayed silent is named at the end.
  bench.lines = {MakeLine("GPIO12", "bumper_left", "collision"), MakeLine("GPIO13", "bumper_left", "collision")};
  bench.program.OnInputs(bench.lines);
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.SetLine("GPIO13", true);
  bench.hardware.bumper = true;
  bench.Press(&Hardware::bumper_left, EmergencyBit("COLLISION"), 0.1, 0.4);
  bench.SetLine("GPIO13", false);
  step = bench.StepState("left");
  EXPECT_EQ(step["result"]["reason"], "LINE_SILENT");
  EXPECT_EQ(bench.Events()[0]["lines"], json::array({"GPIO12"}));
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

TEST(Program, TheRestOfTheImuCountsFromItsFirstSample) {
  // The first sample comes later than the rest lasts, as after a start whose record took long to write.
  Bench bench;
  bench.imu_on = false;
  ASSERT_EQ(bench.Refused(bench.Start("imu")), "");
  bench.Run(0.8);
  // Until the rest is taken the part is not free for its move.
  EXPECT_EQ(bench.StepState("imu")["blocked"][0]["reason"], "INPUT_ACTIVE");
  bench.imu_on = true;
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("imu")["blocked"][0]["reason"], "INPUT_ACTIVE");
  bench.Run(0.8);
  const json step = bench.StepState("imu");
  EXPECT_EQ(step["state"], "running");
  EXPECT_EQ(step["parts"][0]["state"], "waiting");
  EXPECT_TRUE(step["blocked"].empty()) << step["blocked"].dump();
  bench.imu.ax = 9.81 * std::sin(15 * M_PI / 180);
  bench.imu.az = 9.81 * std::cos(15 * M_PI / 180);
  bench.Run(1.5);
  EXPECT_EQ(bench.StepState("imu")["parts"][0]["state"], "ok");
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
    ASSERT_EQ(bench.Refused(bench.program.Answer(
                  {{"step", "imu"}, {"version", bench.program.version()}, {"values", {{"pressed", "yes"}}}},
                  bench.now, kWall + bench.now)),
              "");
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
        bench.program.Answer({{"step", step}, {"version", bench.program.version()}, {"values", values}}, 0, kWall));
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
                              0, kWall));
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
  ASSERT_FALSE(bench.program.Answer(params, 0, kWall));
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

// inputs_assign: restart into learning, the lines untouched, one line per part with its question, the table into the
// parameter file, restart, and hw/inputs held against it.
TEST(Program, TheInputsAreAssignedPartByPart) {
  AssignBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("assign")), "");
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 1);
  EXPECT_NE(bench.layer_text.find("driver:\n  ros__parameters:\n    inputs.learn: true\n"), std::string::npos);
  json state = bench.State();
  EXPECT_EQ(state["restart_pending"], true);
  EXPECT_EQ(state["steps"][1]["state"], "running");
  EXPECT_EQ(state["steps"][1]["restart"], true);
  EXPECT_EQ(state["steps"][0]["restart"], false);

  bench.Restart(true, AssignBench::Candidates());
  json step = bench.StepState("assign");
  EXPECT_EQ(step["state"], "running");
  EXPECT_EQ(step["parts"][0]["state"], "open");
  EXPECT_EQ(step["lines"].size(), 4u);
  EXPECT_EQ(bench.State()["restart_pending"], false);
  bench.Run(3.2);
  step = bench.StepState("assign");
  EXPECT_EQ(step["parts"][0]["state"], "waiting");
  EXPECT_EQ(step["progress"], (json{{"n", 1}, {"of", 2}}));
  EXPECT_TRUE(step["ask"].is_null());
  EXPECT_EQ(step["inputs"][0]["name"], "accept");
  EXPECT_EQ(step["inputs"][0]["ask"], false);
  EXPECT_EQ(step["inputs"][0]["values"][0]["id"], "yes");
  EXPECT_EQ(step["parts"][0]["texts"]["en"]["name"], "STOP button");

  bench.SetLine("GPIO5", true);
  bench.Run(0.5);
  step = bench.StepState("assign");
  EXPECT_EQ(step["ask"], (json{{"part", "stop_top"}, {"line", "GPIO5"}, {"active_high", true}}));
  EXPECT_EQ(step["inputs"][0]["ask"], true);
  EXPECT_EQ(bench.Refused(bench.Accept("maybe")), "INVALID_VALUE");
  ASSERT_EQ(bench.Refused(bench.Accept("yes")), "");
  bench.Run(0.2);
  step = bench.StepState("assign");
  EXPECT_EQ(step["parts"][0]["state"], "ok");
  EXPECT_EQ(step["values"][0]["value"], "GPIO5");
  // The next part begins with the lines back at rest.
  EXPECT_EQ(step["blocked"][0]["reason"], "INPUT_ACTIVE");
  EXPECT_EQ(bench.Refused(bench.Accept("yes")), "WRONG_METHOD");
  bench.SetLine("GPIO5", false);
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("assign")["parts"][1]["state"], "waiting");

  // Two lines shortly after one another are no answer, and a line a part took is none either.
  bench.SetLine("GPIO10", true);
  bench.Run(0.1);
  bench.SetLine("GPIO12", false);
  bench.Run(0.5);
  EXPECT_TRUE(bench.StepState("assign")["ask"].is_null());
  bench.SetLine("GPIO10", false);
  bench.SetLine("GPIO12", true);
  bench.Run(0.2);
  bench.SetLine("GPIO5", true);
  bench.Run(0.5);
  EXPECT_TRUE(bench.StepState("assign")["ask"].is_null());
  bench.SetLine("GPIO5", false);
  // No leaves the part waiting, a line that goes low is taken as active low.
  bench.Take("GPIO10", true, "no");
  EXPECT_EQ(bench.StepState("assign")["parts"][1]["state"], "waiting");
  bench.SetLine("GPIO12", false);
  bench.Run(0.5);
  EXPECT_EQ(bench.StepState("assign")["ask"],
            (json{{"part", "lift_left"}, {"line", "GPIO12"}, {"active_high", false}}));
  ASSERT_EQ(bench.Refused(bench.Accept("yes")), "");
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 1);
  bench.SetLine("GPIO12", true);
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 2);
  const std::string values = bench.layer_text.substr(bench.layer_text.find("driver:"));
  EXPECT_EQ(values, "driver:\n  ros__parameters:\n"
                    "    inputs.gpio_active_high: [true, false]\n"
                    "    inputs.gpio_delays_ms: [10, 2500]\n"
                    "    inputs.gpio_lines: [\"GPIO5\", \"GPIO12\"]\n"
                    "    inputs.gpio_reasons: [\"stop\", \"lift\"]\n"
                    "    inputs.gpio_roles: [\"stop\", \"lift\"]\n");
  bench.Events();

  // Back without learning, the driver shows the table: done.
  bench.Restart(false, AssignBench::Table());
  step = bench.StepState("assign");
  EXPECT_EQ(step["state"], "ok");
  EXPECT_EQ(step["values"][1]["value"], "GPIO12");
  EXPECT_EQ(step["parts"][1]["state"], "ok");
  EXPECT_EQ(bench.State()["restart_pending"], false);
  const auto events = bench.Events();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events[0]["step"], "assign");
  EXPECT_EQ(events[0]["state"], "ok");
  bench.Run(30);
  EXPECT_EQ(bench.restarts, 2);
}

TEST(Program, AssigningNeedsAMowerAtRestWithoutEmergency) {
  AssignBench bench;
  bench.world.state = "MOWING";
  bench.program.OnWorld(bench.world);
  EXPECT_EQ(bench.Refused(bench.Start("assign")), "BUSY");
  bench.world.state = "IDLE";
  bench.program.OnWorld(bench.world);
  bench.hardware.emergency_reason = EmergencyBit("STOP") | EmergencyBit("LATCH");
  bench.Run(0.2);
  EXPECT_EQ(bench.Refused(bench.Start("assign")), "EMERGENCY");
  EXPECT_EQ(bench.StepState("assign")["blocked"][0]["reason"], "EMERGENCY");
  bench.hardware.emergency_reason = 0;
  bench.driver = false;
  bench.Run(1.5);
  EXPECT_EQ(bench.Refused(bench.Start("assign")), "DRIVER_MISSING");
  EXPECT_EQ(bench.restarts, 0);
  EXPECT_EQ(bench.layer_text, "");

  // Without hw/inputs or without a place for the file there is nothing to assign.
  Bench plain(kAssignList);
  World world;
  world.state = "IDLE";
  plain.program.OnWorld(world);
  EXPECT_EQ(plain.Refused(plain.Start("assign")), "STORAGE_NOT_WRITABLE");
  plain.program.SetLayer(Layer(), {});
  EXPECT_EQ(plain.Refused(plain.Start("assign")), "DRIVER_MISSING");
}

// Another emergency while the driver learns holds the restart back until it is released.
TEST(Program, ARestartNeverTakesAnEmergencyAway) {
  AssignBench bench;
  bench.ToTheParts();
  bench.TakeBothParts();
  ASSERT_EQ(bench.restarts, 2);

  AssignBench tilted;
  tilted.ToTheParts();
  tilted.hardware.emergency_reason |= EmergencyBit("TILT");
  tilted.TakeBothParts();
  tilted.Run(5);
  EXPECT_EQ(tilted.restarts, 1);
  EXPECT_EQ(tilted.StepState("assign")["blocked"][0]["reason"], "EMERGENCY");
  EXPECT_EQ(tilted.State()["restart_pending"], true);
  tilted.world.state = "AREA_RECORDING";
  tilted.program.OnWorld(tilted.world);
  tilted.hardware.emergency_reason &= ~EmergencyBit("TILT");
  tilted.Run(1);
  EXPECT_EQ(tilted.restarts, 1);
  EXPECT_EQ(tilted.StepState("assign")["blocked"][0]["reason"], "BUSY");
  tilted.world.state = "IDLE";
  tilted.program.OnWorld(tilted.world);
  tilted.Run(0.2);
  EXPECT_EQ(tilted.restarts, 2);
}

TEST(Program, ARestartThatDoesNotComeFailsTheStep) {
  AssignBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("assign")), "");
  // Not before the record is on disk: the step goes on from it.
  bench.program.SetWritable(false);
  bench.Run(1);
  EXPECT_EQ(bench.restarts, 0);
  bench.program.SetWritable(true);
  bench.Run(0.1);
  EXPECT_EQ(bench.restarts, 1);
  bench.Run(18.9);
  EXPECT_EQ(bench.StepState("assign")["state"], "running");
  bench.Events();
  bench.Run(2);
  const json step = bench.StepState("assign");
  EXPECT_EQ(step["state"], "failed");
  EXPECT_EQ(step["result"]["reason"], "RESTART_FAILED");
  EXPECT_EQ(bench.Events()[0]["limit"], 20.0);
  // The next start does not learn, and nothing restarts for a driver that never learned.
  EXPECT_EQ(bench.layer_text, "");
  bench.Run(30);
  EXPECT_EQ(bench.restarts, 1);
}

// However the step ends while the driver learns, the program takes the learning back and restarts.
TEST(Program, AStepThatEndsWhileLearningLeavesTheLearning) {
  AssignBench flutter;
  ASSERT_EQ(flutter.Refused(flutter.Start("assign")), "");
  flutter.Run(0.2);
  flutter.Restart(true, AssignBench::Candidates());
  flutter.Run(1);
  flutter.SetLine("GPIO10", true);
  flutter.SetLine("GPIO10", false);
  flutter.Events();
  flutter.Run(2.5);
  EXPECT_EQ(flutter.StepState("assign")["result"]["reason"], "FLUTTER");
  EXPECT_EQ(flutter.Events()[0]["lines"], json::array({"GPIO10"}));
  EXPECT_EQ(flutter.restarts, 2);
  EXPECT_EQ(flutter.layer_text, "");

  AssignBench stopped;
  stopped.ToTheParts();
  stopped.Take("GPIO5", true, "yes");
  ASSERT_EQ(stopped.Refused(stopped.program.Abort(nullptr, stopped.now, kWall)), "");
  stopped.Run(0.2);
  json step = stopped.StepState("assign");
  EXPECT_EQ(step["state"], "open");
  EXPECT_EQ(step["result"]["reason"], "STOPPED");
  EXPECT_EQ(stopped.restarts, 2);
  EXPECT_EQ(stopped.layer_text, "");
  EXPECT_EQ(stopped.State()["restart_pending"], false);
  stopped.Run(30);
  EXPECT_EQ(stopped.restarts, 2);

  // No line in time, or only several at once.
  AssignBench silent;
  silent.ToTheParts();
  silent.Events();
  silent.Run(10.2);
  step = silent.StepState("assign");
  EXPECT_EQ(step["result"]["reason"], "NO_LINE");
  EXPECT_EQ(step["result"]["part"], "stop_top");
  EXPECT_EQ(step["parts"][0]["state"], "failed");
  EXPECT_EQ(silent.restarts, 2);

  AssignBench both;
  both.ToTheParts();
  both.SetLine("GPIO1", true);
  both.SetLine("GPIO5", true);
  both.Run(0.5);
  both.SetLine("GPIO1", false);
  both.SetLine("GPIO5", false);
  both.Events();
  both.Run(10.2);
  EXPECT_EQ(both.StepState("assign")["result"]["reason"], "AMBIGUOUS");
  EXPECT_EQ(both.Events()[0]["lines"], json::array({"GPIO1", "GPIO5"}));
}

// The record is lost while the driver learns: the learning of the program's own file goes, one a file kept by hand
// asks for stays.
TEST(Program, ALearningNoStepIsAtIsTakenBack) {
  AssignBench bench;
  bench.saved.clear();
  bench.layer_text = "driver:\n  ros__parameters:\n    inputs.learn: true\n";
  bench.Restart(true, AssignBench::Candidates());
  EXPECT_EQ(bench.restarts, 1);
  EXPECT_EQ(bench.layer_text, "");
  EXPECT_EQ(bench.StepState("assign")["blocked"][0]["reason"], "BUSY");

  AssignBench by_hand;
  by_hand.Restart(true, AssignBench::Candidates());
  by_hand.Run(5);
  EXPECT_EQ(by_hand.restarts, 0);
  // No input stops anything while the driver learns: none is checked.
  EXPECT_EQ(by_hand.Refused(by_hand.Start("stop")), "BUSY");
}

TEST(Program, ATableTheDriverDoesNotShowIsNotApplied) {
  AssignBench bench;
  bench.ToTheParts();
  bench.TakeBothParts();
  ASSERT_EQ(bench.restarts, 2);
  // The file kept by hand names other lines: the driver comes back with those.
  bench.shadowed = {"driver.inputs.gpio_lines", "logic.inputs.x"};
  bench.Restart(false, {MakeLine("GPIO1", "stop", "stop"), MakeLine("GPIO10", "lift", "lift")});
  bench.Events();
  bench.Run(19.5);
  EXPECT_EQ(bench.StepState("assign")["state"], "running");
  // The right lines with another level or delay are not the table either.
  std::vector<Line> close = AssignBench::Table();
  close[1].active_high = true;
  bench.lines = close;
  bench.program.OnInputs(bench.lines, false);
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("assign")["state"], "running");
  close = AssignBench::Table();
  close[0].delay_ms = 20;
  bench.lines = close;
  bench.program.OnInputs(bench.lines, false);
  bench.Run(1);
  EXPECT_EQ(bench.StepState("assign")["result"]["reason"], "NOT_APPLIED");
  EXPECT_EQ(bench.Events()[0]["key"], "driver.inputs.gpio_lines");
  EXPECT_EQ(bench.restarts, 2);

  // A file that cannot be written ends the step before any restart.
  AssignBench full;
  full.layer_ok = false;
  ASSERT_EQ(full.Refused(full.Start("assign")), "");
  full.Run(0.2);
  EXPECT_EQ(full.StepState("assign")["result"]["reason"], "NOT_APPLIED");
  EXPECT_EQ(full.restarts, 0);
}

// The checks of the inputs count only with the table they ran with.
TEST(Program, ANewTableOpensTheChecksOfTheInputsAgain) {
  AssignBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("stop")), "");
  bench.Run(0.2);
  bench.SetLine("GPIO1", true);
  bench.Press(&Hardware::stop, 4, 0.1, 0.4);
  bench.SetLine("GPIO1", false);
  bench.hardware.emergency_reason = 0;
  bench.Run(0.2);
  ASSERT_EQ(bench.StepState("stop")["state"], "ok");
  bench.ToTheParts();
  EXPECT_EQ(bench.StepState("stop")["state"], "ok");
  // While the driver learns no check of an input starts.
  EXPECT_EQ(bench.Refused(bench.Start("stop")), "BUSY");
  bench.TakeBothParts();
  EXPECT_EQ(bench.StepState("stop")["state"], "open");
  EXPECT_TRUE(bench.StepState("stop")["result"].is_null());
}

TEST(Program, TheDelaysOfAFoundTableGetShorterOnly) {
  AssignBench bench;
  const auto delays = [&bench](json values) {
    return bench.Refused(bench.program.Answer(
        {{"step", "assign"}, {"version", bench.program.version()}, {"values", values}}, bench.now, kWall + bench.now));
  };
  bench.ToTheParts();
  bench.TakeBothParts();
  ASSERT_EQ(bench.restarts, 2);
  bench.Restart(false, AssignBench::Table());
  ASSERT_EQ(bench.StepState("assign")["state"], "ok");
  ASSERT_FALSE(bench.program.Skip({{"step", "stop"}, {"version", bench.program.version()}, {"reason", "later"}},
                                  kWall + bench.now));
  // One delay per part beside the question of the step, with the one of the list as its largest value.
  json input = bench.StepState("assign")["inputs"][2];
  EXPECT_EQ(input["name"], "delay_lift_left");
  EXPECT_EQ(input["type"], "number");
  EXPECT_EQ(input["advanced"], true);
  EXPECT_EQ(input["value"], 2500);
  EXPECT_EQ(input["unit"], "ms");
  EXPECT_EQ(input["min"], 0);
  EXPECT_EQ(input["max"], 2500);
  EXPECT_EQ(input["texts"]["en"]["name"], "Lifting");
  EXPECT_FALSE(input["texts"]["de"]["help"].get<std::string>().empty());
  EXPECT_EQ(bench.StepState("assign")["inputs"][0]["advanced"], false);

  // Never longer than the platform gives, a whole number, of a part of the step.
  EXPECT_EQ(delays({{"delay_lift_left", 2501}}), "INVALID_VALUE");
  EXPECT_EQ(delays({{"delay_lift_left", -1}}), "INVALID_VALUE");
  EXPECT_EQ(delays({{"delay_lift_left", 100.5}}), "INVALID_VALUE");
  EXPECT_EQ(delays({{"delay_stop_top", 5}, {"delay_nothing", 5}}), "INVALID_VALUE");
  EXPECT_EQ(delays({{"accept", "yes"}}), "INVALID_VALUE");
  EXPECT_EQ(delays(json::object()), "param values");
  // The values of now change nothing.
  EXPECT_EQ(delays({{"delay_lift_left", 2500}, {"delay_stop_top", 10}}), "");
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 2);
  // Only a mower at rest without an emergency restarts for it.
  bench.hardware.emergency_reason = EmergencyBit("STOP");
  bench.Run(0.2);
  EXPECT_EQ(delays({{"delay_lift_left", 1000}}), "EMERGENCY");
  bench.hardware.emergency_reason = 0;
  bench.world.state = "MOWING";
  bench.program.OnWorld(bench.world);
  bench.Run(0.2);
  EXPECT_EQ(delays({{"delay_lift_left", 1000}}), "BUSY");
  bench.world.state = "IDLE";
  bench.program.OnWorld(bench.world);
  bench.Run(0.2);
  EXPECT_EQ(bench.StepState("assign")["state"], "ok");
  EXPECT_EQ(bench.StepState("assign")["attempt"], 1);
  EXPECT_EQ(bench.restarts, 2);
  bench.Events();

  // A shorter one: the table is written again, the OS restarts, the checks of the inputs are open.
  ASSERT_EQ(delays({{"delay_lift_left", 1000}}), "");
  EXPECT_EQ(bench.StepState("assign")["state"], "running");
  EXPECT_EQ(bench.StepState("assign")["attempt"], 2);
  EXPECT_EQ(bench.StepState("stop")["state"], "open");
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 3);
  EXPECT_NE(bench.layer_text.find("    inputs.gpio_delays_ms: [10, 1000]\n"), std::string::npos) << bench.layer_text;
  EXPECT_NE(bench.layer_text.find("    inputs.gpio_lines: [\"GPIO5\", \"GPIO12\"]\n"), std::string::npos);
  EXPECT_EQ(bench.layer_text.find("inputs.learn"), std::string::npos);
  std::vector<Line> table = AssignBench::Table();
  table[1].delay_ms = 1000;
  bench.Restart(false, table);
  json step = bench.StepState("assign");
  EXPECT_EQ(step["state"], "ok");
  EXPECT_EQ(step["inputs"][2]["value"], 1000);
  EXPECT_EQ(step["inputs"][1]["value"], 10);
  EXPECT_EQ(step["values"][1]["value"], "GPIO12");
  const auto events = bench.Events();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events[0]["state"], "ok");
  EXPECT_EQ(events[0]["attempt"], 2);

  // A driver that does not show the delay asked for: the step fails as after its parts.
  ASSERT_EQ(delays({{"delay_stop_top", 0}}), "");
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 4);
  EXPECT_NE(bench.layer_text.find("    inputs.gpio_delays_ms: [0, 1000]\n"), std::string::npos) << bench.layer_text;
  bench.Restart(false, table);
  bench.Run(21);
  step = bench.StepState("assign");
  EXPECT_EQ(step["state"], "failed");
  EXPECT_EQ(step["result"]["reason"], "NOT_APPLIED");
  // Not before the step found a table again.
  EXPECT_EQ(delays({{"delay_stop_top", 5}}), "WRONG_METHOD");
  // setup.reset forgets the delays.
  ASSERT_FALSE(bench.program.Reset({{"version", bench.program.version()}}));
  EXPECT_EQ(bench.StepState("assign")["inputs"][1]["value"], 10);
  EXPECT_EQ(bench.StepState("assign")["inputs"][2]["value"], 2500);
  // A delay of the record above the one of the list, as after a list that made it shorter, counts as that one.
  bench.Run(0.2);
  json record = json::parse(bench.saved);
  record["steps"]["assign"]["answers"]["delay_lift_left"] = 9000;
  bench.saved = record.dump();
  bench.Restart(false, table);
  EXPECT_EQ(bench.StepState("assign")["inputs"][2]["value"], 2500);
}

TEST(Program, AnUpdateThatHoldsTheMowerKeepsTheRestartBack) {
  AssignBench bench;
  bench.hardware.hold = true;
  bench.Run(0.2);
  EXPECT_EQ(bench.Refused(bench.Start("assign")), "BUSY");
  bench.hardware.hold = false;
  bench.Run(0.2);
  ASSERT_EQ(bench.Refused(bench.Start("assign")), "");
  bench.hardware.hold = true;
  bench.Run(0.5);
  EXPECT_EQ(bench.restarts, 0);
  EXPECT_EQ(bench.StepState("assign")["blocked"][0]["reason"], "BUSY");
  bench.hardware.hold = false;
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 1);
}

TEST(Program, AssigningNeedsTheLevelsOfTheLines) {
  // A learning driver that does not know its lines yet: the lines are not taken as at rest.
  AssignBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("assign")), "");
  bench.Run(0.2);
  bench.Restart(true, {});
  bench.Run(5);
  EXPECT_EQ(bench.StepState("assign")["parts"][0]["state"], "open");
  bench.lines = AssignBench::Candidates();
  bench.program.OnInputs(bench.lines, true);
  bench.Run(3.2);
  ASSERT_EQ(bench.StepState("assign")["parts"][0]["state"], "waiting");
  // It loses them again: nothing is watched any more.
  bench.program.OnInputs({}, true);
  bench.Run(0.2);
  ASSERT_EQ(bench.StepState("assign")["result"]["reason"], "NO_DATA");
  EXPECT_EQ(bench.Events()[0]["source"], "hw/inputs");

  // A driver that falls silent while the lines are watched untouched.
  AssignBench silent;
  ASSERT_EQ(silent.Refused(silent.Start("assign")), "");
  silent.Run(0.2);
  silent.Restart(true, AssignBench::Candidates());
  silent.Run(1);
  silent.driver = false;
  silent.Run(3.5);
  ASSERT_EQ(silent.StepState("assign")["result"]["reason"], "NO_DATA");
  EXPECT_EQ(silent.Events()[0]["source"], "hw/status");
}

TEST(Program, AResetBeforeTheRestartTakesTheLearningBack) {
  AssignBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("assign")), "");
  bench.world.state = "DRIVING";
  bench.program.OnWorld(bench.world);
  bench.Run(0.3);
  EXPECT_EQ(bench.restarts, 0);
  EXPECT_NE(bench.layer_text.find("inputs.learn: true"), std::string::npos);
  ASSERT_FALSE(bench.program.Reset({{"version", bench.program.version()}}));
  bench.world.state = "IDLE";
  bench.program.OnWorld(bench.world);
  bench.Run(0.3);
  EXPECT_EQ(bench.restarts, 0);
  EXPECT_EQ(bench.layer_text.find("inputs.learn"), std::string::npos) << bench.layer_text;
}

TEST(Program, OnlyAWholeTableIsKeptAndGoneOnWith) {
  // A table that could not be written is not kept for a later write, and the learning is taken back once the file
  // can be written again.
  AssignBench late;
  late.ToTheParts();
  late.layer_ok = false;
  late.TakeBothParts();
  late.Run(0.2);
  EXPECT_EQ(late.StepState("assign")["result"]["reason"], "NOT_APPLIED");
  EXPECT_EQ(late.restarts, 1);
  late.layer_ok = true;
  late.Run(6);
  EXPECT_EQ(late.layer_text, "");
  EXPECT_EQ(late.restarts, 2);

  // A record whose table lacks a part does not go on after the restart.
  AssignBench broken;
  broken.ToTheParts();
  broken.TakeBothParts();
  ASSERT_EQ(broken.restarts, 2);
  json record = json::parse(broken.saved);
  ASSERT_EQ(record["steps"]["assign"]["assign"]["phase"], "apply");
  record["steps"]["assign"]["assign"]["parts"]["lift_left"] = 3;
  broken.saved = record.dump();
  broken.Restart(false, AssignBench::Table());
  const json step = broken.StepState("assign");
  EXPECT_EQ(step["state"], "open");
  EXPECT_EQ(step["result"]["reason"], "INTERRUPTED");

  // The delays go with the table of the parts as the list has them now.
  AssignBench other;
  other.ToTheParts();
  other.TakeBothParts();
  other.Restart(false, AssignBench::Table());
  ASSERT_EQ(other.StepState("assign")["state"], "ok");
  const std::string roles = "inputs.gpio_roles: [\"stop\", \"lift\"]";
  const size_t at = other.layer_text.find(roles);
  ASSERT_NE(at, std::string::npos);
  other.layer_text.replace(at, roles.size(), "inputs.gpio_roles: [\"lift\", \"stop\"]");
  other.Restart(false, AssignBench::Table());
  EXPECT_EQ(other.Refused(other.program.Answer(
                {{"step", "assign"}, {"version", other.program.version()}, {"values", {{"delay_lift_left", 1000}}}},
                other.now, kWall + other.now)),
            "WRONG_METHOD");
  EXPECT_EQ(other.restarts, 2);
}

// A bench with a safety measurement date, the board passed and the events taken.
struct LockBench : Bench {
  LockBench() {
    capabilities.safety_measurement_date = "2026-11-17";
    program.OnCapabilities(capabilities);
    PassBoard();
    Events();
  }
  json Safety() {
    return State()["safety"];
  }
  // bumper_left switches; with its emergency 0.1 s later when bring; let go after the time of the emergency.
  void Bump(bool bring) {
    hardware.bumper = hardware.bumper_left = true;
    Run(0.1);
    if (bring) hardware.emergency_reason |= bumper;
    Run(1.3);
    hardware.bumper = hardware.bumper_left = false;
    Run(0.3);
  }
  std::string AnswerPressed(const std::string& id, const std::string& value) {
    return Refused(program.Answer({{"step", id}, {"version", program.version()}, {"values", {{"pressed", value}}}},
                                  now, kWall + now));
  }
  json Pressed(const std::string& id) {
    const json step = StepState(id);
    for (const auto& input : step["inputs"]) {
      if (input["name"] == "pressed") return input;
    }
    return nullptr;
  }
  const uint32_t bumper =
      EmergencyBit(mowbite_api::FindSetupName(mowbite_api::SetupRoles(), "bumper_left")->emergency);
};

TEST(Lock, AFailedSafetyStepClosesTheLockUntilItPassesAgain) {
  LockBench bench;
  EXPECT_EQ(bench.Safety(), (json{{"open", true},
                                  {"date", "2026-11-17"},
                                  {"firmware", nullptr},
                                  {"why", nullptr},
                                  {"source", "hand"},
                                  {"closed", json::array()}}));
  EXPECT_EQ(bench.StepState("left")["safety"], true);
  EXPECT_EQ(bench.StepState("board")["safety"], false);
  EXPECT_FALSE(bench.program.TakeLock());
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.Bump(false);
  EXPECT_EQ(bench.StepState("left")["state"], "failed");
  auto events = bench.Events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0]["reason"], "NO_EMERGENCY");
  EXPECT_EQ(events[1],
            (json{{"what", "safety_closed"}, {"step", "left"}, {"cause", "NO_EMERGENCY"}, {"type", "SETUP"}}));
  const auto text = bench.program.TakeLock();
  ASSERT_TRUE(text && !text->empty());
  const auto record = mowbite_hold::ParseSafetyLock(*text);
  ASSERT_TRUE(record);
  EXPECT_EQ(record->failures, 1u);
  ASSERT_EQ(record->causes.size(), 1u);
  EXPECT_EQ(record->causes[0].step, "left");
  EXPECT_EQ(record->causes[0].reason, "NO_EMERGENCY");
  EXPECT_EQ(*record->causes[0].limit, 1.0);
  EXPECT_EQ(record->causes[0].firmware, "chassis 0.6.36, cutter 0.6.34");
  EXPECT_FALSE(bench.program.TakeLock());
  json safety = bench.Safety();
  EXPECT_EQ(safety["open"], false);
  EXPECT_EQ(safety["why"], "CHECK_FAILED");
  ASSERT_EQ(safety["closed"].size(), 1u);
  EXPECT_EQ(safety["closed"][0]["step"], "left");
  EXPECT_EQ(safety["closed"][0]["reason"], "NO_EMERGENCY");

  // Skipped or reset, the step keeps the lock closed.
  bench.hardware.emergency_reason = 0;
  ASSERT_EQ(bench.Refused(bench.program.Skip(
                {{"step", "left"}, {"version", bench.program.version()}, {"reason", "later"}}, kWall)),
            "");
  EXPECT_EQ(bench.Safety()["open"], false);
  ASSERT_EQ(bench.Refused(bench.program.Reset({{"version", bench.program.version()}})), "");
  EXPECT_EQ(bench.Safety()["open"], false);
  EXPECT_FALSE(bench.program.TakeLock());
  bench.Events();

  // A failure again counts again.
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.Bump(false);
  EXPECT_EQ(mowbite_hold::ParseSafetyLock(*bench.program.TakeLock())->failures, 2u);
  bench.hardware.emergency_reason = 0;
  bench.Events();

  // Passed, the lock opens and the file goes.
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.Bump(true);
  EXPECT_EQ(bench.StepState("left")["state"], "ok");
  events = bench.Events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1], (json{{"what", "safety_opened"}, {"type", "SETUP"}}));
  EXPECT_EQ(bench.program.TakeLock(), std::optional<std::string>(""));
  safety = bench.Safety();
  EXPECT_EQ(safety["open"], true);
  EXPECT_TRUE(safety["why"].is_null());
  EXPECT_TRUE(safety["closed"].empty());
}

TEST(Lock, WithoutADateTheLockStaysClosedAfterThePass) {
  Bench bench;
  bench.PassBoard();
  EXPECT_EQ(bench.State()["safety"]["why"], "NO_SAFETY_MEASUREMENT");
  EXPECT_TRUE(bench.State()["safety"]["source"].is_null());
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(10.3);
  ASSERT_EQ(bench.Refused(bench.program.Answer(
                {{"step", "left"}, {"version", bench.program.version()}, {"values", {{"pressed", "yes"}}}}, bench.now,
                kWall + bench.now)),
            "");
  bench.Events();
  EXPECT_EQ(bench.State()["safety"]["why"], "CHECK_FAILED");
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.hardware.bumper = bench.hardware.bumper_left = true;
  bench.Run(0.1);
  bench.hardware.emergency_reason |=
      EmergencyBit(mowbite_api::FindSetupName(mowbite_api::SetupRoles(), "bumper_left")->emergency);
  bench.Run(0.5);
  bench.hardware.bumper = bench.hardware.bumper_left = false;
  bench.Run(0.3);
  ASSERT_EQ(bench.StepState("left")["state"], "ok");
  // The cause is gone, yet without a date it is not open: no safety_opened.
  const auto events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(bench.program.TakeLock(), std::optional<std::string>(""));
  EXPECT_EQ(bench.State()["safety"]["why"], "NO_SAFETY_MEASUREMENT");
}

TEST(Lock, NothingSeenAsksAndOnlyYesCloses) {
  LockBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  EXPECT_EQ(bench.Pressed("left")["ask"], false);
  bench.Run(10.3);
  json step = bench.StepState("left");
  EXPECT_EQ(step["state"], "running");
  EXPECT_EQ(bench.Pressed("left")["ask"], true);
  EXPECT_EQ(bench.Pressed("left")["values"].size(), 2u);
  EXPECT_NEAR(step["ends_at"].get<double>(), kWall + bench.now + mowbite_api::kSetupPressedTime, 1.0);
  EXPECT_TRUE(step["blocked"].empty());
  EXPECT_TRUE(bench.Events().empty());
  EXPECT_EQ(bench.AnswerPressed("left", "maybe"), "INVALID_VALUE");

  // No: open again, nothing closes.
  EXPECT_EQ(bench.AnswerPressed("left", "no"), "");
  step = bench.StepState("left");
  EXPECT_EQ(step["state"], "open");
  EXPECT_EQ(step["result"]["reason"], "NOT_DONE");
  auto events = bench.Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["state"], "failed");
  EXPECT_EQ(events[0]["reason"], "NOT_DONE");
  EXPECT_EQ(events[0]["severity"], "info");
  EXPECT_FALSE(bench.program.TakeLock());
  EXPECT_EQ(bench.Safety()["open"], true);
  EXPECT_EQ(bench.AnswerPressed("left", "yes"), "WRONG_METHOD");

  // No answer within its time: the same.
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(10.3);
  bench.Run(mowbite_api::kSetupPressedTime - 0.5);
  EXPECT_EQ(bench.StepState("left")["state"], "running");
  bench.Run(1.0);
  EXPECT_EQ(bench.StepState("left")["result"]["reason"], "NOT_DONE");
  EXPECT_FALSE(bench.program.TakeLock());
  bench.Events();

  // Yes: the sensor is dead, the lock closes.
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(10.3);
  EXPECT_EQ(bench.AnswerPressed("left", "yes"), "");
  step = bench.StepState("left");
  EXPECT_EQ(step["state"], "failed");
  EXPECT_EQ(step["result"]["reason"], "NOT_SEEN");
  events = bench.Events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1]["cause"], "NOT_SEEN");
  EXPECT_TRUE(bench.program.TakeLock());
  EXPECT_EQ(bench.Safety()["open"], false);
}

TEST(Lock, ATiltThatDoesNotStopAsksAsWell) {
  const char* list = R"({"catalog": 1, "steps": [
    {"id": "imu", "stage": "inputs", "check": "imu", "params": {"time": 5}, "parts": ["tilt_stop"]}]})";
  Bench bench(list);
  ASSERT_EQ(bench.Refused(bench.Start("imu")), "");
  bench.Run(6.0);
  ASSERT_EQ(bench.StepState("imu")["state"], "running");
  ASSERT_EQ(bench.Refused(bench.program.Answer(
                {{"step", "imu"}, {"version", bench.program.version()}, {"values", {{"pressed", "yes"}}}}, bench.now,
                kWall + bench.now)),
            "");
  EXPECT_EQ(bench.StepState("imu")["result"]["reason"], "NO_EMERGENCY");
  EXPECT_TRUE(bench.program.TakeLock());
}

TEST(Lock, AFailureOfAnotherStepOrReasonLeavesTheLock) {
  LockBench bench;
  // The board is no safety step; WRONG_INPUT and an abort do not close.
  ASSERT_EQ(bench.Refused(bench.Start("board")), "");
  bench.Run(0.5);
  bench.hardware.link_errors = 5;
  bench.Run(1.7);
  ASSERT_EQ(bench.StepState("board")["state"], "failed");
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.hardware.stop = true;
  bench.Run(10.3);
  ASSERT_EQ(bench.StepState("left")["result"]["reason"], "WRONG_INPUT");
  bench.hardware.stop = false;
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  ASSERT_EQ(bench.Refused(bench.program.Abort(nullptr, bench.now, kWall)), "");
  EXPECT_FALSE(bench.program.TakeLock());
  EXPECT_EQ(bench.Safety()["open"], true);
  for (const auto& event : bench.Events()) EXPECT_NE(event.value("what", ""), "safety_closed");
}

TEST(Lock, TheFileOfTheLastStart) {
  using mowbite_hold::LockCause;
  using mowbite_hold::LockRecord;
  {
    // Not there: open, nothing to write.
    LockBench bench;
    bench.program.LoadLock(std::nullopt, kWall);
    EXPECT_FALSE(bench.program.TakeLock());
    EXPECT_EQ(bench.Safety()["open"], true);
  }
  {
    // Unreadable: a reset closes it, written back in the form.
    LockBench bench;
    bench.program.LoadLock(std::string("{"), kWall);
    const auto record = mowbite_hold::ParseSafetyLock(*bench.program.TakeLock());
    ASSERT_TRUE(record);
    EXPECT_EQ(record->failures, 0u);
    ASSERT_EQ(record->causes.size(), 1u);
    EXPECT_TRUE(record->causes[0].reset);
    EXPECT_EQ(record->causes[0].reason, "UNREADABLE");
    const json safety = bench.Safety();
    EXPECT_EQ(safety["why"], "RESET");
    EXPECT_TRUE(safety["closed"][0]["step"].is_null());
    EXPECT_EQ(safety["closed"][0]["reason"], "UNREADABLE");
    // Passing a safety step does not take a reset away.
    ASSERT_EQ(bench.Refused(bench.Start("left")), "");
    bench.Run(0.2);
    bench.Bump(true);
    ASSERT_EQ(bench.StepState("left")["state"], "ok");
    EXPECT_FALSE(bench.program.TakeLock());
    EXPECT_EQ(bench.Safety()["open"], false);
  }
  LockCause left;
  left.step = "left";
  left.reason = "LIMIT";
  left.t = 1700000000;
  {
    // Unchanged: closed, nothing to write; its step opens it.
    LockBench bench;
    bench.program.LoadLock(mowbite_hold::SafetyLockText(LockRecord{2, {left}}), kWall);
    EXPECT_FALSE(bench.program.TakeLock());
    EXPECT_EQ(bench.Safety()["why"], "CHECK_FAILED");
    ASSERT_EQ(bench.Refused(bench.Start("left")), "");
    bench.Run(0.2);
    bench.Bump(true);
    EXPECT_EQ(bench.program.TakeLock(), std::optional<std::string>(""));
    EXPECT_EQ(bench.Safety()["open"], true);
  }
  {
    // A cause of a step the list does not have, or of one that is no safety step, becomes a reset.
    LockCause gone = left;
    gone.step = "gone";
    LockCause board = left;
    board.step = "board";
    LockBench bench;
    bench.program.LoadLock(mowbite_hold::SafetyLockText(LockRecord{5, {left, gone, board}}), kWall);
    const auto record = mowbite_hold::ParseSafetyLock(*bench.program.TakeLock());
    ASSERT_TRUE(record);
    EXPECT_EQ(record->failures, 5u);
    ASSERT_EQ(record->causes.size(), 2u);
    EXPECT_EQ(record->causes[0].step, "left");
    EXPECT_TRUE(record->causes[1].reset);
    EXPECT_EQ(record->causes[1].reason, "RESET");
    EXPECT_EQ(bench.Safety()["why"], "CHECK_FAILED");
  }
}

TEST(Lock, AFileThatCannotBeWrittenBlocksEveryChange) {
  LockBench bench;
  ASSERT_EQ(bench.Refused(bench.Start("left")), "");
  bench.Run(0.2);
  bench.Bump(false);
  ASSERT_TRUE(bench.program.TakeLock());
  bench.program.LockWritten(false);
  bench.hardware.emergency_reason = 0;
  EXPECT_EQ(bench.Refused(bench.Start("left")), "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(bench.StepState("left")["blocked"][0]["reason"], "STORAGE_NOT_WRITABLE");
  // It is due again until a write goes through.
  ASSERT_TRUE(bench.program.TakeLock());
  bench.program.LockWritten(true);
  EXPECT_FALSE(bench.program.TakeLock());
  EXPECT_EQ(bench.Refused(bench.Start("left")), "");
}
