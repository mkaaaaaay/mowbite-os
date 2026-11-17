#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_hold/lock.h"
#include "mowbite_setup/program.h"

using namespace mowbite_setup;

namespace {

const char* const kList = R"({
  "catalog": 1,
  "restart": "host",
  "steps": [
    {"id": "stop", "stage": "inputs", "check": "input", "params": {"role": "stop", "time": 5}},
    {"id": "lift", "stage": "inputs", "check": "input", "params": {"role": "lift"}},
    {"id": "bumper", "stage": "inputs", "check": "input", "params": {"role": "bumper"}},
    {"id": "silence", "stage": "wheels", "check": "silence", "confirm": ["raised", "hand_at_stop"],
     "params": {"rounds": 2, "emergency": ["HIGH_LEVEL"]}},
    {"id": "blade", "stage": "blade", "check": "blade", "params": {"node": "tron_mcu", "emergency": ["HIGH_LEVEL"]},
     "confirm": ["blade_removed", "hand_at_stop"]},
    {"id": "summary", "stage": "safety", "check": "summary", "params": {"node": "tron_mcu", "restart_time": 20},
     "confirm": ["seen_myself"]}
  ]})";

// Every safety step passed; silence stood within 0.25 and 0.31 s.
const char* const kPassed = R"({"schema": 1, "platform": "tron", "steps": {
  "stop": {"state": "ok"}, "lift": {"state": "ok"}, "bumper": {"state": "ok"}, "blade": {"state": "ok"},
  "silence": {"state": "ok", "values": [{"id": "round_1", "value": 0.25}, {"id": "round_2", "value": 0.31},
                                         {"id": "spread", "value": 0.06}]}}})";

constexpr double kWall = 1700000000.0;  // 2023-11-14 22:13:20 UTC
const char* const kToday = "2023-11-14";
const char* const kDateKey = "tron_mcu.safety_measurement_date";
const json kSeen = json::array({"seen_myself"});

// The program as the node runs it, with what a restart of the OS does: a new program from the record, the lock and
// the parameter file on disk, and a driver that takes its date and stand time from that file unless the file kept by
// hand sets them.
struct Bench {
  explicit Bench(const char* record = kPassed, std::set<std::string> hand = {}) : hand(std::move(hand)) {
    record_text = record;
    Boot();
  }

  void Boot() {
    program = std::make_unique<Program>(*ParseSteps(json::parse(kList), nullptr), "tron");
    EXPECT_TRUE(program->Load(record_text, kWall + now, nullptr));
    program->LoadLock(lock_text, kWall + now);
    Layer layer;
    if (!layer_text.empty()) {
      EXPECT_TRUE(layer.Parse(layer_text, nullptr));
    }
    program->SetLayer(layer, hand);
    capabilities.firmware = {{"chassis", "0.6.36"}};
    if (!hand.count(kDateKey)) {
      const json date = layer.Get("tron_mcu", mowbite_api::kSafetyDate);
      capabilities.safety_measurement_date = date.is_string() && driver_takes ? date.get<std::string>() : "";
    }
    if (!hand.count("tron_mcu.failsafe_without_host")) {
      const json failsafe = layer.Get("tron_mcu", mowbite_api::kFailsafeWithoutHost);
      capabilities.failsafe_without_host_s = failsafe.is_number() && driver_takes ? failsafe.get<double>() : -1;
    }
    capabilities.silence_max_s = 1.0;
    program->OnCapabilities(capabilities);
    program->OnClock(clock_ok, clock_ok ? "" : "never_synced");
    World idle;
    idle.state = "IDLE";
    program->OnWorld(idle);
    hardware.telemetry_ok = true;
    Run(0.3);
  }

  void Run(double seconds) {
    for (double end = now + seconds - 1e-9; now < end;) {
      now += 0.1;
      program->OnHardware(hardware, now);
      program->Tick(now, kWall + now);
      if (const auto lock = program->TakeLock()) {
        lock_text = lock->empty() ? std::nullopt : lock;
        program->LockWritten(true);
        ++lock_writes;
      }
      if (program->TakeChanged()) record_text = program->Text();
      if (const auto layer = program->TakeLayer()) {
        layer_text = *layer;
        program->LayerWritten(true, now, kWall + now);
      }
      if (program->TakeRestart(now)) ++restarts;
    }
  }

  json Step(const std::string& id) {
    const json state = program->State(now, {});
    for (const auto& step : state["steps"]) {
      if (step["id"] == id) return step;
    }
    return nullptr;
  }
  json Safety() {
    return program->State(now, {})["safety"];
  }
  json Layered(const char* key) {
    Layer layer;
    if (!layer_text.empty()) layer.Parse(layer_text, nullptr);
    return layer.Get("tron_mcu", key);
  }
  std::string Start(json confirm = kSeen) {
    const auto refusal = program->Start({{"step", "summary"}, {"version", program->version()}, {"confirm", confirm}},
                                        now, kWall + now);
    return refusal ? refusal->reason : "";
  }
  std::vector<json> Events(const char* type = "SETUP") {
    std::vector<json> out;
    for (const Event& event : program->TakeEvents()) {
      json details = event.details;
      details["id"] = "x";
      details["t"] = kWall;
      details["type"] = event.type;
      const auto problems = mowbite_api::CheckEvent(event.type, mowbite_api::Json(details));
      EXPECT_TRUE(problems.empty()) << event.type << ": " << (problems.empty() ? "" : problems[0]);
      if (event.type == type) out.push_back(event.details);
    }
    return out;
  }
  std::vector<std::string> Blocked() {
    std::vector<std::string> reasons;
    const json step = Step("summary");
    for (const auto& entry : step["blocked"]) reasons.push_back(entry["reason"].get<std::string>());
    return reasons;
  }
  bool Resets() {
    if (!lock_text) return false;
    const auto record = mowbite_hold::ParseSafetyLock(*lock_text);
    for (const auto& cause : record->causes) {
      if (cause.reset) return true;
    }
    return false;
  }

  std::unique_ptr<Program> program;
  std::set<std::string> hand;
  std::string record_text;
  std::optional<std::string> lock_text;
  std::string layer_text;
  Capabilities capabilities;
  Hardware hardware;
  bool clock_ok = true;
  bool driver_takes = true;
  double now = 0;
  int restarts = 0;
  int lock_writes = 0;
};

}  // namespace

TEST(Summary, TheStepListHasItAfterASilence) {
  std::string why;
  json list = json::parse(kList);
  ASSERT_TRUE(ParseSteps(list, &why)) << why;
  json early = list;
  early["steps"].erase(3);
  EXPECT_FALSE(ParseSteps(early, &why));
  EXPECT_NE(why.find("comes after a step silence"), std::string::npos) << why;
  json twice = list;
  twice["steps"].push_back(list["steps"][5]);
  twice["steps"][6]["id"] = "summary_2";
  EXPECT_FALSE(ParseSteps(twice, &why));
  json without = list;
  without.erase("restart");
  EXPECT_FALSE(ParseSteps(without, &why));
}

TEST(Summary, WritesTheDateRestartsAndOpensTheLock) {
  Bench bench;
  EXPECT_EQ(bench.Safety()["why"], "NO_SAFETY_MEASUREMENT");
  EXPECT_TRUE(bench.Blocked().empty());
  EXPECT_EQ(bench.Start(json::array()), "CONFIRM_MISSING");
  ASSERT_EQ(bench.Start(), "");
  json step = bench.Step("summary");
  EXPECT_EQ(step["state"], "running");
  EXPECT_EQ(step["restart"], true);
  EXPECT_EQ(bench.program->State(bench.now, {})["restart_pending"], true);
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 1);
  EXPECT_EQ(bench.Layered(mowbite_api::kSafetyDate), kToday);
  EXPECT_EQ(bench.Layered(mowbite_api::kFailsafeWithoutHost), 0.31);
  // Until it passes, the date the program wrote does not open the lock.
  EXPECT_TRUE(bench.Resets());
  EXPECT_EQ(json::parse(bench.record_text)["steps"]["summary"]["applied"]["date"], kToday);
  bench.Events();

  bench.Boot();
  step = bench.Step("summary");
  EXPECT_EQ(step["state"], "ok");
  EXPECT_EQ(step["values"][0]["value"], kToday);
  EXPECT_EQ(step["values"][1]["value"], 0.31);
  EXPECT_FALSE(bench.Resets());
  EXPECT_FALSE(bench.lock_text);
  const auto events = bench.Events();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events[0], (json{{"what", "safety_opened"}, {"date", kToday}}));
  const json safety = bench.Safety();
  EXPECT_EQ(safety["open"], true);
  EXPECT_EQ(safety["source"], "program");
  EXPECT_EQ(safety["firmware"], "chassis 0.6.36");
  EXPECT_EQ(safety["date"], kToday);
  EXPECT_FALSE(json::parse(bench.record_text)["steps"]["summary"].contains("applied"));
  EXPECT_EQ(bench.restarts, 1);
}

TEST(Summary, NamesWhatKeepsItFromStarting) {
  Bench bench(R"({"schema": 1, "platform": "tron", "steps": {"stop": {"state": "ok"}, "lift": {"state": "skipped"},
                  "silence": {"state": "ok", "values": [{"id": "round_1", "value": 0.2}]}}})");
  bench.program->OnClock(false, "never_synced");
  bench.hardware.emergency_reason = EmergencyBit("STOP");
  bench.Run(0.2);
  EXPECT_EQ(bench.Blocked(), (std::vector<std::string>{"SAFETY_STEPS", "CLOCK_UNSAFE", "EMERGENCY"}));
  EXPECT_EQ(bench.Start(), "SAFETY_STEPS");
  const auto refusal =
      bench.program->Start({{"step", "summary"}, {"version", bench.program->version()}, {"confirm", kSeen}}, bench.now,
                           kWall + bench.now);
  ASSERT_TRUE(refusal);
  EXPECT_EQ(refusal->data["steps"], json::array({"lift", "bumper", "blade"}));
  EXPECT_EQ(refusal->data["skipped"], json::array({"lift"}));
  EXPECT_EQ(bench.restarts, 0);
  EXPECT_TRUE(bench.layer_text.empty());
}

TEST(Summary, ADateKeptByHandCountsAndResetLeavesTheLock) {
  Bench bench(kPassed, {kDateKey, "tron_mcu.failsafe_without_host"});
  bench.capabilities.safety_measurement_date = "2026-11-08";
  bench.capabilities.failsafe_without_host_s = 0.3;
  bench.program->OnCapabilities(bench.capabilities);
  EXPECT_EQ(bench.Safety()["source"], "hand");
  EXPECT_EQ(bench.Safety()["open"], true);
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.2);
  // The date by hand stays in force: no reset for the time of the step.
  EXPECT_FALSE(bench.Resets());
  EXPECT_EQ(bench.restarts, 1);
  bench.Boot();
  EXPECT_EQ(bench.Step("summary")["state"], "ok");
  const json safety = bench.Safety();
  EXPECT_EQ(safety["open"], true);
  EXPECT_EQ(safety["source"], "hand");
  EXPECT_TRUE(safety["firmware"].is_null());
  bench.Events();

  // setup.reset takes what the program wrote out of the file, the lock by hand stays as it is.
  EXPECT_EQ(bench.Layered(mowbite_api::kSafetyDate), kToday);
  ASSERT_FALSE(bench.program->Reset({{"version", bench.program->version()}}, kWall + bench.now));
  bench.Run(0.2);
  EXPECT_TRUE(bench.Layered(mowbite_api::kSafetyDate).is_null());
  EXPECT_FALSE(bench.Resets());
  EXPECT_EQ(bench.Safety()["open"], true);
}

TEST(Summary, AnEmptyDateByHandIsNotApplied) {
  Bench bench(kPassed, {kDateKey});
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.2);
  bench.Boot();
  const json step = bench.Step("summary");
  EXPECT_EQ(step["state"], "failed");
  EXPECT_EQ(step["result"]["reason"], "NOT_APPLIED");
  const auto events = bench.Events("SETUP_STEP");
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["key"], kDateKey);
  bench.Run(0.2);
  // The file gets back what it had.
  EXPECT_TRUE(bench.Layered(mowbite_api::kSafetyDate).is_null());
  EXPECT_EQ(bench.Safety()["open"], false);
  EXPECT_EQ(bench.Safety()["why"], "NO_SAFETY_MEASUREMENT");
}

TEST(Summary, ADriverThatDoesNotTakeItLeavesTheLockClosed) {
  Bench bench;
  bench.driver_takes = false;
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.2);
  bench.Boot();
  EXPECT_EQ(bench.Step("summary")["result"]["reason"], "NOT_APPLIED");
  bench.Run(0.2);
  EXPECT_TRUE(bench.Layered(mowbite_api::kSafetyDate).is_null());
  EXPECT_TRUE(bench.Resets());
  // Should the driver show the date later after all, the reset holds the lock.
  bench.driver_takes = true;
  bench.layer_text = "";
  bench.capabilities.safety_measurement_date = kToday;
  bench.program->OnCapabilities(bench.capabilities);
  EXPECT_EQ(bench.Safety()["open"], false);
  EXPECT_EQ(bench.Safety()["why"], "RESET");
}

TEST(Summary, StoppedOrWithoutRestartItGivesTheFileBack) {
  Bench bench;
  // An emergency after the start holds the restart back.
  ASSERT_EQ(bench.Start(), "");
  bench.hardware.emergency_reason = EmergencyBit("STOP");
  bench.Run(0.5);
  EXPECT_EQ(bench.restarts, 0);
  EXPECT_EQ(bench.Blocked(), std::vector<std::string>{"EMERGENCY"});
  ASSERT_FALSE(bench.program->Abort(json{{"step", "summary"}}, bench.now, kWall + bench.now));
  bench.Run(0.2);
  EXPECT_EQ(bench.Step("summary")["state"], "open");
  EXPECT_TRUE(bench.Layered(mowbite_api::kSafetyDate).is_null());
  EXPECT_EQ(bench.Safety()["why"], "RESET");

  // The restart asked for does not come.
  bench.hardware.emergency_reason = 0;
  bench.Run(0.2);
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.2);
  EXPECT_EQ(bench.restarts, 1);
  EXPECT_EQ(bench.Blocked(), std::vector<std::string>{});
  bench.Run(20.5);
  EXPECT_EQ(bench.Step("summary")["result"]["reason"], "RESTART_FAILED");
  EXPECT_TRUE(bench.Layered(mowbite_api::kSafetyDate).is_null());
  EXPECT_TRUE(bench.Resets());
}

TEST(Summary, AFailedSafetyStepOpensItAgainAndResetClosesTheLock) {
  Bench bench;
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.2);
  bench.Boot();
  ASSERT_EQ(bench.Step("summary")["state"], "ok");
  bench.Events();

  // stop fails: summary is open again, the lock closes for stop alone.
  ASSERT_FALSE(bench.program->Start({{"step", "stop"}, {"version", bench.program->version()}}, bench.now,
                                    kWall + bench.now));
  bench.Run(5.3);
  ASSERT_FALSE(bench.program->Answer(
      {{"step", "stop"}, {"version", bench.program->version()}, {"values", {{"pressed", "yes"}}}}, bench.now,
      kWall + bench.now));
  bench.Run(0.2);
  EXPECT_EQ(bench.Step("stop")["state"], "failed");
  EXPECT_EQ(bench.Step("summary")["state"], "open");
  EXPECT_EQ(bench.Safety()["why"], "CHECK_FAILED");
  EXPECT_EQ(bench.Blocked(), std::vector<std::string>{"SAFETY_STEPS"});
  bench.Events();

  // setup.reset: the program opened the lock, so it closes with a reset; the settings leave the file.
  ASSERT_FALSE(bench.program->Reset({{"version", bench.program->version()}}, kWall + bench.now));
  bench.Run(0.2);
  EXPECT_TRUE(bench.Resets());
  EXPECT_TRUE(bench.Layered(mowbite_api::kSafetyDate).is_null());
  EXPECT_TRUE(bench.Layered(mowbite_api::kFailsafeWithoutHost).is_null());
  const json safety = bench.Safety();
  EXPECT_EQ(safety["source"], "program");
  EXPECT_EQ(safety["open"], false);
}
