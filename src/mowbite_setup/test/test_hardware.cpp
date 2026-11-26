#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_setup/program.h"

using namespace mowbite_setup;

namespace {

const char* const kList = R"({
  "catalog": 1,
  "restart": "host",
  "steps": [
    {"id": "hardware", "stage": "device", "check": "hardware", "params": {"restart_time": 20}},
    {"id": "stop", "stage": "inputs", "check": "input", "params": {"role": "stop"}}
  ]})";

constexpr double kWall = 1700000000.0;

// The program as the node runs it; Boot is the start after a restart, with the record on disk and the drivers as
// MOWBITE_HARDWARE has them then.
struct Bench {
  explicit Bench(bool drivers = false) : drivers(drivers) {
    Boot();
  }

  void Boot() {
    std::string why;
    const auto steps = ParseSteps(json::parse(list), &why);
    ASSERT_TRUE(steps) << why;
    program = std::make_unique<Program>(*steps, "tron");
    program->SetDrivers(drivers);
    if (!record_text.empty()) {
      EXPECT_TRUE(program->Load(record_text, kWall + now, nullptr));
    }
    program->SetLayer(Layer(), {});
    World idle;
    idle.state = world;
    program->OnWorld(idle);
    Run(0.3);
  }

  void Run(double seconds) {
    for (double end = now + seconds - 1e-9; now < end;) {
      now += 0.1;
      program->Tick(now, kWall + now);
      if (program->TakeChanged()) record_text = program->Text();
      if (const auto layer = program->TakeLayer()) program->LayerWritten(true, now, kWall + now);
      if (program->TakeRestart(now)) {
        ++restarts;
        hardware_asked = program->HardwareRestart();
      }
      for (const Event& event : program->TakeEvents()) {
        json details = event.details;
        details["id"] = "x";
        details["t"] = kWall;
        details["type"] = event.type;
        const auto problems = mowbite_api::CheckEvent(event.type, mowbite_api::Json(details));
        EXPECT_TRUE(problems.empty()) << event.type << ": " << (problems.empty() ? "" : problems[0]);
        if (event.type == "SETUP_STEP") ends.push_back(event.details);
      }
    }
  }

  json Step() {
    const json state = program->State(now, {});
    for (const auto& step : state["steps"]) {
      if (step["id"] == "hardware") return step;
    }
    return nullptr;
  }

  std::string Start() {
    const auto refusal = program->Start({{"step", "hardware"}, {"version", program->version()}}, now, kWall + now);
    return refusal ? refusal->reason : "";
  }

  std::unique_ptr<Program> program;
  std::string list = kList;
  std::string record_text;
  std::string world = "IDLE";
  bool drivers;
  double now = 0;
  int restarts = 0;
  bool hardware_asked = false;
  std::vector<json> ends;
};

}  // namespace

TEST(Hardware, OkWhileTheDriversRun) {
  Bench bench(true);
  EXPECT_EQ(bench.Step()["state"], "ok");
  EXPECT_EQ(bench.Start(), "NOT_OFFERED");
  EXPECT_TRUE(bench.Step()["blocked"].empty());
}

TEST(Hardware, TheStartAsksTheHostAndTheRestartMakesItOk) {
  Bench bench;
  EXPECT_EQ(bench.Step()["state"], "open");
  EXPECT_EQ(bench.Step()["restart"], true);
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.5);
  EXPECT_EQ(bench.restarts, 1);
  EXPECT_TRUE(bench.hardware_asked);
  EXPECT_EQ(bench.program->State(bench.now, {})["restart_pending"], true);
  // mowbite-host wrote MOWBITE_HARDWARE=true and restarted the OS.
  bench.drivers = true;
  bench.Boot();
  ASSERT_FALSE(bench.ends.empty());
  EXPECT_EQ(bench.ends.back()["state"], "ok") << bench.ends.back().dump();
  EXPECT_EQ(bench.Step()["state"], "ok");
  EXPECT_EQ(bench.restarts, 1);
}

TEST(Hardware, StillOffAfterTheRestartIsNotApplied) {
  Bench bench;
  ASSERT_EQ(bench.Start(), "");
  bench.Run(0.5);
  bench.Boot();
  ASSERT_FALSE(bench.ends.empty());
  EXPECT_EQ(bench.ends.back()["reason"], "NOT_APPLIED") << bench.ends.back().dump();
  EXPECT_EQ(bench.Step()["state"], "failed");
  // It may start again.
  EXPECT_EQ(bench.Start(), "");
}

TEST(Hardware, ARestartThatDoesNotComeFails) {
  Bench bench;
  ASSERT_EQ(bench.Start(), "");
  bench.Run(21);
  ASSERT_FALSE(bench.ends.empty());
  EXPECT_EQ(bench.ends.back()["reason"], "RESTART_FAILED") << bench.ends.back().dump();
  EXPECT_FALSE(bench.hardware_asked && bench.program->HardwareRestart());
}

TEST(Hardware, OnlyInIdleAndOnlyByTheHost) {
  Bench busy;
  busy.world = "MOWING";
  busy.Boot();
  EXPECT_EQ(busy.Start(), "BUSY");

  Bench self;
  self.list = std::string(kList).replace(std::string(kList).find("\"host\""), 6, "\"self\"");
  self.Boot();
  EXPECT_EQ(self.Start(), "NOT_OFFERED");
  EXPECT_EQ(self.Step()["blocked"][0]["reason"], "NOT_OFFERED");
}

TEST(Hardware, DriversSwitchedOffByHandOpenItAgain) {
  Bench bench(true);
  EXPECT_EQ(bench.Step()["state"], "ok");
  bench.drivers = false;
  bench.Boot();
  EXPECT_EQ(bench.Step()["state"], "open");
}
