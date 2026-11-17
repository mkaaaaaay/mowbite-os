#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

// With MOWBITE_EVENT_TRACE set, every event of the core goes as a line "test, type, details" into that file: the trace
// of a commit that only moves code has to equal the one before it.
FILE* trace = nullptr;

void Trace(const std::string& type, const json& details) {
  if (!trace) return;
  const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string name = test ? std::string(test->test_suite_name()) + "." + test->name() : "-";
  std::fprintf(trace, "%s\t%s\t%s\n", name.c_str(), type.c_str(), details.dump().c_str());
}

// Every event the core adds in any test of the logic goes against the catalog of the interface.
void CheckAgainstCatalog(const std::string& type, const json& details) {
  Trace(type, details);
  for (const auto& problem : mowbite_api::CheckEvent(type, details)) ADD_FAILURE() << problem;
}

class EventCatalog : public ::testing::Environment {
 public:
  void SetUp() override {
    if (const char* path = std::getenv("MOWBITE_EVENT_TRACE")) {
      trace = std::fopen(path, "w");
      if (!trace) ADD_FAILURE() << "cannot write the event trace " << path;
    }
    SetEventObserver(&CheckAgainstCatalog);
  }
  void TearDown() override {
    SetEventObserver(nullptr);
    if (trace) std::fclose(trace);
    trace = nullptr;
  }
};

[[maybe_unused]] ::testing::Environment* const kEventCatalog =
    ::testing::AddGlobalTestEnvironment(new EventCatalog);

int seen = 0;

void Count(const std::string& type, const json& details) {
  Trace(type, details);
  seen++;
}

}  // namespace

TEST(EventCatalog, SeesTheEventsOfTheCore) {
  SetEventObserver(&Count);
  TestCore core{LogicConfig()};
  // Without a safety measurement every drive is refused with an event.
  EXPECT_FALSE(core.StartDrive(1.0, 2.0).empty());
  const size_t events = core.TakeEvents().size();
  SetEventObserver(&CheckAgainstCatalog);
  EXPECT_GT(events, 0u);
  EXPECT_EQ(seen, static_cast<int>(events));
}

TEST(EventCatalog, FindsAnEventOutsideOfIt) {
  EXPECT_FALSE(mowbite_api::CheckEvent("STATE", json{{"state", "IDLE"}, {"x", 0.0}, {"y", 0.0}, {"odd", 1}}).empty());
  EXPECT_FALSE(mowbite_api::CheckEvent("NOT_A_TYPE", json::object()).empty());
}

// The sensors of the logic are those of the interface with source logic, with the same name, unit and kind.
TEST(EventCatalog, SensorsOfTheLogicAreThoseOfTheInterface) {
  TestCore core{LogicConfig()};
  size_t logic = 0;
  for (const auto& sensor : mowbite_api::Sensors()) logic += sensor.source == "logic" ? 1 : 0;
  const auto sensors = core.Sensors();
  EXPECT_EQ(sensors.size(), logic);
  for (const auto& sensor : sensors) {
    const mowbite_api::SensorEntry* entry = mowbite_api::FindSensor(sensor.id);
    ASSERT_NE(entry, nullptr) << sensor.id;
    EXPECT_EQ(entry->source, "logic") << sensor.id;
    EXPECT_EQ(entry->name, sensor.name) << sensor.id;
    EXPECT_EQ(entry->unit, sensor.unit) << sensor.id;
    EXPECT_EQ(entry->text, sensor.text) << sensor.id;
  }
}
