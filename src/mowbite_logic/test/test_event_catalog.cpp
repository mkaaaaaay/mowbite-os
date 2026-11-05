#include <gtest/gtest.h>

#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

// Every event the core adds in any test of the logic goes against the catalog of the interface.
void CheckAgainstCatalog(const std::string& type, const json& details) {
  for (const auto& problem : mowbite_api::CheckEvent(type, details)) ADD_FAILURE() << problem;
}

class EventCatalog : public ::testing::Environment {
 public:
  void SetUp() override {
    SetEventObserver(&CheckAgainstCatalog);
  }
  void TearDown() override {
    SetEventObserver(nullptr);
  }
};

[[maybe_unused]] ::testing::Environment* const kEventCatalog =
    ::testing::AddGlobalTestEnvironment(new EventCatalog);

int seen = 0;

void Count(const std::string&, const json&) {
  seen++;
}

}  // namespace

TEST(EventCatalog, SeesTheEventsOfTheCore) {
  SetEventObserver(&Count);
  LogicCore core{LogicConfig()};
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
  LogicCore core{LogicConfig()};
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
