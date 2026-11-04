#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>

#include "mowbite_api/catalog.h"

using namespace mowbite_api;

namespace {

// The table of the logic before the catalog (reasons.cpp at 29bc246), kept to show that RateReason did not change.
Rating OldRateReason(const std::string& event, const std::string& code, const std::string& action) {
  const auto in = [&code](const std::set<std::string>& names) { return names.count(code) != 0; };
  const std::set<std::string> position = {"NO_POSITION", "POSE_LOST", "POSITION_ACCURACY", "HEADING", "NO_HEADING"};
  if (event == "MOWING_REFUSED" || event == "DRIVE_REFUSED" || event == "RECORDING_REFUSED") {
    if (event == "RECORDING_REFUSED" && action == "point" && code == "POSITION_ACCURACY") return {"warning", "auto"};
    if (in({"BUSY", "STILL_STOPPING", "STILL_SAVING", "ON_DOCK", "RAIN", "ALREADY_THERE", "HOLD"})) {
      return {"info", "user"};
    }
    if (in({"EMERGENCY", "NO_SAFETY_MEASUREMENT", "MAP_MISMATCH", "MAP_UNREADABLE", "MAP_UNKNOWN", "PLANNER_MISSING",
            "FOLLOWER_MISSING", "SAVE_FAILED"})) {
      return {"error", "user"};
    }
    return {"warning", "user"};
  }
  if (event == "CALIBRATION_REFUSED") {
    if (in({"BUSY", "STILL_STOPPING", "ON_DOCK", "HOLD"})) return {"info", "user"};
    if (in({"EMERGENCY", "NO_SAFETY_MEASUREMENT", "MAP_MISMATCH", "MAP_UNREADABLE", "MAP_UNKNOWN",
            "CALIBRATION_MISSING", "SENSOR_MISSING", "IMU_GRAVITY"})) {
      return {"error", "user"};
    }
    return {"warning", "user"};
  }
  if (event == "CALIBRATION_FAILED") {
    if (code == "STOPPED") return {"info", "user"};
    if (in(position) || in({"NO_RTK", "BATTERY_LOW", "NO_ROOM", "SLOPE"})) return {"warning", "user"};
    return {"error", "user"};
  }
  if (event == "STATE") {
    if (code == "MANUAL") return {"info", "user"};
    if (in(position)) return {"warning", "auto"};
    return {"error", "user"};
  }
  if (event == "DRIVE") {
    if (in({"DONE", "ALREADY_THERE", "STOPPED", "CANCELED"})) return {"info", "user"};
    if (in(position)) return {"warning", "user"};
    return {"error", "user"};
  }
  if (event == "POSITION_JUMP") return {"warning", "auto"};
  if (event == "AREA_SKIPPED") return {"info", "auto"};
  if (event == "UNDOCKING_FAILED") return {"error", "user"};
  if (event == "DOCKING_RETRY") return {"warning", "auto"};
  if (event == "DOCKING_FAILED") return {"error", "user"};
  if (event == "EMERGENCY_RESET_REFUSED") return {code == "STILL_TILTED" ? "warning" : "error", "user"};
  return {"warning", "user"};
}

// The reasons of the catalog for this event and one it does not know.
std::set<std::string> CodesOf(const std::string& type) {
  std::set<std::string> codes = {"SOMETHING_NEW"};
  if (const EventEntry* event = FindEvent(type)) {
    for (const auto& reason : event->reasons) codes.insert(reason.code);
  }
  return codes;
}

}  // namespace

// RateReason reads the catalog now and rates every reason of an event as the table did, also one it does not know.
TEST(Catalog, RateReasonAsTheTableBefore) {
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED", "RECORDING_REFUSED", "CALIBRATION_REFUSED",
                            "CALIBRATION_FAILED", "STATE", "DRIVE", "POSITION_JUMP", "AREA_SKIPPED", "UNDOCKING_FAILED",
                            "DOCKING_RETRY", "DOCKING_FAILED", "EMERGENCY_RESET_REFUSED", "NOT_AN_EVENT"}) {
    for (const auto& code : CodesOf(event)) {
      for (const char* action : {"", "point", "collect_point", "record_dock", "start_area_recording"}) {
        const Rating now = RateReason(event, code, action);
        const Rating before = OldRateReason(event, code, action);
        EXPECT_STREQ(now.severity, before.severity) << event << " " << code << " " << action;
        EXPECT_STREQ(now.resume, before.resume) << event << " " << code << " " << action;
      }
    }
  }
}

TEST(Catalog, EntriesAreWellFormed) {
  const std::set<std::string> severities = {"info", "warning", "error"};
  const std::set<std::string> resumes = {"auto", "user"};
  std::set<std::string> types;
  for (const auto& event : Events()) {
    EXPECT_TRUE(types.insert(event.type).second) << "twice: " << event.type;
    EXPECT_TRUE(std::set<std::string>({"logic", "map", "ntrip", "bridge"}).count(event.emitter)) << event.type;
    EXPECT_FALSE(event.text.empty()) << event.type;
    EXPECT_TRUE(severities.count(event.other_severity)) << event.type;
    EXPECT_TRUE(resumes.count(event.other_resume)) << event.type;
    // One name, one type and unit within an event, whoever brings the field.
    // The unit may differ only between the reasons (limit), where x-reasons names it.
    std::map<std::string, std::pair<const Field*, bool>> seen;
    const auto add = [&](const Field& field, bool of_reason) {
      EXPECT_FALSE(field.text.empty()) << event.type << "." << field.name;
      const auto known = seen.find(field.name);
      if (known == seen.end()) {
        seen[field.name] = {&field, of_reason};
        return;
      }
      const Field* first = known->second.first;
      EXPECT_EQ(static_cast<int>(first->type), static_cast<int>(field.type)) << event.type << "." << field.name;
      if (!of_reason || !known->second.second) {
        EXPECT_EQ(first->unit, field.unit) << event.type << "." << field.name;
      }
    };
    for (const auto& field : Envelope()) add(field, false);
    if (event.emitter == "logic") {
      for (const auto& field : LogicFields()) add(field, false);
    }
    for (const auto& field : event.fields) add(field, false);
    std::set<std::string> codes;
    for (const auto& reason : event.reasons) {
      EXPECT_TRUE(severities.count(reason.severity)) << event.type << " " << reason.code;
      EXPECT_TRUE(resumes.count(reason.resume)) << event.type << " " << reason.code;
      EXPECT_FALSE(reason.text.empty()) << event.type << " " << reason.code;
      if (reason.action.empty()) {
        EXPECT_TRUE(codes.insert(reason.code).second) << event.type << " " << reason.code;
      }
      for (const auto& field : reason.fields) add(field, true);
    }
    if (!event.reasons.empty()) {
      EXPECT_TRUE(seen.count("reason")) << event.type << " has reasons but no field reason";
    }
  }
  std::set<std::string> names;
  for (const auto& command : Commands()) {
    EXPECT_TRUE(names.insert(command.name).second) << command.name;
    EXPECT_FALSE(command.text.empty()) << command.name;
    if (!command.refused.empty()) {
      EXPECT_NE(FindEvent(command.refused), nullptr) << command.name;
    }
  }
  for (const auto& field : RobotState()) EXPECT_FALSE(field.text.empty()) << field.name;
}

TEST(Catalog, EmergencyBits) {
  ASSERT_EQ(EmergencyBits().size(), 21u);
  for (size_t i = 0; i < EmergencyBits().size(); i++) {
    EXPECT_EQ(EmergencyBits()[i].bit, static_cast<int>(i));
    EXPECT_FALSE(EmergencyBits()[i].text.empty()) << i;
  }
  EXPECT_EQ(EmergencyNames(0), "");
  EXPECT_EQ(EmergencyNames((1 << 0) | (1 << 3)), "LATCH, LIFT");
  EXPECT_EQ(EmergencyNames((1 << 13) | (1 << 14) | (1 << 15)), "TILT, FENCE, STUCK");
  EXPECT_EQ(EmergencyNames((1 << 18) | (1 << 19) | (1 << 20)), "WHEEL_RUNAWAY, FOREIGN_HOST, DRIVER_STALL");
  EXPECT_EQ(EmergencyNames(1 << 25), "");
}

// Every Reason("NAME") in the sources of the OS is in the catalog, for some event.
TEST(Catalog, EveryReasonOfTheSources) {
  std::set<std::string> codes;
  for (const auto& event : Events()) {
    for (const auto& reason : event.reasons) codes.insert(reason.code);
  }
  const std::regex literal(R"(Reason\("([A-Za-z_]+)\")");
  int found = 0;
  const std::filesystem::path root = std::filesystem::canonical(MOWBITE_SOURCE_ROOT);
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    const std::string path = "/" + std::filesystem::relative(entry.path(), root).string();
    const std::string ext = entry.path().extension().string();
    if (!entry.is_regular_file() || (ext != ".cpp" && ext != ".h")) continue;
    if (path.find("/test/") != std::string::npos || path.find("/mowbite_api/") != std::string::npos) continue;
    std::ifstream in(entry.path());
    std::stringstream text;
    text << in.rdbuf();
    const std::string source = text.str();
    for (std::sregex_iterator it(source.begin(), source.end(), literal), end; it != end; ++it) {
      found++;
      EXPECT_TRUE(codes.count((*it)[1])) << path << ": " << (*it)[1] << " is not in the catalog";
    }
  }
  EXPECT_GT(found, 40);
}
