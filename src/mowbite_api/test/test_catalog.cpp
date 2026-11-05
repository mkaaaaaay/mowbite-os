#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
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

// RateReason reads the catalog now and rates every reason of an event as the table did, also one it does not know;
// the pauses of api 5 (O11) are rated anew.
TEST(Catalog, RateReasonAsTheTableBefore) {
  const std::map<std::string, Rating> paused_since_api_5 = {{"BATTERY_HOT", {"warning", "user"}},
                                                             {"MOTOR_HOT", {"warning", "user"}},
                                                             {"RAIN", {"info", "user"}},
                                                             {"DARK", {"info", "user"}}};
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED", "RECORDING_REFUSED", "CALIBRATION_REFUSED",
                            "CALIBRATION_FAILED", "STATE", "DRIVE", "POSITION_JUMP", "AREA_SKIPPED", "UNDOCKING_FAILED",
                            "DOCKING_RETRY", "DOCKING_FAILED", "EMERGENCY_RESET_REFUSED", "NOT_AN_EVENT"}) {
    for (const auto& code : CodesOf(event)) {
      for (const char* action : {"", "point", "collect_point", "record_dock", "start_area_recording"}) {
        const Rating now = RateReason(event, code, action);
        const auto anew = paused_since_api_5.find(code);
        const Rating before = std::string(event) == "STATE" && anew != paused_since_api_5.end()
                                  ? anew->second
                                  : OldRateReason(event, code, action);
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
      EXPECT_NE(FindReasonGroup(command.refused), nullptr) << command.name;
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
}

// Home Assistant reads the names from the log line with [A-Z_, ]: each bit, set alone, gives a name that fits it.
TEST(Catalog, EmergencyNamesOfEachBit) {
  const std::regex line("^[A-Z_, ]+$");
  std::set<std::string> names;
  for (const auto& bit : EmergencyBits()) {
    EXPECT_TRUE(std::regex_match(bit.name, std::regex("^[A-Z_]+$"))) << bit.name;
    EXPECT_NE(bit.name, kUnknownEmergency);
    EXPECT_TRUE(names.insert(bit.name).second) << bit.name;
  }
  for (int i = 0; i < 32; i++) {
    const int bits = static_cast<int>(1u << i);
    const std::string expected = i < static_cast<int>(EmergencyBits().size()) ? EmergencyBits()[i].name : "UNKNOWN";
    EXPECT_EQ(EmergencyNames(bits), expected) << i;
    EXPECT_TRUE(std::regex_match(EmergencyNames(bits), line)) << i;
  }
  // One UNKNOWN for any number of bits without a name, after the named ones.
  EXPECT_EQ(EmergencyNames((1 << 21) | (1 << 25) | static_cast<int>(1u << 31)), "UNKNOWN");
  EXPECT_EQ(EmergencyNames((1 << 0) | (1 << 25) | (1 << 3) | (1 << 30)), "LATCH, LIFT, UNKNOWN");
  const std::string all = EmergencyNames(-1);
  EXPECT_TRUE(std::regex_match(all, line));
  EXPECT_EQ(all.substr(all.size() - 9), ", UNKNOWN");
  EXPECT_EQ(std::count(all.begin(), all.end(), ','), static_cast<long>(EmergencyBits().size()));
}

// Every bit the message of hw/status or a driver, the safety filter or the logic defines has the same name in the
// catalog: REASON_IMU_TIMEOUT and kReasonImuTimeout are bit 16, IMU_TIMEOUT.
TEST(Catalog, EmergencyNamesOfTheSources) {
  const auto name_of = [](int bit) {
    for (const auto& entry : EmergencyBits()) {
      if (entry.bit == bit) return entry.name;
    }
    return std::string();
  };
  const std::filesystem::path root = std::filesystem::canonical(MOWBITE_SOURCE_ROOT);
  const auto read = [](const std::filesystem::path& path) {
    std::ifstream in(path);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
  };

  const std::string msg = read(root / "mowbite_msgs/msg/HardwareStatus.msg");
  const std::regex constant(R"(uint32 REASON_([A-Z_]+) = (\d+))");
  int in_msg = 0;
  for (std::sregex_iterator it(msg.begin(), msg.end(), constant), end; it != end; ++it) {
    const unsigned long value = std::stoul((*it)[2]);
    ASSERT_TRUE(value != 0 && (value & (value - 1)) == 0) << (*it)[1] << " is no single bit";
    int bit = 0;
    while ((1ul << bit) != value) bit++;
    EXPECT_EQ(name_of(bit), (*it)[1].str()) << "REASON_" << (*it)[1] << " = 1 << " << bit;
    in_msg++;
  }
  EXPECT_GE(in_msg, 15);

  // kReasonTilt = 1u << 13 and kEmergencyLift = 1 << 3, kLift = 1u << 3 in a namespace reason; the name in CamelCase.
  const std::regex shift(R"(k(?:Reason|Emergency)([A-Z][A-Za-z]*) = 1u? << (\d+);)");
  const std::regex in_namespace(R"(namespace reason \{([^}]*)\})");
  const std::regex plain(R"(k([A-Z][A-Za-z]*) = 1u? << (\d+);)");
  const auto upper_snake = [](const std::string& camel) {
    std::string out;
    for (char c : camel) {
      if (std::isupper(static_cast<unsigned char>(c)) && !out.empty()) out += '_';
      out += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return out;
  };
  int in_sources = 0;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    const std::string path = "/" + std::filesystem::relative(entry.path(), root).string();
    const std::string ext = entry.path().extension().string();
    if (!entry.is_regular_file() || (ext != ".cpp" && ext != ".h")) continue;
    if (path.find("/test/") != std::string::npos) continue;
    const std::string source = read(entry.path());
    const auto check = [&](const std::string& text, const std::regex& pattern) {
      for (std::sregex_iterator it(text.begin(), text.end(), pattern), end; it != end; ++it) {
        const int bit = std::stoi((*it)[2]);
        EXPECT_EQ(name_of(bit), upper_snake((*it)[1])) << path << ": " << (*it)[0];
        in_sources++;
      }
    };
    check(source, shift);
    for (std::sregex_iterator it(source.begin(), source.end(), in_namespace), end; it != end; ++it) {
      check((*it)[1].str(), plain);
    }
  }
  EXPECT_GE(in_sources, 50);
}

// Every Reason("NAME") in the sources of the OS is in the catalog, for some event.
TEST(Catalog, EveryReasonOfTheSources) {
  std::set<std::string> codes;
  for (const auto& event : Events()) {
    for (const auto& reason : event.reasons) codes.insert(reason.code);
  }
  for (const auto& group : Refusals()) {
    for (const auto& reason : group.reasons) codes.insert(reason.code);
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

// COMMAND_REFUSED and REQUEST_REFUSED are groups of answers only: rated like an event, but never one.
TEST(Catalog, RefusalsAreNoEvents) {
  ASSERT_EQ(Refusals().size(), 2u);
  const EventEntry& answers = Refusals().front();
  EXPECT_EQ(answers.type, "COMMAND_REFUSED");
  EXPECT_EQ(FindEvent("COMMAND_REFUSED"), nullptr);
  EXPECT_EQ(FindReasonGroup("COMMAND_REFUSED"), &answers);
  for (const char* code : {"WRONG_STATE", "NO_JOB", "CHANGED"}) {
    const Rating rating = RateReason("COMMAND_REFUSED", code);
    EXPECT_STREQ(rating.severity, "info") << code;
    EXPECT_STREQ(rating.resume, "user") << code;
  }
  const EventEntry& requests = Refusals().back();
  EXPECT_EQ(requests.type, "REQUEST_REFUSED");
  EXPECT_EQ(FindEvent("REQUEST_REFUSED"), nullptr);
  EXPECT_EQ(FindReasonGroup("REQUEST_REFUSED"), &requests);
  // The fields of COMMAND_REFUSED but event_id, so one reader takes the data of both.
  std::vector<std::string> names, expected;
  for (const auto& field : requests.fields) names.push_back(field.name + (field.required ? "!" : ""));
  for (const auto& field : answers.fields) {
    if (field.name != "event_id") expected.push_back(field.name + (field.required ? "!" : ""));
  }
  EXPECT_EQ(names, expected);
  for (const char* code : {"RESPONSE_TOPIC", "CORRELATION_DATA"}) {
    const Rating rating = RateReason("REQUEST_REFUSED", code);
    EXPECT_STREQ(rating.severity, "error") << code;
    EXPECT_STREQ(rating.resume, "user") << code;
  }
}

// Every action of the logic but manual mowing stands for exactly one command, with params that command takes.
TEST(Catalog, ActionsAndCommands) {
  std::set<std::string> actions;
  for (const auto& command : Commands()) {
    const bool logic = std::any_of(command.params.begin(), command.params.end(),
                                   [](const Field& param) { return param.name == "client"; });
    EXPECT_EQ(logic, command.name.rfind("system.", 0) != 0) << command.name;
    for (const auto& entry : command.actions) {
      EXPECT_TRUE(actions.insert(entry.action).second) << entry.action;
      Json params;
      EXPECT_EQ(CommandOfAction(entry.action, &params), &command) << entry.action;
      EXPECT_EQ(params, entry.params) << entry.action;
      std::string field;
      EXPECT_EQ(CheckParams(command, params, &field), "") << entry.action;
    }
  }
  EXPECT_EQ(actions.size(), 21u);
  EXPECT_EQ(CommandOfAction("mower_logic:area_recording/start_manual_mowing", nullptr), nullptr);
  EXPECT_EQ(FindCommand("mowing.start")->actions[0].action, "mower_logic:idle/start_mowing");
  EXPECT_EQ(FindCommand("no.such"), nullptr);
}

TEST(Catalog, ParamsOfACommand) {
  const Command& save = *FindCommand("recording.save");
  std::string field;
  EXPECT_EQ(CheckParams(save, Json{{"type", "mow"}, {"client", "ha"}}, &field), "");
  EXPECT_NE(CheckParams(save, Json::object(), &field), "");
  EXPECT_EQ(field, "type");
  EXPECT_NE(CheckParams(save, Json{{"type", "lawn"}}, &field), "");
  EXPECT_EQ(field, "type");
  EXPECT_NE(CheckParams(save, Json{{"type", "mow"}, {"area", 1}}, &field), "");
  EXPECT_EQ(field, "area");
  EXPECT_NE(CheckParams(save, Json::array({"mow"}), &field), "");
  EXPECT_EQ(field, "");
  const Command& pause = *FindCommand("mowing.pause");
  for (const char* good : {"app", "ha", "a", "mowbite-app_2", "abcdefghijklmnop"}) {
    EXPECT_EQ(CheckParams(pause, Json{{"client", good}}, &field), "") << good;
  }
  for (const Json& bad : {Json("App"), Json("1app"), Json(""), Json("abcdefghijklmnopq"), Json("a b"), Json("a/b"),
                          Json(7)}) {
    EXPECT_NE(CheckParams(pause, Json{{"client", bad}}, &field), "") << bad.dump();
    EXPECT_EQ(field, "client");
  }
  const Command& skip = *FindCommand("mowing.skip_area");
  EXPECT_EQ(CheckParams(skip, Json{{"area", 2}}, &field), "");
  EXPECT_EQ(CheckParams(skip, Json{{"area", 2.0}}, &field), "");
  EXPECT_NE(CheckParams(skip, Json{{"area", 2.5}}, &field), "");
  EXPECT_NE(CheckParams(*FindCommand("recording.auto_points"), Json{{"enabled", 1}}, &field), "");
  EXPECT_EQ(CheckParams(*FindCommand("mowing.start"), Json{{"areas", Json::array({"front"})}}, &field), "");
}

// Every RPC method once, each with its node; the commands are none of them.
TEST(Catalog, MethodsSensorsSettingsAndGeometry) {
  const std::set<std::string> nodes = {"mowbite_bridge", "mowbite_logic", "mowbite_map", "mowbite_ntrip",
                                       "mowbite_calibration"};
  std::set<std::string> names;
  for (const auto& command : Commands()) names.insert(command.name);
  for (const auto& method : Methods()) {
    EXPECT_TRUE(names.insert(method.name).second) << method.name;
    EXPECT_TRUE(nodes.count(method.node)) << method.name << " " << method.node;
    EXPECT_FALSE(method.text.empty()) << method.name;
    EXPECT_EQ(method.name.rfind("planner.", 0), std::string::npos) << method.name;
    EXPECT_EQ(method.name.rfind("sim.", 0), std::string::npos) << method.name;
    EXPECT_EQ(NodeOf(method.name), method.node);
  }
  EXPECT_EQ(NodeOf("system.info"), "mowbite_bridge");
  EXPECT_EQ(NodeOf("mowing.start"), "mowbite_logic");
  EXPECT_EQ(NodeOf("no.such"), "");
  std::set<std::string> ids;
  for (const auto& sensor : Sensors()) {
    EXPECT_TRUE(ids.insert(sensor.id).second) << sensor.id;
    EXPECT_TRUE(sensor.source == "logic" || sensor.source == "gnss" || sensor.source == "ntrip") << sensor.id;
    EXPECT_EQ(FindSensor(sensor.id), &sensor);
  }
  EXPECT_EQ(Sensors().size(), 13u);
  EXPECT_EQ(FindSensor("no_such"), nullptr);
  std::set<std::string> settings;
  for (const auto& setting : LogicSettings()) {
    EXPECT_TRUE(settings.insert(setting.name).second) << setting.name;
    EXPECT_LE(setting.min, setting.def) << setting.name;
    EXPECT_LE(setting.def, setting.max) << setting.name;
  }
  EXPECT_EQ(settings.size(), 13u);
  std::set<std::string> keys;
  for (const auto& key : GeometryKeys()) {
    EXPECT_TRUE(keys.insert(key.name).second) << key.name;
    if (!key.points) {
      EXPECT_LT(key.min, key.max) << key.name;
    }
  }
}
