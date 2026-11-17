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
#include "mowbite_api/setup.h"

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
// the pauses of api 5 (O11) and the release refusals of api 10 are rated anew, STORAGE_NOT_WRITABLE of api 12,
// REVERSING of api 21 and the reasons of the schedule of api 25 are new.
TEST(Catalog, RateReasonAsTheTableBefore) {
  const std::map<std::string, Rating> paused_since_api_5 = {{"BATTERY_HOT", {"warning", "user"}},
                                                             {"MOTOR_HOT", {"warning", "user"}},
                                                             {"RAIN", {"info", "user"}},
                                                             {"DARK", {"info", "user"}}};
  const std::map<std::string, Rating> refused_since_api_10 = {{"STILL_ACTIVE", {"warning", "user"}},
                                                              {"LATCH_CHANGED", {"warning", "user"}}};
  const std::map<std::string, Rating> schedule_since_api_25 = {{"MOWING_REFUSED NOT_DOCKED", {"info", "user"}},
                                                               {"MOWING_REFUSED CLOCK_UNSAFE", {"info", "user"}},
                                                               {"MOWING_REFUSED DARK", {"info", "user"}},
                                                               {"STATE SCHEDULE", {"info", "user"}},
                                                               {"AREA_SKIPPED SCHEDULE", {"info", "auto"}}};
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED", "RECORDING_REFUSED", "CALIBRATION_REFUSED",
                            "CALIBRATION_FAILED", "STATE", "DRIVE", "POSITION_JUMP", "AREA_SKIPPED", "UNDOCKING_FAILED",
                            "DOCKING_RETRY", "DOCKING_FAILED", "EMERGENCY_RESET_REFUSED", "NOT_AN_EVENT"}) {
    for (const auto& code : CodesOf(event)) {
      for (const char* action : {"", "point", "collect_point", "record_dock", "start_area_recording"}) {
        const Rating now = RateReason(event, code, Json{{"action", action}});
        const auto paused = paused_since_api_5.find(code);
        const auto refused = refused_since_api_10.find(code);
        const auto scheduled = schedule_since_api_25.find(std::string(event) + " " + code);
        const Rating before = scheduled != schedule_since_api_25.end() ? scheduled->second
                              : code == "STORAGE_NOT_WRITABLE"   ? Rating{"error", "user"}
                              : code == "REVERSING" ? Rating{"info", "auto"}
                              : code == "LEARNING" ? Rating{"warning", "user"}
                              : std::string(event) == "STATE" && paused != paused_since_api_5.end() ? paused->second
                              : std::string(event) == "EMERGENCY_RESET_REFUSED" && refused != refused_since_api_10.end()
                                  ? refused->second
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
    const std::set<std::string> emitters = {"logic", "map", "ntrip", "camera", "bridge", "stats", "setup"};
    EXPECT_TRUE(emitters.count(event.emitter)) << event.type;
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
    std::set<std::string> envelope;
    for (const auto& field : Envelope()) {
      add(field, false);
      envelope.insert(field.name);
    }
    if (event.emitter == "logic") {
      for (const auto& field : LogicFields()) add(field, false);
    }
    // The envelope (id, t, type) wins over a detail of the same name.
    for (const auto& field : event.fields) {
      EXPECT_EQ(envelope.count(field.name), 0u) << event.type << "." << field.name;
      add(field, false);
    }
    std::set<std::string> codes;
    for (const auto& reason : event.reasons) {
      EXPECT_TRUE(severities.count(reason.severity)) << event.type << " " << reason.code;
      EXPECT_TRUE(resumes.count(reason.resume)) << event.type << " " << reason.code;
      EXPECT_FALSE(reason.text.empty()) << event.type << " " << reason.code;
      if (reason.when.empty()) {
        EXPECT_TRUE(codes.insert(reason.code).second) << event.type << " " << reason.code;
      }
      for (const auto& field : reason.fields) {
        EXPECT_EQ(envelope.count(field.name), 0u) << event.type << " " << reason.code << "." << field.name;
        add(field, true);
      }
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

// The values of driven_reason in the order the recorder checks them, each with its texts.
TEST(Catalog, DrivenReasonsInTheOrderOfTheCheck) {
  const EventEntry* recorded = FindEvent("AREA_RECORDED");
  ASSERT_NE(recorded, nullptr);
  const auto field = std::find_if(recorded->fields.begin(), recorded->fields.end(),
                                  [](const Field& f) { return f.name == "driven_reason"; });
  ASSERT_NE(field, recorded->fields.end());
  EXPECT_EQ(field->values, (std::vector<std::string>{"MANUAL_POINT", "NO_HEADING", "INTERRUPTED", "REVERSED",
                                                     "CLOSING", "GAP", "DIRECTION", "SHARP_TURN"}));
  for (const auto& value : field->values) {
    const auto texts = field->value_texts.find(value);
    ASSERT_NE(texts, field->value_texts.end()) << value;
    for (const char* language : {"de", "en"}) {
      const auto text = texts->second.find(language);
      ASSERT_NE(text, texts->second.end()) << value << " " << language;
      EXPECT_FALSE(text->second.name.empty()) << value << " " << language;
      EXPECT_FALSE(text->second.fix.empty()) << value << " " << language;
      const std::string edge = std::string(language) == "de" ? "ohne den genauen Rand" : "without the exact edge";
      EXPECT_NE(text->second.help.find(edge), std::string::npos) << value << " " << language;
    }
  }
}

TEST(Catalog, EmergencyBits) {
  ASSERT_EQ(EmergencyBits().size(), 23u);
  for (size_t i = 0; i < EmergencyBits().size(); i++) {
    EXPECT_EQ(EmergencyBits()[i].bit, static_cast<int>(i));
    EXPECT_FALSE(EmergencyBits()[i].text.empty()) << i;
  }
  EXPECT_EQ(EmergencyNames(0), "");
  EXPECT_EQ(EmergencyNames((1 << 0) | (1 << 3)), "LATCH, LIFT");
  EXPECT_EQ(EmergencyNames((1 << 13) | (1 << 14) | (1 << 15)), "TILT, FENCE, STUCK");
  EXPECT_EQ(EmergencyNames((1 << 18) | (1 << 19) | (1 << 20)), "WHEEL_RUNAWAY, FOREIGN_HOST, DRIVER_STALL");
  EXPECT_EQ(EmergencyNames((1 << 0) | (1 << 21)), "LATCH, UNEXPECTED_RESTART");
  EXPECT_EQ(EmergencyNames((1 << 0) | (1 << 22)), "LATCH, INPUTS_LEARNING");
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
  EXPECT_EQ(EmergencyNames((1 << 23) | (1 << 25) | static_cast<int>(1u << 31)), "UNKNOWN");
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

// COMMAND_REFUSED, REQUEST_REFUSED, SOUND_REFUSED and CAMERA_REFUSED are groups of answers only: rated like an event,
// but never one.
TEST(Catalog, RefusalsAreNoEvents) {
  ASSERT_EQ(Refusals().size(), 9u);
  const EventEntry& answers = Refusals().front();
  EXPECT_EQ(answers.type, "COMMAND_REFUSED");
  EXPECT_EQ(FindEvent("COMMAND_REFUSED"), nullptr);
  EXPECT_EQ(FindReasonGroup("COMMAND_REFUSED"), &answers);
  for (const char* code : {"WRONG_STATE", "NO_JOB", "CHANGED"}) {
    const Rating rating = RateReason("COMMAND_REFUSED", code);
    EXPECT_STREQ(rating.severity, "info") << code;
    EXPECT_STREQ(rating.resume, "user") << code;
  }
  const EventEntry& requests = *FindReasonGroup("REQUEST_REFUSED");
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
  ASSERT_NE(FindReasonGroup("SOUND_REFUSED"), nullptr);
  EXPECT_EQ(FindEvent("SOUND_REFUSED"), nullptr);
  for (const char* code : {"NO_FILE", "EMERGENCY"}) {
    const Rating rating = RateReason("SOUND_REFUSED", code);
    EXPECT_STREQ(rating.severity, "info") << code;
    EXPECT_STREQ(rating.resume, "user") << code;
  }
}

// Every reason hw/cameras may give is one of CAMERA_REFUSED and of CAMERA_CHANGED, with texts in both languages;
// CAMERA_BUSY is the OS's own and never the state of a camera. The event only tells that the list changed.
TEST(Catalog, ReasonsOfTheCameras) {
  const std::filesystem::path root = std::filesystem::canonical(MOWBITE_SOURCE_ROOT);
  std::ifstream in(root / "mowbite_msgs/msg/Camera.msg");
  std::stringstream text;
  text << in.rdbuf();
  const std::string msg = text.str();
  const std::regex constant(R"(string REASON_([A-Z_]+)=([A-Z_]+))");
  const EventEntry* refused = FindReasonGroup("CAMERA_REFUSED");
  const EventEntry* changed = FindEvent("CAMERA_CHANGED");
  ASSERT_NE(refused, nullptr);
  ASSERT_NE(changed, nullptr);
  EXPECT_EQ(FindEvent("CAMERA_REFUSED"), nullptr);
  const auto find = [](const EventEntry& group, const std::string& code) -> const ReasonEntry* {
    for (const auto& reason : group.reasons) {
      if (reason.code == code) return &reason;
    }
    return nullptr;
  };
  int reasons = 0;
  for (std::sregex_iterator it(msg.begin(), msg.end(), constant), end; it != end; ++it) {
    const std::string code = (*it)[2];
    EXPECT_EQ((*it)[1], code);
    reasons++;
    for (const EventEntry* group : {refused, changed}) {
      const ReasonEntry* reason = find(*group, code);
      ASSERT_NE(reason, nullptr) << group->type << " " << code;
      for (const char* language : {"de", "en"}) {
        ASSERT_TRUE(reason->texts.count(language)) << group->type << " " << code;
        const UserText& words = reason->texts.at(language);
        EXPECT_FALSE(words.name.empty() || words.help.empty() || words.fix.empty()) << group->type << " " << code;
      }
    }
    const Rating notice = RateReason("CAMERA_CHANGED", code);
    EXPECT_STREQ(notice.severity, "info") << code;
    EXPECT_STREQ(notice.resume, "auto") << code;
  }
  EXPECT_EQ(reasons, 7);
  EXPECT_EQ(refused->reasons.size(), 8u);
  EXPECT_EQ(changed->reasons.size(), 7u);
  ASSERT_NE(find(*refused, "CAMERA_BUSY"), nullptr);
  EXPECT_EQ(find(*refused, "CAMERA_BUSY")->texts.count("de"), 1u);
  EXPECT_EQ(find(*changed, "CAMERA_BUSY"), nullptr);
  EXPECT_STREQ(RateReason("CAMERA_REFUSED", "CAMERA_BUSY").resume, "user");
  EXPECT_STREQ(RateReason("CAMERA_REFUSED", "SELF_TEST").resume, "auto");
  EXPECT_STREQ(RateReason("CAMERA_REFUSED", "LOCKED_AFTER_CRASH").severity, "warning");

  // The events as the camera node and the bridge send them.
  const Json offer = {{"protocols", {"webrtc", "mjpeg"}}, {"width", 1280}, {"height", 720}};
  const auto camera_changed = [&offer](Json details) {
    details.update(offer);
    return CheckEvent("CAMERA_CHANGED", details);
  };
  EXPECT_TRUE(camera_changed({{"camera_id", "stereo"}, {"available", true}, {"severity", "info"}, {"resume", "auto"}})
                  .empty());
  EXPECT_TRUE(camera_changed({{"camera_id", "stereo"}, {"available", false}, {"reason", "TOO_HOT"},
                              {"message", "Stereo camera: TOO_HOT (SoC 81 C)"}, {"severity", "info"},
                              {"resume", "auto"}}).empty());
  EXPECT_FALSE(camera_changed({{"camera_id", "stereo"}, {"available", false}, {"reason", "CAMERA_BUSY"},
                               {"severity", "info"}, {"resume", "auto"}}).empty());
  EXPECT_FALSE(camera_changed({{"id", "stereo"}, {"available", true}, {"severity", "info"}, {"resume", "auto"}})
                   .empty());
  // How it streams is always there.
  EXPECT_FALSE(CheckEvent("CAMERA_CHANGED", Json{{"camera_id", "stereo"}, {"available", true}, {"severity", "info"},
                                                 {"resume", "auto"}}).empty());
  EXPECT_TRUE(CheckEvent("METHODS_CHANGED", Json{{"added", {"camera.list", "camera.stream"}},
                                                 {"removed", Json::array()}, {"severity", "info"},
                                                 {"resume", "auto"}}).empty());
  EXPECT_FALSE(CheckEvent("METHODS_CHANGED", Json{{"added", Json::array()}, {"severity", "info"},
                                                  {"resume", "auto"}}).empty());
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
  EXPECT_EQ(actions.size(), 22u);
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
  const Command& undo = *FindCommand("recording.undo");
  EXPECT_EQ(CheckParams(undo, Json{{"meters", 0.1}}, &field), "");
  EXPECT_EQ(CheckParams(undo, Json{{"meters", 50}}, &field), "");
  EXPECT_EQ(CheckParams(undo, Json{{"meters", 0.05}}, &field), "recording.undo.meters: 0.05 is below 0.1");
  EXPECT_EQ(field, "meters");
  EXPECT_EQ(CheckParams(undo, Json{{"meters", 51}}, &field), "recording.undo.meters: 51 is above 50.0");
}

// Every RPC method once, each with its node; the commands are none of them.
TEST(Catalog, MethodsSensorsSettingsAndGeometry) {
  const std::set<std::string> nodes = {"mowbite_bridge",      "mowbite_logic", "mowbite_map",   "mowbite_ntrip",
                                       "mowbite_calibration", "mowbite_light", "mowbite_sound", "mowbite_camera",
                                       "mowbite_schedule",    "mowbite_stats", "mowbite_net",   "mowbite_host",
                                       "mowbite_setup"};
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
  EXPECT_EQ(settings.size(), 16u);
  std::set<std::string> keys;
  for (const auto& key : GeometryKeys()) {
    EXPECT_TRUE(keys.insert(key.name).second) << key.name;
    if (!key.points && !key.shape) {
      EXPECT_LT(key.min, key.max) << key.name;
    }
  }
}

namespace {

// The reasons of an event or of a group of refusals, by code.
std::map<std::string, const ReasonEntry*> ReasonsOf(const std::string& type) {
  std::map<std::string, const ReasonEntry*> out;
  for (const auto* list : {&Events(), &Refusals()}) {
    for (const auto& entry : *list) {
      if (entry.type != type) continue;
      for (const auto& reason : entry.reasons) out[reason.code] = &reason;
    }
  }
  return out;
}

// Texts in both languages, each with a name; help and fix as well where a failure has to be explained.
void ExpectTexts(const UserTexts& texts, const std::string& where, bool explained) {
  for (const char* language : {"de", "en"}) {
    ASSERT_TRUE(texts.count(language)) << where << " " << language;
    const UserText& text = texts.at(language);
    EXPECT_FALSE(text.name.empty()) << where << " " << language;
    if (!explained) continue;
    EXPECT_FALSE(text.help.empty()) << where << " " << language;
    EXPECT_FALSE(text.fix.empty()) << where << " " << language;
  }
}

}  // namespace

// What a step list can name: unique ids of the pattern, and texts in both languages where the app shows them.
TEST(Catalog, TheNamesOfTheSetupProgram) {
  const std::regex id(kSetupIdPattern);
  const auto named = [&id](const auto& names, const std::string& what, bool shown) {
    std::set<std::string> seen;
    EXPECT_FALSE(names.empty()) << what;
    for (const SetupName& name : names) {
      EXPECT_TRUE(seen.insert(name.id).second) << what << " " << name.id;
      EXPECT_TRUE(std::regex_match(name.id, id)) << what << " " << name.id;
      EXPECT_FALSE(name.text.empty()) << what << " " << name.id;
      if (shown) ExpectTexts(name.texts, what + " " + name.id, false);
    }
  };
  named(SetupStages(), "stage", true);
  named(SetupRoles(), "role", true);
  named(SetupMoves(), "move", true);
  named(SetupConditions(), "condition", true);
  named(SetupPoints(), "point", true);
  named(SetupSkipReasons(), "skip reason", true);
  named(SetupModes(), "mode", false);
  named(SetupAcceptValues(), "accept", true);
  named(SetupRestarts(), "restart", false);
  named(SetupWheelParts(), "wheel part", true);
  named(SetupGroundParts(), "ground part", true);
  named(SetupBladeParts(), "blade part", true);
  named(SetupWheelValues(), "wheel", true);
  ExpectTexts(SetupAccept().texts, "accept", false);
  ExpectTexts(SetupWheelQuestion().texts, "wheel", false);
  // The emergencies are names of the bits of hw/status.
  for (const auto& name : SetupEmergencies()) {
    if (name == "NONE") continue;
    bool known = false;
    for (const auto& bit : EmergencyBits()) known = known || bit.name == name;
    EXPECT_TRUE(known) << name;
  }
  // A role is a flag of hw/status, its group a role itself, its emergency one a step can name.
  std::ifstream in(std::string(MOWBITE_SOURCE_ROOT) + "/mowbite_msgs/msg/HardwareStatus.msg");
  ASSERT_TRUE(in.good());
  std::stringstream status;
  status << in.rdbuf();
  const Json catalog = SetupCatalog();
  for (const auto& role : SetupRoles()) {
    EXPECT_NE(status.str().find("\nbool " + role.id + "\n"), std::string::npos) << role.id;
    EXPECT_NE(FindSetupName(SetupRoles(), role.group), nullptr) << role.id;
    EXPECT_EQ(std::count(SetupEmergencies().begin(), SetupEmergencies().end(), role.emergency), 1) << role.id;
    EXPECT_NE(role.emergency, "NONE") << role.id;
    EXPECT_EQ(catalog["roles"][role.id]["group"], role.group);
    EXPECT_EQ(catalog["roles"][role.id]["emergency"], role.emergency);
  }
  // The screen of a condition or of a check is one the app knows.
  for (const auto& condition : SetupConditions()) {
    EXPECT_EQ(std::count(SetupOpens().begin(), SetupOpens().end(), condition.open), 1) << condition.id;
    EXPECT_EQ(catalog["conditions"][condition.id]["open"], condition.open);
  }
  for (const auto& check : SetupChecks()) {
    if (check.open.empty()) continue;
    EXPECT_EQ(std::count(SetupOpens().begin(), SetupOpens().end(), check.open), 1) << check.name;
    EXPECT_EQ(catalog["checks"][check.name]["open"], check.open);
  }
  EXPECT_EQ(catalog["accept"]["values"].size(), SetupAcceptValues().size());
  EXPECT_EQ(catalog["restart"].size(), SetupRestarts().size());
  EXPECT_EQ(catalog["wheel_parts"].size(), SetupWheelParts().size());
  EXPECT_EQ(catalog["ground_parts"].size(), SetupGroundParts().size());
  EXPECT_EQ(catalog["blade_parts"].size(), SetupBladeParts().size());
  EXPECT_EQ(catalog["wheel"]["values"].size(), SetupWheelValues().size());
  ASSERT_NE(FindSetupName(SetupWheelValues(), "right"), nullptr);
  ASSERT_NE(FindSetupName(SetupWheelParts(), "one_wheel"), nullptr);
}

// Every check: its params within bounds, its reasons those of SETUP_STEP, texts where the step list brings none.
TEST(Catalog, TheChecksOfTheSetupProgram) {
  const auto reasons = ReasonsOf("SETUP_STEP");
  ASSERT_FALSE(reasons.empty());
  std::set<std::string> names;
  for (const auto& check : SetupChecks()) {
    EXPECT_TRUE(names.insert(check.name).second) << check.name;
    EXPECT_EQ(FindSetupCheck(check.name), &check);
    EXPECT_TRUE(check.kind == "confirm" || check.kind == "input" || check.kind == "measure" ||
                check.kind == "drive" || check.kind == "blade")
        << check.name;
    EXPECT_FALSE(check.text.empty()) << check.name;
    for (const auto& reason : check.reasons) {
      EXPECT_TRUE(reasons.count(reason)) << check.name << " " << reason;
    }
    // What setup.start begins and watches can be stopped and cut off.
    if (check.start && (check.kind == "measure" || check.kind == "drive" || check.kind == "blade")) {
      for (const char* reason : {"STOPPED", "INTERRUPTED"}) {
        EXPECT_EQ(std::count(check.reasons.begin(), check.reasons.end(), reason), 1) << check.name << " " << reason;
      }
    }
    std::set<std::string> params;
    for (const auto& param : check.params) {
      EXPECT_TRUE(params.insert(param.name).second) << check.name << "." << param.name;
      EXPECT_FALSE(param.text.empty()) << check.name << "." << param.name;
      // A text a step may leave out is one of its values.
      if (!param.word.empty()) {
        EXPECT_EQ(param.type, Type::kString) << check.name << "." << param.name;
        EXPECT_EQ(std::count(param.values.begin(), param.values.end(), param.word), 1)
            << check.name << "." << param.name;
        EXPECT_EQ(SetupCatalog()["checks"][check.name]["params"][param.name]["default"], param.word);
      }
      if (param.type != Type::kNumber || param.required) continue;
      EXPECT_LT(param.min, param.max) << check.name << "." << param.name;
      EXPECT_GE(param.def, param.min) << check.name << "." << param.name;
      EXPECT_LE(param.def, param.max) << check.name << "." << param.name;
    }
    for (const auto& value : check.values) {
      EXPECT_FALSE(value.text.empty()) << check.name << " " << value.id;
      if (!value.texts.empty()) ExpectTexts(value.texts, check.name + " " + value.id, false);
    }
    if (!check.texts.empty()) ExpectTexts(check.texts, check.name, true);
    // A step that moves the wheels stays within the bounds of a mower jacked up or, ground alone, on the ground, and
    // a stop or end it asks for names emergencies the hardware has.
    if (check.kind != "drive") continue;
    EXPECT_FALSE(check.texts.empty()) << check.name;
    for (const char* reason : {"NO_START", "LOGIC_REFUSED", "RELEASED", "EMERGENCY"}) {
      EXPECT_EQ(std::count(check.reasons.begin(), check.reasons.end(), reason), 1) << check.name << " " << reason;
    }
    for (const auto& param : check.params) {
      if (param.unit == "m/s") {
        EXPECT_LE(param.max, check.name == "ground" ? kSetupGroundSpeed : kSetupRaisedSpeed) << check.name;
      }
      if (param.unit == "rad/s") {
        EXPECT_LE(param.max, check.name == "ground" ? kSetupGroundTurn : kSetupRaisedTurn) << check.name;
      }
      if (param.unit == "s" && param.name != "return_max") {
        EXPECT_LE(param.max, kSetupMoveTime) << check.name;
      }
      if (param.name != "emergency") continue;
      EXPECT_TRUE(param.required) << check.name;
      for (const auto& name : param.values) EXPECT_NE(name, "LATCH") << check.name;
      EXPECT_EQ(param.values.size() + 1, EmergencyBits().size()) << check.name;
    }
  }
  for (const auto& [code, reason] : reasons) ExpectTexts(reason->texts, "SETUP_STEP " + code, true);
  EXPECT_EQ(FindSetupCheck("nothing"), nullptr);
  // x-setup of the schema carries all of it.
  const Json catalog = SetupCatalog();
  EXPECT_EQ(catalog["checks"].size(), SetupChecks().size());
  for (const auto& check : SetupChecks()) {
    ASSERT_TRUE(catalog["checks"].contains(check.name)) << check.name;
    EXPECT_EQ(catalog["checks"][check.name]["params"].size(), check.params.size()) << check.name;
    EXPECT_EQ(catalog["checks"][check.name]["reasons"], Json(check.reasons)) << check.name;
  }
  EXPECT_EQ(catalog["stages"].size(), SetupStages().size());
  EXPECT_EQ(catalog["limits"]["steps"], kSetupMaxSteps);
  EXPECT_EQ(catalog["limits"]["image_bytes"], kSetupImageBytes);
  EXPECT_EQ(catalog["limits"]["shape_points"], kShapeMaxPoints);
  EXPECT_EQ(catalog["limits"]["raised_speed"], kSetupRaisedSpeed);
  EXPECT_EQ(catalog["limits"]["move_time"], kSetupMoveTime);
  EXPECT_EQ(catalog["limits"]["keep"], kSetupKeepTimeout);
}

// Every reason the program gives is one of its group: Fail() of SETUP_STEP, Refuse() of SETUP_REFUSED and what
// blocks a step of SETUP_BLOCKED, each with texts.
TEST(Catalog, TheReasonsOfTheSetupProgram) {
  std::stringstream text;
  for (const char* file : {"program.cpp", "motion.cpp"}) {
    std::ifstream in(std::string(MOWBITE_SOURCE_ROOT) + "/mowbite_setup/src/" + file);
    ASSERT_TRUE(in.good()) << file;
    text << in.rdbuf();
  }
  const std::string source = text.str();
  struct Call {
    const char* pattern;
    const char* group;
    int least;
  };
  const Call calls[] = {{R"(\bFail\("([A-Z_]+)\")", "SETUP_STEP", 9},
                        {R"(\bEnd\(step, "open", "([A-Z_]+)\")", "SETUP_STEP", 1},
                        {R"(\bRefuse\("([A-Z_]+)\")", "SETUP_REFUSED", 10},
                        {R"(\badd\("([A-Z_]+)\")", "SETUP_BLOCKED", 6}};
  for (const Call& call : calls) {
    const auto reasons = ReasonsOf(call.group);
    const std::regex pattern(call.pattern);
    std::set<std::string> found;
    for (std::sregex_iterator it(source.begin(), source.end(), pattern), end; it != end; ++it) found.insert((*it)[1]);
    EXPECT_GE(static_cast<int>(found.size()), call.least) << call.group;
    for (const auto& code : found) {
      const auto reason = reasons.find(code);
      ASSERT_NE(reason, reasons.end()) << call.group << " has no " << code;
      ExpectTexts(reason->second->texts, std::string(call.group) + " " + code, true);
    }
  }
}


// ground is a safety step: a mower that does not stop closes the lock, a step not done or without a fix does not.
TEST(Catalog, GroundIsASafetyCheck) {
  ASSERT_NE(FindSetupCheck("ground"), nullptr);
  EXPECT_TRUE(FindSetupCheck("ground")->safety);
  EXPECT_EQ(FindSetupCheck("ground")->kind, "drive");
  for (const char* reason :
       {"LIMIT", "NO_STANDSTILL", "STARTED_AGAIN", "NO_EMERGENCY", "WRONG_DIRECTION", "NOT_SEEN"}) {
    EXPECT_TRUE(SetupClosesLock("ground", reason)) << reason;
  }
  for (const char* reason : {"NOT_DONE", "NO_RTK", "STOPPED", "RELEASED", "NOT_RELEASED", "NO_START"}) {
    EXPECT_FALSE(SetupClosesLock("ground", reason)) << reason;
  }
  EXPECT_FALSE(SetupClosesLock("board", "LIMIT"));
  std::vector<std::string> parts;
  for (const auto& part : SetupGroundParts()) parts.push_back(part.id);
  EXPECT_EQ(parts, (std::vector<std::string>{"straight", "turn", "stop", "bumper"}));
  const Json catalog = SetupCatalog();
  EXPECT_EQ(catalog["limits"]["ground_speed"], kSetupGroundSpeed);
  EXPECT_NE(FindSetupName(SetupPoints(), "area_clear"), nullptr);
}

// blade is a safety step: a blade that does not stand or starts again closes the lock, one that does not come up or a
// driver that refuses does not.
TEST(Catalog, BladeIsASafetyCheck) {
  ASSERT_NE(FindSetupCheck("blade"), nullptr);
  EXPECT_TRUE(FindSetupCheck("blade")->safety);
  EXPECT_EQ(FindSetupCheck("blade")->kind, "blade");
  for (const char* reason : {"LIMIT", "STARTED_AGAIN", "NO_EMERGENCY", "NOT_SEEN"}) {
    EXPECT_TRUE(SetupClosesLock("blade", reason)) << reason;
  }
  for (const char* reason : {"NO_START", "DRIVER_REFUSED", "NOT_DONE", "STOPPED", "RELEASED", "NOT_RELEASED"}) {
    EXPECT_FALSE(SetupClosesLock("blade", reason)) << reason;
  }
  std::vector<std::string> parts;
  for (const auto& part : SetupBladeParts()) parts.push_back(part.id);
  EXPECT_EQ(parts, (std::vector<std::string>{"on_off", "lease", "silent", "stop", "lift", "bumper"}));
  EXPECT_NE(FindSetupName(SetupPoints(), "blade_removed"), nullptr);
  const auto blocks = ReasonsOf("SETUP_BLOCKED");
  EXPECT_TRUE(blocks.count("BLADE_NOT_ALLOWED"));
  EXPECT_TRUE(ReasonsOf("SETUP_REFUSED").count("BLADE_NOT_ALLOWED"));
}

// summary opens the lock with one tap: it is no safety step itself, restarts the OS and names why it cannot start.
TEST(Catalog, SummaryOpensTheLock) {
  const SetupCheck* summary = FindSetupCheck("summary");
  ASSERT_NE(summary, nullptr);
  EXPECT_FALSE(summary->safety);
  EXPECT_EQ(summary->kind, "confirm");
  EXPECT_TRUE(summary->start);
  for (const char* reason : {"NOT_APPLIED", "RESTART_FAILED", "STOPPED", "INTERRUPTED"}) {
    EXPECT_EQ(std::count(summary->reasons.begin(), summary->reasons.end(), reason), 1) << reason;
    EXPECT_FALSE(SetupClosesLock("summary", reason)) << reason;
  }
  EXPECT_NE(FindSetupName(SetupPoints(), "seen_myself"), nullptr);
  for (const char* reason : {"SAFETY_STEPS", "CLOCK_UNSAFE"}) {
    EXPECT_TRUE(ReasonsOf("SETUP_BLOCKED").count(reason)) << reason;
    EXPECT_TRUE(ReasonsOf("SETUP_REFUSED").count(reason)) << reason;
  }
}
