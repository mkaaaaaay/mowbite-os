#include <gtest/gtest.h>

#include "mowbite_api/catalog.h"

using namespace mowbite_api;

namespace {

bool Has(const std::vector<std::string>& problems, const std::string& part) {
  for (const auto& problem : problems) {
    if (problem.find(part) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

TEST(CheckEvent, TheEnvelopeAndUnknownTypes) {
  EXPECT_TRUE(CheckEvent("BOOTED", Json::object()).empty());
  EXPECT_TRUE(CheckEvent("BOOTED", Json{{"id", "a1"}, {"t", 5.5}, {"type", "BOOTED"}}).empty());
  EXPECT_TRUE(Has(CheckEvent("BOOTED", Json{{"t", "now"}}), "BOOTED.t"));
  EXPECT_TRUE(Has(CheckEvent("BOOTED", Json{{"type", "SHUTDOWN"}}), "type is"));
  EXPECT_TRUE(Has(CheckEvent("BOOTED", Json{{"odd", 1}}), "field odd is not in the catalog"));
  EXPECT_TRUE(Has(CheckEvent("PLANNER_THING", Json::object()), "does not know"));
  EXPECT_TRUE(CheckEvent("PLANNER_THING", Json{{"any", 1}}, false).empty());
  EXPECT_FALSE(CheckEvent("BOOTED", Json::array()).empty());
}

TEST(CheckEvent, TheStateWithItsValuesAndTheFieldsOfTheLogic) {
  const Json idle{{"state", "IDLE"}, {"x", 1.0}, {"y", -2.5}, {"job_id", "j1"}, {"session_id", "s1"}};
  EXPECT_TRUE(CheckEvent("STATE", idle).empty());
  Json driving = idle;
  driving["sub_state"] = "DRIVING";
  EXPECT_TRUE(CheckEvent("STATE", driving).empty());
  Json lower = idle;
  lower["state"] = "idle";
  EXPECT_TRUE(Has(CheckEvent("STATE", lower), "STATE.state: value \"idle\""));
  Json number = idle;
  number["state"] = 3;
  EXPECT_TRUE(Has(CheckEvent("STATE", number), "STATE.state: string expected"));
  EXPECT_TRUE(Has(CheckEvent("STATE", Json{{"state", "IDLE"}, {"y", 0.0}}), "STATE: x is missing"));
  EXPECT_TRUE(Has(CheckEvent("STATE", Json{{"x", 0.0}, {"y", 0.0}}), "STATE: state is missing"));
}

TEST(CheckEvent, AReasonWithItsRatingAndItsOwnFields) {
  const Json at{{"x", 0.0}, {"y", 0.0}};
  Json paused = at;
  paused.update(Json{{"state", "PAUSED"}, {"reason", "NO_POSITION"}, {"message", "No position"},
                     {"severity", "warning"}, {"resume", "auto"}});
  EXPECT_TRUE(CheckEvent("STATE", paused).empty());
  Json wrong = paused;
  wrong["resume"] = "user";
  EXPECT_TRUE(Has(CheckEvent("STATE", wrong), "STATE NO_POSITION: resume \"user\", the catalog says auto"));
  wrong = paused;
  wrong["severity"] = "error";
  EXPECT_TRUE(Has(CheckEvent("STATE", wrong), "severity \"error\", the catalog says warning"));
  wrong = paused;
  wrong["reason"] = "HOLD";
  EXPECT_TRUE(Has(CheckEvent("STATE", wrong), "reason HOLD is not in the catalog for this event"));

  Json low = at;
  low.update(Json{{"reason", "BATTERY_LOW"}, {"severity", "warning"}, {"resume", "user"},
                  {"battery_voltage", 21.4}, {"battery_percentage", 12.0}, {"limit", 22}});
  EXPECT_TRUE(CheckEvent("MOWING_REFUSED", low).empty());
  low["battery_percentage"] = 12.5;
  EXPECT_TRUE(Has(CheckEvent("MOWING_REFUSED", low), "battery_percentage: integer expected"));
  // The fields of one reason do not belong to another.
  Json busy = at;
  busy.update(Json{{"reason", "BUSY"}, {"severity", "info"}, {"resume", "user"}, {"state", "MOWING"},
                   {"battery_voltage", 21.4}});
  EXPECT_TRUE(Has(CheckEvent("MOWING_REFUSED", busy), "field battery_voltage is not in the catalog"));
  // A refusal always carries its reason and rating.
  EXPECT_TRUE(Has(CheckEvent("MOWING_REFUSED", at), "MOWING_REFUSED: reason is missing"));
  EXPECT_TRUE(Has(CheckEvent("MOWING_REFUSED", at), "MOWING_REFUSED: severity is missing"));
}

TEST(CheckEvent, TheRatingOfARecordingStepDependsOnTheAction) {
  Json point{{"x", 0.0}, {"y", 0.0}, {"reason", "POSITION_ACCURACY"}, {"action", "point"},
             {"severity", "warning"}, {"resume", "auto"}, {"accuracy", 0.4}, {"limit", 0.05}};
  EXPECT_TRUE(CheckEvent("RECORDING_REFUSED", point).empty());
  Json collect = point;
  collect["action"] = "collect_point";
  EXPECT_TRUE(Has(CheckEvent("RECORDING_REFUSED", collect), "resume \"auto\", the catalog says user"));
  collect["resume"] = "user";
  EXPECT_TRUE(CheckEvent("RECORDING_REFUSED", collect).empty());
  collect["action"] = "jump";
  EXPECT_TRUE(Has(CheckEvent("RECORDING_REFUSED", collect), "action: value \"jump\""));
}

TEST(CheckEvent, EventsWhoseReasonIsNotAName) {
  // The emergency carries the bits as its reason.
  const Json on{{"x", 0.0}, {"y", 0.0}, {"emergency", true}, {"reason", 5}, {"severity", "error"}, {"resume", "user"}};
  EXPECT_TRUE(CheckEvent("EMERGENCY", on).empty());
  Json named = on;
  named["reason"] = "LATCH";
  EXPECT_TRUE(Has(CheckEvent("EMERGENCY", named), "EMERGENCY.reason: integer expected"));
  // The caster's refusal carries the HTTP status, a failed save its code as text.
  EXPECT_TRUE(CheckEvent("CORRECTIONS", Json{{"reason", "REJECTED"}, {"message", "401"}, {"severity", "error"},
                                             {"resume", "user"}, {"code", 401}})
                  .empty());
  const Json saved{{"x", 0.0},           {"y", 0.0},          {"action", "finish_mowing_area"},
                   {"reason", "SAVE_FAILED"}, {"severity", "error"}, {"resume", "user"},
                   {"code", "STORAGE"}};
  EXPECT_TRUE(CheckEvent("RECORDING_REFUSED", saved).empty());
  Json odd = saved;
  odd["code"] = "FULL";
  EXPECT_TRUE(Has(CheckEvent("RECORDING_REFUSED", odd), "code: value \"FULL\""));
}

TEST(CheckEvent, EveryReasonOfTheCatalogPassesWithItsRating) {
  for (const auto& event : Events()) {
    for (const auto& reason : event.reasons) {
      if (event.type == "EMERGENCY") continue;
      Json details{{"reason", reason.code}};
      details.update(reason.when);
      // GEOMETRY_CHANGED carries no resume.
      for (const auto& field : event.fields) {
        if (field.name == "severity") details["severity"] = reason.severity;
        if (field.name == "resume") details["resume"] = reason.resume;
      }
      if (event.emitter == "logic") details.update(Json{{"x", 0.0}, {"y", 0.0}});
      for (const auto& field : event.fields) {
        if (!field.required || details.contains(field.name)) continue;
        if (field.name == "action") details["action"] = "collect_point";
        else if (field.type == Type::kString) details[field.name] = field.values.empty() ? "a" : field.values.front();
        else if (field.type == Type::kArray) details[field.name] = Json::array();
        else if (field.type == Type::kObject && field.nullable) details[field.name] = nullptr;
        else if (field.type == Type::kBoolean) details[field.name] = true;
        else details[field.name] = 1;
      }
      EXPECT_TRUE(CheckEvent(event.type, details).empty())
          << event.type << " " << reason.code << ": " << Json(CheckEvent(event.type, details)).dump();
    }
  }
}

// The events of a command carry its client; the end of an emergency who released it.
TEST(CheckEvent, ClientAndSource) {
  const Json state{{"x", 0.0}, {"y", 0.0}, {"state", "MOWING"}, {"client", "app"}};
  EXPECT_TRUE(CheckEvent("STATE", state).empty());
  Json odd = state;
  odd["client"] = "App";
  EXPECT_TRUE(Has(CheckEvent("STATE", odd), "does not match"));
  const Json released{{"x", 0.0},          {"y", 0.0},      {"emergency", false}, {"reason", 0},
                      {"severity", "info"}, {"resume", "auto"}, {"source", "rpc"},  {"client", "ha"}};
  EXPECT_TRUE(CheckEvent("EMERGENCY", released).empty());
  Json ble = released;
  ble["source"] = "ble";
  EXPECT_TRUE(Has(CheckEvent("EMERGENCY", ble), "source: value"));
  // Never an event.
  EXPECT_TRUE(Has(CheckEvent("COMMAND_REFUSED", Json::object()), "does not know"));
}

// The items of a list of one type are checked too (lists of names in events, such as METHODS_CHANGED).
TEST(CheckEvent, ItemsOfAListOfOneType) {
  int checked = 0;
  for (const EventEntry& event : Events()) {
    for (const Field& list : event.fields) {
      if (list.type != Type::kArray || !list.fields.empty() || list.items != Type::kString) continue;
      EXPECT_TRUE(Has(CheckEvent(event.type, Json{{list.name, Json::array({7})}}), list.name + "[0]: string expected"))
          << event.type;
      EXPECT_FALSE(Has(CheckEvent(event.type, Json{{list.name, Json::array({"a"})}}), list.name + "[0]"))
          << event.type;
      ++checked;
    }
  }
  EXPECT_GT(checked, 0);
}

// A list of rows: each one an object of the fields of the catalog.
TEST(CheckParams, EachRowOfAListOfObjects) {
  const Command& start = *FindCommand("mowing.start");
  std::string field;
  const Json lawn = Json::object({{"area_id", "lawn"}});
  EXPECT_EQ(CheckParams(start, Json{{"tasks", Json::array({lawn, lawn})}, {"restart", true}}, &field), "");
  for (const Json& row : {Json("lawn"), Json::object(), Json::object({{"area_id", 1}}),
                          Json::object({{"area_id", "lawn"}, {"angle", 30}})}) {
    field.clear();
    EXPECT_NE(CheckParams(start, Json{{"tasks", Json::array({lawn, row})}}, &field), "") << row.dump();
    EXPECT_EQ(field, "tasks");
  }
  EXPECT_NE(CheckParams(start, Json{{"tasks", Json::array({Json::object()})}}, &field).find("tasks[0].area_id"),
            std::string::npos);
}

// calibration.start takes client and lets other keys through, as before api 66.
TEST(CheckParams, AMethodWithOpenParams) {
  const Method* start = FindMethod("calibration.start");
  ASSERT_NE(start, nullptr);
  std::string field;
  EXPECT_EQ(CheckParams(*start, Json::object(), &field), "");
  EXPECT_EQ(CheckParams(*start, Json{{"client", "app"}, {"other", 1}}, &field), "");
  EXPECT_NE(CheckParams(*start, Json{{"client", "App!"}}, &field), "");
  EXPECT_EQ(field, "client");
  EXPECT_NE(CheckParams(*start, Json::array(), &field), "");
  EXPECT_EQ(FindMethod("mowing.start"), nullptr);
}

// The setup condition calibration: a done result, the note of calibration.apply for it, the six values unchanged.
TEST(CalibrationTaken, ByTheNoteAndTheValues) {
  const Json result = {{"id", "calib-1"}, {"status", "done"}};
  Json values = Json::object();
  for (const auto& key : CalibrationKeys()) values[key] = 0.5;
  Json params = values;
  params["calibration_applied"] = {{"result_id", "calib-1"}, {"keys", Json::array()}, {"values", values}};
  EXPECT_EQ(CalibrationKeys().size(), 6u);
  EXPECT_TRUE(CalibrationTaken(result, params));
  Json other = params;
  other["track"] = 0.5 + 1e-6;
  EXPECT_FALSE(CalibrationTaken(result, other));
  other = params;
  other["blade_ahead"] = 0.3;
  EXPECT_TRUE(CalibrationTaken(result, other));
  EXPECT_FALSE(CalibrationTaken({{"id", "calib-2"}, {"status", "done"}}, params));
  EXPECT_FALSE(CalibrationTaken({{"id", "calib-1"}, {"status", "not_evaluated"}}, params));
  other = params;
  other["calibration_applied"] = nullptr;
  EXPECT_FALSE(CalibrationTaken(result, other));
  EXPECT_FALSE(CalibrationTaken(Json(), params));
}
