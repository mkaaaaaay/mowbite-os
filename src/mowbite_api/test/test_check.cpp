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
