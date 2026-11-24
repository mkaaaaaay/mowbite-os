#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_api/planner_schema.h"

using namespace mowbite_api;

namespace {

Json Read(const std::filesystem::path& file) {
  std::ifstream in(file);
  return Json::parse(in, nullptr, false);
}

bool Has(const std::vector<std::string>& problems, const std::string& part) {
  for (const auto& problem : problems) {
    if (problem.find(part) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

TEST(Schema, TheFileInTheRepoIsTheGeneratedOne) {
  const Json file = Read(std::filesystem::path(MOWBITE_API_DIR) / "api.schema.json");
  ASSERT_FALSE(file.is_discarded()) << "api.schema.json is not JSON";
  if (file == Schema()) return;
  if (file.value("x-api", 0) == kApi) {
    FAIL() << "The catalog changed: raise kApi in catalog.h, then mowbite_api_schema --write " MOWBITE_API_DIR;
  }
  FAIL() << "api.schema.json is of api " << file.value("x-api", 0) << ": mowbite_api_schema --write " MOWBITE_API_DIR;
}

// Every older version stays in schema/api-<N>.schema.json, and the current one only adds to those of its break.
TEST(Schema, OlderVersionsAreOnlyAddedTo) {
  const std::filesystem::path dir = std::filesystem::path(MOWBITE_API_DIR) / "schema";
  if (!std::filesystem::exists(dir)) {
    EXPECT_EQ(kApi, 1) << "api " << kApi << " without the older versions in " << dir;
    return;
  }
  int older = 0;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const Json old = Read(entry.path());
    ASSERT_FALSE(old.is_discarded()) << entry.path();
    EXPECT_EQ(entry.path().filename(), "api-" + std::to_string(old.value("x-api", 0)) + ".schema.json");
    EXPECT_LT(old.value("x-api", 0), kApi) << entry.path();
    EXPECT_LE(ApiBreakOf(old), kApiBreak) << entry.path();
    if (ApiBreakOf(old) != kApiBreak) continue;
    for (const auto& problem : CheckAdditive(old, Schema())) ADD_FAILURE() << entry.path() << ": " << problem;
    older++;
  }
  EXPECT_EQ(older, kApi - 1);
}

TEST(Schema, IdAndVersion) {
  const Json& schema = Schema();
  EXPECT_EQ(schema["$schema"], "https://json-schema.org/draft/2020-12/schema");
  EXPECT_EQ(schema["$id"], "urn:mowbite-os:api:" + std::to_string(kApi));
  EXPECT_EQ(schema["x-api"], kApi);
  EXPECT_EQ(schema["x-api-break"], kApiBreak);
  EXPECT_EQ(ApiBreakOf(schema), kApiBreak);
}

// A schema from before x-api-break is of break 1.
TEST(Schema, BreakOfAnOlderSchema) {
  EXPECT_EQ(ApiBreakOf(Json::object()), 1);
  EXPECT_EQ(ApiBreakOf(Json{{"x-api", 30}}), 1);
  EXPECT_EQ(ApiBreakOf(Json{{"x-api", 40}, {"x-api-break", 2}}), 2);
}

// Every event of the catalog by its type, any other type only with the envelope.
TEST(Schema, EventsOneOfByType) {
  const Json& defs = Schema()["$defs"];
  const Json& one_of = defs["event"]["oneOf"];
  for (const auto& event : Events()) {
    const std::string ref = "#/$defs/event." + event.type;
    EXPECT_NE(std::find(one_of.begin(), one_of.end(), Json{{"$ref", ref}}), one_of.end()) << event.type;
    EXPECT_EQ(defs["event." + event.type]["properties"]["type"]["const"], event.type);
    EXPECT_EQ(defs["event." + event.type]["additionalProperties"], false);
    const Json& known = defs["event.other"]["properties"]["type"]["not"]["enum"];
    EXPECT_NE(std::find(known.begin(), known.end(), event.type), known.end());
  }
  EXPECT_EQ(defs["event.other"]["required"], Json({"id", "t", "type"}));
  EXPECT_FALSE(defs["event.other"].contains("additionalProperties"));
}

TEST(Schema, CommandsAndBits) {
  const Json& schema = Schema();
  for (const auto& command : Commands()) {
    EXPECT_TRUE(schema["x-commands"].contains(command.name)) << command.name;
    EXPECT_TRUE(schema["$defs"].contains(command.name + ".params")) << command.name;
    EXPECT_TRUE(schema["$defs"].contains(command.name + ".result")) << command.name;
  }
  EXPECT_EQ(schema["x-emergency-bits"].size(), EmergencyBits().size());
  EXPECT_EQ(schema["$defs"]["system.info.result"]["properties"]["api"]["type"], "integer");
  EXPECT_EQ(schema["$defs"]["system.info.result"]["properties"]["api_break"]["type"], "integer");
  EXPECT_EQ(schema["$defs"]["system.api.result"]["properties"]["api_break"]["type"], "integer");
  EXPECT_EQ(schema["$defs"]["system.info.result"]["properties"]["mqtt"]["type"], "integer");
  // Action and RPC: recording.save stands for both finish actions, with its type.
  const Json& save = schema["x-commands"]["recording.save"];
  EXPECT_EQ(save["refused"], "RECORDING_REFUSED");
  EXPECT_EQ(save["actions"], Json::parse(R"([
      {"action": "mower_logic:area_recording/finish_mowing_area", "params": {"type": "mow"}},
      {"action": "mower_logic:area_recording/finish_navigation_area", "params": {"type": "nav"}}])"));
  EXPECT_EQ(schema["x-commands"]["docking.start"]["actions"], Json::array());
  EXPECT_EQ(schema["$defs"]["mowing.pause.params"]["properties"]["client"]["pattern"], kClientPattern);
  EXPECT_EQ(schema["$defs"]["mowing.reset_job.result"]["required"], Json({"state", "sub_state", "job_id"}));
  // COMMAND_REFUSED only in x-reasons and in the data of a refusal, never an event.
  EXPECT_TRUE(schema["x-reasons"]["COMMAND_REFUSED"].contains("WRONG_STATE"));
  EXPECT_FALSE(schema["$defs"].contains("event.COMMAND_REFUSED"));
  // REQUEST_REFUSED: the bridge's refusal of an MQTT 5 request, in the same form.
  EXPECT_TRUE(schema["x-reasons"]["REQUEST_REFUSED"].contains("RESPONSE_TOPIC"));
  EXPECT_TRUE(schema["x-reasons"]["REQUEST_REFUSED"].contains("CORRELATION_DATA"));
  EXPECT_FALSE(schema["$defs"].contains("event.REQUEST_REFUSED"));
  const Json& refusal = schema["$defs"]["refusal"];
  EXPECT_EQ(refusal["required"], Json({"event", "reason"}));
  EXPECT_FALSE(refusal.contains("additionalProperties"));
  // A blocked item carries the fields of its reason (SAFETY_STEPS: steps, skipped).
  const Json& blocked = schema["$defs"]["setup"]["properties"]["steps"]["items"]["properties"]["blocked"]["items"];
  EXPECT_FALSE(blocked.contains("additionalProperties"));
  EXPECT_TRUE(schema["x-reasons"]["SETUP_BLOCKED"]["SAFETY_STEPS"].dump().find("skipped") != std::string::npos);
}

// The named corrections count only against versions before the one that made them.
TEST(Additive, ACorrectionCountsOnlyAgainstOlderVersions) {
  Json older = Schema();
  Json newer = Schema();
  Json& required = newer["$defs"]["refusal"]["required"];
  required = Json::array({"event", "reason", "message"});
  older["$defs"]["refusal"]["required"] = Json::array({"event", "reason", "message", "state"});
  older["x-api"] = 53;
  EXPECT_FALSE(Has(CheckAdditive(older, newer), "refusal.state: no longer always there"));
  older["x-api"] = 54;
  EXPECT_TRUE(Has(CheckAdditive(older, newer), "refusal.state: no longer always there"));
  // Other fields of the refusal stay strict.
  older["x-api"] = 53;
  older["$defs"]["refusal"]["required"].push_back("event_id");
  EXPECT_TRUE(Has(CheckAdditive(older, newer), "refusal.event_id: no longer always there"));
}

TEST(Additive, WhatIsTakenAwayIsFound) {
  const Json& older = Schema();
  const auto changed = [&older](const std::function<void(Json&)>& change) {
    Json newer = older;
    change(newer);
    return CheckAdditive(older, newer);
  };
  EXPECT_TRUE(CheckAdditive(older, older).empty());
  EXPECT_TRUE(Has(changed([](Json& s) { s["$defs"]["robot_state"]["properties"].erase("current_state"); }),
                  "robot_state.current_state: gone"));
  EXPECT_TRUE(Has(changed([](Json& s) {
                    s["$defs"]["robot_state"]["properties"]["current_state"]["type"] = "integer";
                  }),
                  "type string gone"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["$defs"].erase("event.STATE"); }), "event.STATE: gone"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["$defs"]["event.STATE"]["properties"]["state"]["enum"].erase(0); }),
                  "gone"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-reasons"]["STATE"].erase("MANUAL"); }), "x-reasons.STATE.MANUAL"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-emergency-bits"][3]["name"] = "LIFTED"; }), "x-emergency-bits"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-commands"].erase("docking.start"); }), "x-commands.docking.start"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-methods"].erase("map.replace"); }), "x-methods.map.replace: gone"));
  // Action ids, units, the words of schedule.log and the keys of the setup are read by HA and the app.
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-commands"]["docking.abort"]["actions"][0]["action"] = "x:y/z"; }),
                  "action mower_logic:docking/abort_docking gone"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["$defs"]["camera.stream.result"]["properties"]["expires"]["x-unit"] = "s"; }),
                  "unit changed"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-schedule-log"].erase("skip_battery"); }), "x-schedule-log.skip_battery"));
  EXPECT_TRUE(Has(changed([](Json& s) { s["x-setup"].erase("checks"); }), "x-setup.checks: gone"));
  // A parameter newly limited breaks callers; a result's limit may only narrow.
  EXPECT_TRUE(Has(changed([](Json& s) { s["$defs"]["docking.test.params"]["properties"]["runs"]["maximum"] = 10; }),
                  "maximum tighter"));
  EXPECT_TRUE(changed([](Json& s) { s["$defs"]["robot_state"]["properties"]["battery_percentage"]["maximum"] = 1; })
                  .empty());
  EXPECT_TRUE(Has(changed([](Json& s) { s["$defs"]["robot_state"]["required"] = Json::array(); }),
                  "no longer always there"));
  EXPECT_TRUE(Has(changed([](Json& s) {
                    s["$defs"]["docking.start.params"]["properties"]["client"] = {{"type", "string"}};
                    s["$defs"]["docking.start.params"]["required"].push_back("client");
                  }),
                  "newly required"));
  // Adding is fine: a field, a value, a reason, an event, a command.
  EXPECT_TRUE(changed([](Json& s) {
                s["$defs"]["robot_state"]["properties"]["new_value"] = {{"type", "number"}};
                s["$defs"]["event.STATE"]["properties"]["state"]["enum"].push_back("NEW_STATE");
                s["x-reasons"]["STATE"]["NEW_REASON"] = {{"severity", "info"}, {"resume", "user"}};
                s["$defs"]["event.NEW"] = {{"type", "object"}};
                s["$defs"]["event"]["oneOf"].push_back({{"$ref", "#/$defs/event.NEW"}});
                s["x-commands"]["new.command"] = Json::object();
                s["$defs"]["docking.start.params"]["properties"]["client"] = {{"type", "string"}};
              }).empty());
}

// The planner's part, once its pin has one: its events become known types, its topics definitions.
TEST(Schema, WithThePartOfThePlanner) {
  const Json part = {{"version", 2},
                     {"events", {{"LEFTOVERS_ARMED", {{"description", "Leftovers are armed"},
                                                      {"properties", {{"area_id", {{"type", "string"}}}}},
                                                      {"required", {"area_id"}}}}}},
                     {"topics", {{"mowbite/planner/progress", {{"type", "object"}}}}}};
  const Json schema = SchemaWithPlanner(part);
  const Json& defs = schema["$defs"];
  EXPECT_EQ(defs["event.LEFTOVERS_ARMED"]["properties"]["type"]["const"], "LEFTOVERS_ARMED");
  EXPECT_EQ(defs["event.LEFTOVERS_ARMED"]["required"], Json({"id", "t", "type", "area_id"}));
  const Json& known = defs["event.other"]["properties"]["type"]["not"]["enum"];
  EXPECT_NE(std::find(known.begin(), known.end(), "LEFTOVERS_ARMED"), known.end());
  EXPECT_EQ(defs["topic.mowbite/planner/progress"], Json({{"type", "object"}}));
  EXPECT_EQ(schema["x-planner"]["version"], 2);
  EXPECT_TRUE(CheckAdditive(SchemaWithPlanner(nullptr), schema).empty());
}

// I1: the file's head names version and owner, every method is in x-methods, and every object is sorted by key.
TEST(Schema, HeadMethodsAndOrder) {
  const Json& schema = Schema();
  EXPECT_EQ(schema["x-owner"], kOwner);
  for (const auto& command : Commands()) EXPECT_TRUE(schema["x-methods"].contains(command.name)) << command.name;
  for (const auto& method : Methods()) {
    ASSERT_TRUE(schema["x-methods"].contains(method.name)) << method.name;
    EXPECT_EQ(schema["x-methods"][method.name]["node"], method.node);
    EXPECT_TRUE(schema["$defs"].contains(method.name + ".result")) << method.name;
  }
  EXPECT_EQ(schema["x-methods"].size(), Commands().size() + Methods().size());
  // Each sensor under its own id, and under its id of OpenMower until the next break; both lead to the key.
  size_t shown = 0;
  for (const auto& sensor : Sensors()) {
    EXPECT_EQ(schema["x-sensors"][sensor.id]["key"], sensor.id);
    EXPECT_EQ(schema["x-sensors"][sensor.id]["name"], sensor.name);
    if (sensor.openmower_id.empty()) continue;
    shown++;
    EXPECT_EQ(schema["x-sensors"][sensor.openmower_id]["key"], sensor.id);
    EXPECT_EQ(schema["x-sensors"][sensor.openmower_id]["name"], sensor.openmower_name);
  }
  EXPECT_EQ(schema["x-sensors"].size(), Sensors().size() + shown);
  EXPECT_EQ(schema["x-sensors"]["os_ntrip_state"]["kind"], "text");
  EXPECT_EQ(schema["x-settings"]["undock_distance"]["min"], 0.5);
  EXPECT_EQ(schema["x-settings"]["docking_retry_count"]["type"], "integer");
  EXPECT_EQ(schema["x-geometry"]["robot_outline"]["type"], "points");
  EXPECT_FALSE(schema["x-geometry"]["dock_contact_x"]["settable"].get<bool>());
  // actions/json: reason only with the names of MOWING_REFUSED.
  const Json& reason = schema["$defs"]["actions"]["items"]["properties"]["reason"];
  EXPECT_EQ(reason["enum"].size(), schema["x-reasons"]["MOWING_REFUSED"].size());
  EXPECT_EQ(schema["$defs"]["actions"]["items"]["required"], Json({"action_id", "action_name", "enabled"}));
  std::function<void(const Json&, const std::string&)> sorted = [&](const Json& value, const std::string& path) {
    if (value.is_array()) {
      for (const auto& item : value) sorted(item, path + "[]");
    }
    if (!value.is_object()) return;
    std::string last;
    for (const auto& [key, item] : value.items()) {
      EXPECT_LT(last, key) << path;
      last = key;
      sorted(item, path + "." + key);
    }
  };
  sorted(schema, "");
}
