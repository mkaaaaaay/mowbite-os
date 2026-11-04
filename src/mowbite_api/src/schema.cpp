#include <algorithm>
#include <map>
#include <set>

#include "mowbite_api/catalog.h"
#include "mowbite_api/planner_schema.h"

namespace mowbite_api {

namespace {

const char* TypeName(Type type) {
  switch (type) {
    case Type::kString: return "string";
    case Type::kNumber: return "number";
    case Type::kInteger: return "integer";
    case Type::kBoolean: return "boolean";
    case Type::kObject: return "object";
    case Type::kArray: return "array";
  }
  return "string";
}

Json Property(const Field& field) {
  Json property = Json::object();
  if (field.nullable) {
    property["type"] = Json::array({TypeName(field.type), "null"});
  } else {
    property["type"] = TypeName(field.type);
  }
  if (!field.values.empty()) {
    property["enum"] = field.values;
    if (field.nullable) property["enum"].push_back(nullptr);
  }
  if (!field.pattern.empty()) property["pattern"] = field.pattern;
  property["description"] = field.text;
  if (!field.unit.empty()) property["x-unit"] = field.unit;
  if (!field.fields.empty()) {
    Json properties = Json::object();
    Json required = Json::array();
    for (const auto& inner : field.fields) {
      properties[inner.name] = Property(inner);
      if (inner.required) required.push_back(inner.name);
    }
    property["properties"] = properties;
    property["required"] = required;
    property["additionalProperties"] = false;
  }
  return property;
}

// An object of these fields; a name that comes twice keeps its first entry.
Json Object(const std::string& text, const std::vector<const Field*>& fields, bool closed) {
  Json properties = Json::object();
  Json required = Json::array();
  std::map<std::string, std::string> units;
  for (const Field* field : fields) {
    if (properties.contains(field->name)) {
      // limit of two reasons in two units: the unit is in x-reasons then.
      if (units[field->name] != field->unit) properties[field->name].erase("x-unit");
      continue;
    }
    properties[field->name] = Property(*field);
    units[field->name] = field->unit;
    if (field->required) required.push_back(field->name);
  }
  Json object = {{"type", "object"}, {"description", text}, {"properties", properties}, {"required", required}};
  if (closed) object["additionalProperties"] = false;
  return object;
}

std::vector<const Field*> Pointers(const std::vector<Field>& fields) {
  std::vector<const Field*> out;
  for (const auto& field : fields) out.push_back(&field);
  return out;
}

Json EventSchema(const EventEntry& event) {
  std::vector<const Field*> fields = Pointers(Envelope());
  if (event.emitter == "logic") {
    for (const auto& field : LogicFields()) fields.push_back(&field);
  }
  for (const auto& field : event.fields) fields.push_back(&field);
  for (const auto& reason : event.reasons) {
    for (const auto& field : reason.fields) fields.push_back(&field);
  }
  Json schema = Object(event.text, fields, true);
  schema["properties"]["type"] = {{"const", event.type}, {"description", Envelope()[2].text}};
  if (!event.reasons.empty() && schema["properties"].contains("reason")) {
    Json codes = Json::array();
    for (const auto& reason : event.reasons) {
      if (std::find(codes.begin(), codes.end(), reason.code) == codes.end()) codes.push_back(reason.code);
    }
    schema["properties"]["reason"]["enum"] = codes;
  }
  schema["x-emitter"] = event.emitter;
  return schema;
}

// An event of the planner from its part of the schema: the envelope with its own fields.
Json PlannerEventSchema(const std::string& type, const Json& part) {
  Json schema = Object(part.value("description", ""), Pointers(Envelope()), true);
  schema["properties"]["type"] = {{"const", type}, {"description", Envelope()[2].text}};
  // Bound first: a range over items() of a temporary would outlive it.
  const Json properties = part.value("properties", Json::object());
  for (const auto& [name, property] : properties.items()) {
    if (!schema["properties"].contains(name)) schema["properties"][name] = property;
  }
  const Json required = part.value("required", Json::array());
  for (const auto& name : required) schema["required"].push_back(name);
  schema["x-emitter"] = "planner";
  return schema;
}

Json Reasons(const EventEntry& event) {
  Json out = Json::object();
  for (const auto& reason : event.reasons) {
    if (!reason.action.empty()) {
      // A rating only for one action; the plain entry of the code carries it under "when".
      Json when = {{"action", reason.action}, {"severity", reason.severity}, {"resume", reason.resume}};
      out[reason.code]["when"].push_back(when);
      continue;
    }
    Json fields = Json::object();
    for (const auto& field : reason.fields) fields[field.name] = field.unit;
    Json entry = {{"severity", reason.severity}, {"resume", reason.resume}, {"description", reason.text},
                  {"fields", fields}};
    if (out.contains(reason.code) && out[reason.code].contains("when")) entry["when"] = out[reason.code]["when"];
    out[reason.code] = entry;
  }
  return out;
}

Json BuildSchema(const Json& planner) {
  Json defs = Json::object();
  defs["robot_state"] = Object("robot_state/json, retained: the state of the mower for the app", Pointers(RobotState()),
                               true);
  Json one_of = Json::array();
  Json known = Json::array();
  Json reasons = Json::object();
  for (const auto& event : Events()) {
    defs["event." + event.type] = EventSchema(event);
    one_of.push_back({{"$ref", "#/$defs/event." + event.type}});
    known.push_back(event.type);
    if (!event.reasons.empty()) reasons[event.type] = Reasons(event);
  }
  for (const auto& group : Refusals()) reasons[group.type] = Reasons(group);
  // The data of a refusal, whatever its group: the fields of COMMAND_REFUSED and, open, those of its reason.
  defs["refusal"] = Object("data of the -32600 of a refused command; event names the group of its reason in x-reasons",
                           Pointers(Refusals().front().fields), false);
  if (planner.is_object()) {
    const Json events = planner.value("events", Json::object());
    for (const auto& [type, part] : events.items()) {
      defs["event." + type] = PlannerEventSchema(type, part);
      one_of.push_back({{"$ref", "#/$defs/event." + type}});
      known.push_back(type);
      if (part.contains("x-reasons")) reasons[type] = part["x-reasons"];
    }
    const Json topics = planner.value("topics", Json::object());
    for (const auto& [topic, schema] : topics.items()) {
      defs["topic." + topic] = schema;
    }
  }
  Json other = Object("An event of a type this version does not know, from the planner or a newer OS: only the "
                      "envelope is fixed",
                      Pointers(Envelope()), false);
  other["properties"]["type"]["not"] = {{"enum", known}};
  defs["event.other"] = other;
  one_of.push_back({{"$ref", "#/$defs/event.other"}});
  defs["event"] = {{"description", "events/json: one event, of a known type or only with the envelope"},
                   {"oneOf", one_of}};
  Json commands = Json::object();
  for (const auto& command : Commands()) {
    defs[command.name + ".params"] = Object("The params of " + command.name, Pointers(command.params), true);
    defs[command.name + ".result"] = Object("The result of " + command.name, Pointers(command.result), true);
    Json actions = Json::array();
    for (const auto& entry : command.actions) actions.push_back({{"action", entry.action}, {"params", entry.params}});
    commands[command.name] = {{"description", command.text},
                              {"params", "#/$defs/" + command.name + ".params"},
                              {"result", "#/$defs/" + command.name + ".result"},
                              {"refused", command.refused.empty() ? Json() : Json(command.refused)},
                              {"action", command.actions.empty() ? Json() : Json(command.actions[0].action)},
                              {"actions", actions}};
  }
  Json bits = Json::array();
  for (const auto& bit : EmergencyBits()) {
    bits.push_back({{"bit", bit.bit}, {"name", bit.name}, {"description", bit.text}});
  }
  Json schema = {{"$schema", "https://json-schema.org/draft/2020-12/schema"},
                 {"$id", "urn:mowbite-os:api:" + std::to_string(kApi)},
                 {"title", "MowBite OS app interface"},
                 {"description",
                  "robot_state, events and RPC methods of MowBite OS over MQTT, as system.api gives them. "
                  "x-reasons has severity and resume of every reason by event (COMMAND_REFUSED only in refusals), "
                  "x-emergency-bits the bits of emergency, x-commands the methods with params, result, the event "
                  "of their refusal and the actions that do the same."},
                 {"x-api", kApi},
                 {"$defs", defs},
                 {"x-reasons", reasons},
                 {"x-emergency-bits", bits},
                 {"x-commands", commands}};
  if (planner.is_object()) schema["x-planner"] = {{"version", planner.value("version", 0)}};
  return schema;
}

std::set<std::string> Types(const Json& schema) {
  std::set<std::string> types;
  if (!schema.contains("type")) return types;
  if (schema["type"].is_string()) types.insert(schema["type"].get<std::string>());
  if (schema["type"].is_array()) {
    for (const auto& type : schema["type"]) types.insert(type.get<std::string>());
  }
  return types;
}

bool Contains(const Json& list, const Json& value) {
  return list.is_array() && std::find(list.begin(), list.end(), value) != list.end();
}

// What newer takes away from older at this place; params are what clients send, the rest what the OS sends.
void Compare(const Json& older, const Json& newer, const std::string& path, bool params,
             std::vector<std::string>* out) {
  if (!older.is_object()) return;
  if (!newer.is_object()) {
    out->push_back(path + ": no longer a schema");
    return;
  }
  const std::set<std::string> old_types = Types(older);
  const std::set<std::string> new_types = Types(newer);
  for (const auto& type : old_types) {
    // integer may widen to number, never the other way.
    if (!new_types.count(type) && !(type == "integer" && new_types.count("number"))) {
      out->push_back(path + ": type " + type + " gone");
    }
  }
  if (older.contains("const") && (!newer.contains("const") || newer["const"] != older["const"])) {
    out->push_back(path + ": const changed");
  }
  if (older.contains("enum") && newer.contains("enum")) {
    for (const auto& value : older["enum"]) {
      if (!Contains(newer["enum"], value)) out->push_back(path + ": value " + value.dump() + " gone");
    }
  }
  if (params && !older.contains("enum") && newer.contains("enum")) out->push_back(path + ": values newly limited");
  if (older.contains("properties")) {
    const Json& now = newer.value("properties", Json::object());
    for (const auto& [name, property] : older["properties"].items()) {
      if (!now.contains(name)) {
        out->push_back(path + "." + name + ": gone");
        continue;
      }
      Compare(property, now[name], path + "." + name, params, out);
    }
  }
  const Json old_required = older.value("required", Json::array());
  const Json new_required = newer.value("required", Json::array());
  if (params) {
    for (const auto& name : new_required) {
      if (!Contains(old_required, name)) out->push_back(path + "." + name.get<std::string>() + ": newly required");
    }
    if (older.value("additionalProperties", true) != false && newer.value("additionalProperties", true) == false) {
      out->push_back(path + ": no longer takes other fields");
    }
  } else {
    for (const auto& name : old_required) {
      if (!Contains(new_required, name)) {
        out->push_back(path + "." + name.get<std::string>() + ": no longer always there");
      }
    }
  }
  if (older.contains("items")) Compare(older["items"], newer.value("items", Json()), path + "[]", params, out);
  if (older.contains("oneOf")) {
    for (const auto& choice : older["oneOf"]) {
      if (!Contains(newer.value("oneOf", Json::array()), choice)) out->push_back(path + ": " + choice.dump() + " gone");
    }
  }
}

bool EndsWith(const std::string& text, const std::string& end) {
  return text.size() >= end.size() && text.compare(text.size() - end.size(), end.size(), end) == 0;
}

}  // namespace

const Json& Schema() {
  static const Json schema = BuildSchema(PlannerPart());
  return schema;
}

Json SchemaWithPlanner(const Json& planner) {
  return BuildSchema(planner);
}

std::vector<std::string> CheckAdditive(const Json& older, const Json& newer) {
  std::vector<std::string> out;
  const Json& old_defs = older.value("$defs", Json::object());
  const Json& new_defs = newer.value("$defs", Json::object());
  for (const auto& [name, schema] : old_defs.items()) {
    if (!new_defs.contains(name)) {
      out.push_back(name + ": gone");
      continue;
    }
    Compare(schema, new_defs[name], name, EndsWith(name, ".params"), &out);
  }
  const Json& new_reasons = newer.value("x-reasons", Json::object());
  const Json old_reasons = older.value("x-reasons", Json::object());
  for (const auto& [event, codes] : old_reasons.items()) {
    for (const auto& [code, entry] : codes.items()) {
      if (!new_reasons.contains(event) || !new_reasons[event].contains(code)) {
        out.push_back("x-reasons." + event + "." + code + ": gone");
      }
    }
  }
  const Json& new_bits = newer.value("x-emergency-bits", Json::array());
  const Json old_bits = older.value("x-emergency-bits", Json::array());
  for (const auto& bit : old_bits) {
    const bool kept = std::any_of(new_bits.begin(), new_bits.end(), [&bit](const Json& now) {
      return now.value("bit", -1) == bit.value("bit", -2) && now.value("name", "") == bit.value("name", "");
    });
    if (!kept) out.push_back("x-emergency-bits." + bit.value("name", "?") + ": gone or renamed");
  }
  const Json& new_commands = newer.value("x-commands", Json::object());
  const Json old_commands = older.value("x-commands", Json::object());
  for (const auto& [name, command] : old_commands.items()) {
    if (!new_commands.contains(name)) out.push_back("x-commands." + name + ": gone");
  }
  return out;
}

}  // namespace mowbite_api
