#include <algorithm>
#include <cmath>
#include <regex>
#include <set>
#include <string>

#include "mowbite_api/catalog.h"

namespace mowbite_api {

namespace {

bool HasType(const Json& value, Type type) {
  switch (type) {
    case Type::kString: return value.is_string();
    case Type::kNumber: return value.is_number();
    case Type::kInteger:
      return value.is_number_integer() ||
             (value.is_number_float() && std::isfinite(value.get<double>()) &&
              std::floor(value.get<double>()) == value.get<double>());
    case Type::kBoolean: return value.is_boolean();
    case Type::kObject: return value.is_object();
    case Type::kArray: return value.is_array();
    case Type::kAny: return true;
  }
  return false;
}

const char* TypeName(Type type) {
  switch (type) {
    case Type::kString: return "string";
    case Type::kNumber: return "number";
    case Type::kInteger: return "integer";
    case Type::kBoolean: return "boolean";
    case Type::kObject: return "object";
    case Type::kArray: return "array";
    case Type::kAny: return "any";
  }
  return "?";
}

// One field against its entry: type and the fixed values of a text.
void CheckField(const std::string& where, const Field& field, const Json& value, std::vector<std::string>* problems) {
  if (value.is_null() && field.nullable) return;
  if (!HasType(value, field.type)) {
    problems->push_back(where + "." + field.name + ": " + TypeName(field.type) + " expected, got " + value.dump());
    return;
  }
  if (!field.values.empty() && std::find(field.values.begin(), field.values.end(), value) == field.values.end()) {
    problems->push_back(where + "." + field.name + ": value " + value.dump() + " is not one of the catalog's");
  }
  if (value.is_number() && field.min && value.get<double>() < *field.min) {
    problems->push_back(where + "." + field.name + ": " + value.dump() + " is below " + Json(*field.min).dump());
  }
  if (value.is_number() && field.max && value.get<double>() > *field.max) {
    problems->push_back(where + "." + field.name + ": " + value.dump() + " is above " + Json(*field.max).dump());
  }
  if (!field.pattern.empty() && value.is_string() &&
      !std::regex_search(value.get<std::string>(), std::regex(field.pattern, std::regex::ECMAScript))) {
    problems->push_back(where + "." + field.name + ": " + value.dump() + " does not match " + field.pattern);
  }
  if (field.type == Type::kArray && !field.fields.empty()) {
    // Each row is an object of these fields.
    Field row = field;
    row.type = Type::kObject;
    for (size_t i = 0; i < value.size(); ++i) {
      row.name = field.name + "[" + std::to_string(i) + "]";
      CheckField(where, row, value[i], problems);
    }
    return;
  }
  if (field.type != Type::kObject || field.fields.empty()) return;
  std::set<std::string> known;
  for (const auto& inner : field.fields) {
    known.insert(inner.name);
    if (!value.contains(inner.name)) {
      if (inner.required) problems->push_back(where + "." + field.name + "." + inner.name + " is missing");
      continue;
    }
    CheckField(where + "." + field.name, inner, value[inner.name], problems);
  }
  for (const auto& [key, inner] : value.items()) {
    if (!known.count(key)) problems->push_back(where + "." + field.name + "." + key + " is not in the catalog");
  }
}

}  // namespace

std::vector<std::string> CheckEvent(const std::string& type, const Json& details, bool strict) {
  std::vector<std::string> problems;
  if (!details.is_object()) return {type + ": the details are not an object"};
  // The envelope where it is there: the logic's tests see the details before the bridge adds it.
  for (const auto& field : Envelope()) {
    if (details.contains(field.name)) CheckField(type, field, details[field.name], &problems);
  }
  if (details.contains("type") && details["type"] != type) {
    problems.push_back(type + ": type is " + details["type"].dump());
  }
  const EventEntry* event = FindEvent(type);
  if (!event) {
    if (strict) problems.push_back(type + ": a type the catalog does not know");
    return problems;
  }
  std::vector<const Field*> fields;
  fields.reserve(event->fields.size() + LogicFields().size());
  for (const auto& field : event->fields) fields.push_back(&field);
  if (event->emitter == "logic") {
    for (const auto& field : LogicFields()) fields.push_back(&field);
  }
  // The reason first: its own fields belong to the event as well.
  const ReasonEntry* reason = nullptr;
  if (details.contains("reason") && details["reason"].is_string()) {
    const std::string code = details["reason"].get<std::string>();
    for (const auto& entry : event->reasons) {
      if (ReasonFits(entry, code, details)) {
        reason = &entry;
        break;
      }
    }
    if (!reason) {
      problems.push_back(type + ": reason " + code + " is not in the catalog for this event");
    } else {
      for (const auto& field : reason->fields) fields.push_back(&field);
      const Rating rating = RateReason(type, code, details);
      if (details.contains("severity") && details["severity"] != rating.severity) {
        problems.push_back(type + " " + code + ": severity " + details["severity"].dump() + ", the catalog says " +
                           rating.severity);
      }
      if (details.contains("resume") && details["resume"] != rating.resume) {
        problems.push_back(type + " " + code + ": resume " + details["resume"].dump() + ", the catalog says " +
                           rating.resume);
      }
    }
  }
  std::set<std::string> known;
  for (const auto& field : Envelope()) known.insert(field.name);
  for (const Field* field : fields) {
    // A reason's field may repeat one of the event; the event's entry was checked first.
    if (!known.insert(field->name).second) continue;
    if (!details.contains(field->name)) {
      if (field->required) problems.push_back(type + ": " + field->name + " is missing");
      continue;
    }
    CheckField(type, *field, details[field->name], &problems);
  }
  for (const auto& [key, value] : details.items()) {
    if (!known.count(key)) problems.push_back(type + ": field " + key + " is not in the catalog");
  }
  return problems;
}

std::string CheckParams(const Command& command, const Json& params, std::string* field) {
  const auto fail = [field](const std::string& name, const std::string& text) {
    if (field) *field = name;
    return text;
  };
  if (!params.is_object()) return fail("", command.name + ": the params are not an object");
  for (const auto& [key, value] : params.items()) {
    const auto entry = std::find_if(command.params.begin(), command.params.end(),
                                    [&key](const Field& param) { return param.name == key; });
    if (entry == command.params.end()) return fail(key, command.name + " takes no " + key);
    std::vector<std::string> problems;
    CheckField(command.name, *entry, value, &problems);
    if (!problems.empty()) return fail(key, problems.front());
  }
  for (const auto& param : command.params) {
    if (param.required && !params.contains(param.name)) {
      return fail(param.name, command.name + "." + param.name + " is missing");
    }
  }
  return "";
}

}  // namespace mowbite_api
