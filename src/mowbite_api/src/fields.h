// Builders of the fields of methods, topics and the setup program. The events have their own, optional by default.
#ifndef MOWBITE_API_FIELDS_H
#define MOWBITE_API_FIELDS_H

#include <string>
#include <utility>
#include <vector>

#include "mowbite_api/catalog.h"

namespace mowbite_api::fields {

inline Field Number(std::string name, std::string unit, std::string text, bool required = true) {
  return {std::move(name), Type::kNumber, std::move(unit), required, std::move(text), {}, false, {}, ""};
}

inline Field Integer(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kInteger, "", required, std::move(text), {}, false, {}, ""};
}

inline Field Text(std::string name, std::string text, bool required = true, std::vector<std::string> values = {}) {
  return {std::move(name), Type::kString, "", required, std::move(text), std::move(values), false, {}, ""};
}

inline Field Bool(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kBoolean, "", required, std::move(text), {}, false, {}, ""};
}

inline Field Object(std::string name, std::string text, std::vector<Field> fields = {}, bool required = true) {
  return {std::move(name), Type::kObject, "", required, std::move(text), {}, false, std::move(fields), ""};
}

// A list of values of one type.
inline Field List(std::string name, std::string text, Type items, bool required = true) {
  Field field = {std::move(name), Type::kArray, "", required, std::move(text), {}, false, {}, ""};
  field.items = items;
  return field;
}

// A list of objects of these fields.
inline Field Rows(std::string name, std::string text, std::vector<Field> fields, bool required = true) {
  return {std::move(name), Type::kArray, "", required, std::move(text), {}, false, std::move(fields), ""};
}

// Takes the fields of its reason in x-reasons as well.
inline Field Open(Field field) {
  field.open = true;
  return field;
}

inline Field Nullable(Field field) {
  field.nullable = true;
  return field;
}

// reason, message, severity and resume as PutReason of the logic writes them.
inline std::vector<Field> Rated(bool always, std::vector<Field> before = {}) {
  before.push_back(Text("reason", "Why: a fixed name in capitals, rated in x-reasons", always));
  before.push_back(Text("message", "English text for the log and for apps that do not know the reason", false));
  before.push_back(Text("severity", "How bad it is", always, {"info", "warning", "error"}));
  before.push_back(
      Text("resume", "auto when it goes on by itself, user when it waits for the user", always, {"auto", "user"}));
  return before;
}

}  // namespace mowbite_api::fields

#endif  // MOWBITE_API_FIELDS_H
