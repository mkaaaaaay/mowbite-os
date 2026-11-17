#include "mowbite_setup/layer.h"

#include <regex>
#include <sstream>

namespace mowbite_setup {

namespace {

using json = Layer::json;

constexpr char kHead[] =
    "# Written by the setup program of MowBite OS, which writes it anew: values of your own go into the file of this\n"
    "# mower beside it, which is read after this one.\n";
constexpr char kParameters[] = "  ros__parameters:";

bool IsKey(const std::string& name) {
  static const std::regex pattern("[a-z][a-z0-9_]*(\\.[a-z][a-z0-9_]*)*");
  return std::regex_match(name, pattern);
}

// b for a boolean, i for a whole number, f for another number, s for a text, 0 for anything else.
char Kind(const json& value) {
  if (value.is_boolean()) return 'b';
  if (value.is_number_integer()) return 'i';
  if (value.is_number_float()) return 'f';
  if (value.is_string()) return 's';
  return 0;
}

bool IsValue(const json& value) {
  if (!value.is_array()) return Kind(value) != 0;
  if (value.empty()) return false;
  const char kind = Kind(value[0]);
  for (const auto& item : value) {
    if (Kind(item) != kind) return false;
  }
  return kind != 0;
}

// JSON, which a YAML reader takes as it is; a number that is no whole one always with a point, as YAML 1.1 asks.
std::string Scalar(const json& value) {
  std::string text = value.dump(-1, ' ', false, json::error_handler_t::replace);
  if (value.is_number_float() && text.find('.') == std::string::npos) {
    const size_t exponent = text.find('e');
    text.insert(exponent == std::string::npos ? text.size() : exponent, ".0");
  }
  return text;
}

std::string ValueText(const json& value) {
  if (!value.is_array()) return Scalar(value);
  std::string text = "[";
  for (const auto& item : value) text += (text.size() > 1 ? ", " : "") + Scalar(item);
  return text + "]";
}

}  // namespace

bool Layer::IsNode(const std::string& name) {
  static const std::regex pattern("[a-z][a-z0-9_]*");
  return std::regex_match(name, pattern);
}

bool Layer::Set(const std::string& node, const std::string& key, const json& value) {
  if (!IsNode(node) || !IsKey(key) || !IsValue(value)) return false;
  values_[node][key] = value;
  return true;
}

void Layer::Erase(const std::string& node, const std::string& key) {
  const auto found = values_.find(node);
  if (found == values_.end()) return;
  found->second.erase(key);
  if (found->second.empty()) values_.erase(found);
}

json Layer::Get(const std::string& node, const std::string& key) const {
  const auto found = values_.find(node);
  if (found == values_.end() || !found->second.count(key)) return nullptr;
  return found->second.at(key);
}

std::string Layer::Text() const {
  if (values_.empty()) return "";
  std::string text = kHead;
  for (const auto& [node, values] : values_) {
    text += node + ":\n" + kParameters + "\n";
    for (const auto& [key, value] : values) text += "    " + key + ": " + ValueText(value) + "\n";
  }
  return text;
}

bool Layer::Parse(const std::string& text, std::string* why) {
  values_.clear();
  const auto fail = [this, why](int number, const std::string& message) {
    values_.clear();
    if (why) *why = "line " + std::to_string(number) + ": " + message;
    return false;
  };
  std::istringstream in(text);
  std::string line, node;
  bool parameters = false;
  for (int number = 1; std::getline(in, line); ++number) {
    if (line.empty() || line[0] == '#') continue;
    if (line[0] != ' ') {
      if (line.back() != ':' || !IsNode(line.substr(0, line.size() - 1))) return fail(number, "no node");
      node = line.substr(0, line.size() - 1);
      parameters = false;
    } else if (line == kParameters) {
      if (node.empty() || parameters) return fail(number, "ros__parameters without its node");
      parameters = true;
    } else {
      const size_t colon = line.find(": ");
      if (!parameters || line.rfind("    ", 0) != 0 || colon == std::string::npos) return fail(number, "no value");
      const json value = json::parse(line.substr(colon + 2), nullptr, false);
      if (value.is_discarded() || !Set(node, line.substr(4, colon - 4), value)) return fail(number, "no value");
    }
  }
  return true;
}

}  // namespace mowbite_setup
