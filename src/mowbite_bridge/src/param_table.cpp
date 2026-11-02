#include "mowbite_bridge/param_table.h"

#include <algorithm>
#include <cctype>

#include "mowbite_bridge/text.h"

namespace mowbite_bridge {

namespace {

constexpr std::size_t kMaxDepth = 32;

nlohmann::json ParseObject(std::string_view text) {
  if (NestingDepth(text) > kMaxDepth) return nlohmann::json::object();
  nlohmann::json j = nlohmann::json::parse(text.begin(), text.end(), nullptr, false);
  return j.is_object() ? j : nlohmann::json::object();
}

bool IsSecret(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
  return name.find("password") != std::string::npos;
}

// "/name" without a slash at the end, whatever the node wrote.
std::string NodeKey(std::string node) {
  while (!node.empty() && node.back() == '/') node.pop_back();
  if (node.empty() || node.front() != '/') node.insert(node.begin(), '/');
  return node;
}

}  // namespace

ParamTable::ParamTable(std::string_view base) : base_(ParseObject(base)) {
  Render();
}

bool ParamTable::Apply(std::string_view message) {
  const nlohmann::json j = ParseObject(message);
  const auto node = j.find("node");
  const auto params = j.find("params");
  if (node == j.end() || !node->is_string() || params == j.end() || !params->is_object()) return false;
  const std::string key = NodeKey(node->get<std::string>());
  if (key == "/") return false;
  nlohmann::json flat = nlohmann::json::object();
  for (const auto& [name, value] : params->items()) {
    flat[key + "/" + name] = IsSecret(name) ? nlohmann::json() : value;
  }
  nodes_[key] = std::move(flat);
  const std::string before = text_;
  Render();
  return text_ != before;
}

void ParamTable::Render() {
  // nlohmann::json keeps its keys sorted.
  nlohmann::json all = base_;
  for (const auto& [node, flat] : nodes_) all.update(flat);
  text_ = all.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
}

}  // namespace mowbite_bridge
