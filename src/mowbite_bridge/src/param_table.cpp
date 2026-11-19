#include "mowbite_bridge/param_table.h"

#include "mowbite_bridge/text.h"

namespace mowbite_bridge {

namespace {

constexpr std::size_t kMaxDepth = 32;

nlohmann::json ParseObject(std::string_view text) {
  if (NestingDepth(text) > kMaxDepth) return nlohmann::json::object();
  nlohmann::json j = nlohmann::json::parse(text.begin(), text.end(), nullptr, false);
  return j.is_object() ? j : nlohmann::json::object();
}

// "/name" without a slash at the end, whatever the node wrote.
std::string NodeKey(std::string node) {
  while (!node.empty() && node.back() == '/') node.pop_back();
  if (node.empty() || node.front() != '/') node.insert(node.begin(), '/');
  return node;
}

// A value as it may go into the retained params/json: secret names nulled at every depth, texts through HideSecrets.
nlohmann::json Masked(const std::string& name, const nlohmann::json& value, std::size_t depth = 0) {
  if (IsSecretName(name) || depth > kMaxDepth) return nlohmann::json();
  if (value.is_string()) return HideSecrets(value.get<std::string>());
  if (value.is_object()) {
    nlohmann::json out = nlohmann::json::object();
    for (const auto& [key, item] : value.items()) out[key] = Masked(key, item, depth + 1);
    return out;
  }
  if (value.is_array()) {
    nlohmann::json out = nlohmann::json::array();
    for (const auto& item : value) out.push_back(Masked("", item, depth + 1));
    return out;
  }
  return value;
}

nlohmann::json MaskedObject(const nlohmann::json& object) {
  nlohmann::json out = nlohmann::json::object();
  for (const auto& [key, value] : object.items()) out[key] = Masked(key, value);
  return out;
}

}  // namespace

ParamTable::ParamTable(std::string_view base) : base_(MaskedObject(ParseObject(base))) {
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
    flat[key + "/" + name] = Masked(name, value);
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
