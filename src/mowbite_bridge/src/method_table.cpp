#include "mowbite_bridge/method_table.h"

namespace mowbite_bridge {

void MethodTable::AddLocal(const std::string& method) {
  local_.insert(method);
}

void MethodTable::SetNode(const std::string& node, const std::vector<std::string>& methods) {
  if (methods.empty()) {
    nodes_.erase(node);
  } else {
    nodes_[node] = std::set<std::string>(methods.begin(), methods.end());
  }
}

void MethodTable::SetHost(const std::vector<std::string>& methods) {
  host_.clear();
  for (const auto& method : methods) {
    if (method.rfind("update.", 0) == 0 || method.rfind("host.", 0) == 0) host_.insert(method);
  }
}

const std::set<std::string>& MethodTable::Of(const std::string& node) const {
  static const std::set<std::string> none;
  const auto known = nodes_.find(node);
  return known == nodes_.end() ? none : known->second;
}

Route MethodTable::Find(const std::string& method) const {
  if (local_.count(method)) return Route::kLocal;
  if (host_.count(method)) return Route::kHost;
  for (const auto& [node, methods] : nodes_) {
    if (methods.count(method)) return Route::kNode;
  }
  return Route::kUnknown;
}

std::vector<std::string> MethodTable::All() const {
  std::set<std::string> all = local_;
  all.insert(host_.begin(), host_.end());
  for (const auto& [node, methods] : nodes_) all.insert(methods.begin(), methods.end());
  return {all.begin(), all.end()};
}

}  // namespace mowbite_bridge
