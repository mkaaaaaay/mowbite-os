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

Route MethodTable::Find(const std::string& method) const {
  if (local_.count(method)) return Route::kLocal;
  for (const auto& [node, methods] : nodes_) {
    if (methods.count(method)) return Route::kNode;
  }
  return Route::kUnknown;
}

std::vector<std::string> MethodTable::All() const {
  std::set<std::string> all = local_;
  for (const auto& [node, methods] : nodes_) all.insert(methods.begin(), methods.end());
  return {all.begin(), all.end()};
}

}  // namespace mowbite_bridge
