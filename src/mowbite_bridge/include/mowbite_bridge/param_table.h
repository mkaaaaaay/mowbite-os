#ifndef MOWBITE_BRIDGE_PARAM_TABLE_H
#define MOWBITE_BRIDGE_PARAM_TABLE_H

#include <map>
#include <string>
#include <string_view>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// params/json: one flat object "<node>/<name>" over all nodes, names with "password" as null.
class ParamTable {
 public:
  // The object to start from (parameter params_json); text that is not an object counts as {}.
  explicit ParamTable(std::string_view base = "{}");

  // Applies one bridge/params message, replacing the node's list; returns whether params/json changed.
  bool Apply(std::string_view message);

  const std::string& Text() const {
    return text_;
  }

 private:
  void Render();

  nlohmann::json base_;
  std::map<std::string, nlohmann::json> nodes_;
  std::string text_;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_PARAM_TABLE_H
