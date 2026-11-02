#ifndef MOWBITE_BRIDGE_PARAM_TABLE_H
#define MOWBITE_BRIDGE_PARAM_TABLE_H

#include <map>
#include <string>
#include <string_view>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// params/json: one flat object "<node>/<name>" over all nodes, keys sorted, values as the node sent them, every name
// with "password" in it as null.
class ParamTable {
 public:
  // The object to start from (parameter params_json); text that is not an object counts as {}.
  explicit ParamTable(std::string_view base = "{}");

  // One message of bridge/params, {"node": "/name", "params": {...}}: the node's whole list, replacing the one before.
  // Returns whether params/json changed; a message of another shape changes nothing.
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
