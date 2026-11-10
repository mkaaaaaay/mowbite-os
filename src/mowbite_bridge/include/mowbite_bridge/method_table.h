#ifndef MOWBITE_BRIDGE_METHOD_TABLE_H
#define MOWBITE_BRIDGE_METHOD_TABLE_H

#include <map>
#include <set>
#include <string>
#include <vector>

namespace mowbite_bridge {

// Where a method is answered: in the bridge, by a node over bridge/rpc/request, by mowbite-host on the broker (the
// bridge stays silent), or nowhere.
enum class Route { kLocal, kNode, kHost, kUnknown };

class MethodTable {
 public:
  void AddLocal(const std::string& method);
  // The methods a node offers, replacing its earlier list; an empty list removes the node.
  void SetNode(const std::string& node, const std::vector<std::string>& methods);
  // The methods mowbite-host answers itself, replacing its earlier list; only update.* and host.* count.
  void SetHost(const std::vector<std::string>& methods);
  // The methods a node offers now, none for a node without a list.
  const std::set<std::string>& Of(const std::string& node) const;
  // The bridge's own methods come first.
  Route Find(const std::string& method) const;
  // Every method once, sorted.
  std::vector<std::string> All() const;

 private:
  std::set<std::string> local_;
  std::map<std::string, std::set<std::string>> nodes_;
  std::set<std::string> host_;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_METHOD_TABLE_H
