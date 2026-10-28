// The parameters of params/json, collected from the nodes. The app reads them as {"/node/name": value}.
#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace mowbite_bridge {

using json = nlohmann::ordered_json;

class ParamsStore {
 public:
  // Takes the parameters of the base object, keys already in full form.
  explicit ParamsStore(const json& base = json::object());

  // Message of bridge/params: {"node": "/xbot_positioning", "params": {"antenna_offset_x": 0.1}}. Every parameter
  // is stored as "<node>/<name>"; values of names containing "password" become null. Returns true if the payload
  // changed; a message of another shape changes nothing.
  bool Update(const std::string& message);

  const std::string& Payload() const {
    return payload_;
  }

 private:
  json params_ = json::object();
  std::string payload_;
};

}  // namespace mowbite_bridge
