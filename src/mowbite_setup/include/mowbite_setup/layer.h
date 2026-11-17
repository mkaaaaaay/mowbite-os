// The parameter file the setup program writes for this mower, <data_dir>/<platform>.setup.yaml: the start reads it
// after the files of the platform and before the file kept by hand, which has the last word. Only the program writes
// it, in one fixed form, so it reads it back without a YAML reader. Without ROS.
#ifndef MOWBITE_SETUP_LAYER_H
#define MOWBITE_SETUP_LAYER_H

#include <map>
#include <nlohmann/json.hpp>
#include <string>

namespace mowbite_setup {

class Layer {
 public:
  using json = nlohmann::ordered_json;

  // The file as Text() wrote it. False with why for anything else; the layer is empty then.
  bool Parse(const std::string& text, std::string* why);
  // The file; empty without a value.
  std::string Text() const;
  // A parameter of a node: a boolean, a number, a text, or a list of one of these kinds that is not empty (a
  // parameter file cannot tell the type of an empty one). False for another value or a name the file cannot hold.
  bool Set(const std::string& node, const std::string& key, const json& value);
  void Erase(const std::string& node, const std::string& key);
  // null without it.
  json Get(const std::string& node, const std::string& key) const;
  // Whether the file can hold values for a node of this name.
  static bool IsNode(const std::string& name);

 private:
  std::map<std::string, std::map<std::string, json>> values_;
};

}  // namespace mowbite_setup

#endif  // MOWBITE_SETUP_LAYER_H
