// The step list of a platform (setup/setup.yaml of its package, handed over as JSON): read and held against the
// catalog of the interface (mowbite_api/setup.h), with the texts every step shows. Without ROS.
#ifndef MOWBITE_SETUP_STEPS_H
#define MOWBITE_SETUP_STEPS_H

#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mowbite_setup {

using json = nlohmann::ordered_json;

// An id with its texts by language: {"de": {"name", "help", "fix"}, "en": {...}}.
struct Named {
  std::string id;
  json texts;
};

// A part of a step input (mode, role), of a step inputs_assign (mode assign, role, delay_ms) or of a step imu (move).
struct Part {
  std::string id;
  std::string mode;
  std::string role;
  std::string move;
  json texts;
  // The delay the line of the part gets in the table of the driver.
  int delay_ms = 0;
};

struct Step {
  std::string id;
  std::string stage;
  std::string check;
  std::string kind;
  bool start = false;
  bool automatic = false;
  bool skippable = true;
  std::vector<std::string> needs;
  std::vector<Named> confirm;
  std::vector<Named> skip_reasons;
  // The params of the check: numbers with the default of the catalog where the list has none, texts, and lists.
  std::map<std::string, double> numbers;
  std::map<std::string, std::string> words;
  std::map<std::string, std::vector<std::string>> lists;
  std::vector<Part> parts;
  // Texts of the values of a choice, in the order of its values.
  std::vector<Named> choices;
  std::string open;
  std::string image;
  std::string accent;
  std::string symbol;
  std::vector<std::string> mark;
  json texts;

  double Number(const std::string& name) const {
    return numbers.at(name);
  }
  const std::string& Word(const std::string& name) const {
    return words.at(name);
  }
};

struct StepList {
  // Counter of the list: another one tells the app that steps or images changed.
  int catalog = 0;
  // How the OS restarts for a step that needs it, a name of SetupRestarts(); empty for a list without such a step.
  std::string restart;
  std::vector<Step> steps;

  const Step* Find(const std::string& id) const;
  // Every image and accent the steps name, once.
  std::vector<std::string> Images() const;
};

// The list as {"catalog": N, "restart": way, "steps": [...]}. Nothing with the first problem in why: a field the
// catalog does not know, a value outside of the bounds of the code, a step that needs a later one, a text missing in
// de or en.
std::optional<StepList> ParseSteps(const json& list, std::string* why);

// What is wrong with an image of a step, empty for none: SVG of one colour without text, raster images, scripts,
// styles or references, of at most kSetupImageBytes.
std::string CheckImage(const std::string& svg);
// Reads the image of this name in the folder; what is wrong with it or with reading it, empty for none.
std::string LoadImage(const std::string& folder, const std::string& name, std::string* bytes);

// The emergency bit of hw/status a name of SetupEmergencies() stands for, 0 for NONE.
unsigned EmergencyBit(const std::string& name);

}  // namespace mowbite_setup

#endif  // MOWBITE_SETUP_STEPS_H
