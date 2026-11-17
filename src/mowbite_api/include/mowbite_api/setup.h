#ifndef MOWBITE_API_SETUP_H
#define MOWBITE_API_SETUP_H

#include <string>
#include <vector>

#include "mowbite_api/catalog.h"

namespace mowbite_api {

// The setup program (setup.*, mowbite_setup): what a step of setup/setup.yaml of a platform can be, the bounds the code
// holds every platform to, and the texts the app shows. x-setup of the schema is made from it.

// A name of the program: what it means, and how the app words it where it has texts.
struct SetupName {
  std::string id;
  std::string text;
  UserTexts texts = {};
};

// A parameter of a check in setup.yaml. A number stays within min and max, the hard bounds of the code; without an
// entry it is def. A text is one of values, a list holds only values (any text with none).
struct SetupParam {
  std::string name;
  Type type = Type::kNumber;
  std::string unit;
  std::string text;
  double min = 0;
  double max = 0;
  double def = 0;
  std::vector<std::string> values = {};
  bool required = false;
};

// A value a check measures, in values[] of its step.
struct SetupMeasure {
  std::string id;
  // Empty for a text.
  std::string unit;
  std::string text;
  UserTexts texts = {};
};

struct SetupCheck {
  std::string name;
  // How the app shows a step of it: confirm, input or measure.
  std::string kind;
  // setup.start begins it. Without, setup.answer ends it (kind input) or the OS once it sees what the step waits for.
  bool start = false;
  // Its ticks come without a tap, as soon as the OS sees what the user did.
  bool automatic = false;
  std::string text;
  std::vector<SetupParam> params = {};
  std::vector<SetupMeasure> values = {};
  // Reasons of SETUP_STEP a step of it can fail with.
  std::vector<std::string> reasons = {};
  // Texts of a step of it; none where the step list has to bring them.
  UserTexts texts = {};
};

// Stages in the order the app shows them; a step list uses those it has steps for.
const std::vector<SetupName>& SetupStages();
const std::vector<SetupCheck>& SetupChecks();
const SetupCheck* FindSetupCheck(const std::string& name);
// The inputs of hw/status a step input can check; texts of a step and of a part of that role.
const std::vector<SetupName>& SetupRoles();
// How a part of a step input is done: press or short.
const std::vector<SetupName>& SetupModes();
// The moves by hand a step imu can ask for.
const std::vector<SetupName>& SetupMoves();
// What a step done_when can wait for; texts of a step of that condition.
const std::vector<SetupName>& SetupConditions();
// Points a step can ask the user to confirm before it starts; a step list may bring its own with texts.
const std::vector<SetupName>& SetupPoints();
// Why a step is skipped; other goes with a text of the user.
const std::vector<SetupName>& SetupSkipReasons();
// Screens of the app a step can point to.
const std::vector<std::string>& SetupOpens();
// Signs the app draws for a step without an image.
const std::vector<std::string>& SetupSymbols();
// The states of a step.
const std::vector<std::string>& SetupStates();
// The emergency a step input waits for, by the names of x-emergency-bits; NONE for an input that stops nothing.
const std::vector<std::string>& SetupEmergencies();

const SetupName* FindSetupName(const std::vector<SetupName>& names, const std::string& id);

// Bounds of a step list and of setup.*.
constexpr int kSetupMaxSteps = 100;
constexpr int kSetupMaxParts = 8;
constexpr int kSetupMaxInputValues = 12;
// An image of a step: SVG of one colour, at most this many bytes; setup.image gives it in pieces of at most as many.
constexpr int kSetupImageBytes = 65536;
// Characters of the text of setup.skip with reason other.
constexpr int kSetupMaxSkipText = 200;
// ids of steps, parts, inputs and images of a step list.
constexpr char kSetupIdPattern[] = "^[a-z][a-z0-9_]{0,31}$";
constexpr char kSetupImagePattern[] = "^[a-z][a-z0-9_-]{0,31}\\.svg$";
// Gravity a board at rest has to read (m/s²).
constexpr double kSetupGravityMin = 8.8;
constexpr double kSetupGravityMax = 10.8;
// setup/json goes out at most this often while a step runs (s).
constexpr double kSetupLivePeriod = 0.5;

// robot_shape of robot.geometry: parts, points of one part and points of all parts at most.
constexpr int kShapeMaxParts = 32;
constexpr int kShapeMaxPartPoints = 200;
constexpr int kShapeMaxPoints = 1000;
// What a part of robot_shape can be; the app draws a kind it does not know like body.
const std::vector<std::string>& ShapeKinds();

// x-setup of the schema.
Json SetupCatalog();
// The state of the program as setup.get and setup/json give it.
Field SetupState();

}  // namespace mowbite_api

#endif  // MOWBITE_API_SETUP_H
