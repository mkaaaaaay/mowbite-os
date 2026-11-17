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

// An input a step input can check, named as its flag in hw/status; texts of a step and of a part of that role.
struct SetupRole : SetupName {
  // Roles of one group are sensors of one kind: the sides of a bumper.
  std::string group;
  // The emergency a press brings unless the step names another, a name of SetupEmergencies().
  std::string emergency;
};

// What a step done_when can wait for; texts of a step of that condition.
struct SetupCondition : SetupName {
  // The screen of the app a step of it opens unless the step names another, a name of SetupOpens().
  std::string open;
};

// A parameter of a check in setup.yaml. A number stays within min and max, the hard bounds of the code; without an
// entry it is def. A text is one of values and word without an entry, a list holds only values (any text with none).
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
  std::string word = {};
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
  // How the app shows a step of it: confirm, input, measure or drive (the program moves the wheels).
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
  // The screen of the app a step of it opens unless the step names another; empty for none.
  std::string open = {};
  // A safety check: a step of it that fails with a reason that closes (closes of the reason) closes the lock of
  // autonomous driving and mowing until it passes again. Not to be chosen by the step list.
  bool safety = false;
};

// Stages in the order the app shows them; a step list uses those it has steps for.
const std::vector<SetupName>& SetupStages();
const std::vector<SetupCheck>& SetupChecks();
const SetupCheck* FindSetupCheck(const std::string& name);
const std::vector<SetupRole>& SetupRoles();
// How a part is done: press or short in a step input, assign in a step inputs_assign.
const std::vector<SetupName>& SetupModes();
// The moves by hand a step imu can ask for.
const std::vector<SetupName>& SetupMoves();
const std::vector<SetupCondition>& SetupConditions();
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
// The question of a step inputs_assign in inputs[] of the step, and the values setup.answer takes for it.
const SetupName& SetupAccept();
const std::vector<SetupName>& SetupAcceptValues();
// The delay of the line of a part, in inputs[] of a step inputs_assign under this prefix and the id of the part:
// setup.answer takes it once the step is ok, a whole number of ms from 0 to delay_ms of the part.
const SetupName& SetupDelay();
constexpr char kSetupDelayPrefix[] = "delay_";
// How the OS restarts for a step that needs it: restart at the top of a step list.
const std::vector<SetupName>& SetupRestarts();
// The parts of a step wheels, always all of them in this order.
const std::vector<SetupName>& SetupWheelParts();
// The question of the part one_wheel of a step wheels in inputs[] of the step, and the values setup.answer takes.
const SetupName& SetupWheelQuestion();
const std::vector<SetupName>& SetupWheelValues();
// The question of a safety step whose part saw nothing of the action within its time, in inputs[] of the step, and
// the values setup.answer takes: yes ends the step failed with NOT_SEEN (it closes), no or no answer within
// kSetupPressedTime ends it open with NOT_DONE.
const SetupName& SetupPressed();
const std::vector<SetupName>& SetupPressedValues();
constexpr double kSetupPressedTime = 30.0;
// Whether a step of this check that fails with this reason of SETUP_STEP closes the lock.
bool SetupClosesLock(const std::string& check, const std::string& reason);
// Most causes setup.get names in safety.closed.
constexpr int kSetupMaxClosed = 32;

template <typename Name>
const Name* FindSetupName(const std::vector<Name>& names, const std::string& id) {
  for (const auto& name : names) {
    if (name.id == id) return &name;
  }
  return nullptr;
}

// The parameters a driver that learns its inputs takes, under the node a step inputs_assign names: started with
// learn it watches its candidate lines; the others are its table, one entry per line.
constexpr char kInputsLearn[] = "inputs.learn";
constexpr char kInputsLines[] = "inputs.gpio_lines";
constexpr char kInputsActiveHigh[] = "inputs.gpio_active_high";
constexpr char kInputsReasons[] = "inputs.gpio_reasons";
constexpr char kInputsDelays[] = "inputs.gpio_delays_ms";
constexpr char kInputsRoles[] = "inputs.gpio_roles";
// The longest delay of a line the board takes (ms).
constexpr int kInputsMaxDelayMs = 65535;

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
// A step that moves the wheels of a mower jacked up: the most the program asks for (m/s of the body and of each
// wheel, rad/s), and the longest single move (s). mowbite_safety holds its own bounds on top (kSetupMaxLinear).
constexpr double kSetupRaisedSpeed = 0.1;
constexpr double kSetupRaisedTurn = 0.4;
constexpr double kSetupMoveTime = 10.0;
// Such a step stops when setup.keep stays away this long (s).
constexpr double kSetupKeepTimeout = 1.0;

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
