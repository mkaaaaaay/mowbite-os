#ifndef MOWBITE_API_CATALOG_H
#define MOWBITE_API_CATALOG_H

#include <iterator>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_api {

using Json = nlohmann::ordered_json;

// The version of the app interface: one up for every change of the catalog, and only by adding (test_schema.cpp).
constexpr int kApi = 70;

// The break of the app interface: one up for a change that is not only adding (what counts: x-api-break in the
// schema); an app goes with the OS of the same break whose api is at least the one it needs.
constexpr int kApiBreak = 1;

// Who owns the names of the interface file; readers keep a copy of it.
constexpr char kOwner[] = "mowbite-os";

// The JSON type of a field; kNumber takes whole numbers as well, kInteger also 2.0, kAny every value.
enum class Type { kString, kNumber, kInteger, kBoolean, kObject, kArray, kAny };

// What the app shows of a reason, an emergency bit or a value of a field in one language: its name, what happened,
// what to do.
struct UserText {
  std::string name;
  std::string help;
  std::string fix;
};

// Texts by language (de, en); none where the app has its own.
using UserTexts = std::map<std::string, UserText>;
// {language: {name, help, fix}}, as x-reasons and the setup program carry texts.
Json Texts(const UserTexts& texts);

struct Field {
  std::string name;
  Type type = Type::kString;
  // Unit of a measured value (m, s, deg, V, %, unix s, ...), empty for none.
  std::string unit;
  bool required = false;
  // One English sentence.
  std::string text;
  // The only values a text may have; empty for any text.
  std::vector<std::string> values;
  // null is allowed as well.
  bool nullable = false;
  // The fields of an object, none for an object of any fields.
  std::vector<Field> fields;
  // ECMA regular expression a text has to match, empty for any.
  std::string pattern;
  // The type of the items of an array without fields; with fields they are objects of these fields.
  std::optional<Type> items = std::nullopt;
  // Bounds of a number, inclusive.
  std::optional<double> min = std::nullopt;
  std::optional<double> max = std::nullopt;
  // Texts of the values of an enum field, by value.
  std::map<std::string, UserTexts> value_texts = {};
  // The entry of $defs that is this value: the schema points there and does not write the fields out again.
  std::string defined = {};
  // An object (or the objects of a list) with the fields of its reason in x-reasons beside these.
  bool open = false;
};

// A reason as events and refusals carry it: reason, message, severity, resume and its own fields.
struct ReasonEntry {
  std::string code;
  // info, warning or error.
  std::string severity;
  // auto (it goes on by itself) or user (it waits for the user).
  std::string resume;
  std::string text;
  std::vector<Field> fields;
  // Only for details with these field values (RECORDING_REFUSED: action, UNEXPECTED_RESTART: hold), else for all; the
  // first match counts.
  Json when = Json::object();
  UserTexts texts = {};
  // SETUP_STEP: a safety step that fails with it closes the lock of autonomous driving and mowing.
  bool closes = false;
  // kCause or kSymptom, set by the catalog for every reason.
  std::string cls = {};
};

struct EventEntry {
  std::string type;
  // logic, map, ntrip, camera, bridge, stats, setup or obstacles; logic events carry x, y, job_id and session_id as
  // well.
  std::string emitter;
  std::string text;
  // The fields beyond the envelope (id, t, type) and those of the emitter.
  std::vector<Field> fields;
  // The reasons this event may carry; empty for events without a reason.
  std::vector<ReasonEntry> reasons;
  // severity and resume of a reason that is not in the list (kept from the table before the catalog).
  std::string other_severity = "warning";
  std::string other_resume = "user";
};

struct EmergencyBit {
  int bit = 0;
  std::string name;
  std::string text;
  UserTexts texts = {};
  // kCause or kSymptom, set by the catalog for every bit.
  std::string cls = {};
};

// An action of actions/json that does what a command does with these params.
struct CommandAction {
  CommandAction(std::string action_in, Json params_in = Json::object(), std::string variant_in = "")
      : action(std::move(action_in)), params(std::move(params_in)), variant(std::move(variant_in)) {}
  // The action id of OpenMower.
  std::string action;
  Json params;
  // Tells the actions of one command apart in their own ids, empty where it has one.
  std::string variant;
};

// An RPC method of the interface; actions are the actions it stands for, none for a method without one.
struct Command {
  std::string name;
  std::string text;
  std::vector<Field> params;
  std::vector<Field> result;
  // The event type whose reasons a refusal (-32600 with data) carries, empty when it is never refused. Reasons of
  // COMMAND_REFUSED (Refusals) may come with any command.
  std::string refused;
  std::vector<CommandAction> actions;
  // When rpc.methods lists it, empty for always.
  std::string offered = {};
};

// Another RPC method of the OS: what it takes and gives, and which node answers it.
struct Method {
  std::string name;
  // mowbite_bridge, mowbite_logic, mowbite_map, mowbite_ntrip, mowbite_calibration, mowbite_light, mowbite_sound,
  // mowbite_camera, mowbite_schedule, mowbite_stats, mowbite_net, mowbite_setup, mowbite_obstacles or mowbite_host.
  std::string node;
  std::string text;
  // The params as one value, mostly an object of fields.
  Field params;
  Field result;
  // When rpc.methods lists it, empty for always.
  std::string offered;
};

// A sensor of sensor_infos/json and sensors.list: its own id (the key there), name and unit; ranges depend on the
// platform.
struct SensorEntry {
  std::string id;
  std::string name;
  // Empty for a text or a number without a unit.
  std::string unit;
  bool text = false;
  // What sends it: logic, gnss (the GNSS driver of the platform) or ntrip.
  std::string source;
  std::string description;
  // Id and name the bridge shows with openmower_names; empty where the app of OpenMower knows no such sensor.
  std::string openmower_id = "";
  std::string openmower_name = "";
};

// A setting of the logic for params.set; a value outside the range is set to the bound.
struct SettingEntry {
  std::string name;
  bool integer = false;
  double min = 0;
  double max = 0;
  double def = 0;
  std::string unit;
  std::string text;
  // Names of the whole values from 0 on, for a setting that picks one of them; empty for a number.
  std::vector<std::string> values = {};
};

// A key of robot.geometry.get and .set; a number outside the range is refused.
struct GeometryEntry {
  std::string name;
  // A list of points (robot_outline, dock_footprint), else a number.
  bool points = false;
  // The user can set it; wheel_radius and dock_contact_x are the platform's alone.
  bool settable = true;
  // The range of a number in its unit; with relative, around the platform's value.
  double min = 0;
  double max = 0;
  std::string unit = "m";
  bool relative = false;
  std::string text;
  // robot_shape: a list of parts, the platform's alone.
  bool shape = false;
  // The calibration drive measures it and calibration.apply takes it.
  bool calibrated = false;
};

// A situation the OS may play a sound for (sound.*), names as in open_mower_ros #347.
struct SoundSituation {
  std::string name;
  // What makes the OS play it.
  std::string event;
};

// The fields of robot_state/json.
const std::vector<Field>& RobotState();
// Every event the OS sends on events/json; types not in it may come from the planner and carry the envelope.
const std::vector<EventEntry>& Events();
// id, t and type of every event.
const std::vector<Field>& Envelope();
// x, y, job_id and session_id of the events of the logic.
const std::vector<Field>& LogicFields();
const std::vector<EmergencyBit>& EmergencyBits();
const std::vector<Command>& Commands();
// Groups of reasons only answers carry, never an event: COMMAND_REFUSED, REQUEST_REFUSED, SOUND_REFUSED,
// CAMERA_REFUSED, CAMERA_UNLOCK_REFUSED and those of mowbite-host.
const std::vector<EventEntry>& Refusals();
// The RPC methods of the OS beyond Commands(); planner.* are the planner's, sim.* only in the simulation.
const std::vector<Method>& Methods();
// The node that answers a command or method, empty for a name the catalog does not know.
std::string NodeOf(const std::string& method);
const std::vector<SensorEntry>& Sensors();
// By its own id.
const SensorEntry* FindSensor(const std::string& id);
// By its id with openmower_names; null for an own id.
const SensorEntry* FindOpenMowerSensor(const std::string& openmower_id);
// The settings of the logic in the order of the app's groups.
const std::vector<SettingEntry>& LogicSettings();
// A value of the setting of that name as params.set takes it: rounded for a whole number, then within min..max;
// none for an unknown name.
std::optional<double> FitSetting(const std::string& name, double value);
// The keys of the geometry in the order of the answers; robot_outline is the only list the user sets.
const std::vector<GeometryEntry>& GeometryKeys();
// The keys of GeometryKeys() with calibrated.
const std::vector<std::string>& CalibrationKeys();
// calibration_applied of the geometry params notes a calibration.apply of result (its id).
bool AppliedTo(const Json& applied, const Json& result);
// The calibration counts as taken (setup condition calibration): result (calibration.result) is done,
// calibration_applied of the geometry params names it, and every key of CalibrationKeys() still has the value it had
// then.
bool CalibrationTaken(const Json& result, const Json& geometry_params);
// A word of schedule.log: what it means, what its detail and more hold (empty: null).
struct ScheduleLogWord {
  std::string word;
  std::string text;
  std::string detail;
  std::string more;
  bool ready = false;  // also a reason of mowbite/schedule/ready
  bool push = false;   // a start of a plan fell through: schedule_skipped of notify.* tells it
};
const std::vector<ScheduleLogWord>& ScheduleLogWords();
// An event notify.* can send a push for: problem as the app shows it, picked until notify.set picks others.
struct NotifyEvent {
  std::string key;
  bool problem = false;
  bool picked = false;
};
const std::vector<NotifyEvent>& NotifyEvents();
// Bounds of notify.* and stats.*, for the catalog and the checks of the nodes.
constexpr int kNotifyMaxRemind = 1440;
constexpr int kNotifyMaxEmergencyWait = 600;
constexpr int kNotifyMaxSnooze = 86400;
constexpr int kNotifyMaxLog = 50;
constexpr int kNotifyMaxName = 40;
constexpr int kNotifyMaxPushes = 10;
constexpr char kNotifyTopicPattern[] = "^[A-Za-z0-9_-]{0,64}$";
constexpr int kStatsMaxDueHours = 10000;
constexpr int kStatsMaxAnswerHours = 9000;
// stats.stops: ISO weeks of counts and the last stops kept; what the mower did when it stopped, IDLE only for a mark.
constexpr int kStopBookWeeks = 8;
constexpr int kStopBookLast = 200;
inline const std::vector<std::string> kStopActivities = {"MOWING",  "UNDOCKING",   "DOCKING",
                                                         "DRIVING", "CALIBRATING", "IDLE"};
// The most bytes of the answer of sensors.history, position.history and events.history (MQTT ends at 2 MB).
constexpr int kHistoryMaxAnswerBytes = 900000;
// sensors.history: hours kept and the steps (s) from small to large.
constexpr int kSensorsHistoryHours = 48;
constexpr int kSensorsHistorySteps[] = {60, 300, 900};
constexpr int kSensorsHistoryMaxStep = kSensorsHistorySteps[std::size(kSensorsHistorySteps) - 1];
// incidents.*: the most bytes of a timeline in incidents.get, of a piece of incidents.download (raw, before base64) and
// characters of the note of incidents.mark.
constexpr int kIncidentTimelineBytes = 65536;
constexpr int kIncidentChunkBytes = 262144;
constexpr int kIncidentMaxNote = 500;
// vision.obstacles.image: the most bytes of a piece (raw, before base64).
constexpr int kObstacleImageBytes = 65536;
// vision.obstacles.*: the values of obstacle_mode by number, the kinds of an obstacle and what the mower did, the
// latter from the least to the most.
const std::vector<std::string>& ObstacleModes();
const std::vector<std::string>& ObstacleKinds();
const std::vector<std::string>& ObstacleActions();
// The highest obstacle_mode built: avoid, driving around objects.
constexpr int kObstacleModeBuilt = 3;
// obstacle_mode as it acts: the setting, at most mode_max of the platform and what is built, at least off.
int EffectiveObstacleMode(int setting, int mode_max);
// A person or animal (kind of ObstacleKinds()).
bool ObstacleLiving(const std::string& kind);
// The place of an action in ObstacleActions(), -1 for none.
int ObstacleActionRank(const std::string& action);
// The situations of sound.list in their order.
const std::vector<SoundSituation>& SoundSituations();

// The event of this type, nullptr for a type the catalog does not know.
const EventEntry* FindEvent(const std::string& type);
// The event or the group of answers of this type, nullptr for neither.
const EventEntry* FindReasonGroup(const std::string& type);
const Command* FindCommand(const std::string& name);
const Method* FindMethod(const std::string& name);
// The node of the settings in params/json and params.set, and the one the app of OpenMower knows them under.
constexpr char kSettingsNode[] = "/logic";
constexpr char kOpenMowerSettingsNode[] = "/mower_logic";

// The own id of an action: the name of its command, "<name>:<variant>" where the command has several.
std::string ActionId(const Command& command, const CommandAction& action);
// The command an own action id stands for, with the params it means; nullptr for an id without one.
const Command* CommandOfAction(const std::string& id, Json* params);
// The own id of an action id of OpenMower and back; empty for an id without a command.
std::string OwnActionId(const std::string& openmower_action);
std::string OpenMowerActionId(const std::string& id);

// The text of the client field the command RPCs take: [a-z][a-z0-9_-]{0,15}.
constexpr char kClientPattern[] = "^[a-z][a-z0-9_-]{0,15}$";

struct Rating {
  const char* severity;
  const char* resume;
};

// severity and resume of a reason of an event or of a group of answers; details only where a rating depends on a
// field (when of the reason).
Rating RateReason(const std::string& event, const std::string& code, const Json& details = Json::object());
// Whether the entry rates this code with these details.
bool ReasonFits(const ReasonEntry& reason, const std::string& code, const Json& details);
// The entry of the event or group that rates this code with these details, nullptr for none.
const ReasonEntry* FindReason(const std::string& event, const std::string& code, const Json& details = Json::object());

// A reason or bit of class cause names a state of the world, the mower, the data or an act (weather, battery, map,
// RTK quality, setup, the user, a broken precondition) that explains its event without another log. One of class
// symptom only names the outcome or a part that is silent or missing (timeout, abort, invalid, a missing node, a
// crash) and needs another event or an incident. The class only feeds the stop book and its incidents.
constexpr char kCause[] = "cause";
constexpr char kSymptom[] = "symptom";

// The class of a reason of an event or a group of answers; kSymptom for one the catalog does not know.
const std::string& ReasonClass(const std::string& event, const std::string& code);
// The bit that tells most of an emergency: its first set bit of class cause, else its first set one but LATCH, which
// every latched emergency has; nullptr without a named bit set.
const EmergencyBit* EmergencyReasonBit(int bits);

// The name EmergencyNames gives all set bits without a name together.
constexpr char kUnknownEmergency[] = "UNKNOWN";

// The names of the set emergency bits in bit order, "LATCH, LIFT"; set bits without a name add one UNKNOWN at the end.
std::string EmergencyNames(int bits);
// The same names as a list.
std::vector<std::string> EmergencyNameList(int bits);

// The JSON Schema 2020-12 of the interface with the catalog in x-reasons, x-emergency-bits, x-commands, x-methods,
// x-sensors, x-settings, x-geometry and x-setup; texts of reasons and bits by language; every object sorted by key.
const Json& Schema();

// What is wrong with an event against the catalog: an unknown type (strict only), a missing or unknown field, a
// wrong type or value, a reason the event does not know, severity or resume other than the catalog's. details may
// hold the envelope; empty when all is well.
std::vector<std::string> CheckEvent(const std::string& type, const Json& details, bool strict = true);

// What is wrong with the params of a command: not an object, a field it does not take, a missing one, a wrong type,
// value or text. Empty when all is well; field names the first field at fault.
std::string CheckParams(const Command& command, const Json& params, std::string* field);
// The same for another method of the OS, by the fields of its params; open params let other keys through.
std::string CheckParams(const Method& method, const Json& params, std::string* field);

// What in the newer schema takes away from the older one: a definition, field, type, value, reason, emergency bit or
// command gone, a field of robot_state or an event no longer always there, a parameter newly required.
std::vector<std::string> CheckAdditive(const Json& older, const Json& newer);

// The break of a schema: its x-api-break, 1 for a schema from before it (api 30 and older).
int ApiBreakOf(const Json& schema);

}  // namespace mowbite_api

#endif  // MOWBITE_API_CATALOG_H
