#ifndef MOWBITE_API_CATALOG_H
#define MOWBITE_API_CATALOG_H

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace mowbite_api {

using Json = nlohmann::ordered_json;

// The version of the app interface: one up for every change of the catalog, and only by adding (test_schema.cpp).
constexpr int kApi = 4;

// The JSON type of a field; kNumber takes whole numbers as well, kInteger also 2.0.
enum class Type { kString, kNumber, kInteger, kBoolean, kObject, kArray };

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
  // Only with this action field (RECORDING_REFUSED), else for every action; the first match counts.
  std::string action;
};

struct EventEntry {
  std::string type;
  // logic, map, ntrip or bridge; logic events carry x, y, job_id and session_id as well.
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
};

// An action of actions/json that does what a command does with these params.
struct CommandAction {
  CommandAction(std::string action_in, Json params_in = Json::object())
      : action(std::move(action_in)), params(std::move(params_in)) {}
  std::string action;
  Json params;
};

// An RPC method of the interface; actions are the action ids it stands for, none for a method without one.
struct Command {
  std::string name;
  std::string text;
  std::vector<Field> params;
  std::vector<Field> result;
  // The event type whose reasons a refusal (-32600 with data) carries, empty when it is never refused. Reasons of
  // COMMAND_REFUSED (Refusals) may come with any command.
  std::string refused;
  std::vector<CommandAction> actions;
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
// Groups of reasons only answers carry, never an event: COMMAND_REFUSED and REQUEST_REFUSED.
const std::vector<EventEntry>& Refusals();

// The event of this type, nullptr for a type the catalog does not know.
const EventEntry* FindEvent(const std::string& type);
// The event or the group of answers of this type, nullptr for neither.
const EventEntry* FindReasonGroup(const std::string& type);
const Command* FindCommand(const std::string& name);
// The command an action stands for, with the params it means; nullptr for an action without one.
const Command* CommandOfAction(const std::string& action, Json* params);

// The text of the client field the command RPCs take: [a-z][a-z0-9_-]{0,15}.
constexpr char kClientPattern[] = "^[a-z][a-z0-9_-]{0,15}$";

struct Rating {
  const char* severity;
  const char* resume;
};

// severity and resume of a reason of an event or of a group of answers; action only for RECORDING_REFUSED.
Rating RateReason(const std::string& event, const std::string& code, const std::string& action = "");

// The name EmergencyNames gives all set bits without a name together.
constexpr char kUnknownEmergency[] = "UNKNOWN";

// The names of the set emergency bits in bit order, "LATCH, LIFT"; set bits without a name add one UNKNOWN at the end.
std::string EmergencyNames(int bits);

// The JSON Schema 2020-12 of the interface with the catalog in x-reasons, x-emergency-bits and x-commands.
const Json& Schema();

// What is wrong with an event against the catalog: an unknown type (strict only), a missing or unknown field, a
// wrong type or value, a reason the event does not know, severity or resume other than the catalog's. details may
// hold the envelope; empty when all is well.
std::vector<std::string> CheckEvent(const std::string& type, const Json& details, bool strict = true);

// What is wrong with the params of a command: not an object, a field it does not take, a missing one, a wrong type,
// value or text. Empty when all is well; field names the first field at fault.
std::string CheckParams(const Command& command, const Json& params, std::string* field);

// What in the newer schema takes away from the older one: a definition, field, type, value, reason, emergency bit or
// command gone, a field of robot_state or an event no longer always there, a parameter newly required.
std::vector<std::string> CheckAdditive(const Json& older, const Json& newer);

}  // namespace mowbite_api

#endif  // MOWBITE_API_CATALOG_H
