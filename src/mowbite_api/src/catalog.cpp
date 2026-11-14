#include <cstdint>
#include <map>
#include <string>

#include "mowbite_api/catalog.h"

namespace mowbite_api {

namespace {

Field Number(std::string name, std::string unit, std::string text, bool required = true) {
  return {std::move(name), Type::kNumber, std::move(unit), required, std::move(text), {}, false, {}, ""};
}

Field Integer(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kInteger, "", required, std::move(text), {}, false, {}, ""};
}

// An optional whole number with a unit.
Field Count(std::string name, std::string unit, std::string text) {
  return {std::move(name), Type::kInteger, std::move(unit), false, std::move(text), {}, false, {}, ""};
}

Field Text(std::string name, std::string text, bool required = true, std::vector<std::string> values = {}) {
  return {std::move(name), Type::kString, "", required, std::move(text), std::move(values), false, {}, ""};
}

// A number of this unit from min to max.
Field Ranged(std::string name, std::string unit, double min, double max, std::string text, bool required = true) {
  Field field = Number(std::move(name), std::move(unit), std::move(text), required);
  field.min = min;
  field.max = max;
  return field;
}

// A whole number of this unit from min to max.
Field Whole(std::string name, std::string unit, double min, double max, std::string text, bool required = true) {
  Field field = Ranged(std::move(name), std::move(unit), min, max, std::move(text), required);
  field.type = Type::kInteger;
  return field;
}

Field Bool(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kBoolean, "", required, std::move(text), {}, false, {}, ""};
}

Field Client() {
  Field field = Text("client", "Who sent the command (app, ha), for the log and the events of its step", false);
  field.pattern = kClientPattern;
  return field;
}

// state and sub_state as robot_state has them right after the command, then the fields of the command.
std::vector<Field> Answer(std::vector<Field> fields = {}, std::vector<std::string> states = {}) {
  std::vector<Field> out = {
      Text("state", "current_state of robot_state right after the command", true, std::move(states)),
      Text("sub_state", "current_sub_state of robot_state right after the command")};
  for (auto& field : fields) out.push_back(std::move(field));
  return out;
}

// A command of the logic: client and the params of the command, its answer, the event of its refusal and its actions.
Command Logic(std::string name, std::string text, std::vector<Field> params, std::vector<Field> result,
              std::string refused, std::vector<CommandAction> actions) {
  params.insert(params.begin(), Client());
  return {std::move(name), std::move(text), std::move(params), std::move(result), std::move(refused),
          std::move(actions)};
}

const std::string kIdle = "mower_logic:idle/";
const std::string kMow = "mower_logic:mowing/";
const std::string kRecord = "mower_logic:area_recording/";

Field TrimmedPoints() {
  return Integer("trimmed_points", "Points cut off where the ring ran past its start, 0 for none");
}

// A switch as OpenMower sends it in robot_state: 0 or 1.
Field Switch(std::string name, std::string text) {
  return {std::move(name), Type::kInteger, "", true, std::move(text), {}, false, {}, ""};
}

Field Object(std::string name, std::string text, std::vector<Field> fields = {}, bool required = true) {
  return {std::move(name), Type::kObject, "", required, std::move(text), {}, false, std::move(fields), ""};
}

// A list of values of one type.
Field List(std::string name, std::string text, Type items, bool required = true) {
  Field field = {std::move(name), Type::kArray, "", required, std::move(text), {}, false, {}, ""};
  field.items = items;
  return field;
}

// A list of objects of these fields.
Field Rows(std::string name, std::string text, std::vector<Field> fields, bool required = true) {
  return {std::move(name), Type::kArray, "", required, std::move(text), {}, false, std::move(fields), ""};
}

Field Nullable(Field field) {
  field.nullable = true;
  return field;
}

Field NoParams() {
  return Object("params", "None", {}, false);
}

// The answer of ntrip.get, .set and .delete.
Field NtripAccess() {
  return Object("result", "The stored access to the caster; of user and password only whether they are set",
                {Bool("configured", "Whether an access is stored"),
                 Text("host", "Host of the caster", false), Integer("port", "Port of the caster", false),
                 Text("mountpoint", "Mountpoint at the caster", false),
                 Integer("version", "NTRIP version, 1 or 2", false), Bool("has_user", "A user is stored", false),
                 Bool("has_password", "A password is stored", false),
                 Text("error", "Why a stored file cannot be used", false)});
}

Field Deleted() {
  return Object("result", "How many were deleted", {Integer("deleted", "Number of files deleted")});
}

Field GeometryAnswer() {
  return Object("result", "The geometry now",
                {Object("settings",
                        "By key of x-geometry: value, default (the platform's), stored (entered by the user), type "
                        "(number or points), unit, min and max of a number, settable"),
                 Integer("version", "Goes up with every change; robot.geometry.set needs the one read last"),
                 Text("platform", "The platform whose file gives the defaults")});
}

// An effect of the lights, as light.get_profile has its effects and limits.
Field LightEffect(std::string name, std::string text) {
  return Object(std::move(name), std::move(text),
                {Text("effect", "One of the effects of light.get_profile"),
                 List("colors", "1 to limits.colors texts #rrggbb; none for off and rainbow", Type::kString, false),
                 Number("period", "s", "Time of one cycle of the effect; gradient, rainbow and level may be 0", false),
                 Number("brightness", "", "0 to 1 of what the OS allows", false)});
}

std::vector<std::string> SoundSituationNames() {
  std::vector<std::string> names;
  for (const auto& situation : SoundSituations()) names.push_back(situation.name);
  return names;
}

const char kSoundOffered[] = "with a speaker and at least one sound";

// update.* and host.* are answered by mowbite-host on the broker, also while the OS is off.
const char kHostOffered[] = "while mowbite/host/status says online";

const char kCameraOffered[] = "with at least one camera";

Field UnixTime(std::string name, std::string text) {
  return {std::move(name), Type::kInteger, "unix s", true, std::move(text), {}, false, {}, ""};
}

// unix s, or null when it is not known.
Field Time(std::string name, std::string text) {
  return Nullable(UnixTime(std::move(name), std::move(text)));
}

// The schedule of schedule.get; in schedule.set every key optional, each replaced whole.
Field Schedule(bool required) {
  const Field plan = Object(
      "plan", "A start time",
      {List("days", "1 Monday to 7 Sunday, none twice", Type::kInteger), Text("time", "HH:MM local, names the plan"),
       List("areas", "Area ids in order, none for every mowing area", Type::kString, false),
       Text("end", "HH:MM local, home then; not after time: the next day", false),
       Switch("dark", "1: also starts and mows in the dark"), Switch("sunset", "With dark 1: still home at sunset"),
       Switch("fresh", "1: a job of its own, the open one is dropped"),
       Ranged("angle_offset_deg", "deg", -180, 180, "Added to the lane direction of the planner for its job", false)});
  return Object(
      "schedule", "The schedule; its times local after tz",
      {Switch("enabled", "1: the plans start the mower"), Integer("min_battery", "% a plan waits for, 0 to 100"),
       Switch("skip_rain", "1: no start while the rain sensor is wet"),
       Switch("skip_forecast", "1: no start with rain now or in the next forecast_hours"),
       Integer("forecast_hours", "1, 2 or 3"), Switch("skip_cold", "1: no start below min_temp"),
       Integer("min_temp", "deg C: 3, 5, 8 or 10"), Text("tz", "POSIX TZ of the local times"),
       Rows("plans", "At most 32, no two at one time on a day", plan.fields),
       Nullable(Object("pause", "Paused through until, null for none",
                       {Text("until", "YYYY-MM-DD local"),
                        Field{"areas", Type::kAny, "", true, "all, or a list of area ids", {}, false, {}, ""}}))},
      required);
}

Field ScheduleAnswer() {
  return Object(
      "result", "The schedule now",
      {Integer("schema", "Always 1"), Integer("version", "Goes up with every change; schedule.set may name it"),
       Bool("available", "Whether the platform lets the schedule start the mower"),
       Nullable(Text("reason", "platform_off without available, else null", true, {"platform_off"})),
       Bool("clock_ok", "Whether the wall clock is safe for date, weekday and sun"),
       Nullable(Text("clock_reason", "Why it is not safe, null when it is", true,
                     {"never_synced", "clock_jump", "gnss_mismatch"})),
       Time("next_start", "The next start, now while a plan waits or is announced; null without one"),
       Schedule(true),
       Object("sun", "Of the local day after tz at the datum of the map; null without a datum or without one that day",
              {Time("rise", "Sunrise"), Time("set", "Sunset")})});
}

// The settings of the pushes as notify.get gives them; in notify.set every key optional.
std::vector<Field> NotifySettings(bool required) {
  Field topic = Text("topic", "ntfy topic, [A-Za-z0-9_-]{1,64}; empty for none", required);
  topic.pattern = kNotifyTopicPattern;
  std::string keys;
  for (const auto& event : NotifyEvents()) keys += (keys.empty() ? "" : ", ") + event.key;
  const Field events = List("events", "The events that send a push: " + keys, Type::kString, required);
  return {Bool("enabled", "false: nothing is sent, the rest stays", required),
          Text("server", "ntfy server, http(s)://host[:port][/path]", required),
          topic,
          Text("lang", "Language of the texts", required, {"de", "en"}),
          Text("name", "Name of the mower in the texts, at most " + std::to_string(kNotifyMaxName) +
                           " characters; empty: the mower",
               required),
          Text("url", "Address of MowBite for the button in the message; empty for none", required),
          events,
          Whole("remind", "min", 0, kNotifyMaxRemind, "Between reminders of a problem that stays; 0 for none",
                required),
          Whole("emergency_wait", "s", 0, kNotifyMaxEmergencyWait, "How long an emergency lasts before it is told",
                required)};
}

Field NotifyAnswer() {
  std::vector<Field> fields = {Integer("schema", "Always 1")};
  for (auto& field : NotifySettings(true)) fields.push_back(std::move(field));
  fields.push_back(Bool("has_token", "A token for the ntfy server is stored; the token itself never comes back"));
  fields.push_back(List("pushes", "UnifiedPush addresses of the phones, oldest first; past " +
                                      std::to_string(kNotifyMaxPushes) + " the oldest goes",
                        Type::kString));
  fields.push_back(UnixTime("snooze", "Reminders snoozed until then, 0 for not"));
  fields.push_back(Bool("stored", "Saved before: without, nothing is sent"));
  return Object("result", "The settings of the pushes", std::move(fields));
}

std::vector<Field> StatsAmounts() {
  return {Integer("blade_s", "Seconds the blade turned"), Number("mowed_m", "m", "Driven with the blade turning"),
          Number("driven_m", "m", "Driven without it, not in IDLE"),
          Integer("runs", "Left IDLE to undock or to mow")};
}

Field StatsAnswer() {
  std::vector<Field> hour = {UnixTime("t", "Start of the hour, a whole UTC hour")};
  for (auto& field : StatsAmounts()) hour.push_back(std::move(field));
  return Object(
      "result", "The statistics",
      {Integer("schema", "Always 1"), UnixTime("since", "Counting started then"),
       UnixTime("from", "The hours from then on: from of the request, later when the newest " +
                            std::to_string(kStatsMaxAnswerHours) + " are all"),
       Object("total", "Since counting started", StatsAmounts()),
       Rows("hours", "Each hour with something in it from from on, oldest first, the one going on last", hour),
       Rows("parts", "The parts to see to",
            {Text("key", "The part", true, {"blade", "clean"}),
             Number("due_h", "h", "Due again after so many blade hours"),
             Integer("base_blade_s", "total blade_s when it was last seen to"),
             UnixTime("done_at", "Last seen to, 0 not since counting started")})});
}

// The version of a planner or an app; the OS has none besides its revision.
Field Version() {
  return Nullable(Text("version", "Its version, null for os"));
}

Field Revision(std::string name, std::string text) {
  return Nullable(Object(std::move(name), std::move(text),
                         {Text("revision", "The revision, a commit of the component"),
                          Time("date", "When it was built, null when the bundle does not say"), Version()}));
}

// The components the updater keeps.
const std::vector<std::string> kComponents = {"os", "planner", "app", "host"};

Field Component(std::string name = "component") {
  return Text(std::move(name), "The component", true, kComponents);
}

// The automatic update: on or off and its window.
std::vector<Field> Auto(bool required) {
  return {Bool("enabled", "On or off", required), Text("from", "Start of the window, HH:MM local time", required),
          Text("to", "End of the window, HH:MM local time", required)};
}

// reset_jobs of update.start and update.rollback; the planner waits for the end of the mowing job without it.
Field ResetJobs() {
  return Bool("reset_jobs",
              "planner only (else INVALID): switch without waiting for an unfinished mowing job (JOB_OPEN), which "
              "then starts its area anew",
              false);
}

// The params of update.*: app_version in every call, for min_app of a bundle.
Field UpdateParams(std::vector<Field> fields, bool required = false) {
  fields.push_back(Text("app_version", "Version of the app that calls, e.g. 1.4.0 or 1.5.0-beta.2", false));
  return Object("params", "app_version and the fields of the call", std::move(fields), required);
}

// A revision update.status offers, and why it is not installed by itself.
Field Available() {
  return Nullable(Object(
      "available", "The newest revision found that is newer than what runs, null without one",
      {Text("revision", "The revision"), Time("date", "When it was built"), Version(),
       Nullable(Text("min_app", "The oldest app it works with")),
       Nullable(Object("blocked",
                       "Not installed by itself: why, the result of the attempt that ended so and when; update.start "
                       "of this revision tries it anyway",
                       {Text("reason", "A reason of UPDATE in x-reasons, never BLOCKED"),
                        Text("result", "How the attempt ended", false, {"rolled_back", "failed"}),
                        Time("t", "When, null for a reason reckoned from the installed OS (kept nowhere)")}))}));
}

// update.status, the same as retained on mowbite/update/status.
Field UpdateStatus() {
  return Object(
      "result", "What the updater does, what runs and what is available, the last result",
      {Text("phase", "What it does now", true,
            {"idle", "verifying", "unpacking", "waiting_gate", "switching", "checking"}),
       Nullable(Text("component", "The component of the update that runs, null in idle")),
       Nullable(Text("revision", "The revision it installs or goes back to, null in idle")),
       Nullable(Text("reason", "Why the gate waits in waiting_gate: a reason of UPDATE in x-reasons")),
       Rows("components", "Each component the updater keeps",
            {Component("id"), Text("name", "Its name to show"),
             Revision("running", "What runs, null without an installation"), Available(),
             Revision("previous", "Where update.rollback goes, null without one"),
             Time("installed_at", "When what runs was installed"),
             Integer("crashes_since_install", "Ends of the container since then"),
             Bool("auto", "Whether updates install by themselves at night"),
             Text("source", "planner and app: own while their own installation is used, else bundled (the one the OS "
                  "brings along)", false, {"own", "bundled"}),
             Nullable(Object("own_skipped",
                             "planner and app: their own installation the OS does not use, null without one",
                             {Text("revision", "The revision"),
                              Text("reason", "Why: a reason of UPDATE in x-reasons")}, false))}),
       Object("auto", "The automatic update", Auto(true)),
       Nullable(Object(
           "last", "The result of the last update or rollback, null before the first",
           {Text("result", "How it ended", true, {"installed", "rolled_back", "refused", "aborted", "failed"}),
            Nullable(Text("reason", "Why: a reason of UPDATE in x-reasons, null when installed")),
            Text("message", "English text for the log"), Nullable(Text("revision", "The revision")),
            Text("component", "The component"), Integer("t", "When, unix s")}))});
}

// os of mowbite/host/status: how the OS is, from systemd and its files.
std::vector<Field> HostOs() {
  return {Text("state", "How it is", true, {"starting", "running", "stopped", "down", "halted", "updating"}),
          Nullable(Text("reason",
                        "Why: CRASH_LOOP when down, the reason of the halt when halted (HOST_OS in x-reasons); else "
                        "null")),
          Integer("since", "Since when, unix s"), Integer("restarts", "Crashes since its last stable run of 10 min"),
          Time("last_crash", "The last end of its container"), Nullable(Text("revision", "The revision that runs")),
          Nullable(Object("stalled", "The node the watch ended it for at its last end, null without one",
                          {Text("node", "The node"), Number("seconds", "s", "How long it gave no sign"),
                           Number("limit", "s", "Its limit")}))};
}

// The answer of light.set_profile and light.reset.
Field LightProfile() {
  return Object("result", "The profile now", {Object("profile", "An effect by state of light.get_profile")});
}

}  // namespace

const std::vector<Field>& RobotState() {
  static const std::vector<Field> fields = {
      Number("battery_percentage", "", "Charge of the battery from 0 to 1"),
      Number("gps_percentage", "", "Quality of the position from 0 to 1, 0 without a recent pose or heading"),
      Number("current_action_progress", "", "Always 0, kept for the app of OpenMower"),
      Text("current_state", "State of the logic: IDLE, MOWING, DOCKING, UNDOCKING, AREA_RECORDING and the like"),
      Text("current_sub_state", "What the state does right now, empty for nothing in particular"),
      Integer("current_area", "While mowing the index of the area among the mowing areas of the map, otherwise -1"),
      Integer("current_path", "While mowing the index of the path in the plan of the area, otherwise -1"),
      Integer("current_path_index", "While mowing the index of the pose in the path, otherwise -1"),
      Integer("task_index", "While mowing the place of the area in the list of the job from 0, otherwise -1"),
      Integer("task_count", "While mowing the number of areas in the list of the job, otherwise -1"),
      Switch("emergency", "1 while an emergency stops motion and blade"),
      Integer("emergency_reason", "The bits of the emergency (x-emergency-bits), 0 without one"),
      Switch("is_charging", "1 while the mower charges on the docking station"),
      Switch("rain_detected", "1 while the rain sensor reports rain"),
      {"pose",
       Type::kObject,
       "",
       true,
       "The pose of the mower in the map",
       {},
       false,
       {Number("x", "m", "East of the datum of the map"), Number("y", "m", "North of the datum of the map"),
        Number("heading", "rad", "Counter-clockwise from east, -pi to pi"),
        Number("pos_accuracy", "m", "Expected error of the position"),
        Number("heading_accuracy", "rad", "Expected error of the heading"),
        Switch("heading_valid", "1 once the heading is known")},
       ""},
  };
  return fields;
}

const std::vector<Field>& Envelope() {
  static const std::vector<Field> fields = {
      Text("id", "Random id of the event, 32 letters and digits"),
      Number("t", "unix s", "When it happened"),
      Text("type", "What happened; a type this version does not know carries only the envelope for sure"),
  };
  return fields;
}

const std::vector<Field>& LogicFields() {
  static const std::vector<Field> fields = {
      Number("x", "m", "Where the mower was, east of the datum of the map"),
      Number("y", "m", "Where the mower was, north of the datum of the map"),
      Text("job_id", "The job of mowing it belongs to", false),
      Text("session_id", "The session of the logic since its start", false),
      Client(),
      Text("source",
           "schedule while a run of the schedule lasts, from its start until the mower is idle again; an event with "
           "a source of its own names that instead",
           false, {"schedule"}),
  };
  return fields;
}

const std::vector<EmergencyBit>& EmergencyBits() {
  static const std::vector<EmergencyBit> bits = {
      {0, "LATCH", "The emergency holds until it is reset"},
      {1, "TIMEOUT_INPUTS", "The inputs of the board did not report in time"},
      {2, "STOP", "A stop button was pressed"},
      {3, "LIFT", "The mower was lifted"},
      {4, "LIFT_MULTIPLE", "More than one lift sensor triggered"},
      {5, "COLLISION", "A bumper hit something"},
      {6, "TIMEOUT_HIGH_LEVEL", "The board heard nothing from the OS in time"},
      {7, "HIGH_LEVEL", "The OS asked for the emergency"},
      {8, "SERVICE_NOT_READY", "A service of the board was not ready"},
      {9, "COLLISION_MULTIPLE", "More than one bumper triggered"},
      {10, "MOWER_RPM_TIMEOUT", "The blade motor did not report its speed in time"},
      {11, "MOWER_RPM_LIMIT", "The blade motor turned outside its limits"},
      {12, "FIRMWARE_INCOMPATIBLE", "The firmware of the board does not fit the driver"},
      {13, "TILT", "The body tilted beyond its limit"},
      {14, "FENCE", "An autonomous drive took the mower outside of the allowed areas"},
      {15, "STUCK", "The wheels do not move the mower as GNSS shows it"},
      {16, "IMU_TIMEOUT", "No IMU sample for the board, whose tilt detection depends on it"},
      {17, "MOWER_NOT_STOPPED", "The blade still turns after it was switched off"},
      {18, "WHEEL_RUNAWAY", "A wheel turned against the drive command"},
      {19, "FOREIGN_HOST", "Another host talks to the board"},
      {20, "DRIVER_STALL", "The input and output of the driver stalled"},
      {21, "UNEXPECTED_RESTART",
       "The OS started after a kernel crash or after an end without its own stop while the mower worked; it holds "
       "until emergency.reset, also after power off and on",
       {{"de",
         {"Unerwarteter Neustart",
          "MowBite OS wurde während der Arbeit nicht geordnet beendet oder der Kernel ist abgestürzt. Der Mäher "
          "wartet auf die Freigabe, auch nach Aus und Ein.",
          "Mäher und Umgebung ansehen, dann den Notaus zurücksetzen. Das Ereignis UNEXPECTED_RESTART nennt die "
          "Ursache."}},
        {"en",
         {"Unexpected restart",
          "MowBite OS did not end in order while the mower worked, or the kernel crashed. The mower waits for a "
          "release, also after power off and on.",
          "Look at the mower and around it, then reset the emergency. The event UNEXPECTED_RESTART tells the "
          "cause."}}}},
  };
  return bits;
}

const std::vector<Command>& Commands() {
  static const std::vector<Command> commands = {
      {"system.info",
       "Name, version and platform of the OS with the versions of its data and of this interface",
       {},
       {Text("name", "Always mowbite-os"), Text("version", "Version of the OS image"),
        Text("platform", "The platform its driver reports, empty until it did"),
        Integer("schema", "Version of the data in data_dir"),
        Integer("api", "Version of this interface, the x-api of the schema system.api gives"),
        Integer("mqtt", "MQTT version of the bridge at the broker: 5 sends each answer to the response topic of its "
                        "request, 3 sends all to rpc/response"),
        Nullable(Object("stop_release",
                        "Releasing the emergency with STOP at the mower: free for free_s, held for hold_s, let go "
                        "before max_hold_s; null or missing where the platform has no such release",
                        {Number("free_s", "s", "STOP not pressed for this long before"),
                         Number("hold_s", "s", "Held at least this long"),
                         Number("max_hold_s", "s", "Let go before this")},
                        false))},
       "",
       {}},
      {"system.api",
       "This interface as JSON Schema with its version",
       {},
       {Integer("api", "Version of this interface"),
        {"json_schema", Type::kObject, "", true, "This schema", {}, false, {}, ""}},
       "",
       {}},
      Logic("emergency.reset",
            "Asks the safety node to release the emergency and answers with its answer, after at most 3 s",
            {}, Answer({Bool("released", "Always true: released; a reason the board still holds keeps it")}),
            "EMERGENCY_RESET_REFUSED", {{"mower_logic/reset_emergency"}}),
      Logic("mowing.start", "Starts or continues the job of mowing from IDLE",
            {Rows("tasks",
                  "The areas of the job in this order; an inactive one is passed over. The list of the open job "
                  "continues it, another list starts a new one: a caller always sends the whole list of its plan, a "
                  "shorter one is another job. Without it every active mowing area. Unknown, twice or empty: -32602 "
                  "with field tasks",
                  {Text("area_id", "Id of a mowing area of the map")}, false),
             Bool("restart", "True starts a new job even with the list of the open one", false),
             {"areas", Type::kArray, "", false,
              "Not taken (open_mower_ros #359): -32602 with data field areas, never all areas instead; see tasks",
              {}, false, {}, ""}},
            Answer({Bool("check", "True when the reference check on the docking station comes first; its refusal "
                                  "comes as MOWING_REFUSED event")}),
            "MOWING_REFUSED", {{kIdle + "start_mowing"}}),
      Logic("mowing.reset_job", "Drops the unfinished job in IDLE", {},
            Answer({Text("job_id", "Id of the dropped job")}), "COMMAND_REFUSED", {{kIdle + "reset_job"}}),
      Logic("mowing.pause", "Pauses mowing", {}, Answer(), "COMMAND_REFUSED", {{kMow + "pause"}}),
      Logic("mowing.continue", "Continues mowing after a pause that waits for the user", {}, Answer(),
            "MOWING_REFUSED", {{kMow + "continue"}}),
      Logic("mowing.abort", "Ends mowing; the mower goes home where it docks by itself", {}, Answer(),
            "COMMAND_REFUSED", {{kMow + "abort_mowing"}}),
      Logic("mowing.skip_area", "Skips the area being mowed",
            {Integer("area", "current_area of robot_state the client saw; another one now gives CHANGED", false)},
            Answer({Integer("area", "The area skipped, current_area before")}), "COMMAND_REFUSED",
            {{kMow + "skip_area"}}),
      Logic("mowing.skip_path", "Skips the path being mowed",
            {Integer("path", "current_path of robot_state the client saw; another one now gives CHANGED", false)},
            Answer({Integer("path", "The path skipped, current_path before")}), "COMMAND_REFUSED",
            {{kMow + "skip_path"}}),
      Logic("undocking.abort", "Stops leaving the docking station", {}, Answer(), "COMMAND_REFUSED",
            {{"mower_logic:undocking/abort_undocking"}}),
      Logic("docking.abort", "Stops docking", {}, Answer(), "COMMAND_REFUSED",
            {{"mower_logic:docking/abort_docking"}}),
      Logic("docking.start",
            "Drives home from IDLE to the docking station where the OS docks by itself; offered only there", {},
            Answer({}, {"DOCKING"}), "DRIVE_REFUSED", {}),
      Logic("recording.start", "Enters the recording of areas from IDLE", {}, Answer(), "RECORDING_REFUSED",
            {{kIdle + "start_area_recording"}}),
      Logic("recording.exit", "Leaves the recording; what was not saved is gone", {}, Answer(), "COMMAND_REFUSED",
            {{kRecord + "exit_recording_mode"}}),
      Logic("recording.ring_start", "Starts recording a ring: the outline, after it an obstacle", {}, Answer(),
            "RECORDING_REFUSED", {{kRecord + "start_recording"}}),
      Logic("recording.ring_stop",
            "Closes the ring being recorded; one without points just ends, one whose end ran past its start is cut "
            "at the crossing",
            {}, Answer({TrimmedPoints()}), "RECORDING_REFUSED", {{kRecord + "stop_recording"}}),
      Logic("recording.save",
            "Saves the outline and its obstacles as an area, closing an open ring first; AREA_RECORDED follows",
            {Text("type", "mow for a mowing area, nav for a navigation area", true, {"mow", "nav"})},
            Answer({TrimmedPoints()}), "RECORDING_REFUSED",
            {{kRecord + "finish_mowing_area", {{"type", "mow"}}},
             {kRecord + "finish_navigation_area", {{"type", "nav"}}}}),
      Logic("recording.discard", "Drops what was recorded and not saved", {}, Answer(), "RECORDING_REFUSED",
            {{kRecord + "finish_discard"}}),
      Logic("recording.dock",
            "Records a point of the docking station: first in front of it, then on the contacts; DOCK_RECORDED follows",
            {}, Answer(), "RECORDING_REFUSED", {{kRecord + "record_dock"}}),
      Logic("recording.auto_points", "Switches the automatic points of the ring on or off",
            {Bool("enabled", "True: a point every few centimetres by itself; false: only recording.point")}, Answer(),
            "RECORDING_REFUSED",
            {{kRecord + "auto_point_collecting_enable", {{"enabled", true}}},
             {kRecord + "auto_point_collecting_disable", {{"enabled", false}}}}),
      Logic("recording.point", "Takes the next position as a point of the ring", {}, Answer(), "RECORDING_REFUSED",
            {{kRecord + "collect_point"}}),
      Logic("recording.undo",
            "Takes points back from the end of the open ring and switches the automatic points off; the mower stays",
            {Ranged("meters", "m", 0.1, 50, "Path to take back, default 1", false)},
            Answer({Integer("points", "Points left in the open ring")}), "RECORDING_REFUSED",
            {{kRecord + "undo_points"}}),
  };
  return commands;
}

const std::vector<Method>& Methods() {
  static const std::vector<Method> methods = {
      {"rpc.ping", "mowbite_bridge", "Answers pong", NoParams(), Text("result", "Always pong", true, {"pong"}), ""},
      {"rpc.methods", "mowbite_bridge", "The methods offered right now, of the OS and the planner", NoParams(),
       List("result", "Method names", Type::kString), ""},
      {"sensors.list", "mowbite_bridge",
       "Every sensor of sensor_infos/json with its last value; ids selects some, unknown ids are left out",
       Object("params", "None for all", {List("ids", "Sensor ids of x-sensors", Type::kString, false)}, false),
       Rows("result", "An entry of sensor_infos/json with value and stamp",
            {Text("sensor_id", "Id of x-sensors"), Text("sensor_name", "Name to show"),
             Text("value_type", "STRING or DOUBLE", true, {"STRING", "DOUBLE"}),
             Text("value_description", "What it measures", true,
                  {"VOLTAGE", "CURRENT", "TEMPERATURE", "REVOLUTIONS", "UNKNOWN"}),
             Text("unit", "Unit of the value, empty for none"), Switch("has_min_max", "1 with a range"),
             Number("min_value", "", "Low end of the range, 0 without one"),
             Number("max_value", "", "High end of the range, 0 without one"),
             Switch("has_critical_low", "1 with a lower critical value"),
             Number("lower_critical_value", "", "Below this the value is critical, 0 without one"),
             Switch("has_critical_high", "1 with an upper critical value"),
             Number("upper_critical_value", "", "Above this the value is critical, 0 without one"),
             Nullable({"value", Type::kAny, "", true, "The last value, a text for a text sensor; null before one came",
                       {}, false, {}, ""}),
             Nullable(Number("stamp", "unix s", "When the last value came, null before one came"))}),
       ""},
      {"logs.recent", "mowbite_bridge", "The last log lines from WARN on, oldest first, logins hidden",
       Object("params", "All optional",
              {Number("since", "unix s", "Only lines after this time", false),
               Text("level", "Lowest level: DEBUG, INFO, WARN, ERROR or FATAL; another counts as WARN", false),
               Integer("limit", "Only the newest lines", false)},
              false),
       Rows("result", "A log line",
            {Number("t", "unix s", "When it was logged"), Text("level", "Its level", true, {"WARN", "ERROR", "FATAL"}),
             Text("node", "The node that logged it"), Text("msg", "The text, cut after 2000 bytes")}),
       ""},
      {"events.history", "mowbite_bridge", "The events of one local day as events/json sent them",
       Object("params", "None for today; [\"YYYYMMDD\"] works as well",
              {Text("date", "The day as YYYYMMDD", false)}, false),
       List("result", "Events, oldest first; none for an unknown day", Type::kObject), ""},
      {"system.stats", "mowbite_bridge",
       "Memory, disks, load, temperature, uptime and network of the machine the OS runs on, with the keys and units "
       "of system.cgi of the app; a value that cannot be read is left out",
       NoParams(),
       Object("result", "The values that could be read",
              {Count("mem_total", "B", "Memory of the machine"),
               Count("mem_available", "B", "Memory free for new programs"),
               Count("disk_total", "B", "Size of the system disk"),
               Count("disk_free", "B", "Space left on the system disk"),
               Count("data_total", "B", "Size of the disk of data_dir"),
               Count("data_free", "B", "Space left on the disk of data_dir"),
               Number("load", "", "Load average over one minute", false), Count("cpus", "", "Number of CPU cores"),
               Number("cpu_temp", "deg.C", "Temperature of the CPU or SoC", false),
               Count("uptime", "s", "Time since the machine started"),
               Text("wifi_iface", "The first wifi interface", false),
               Count("wifi_signal", "dBm", "Signal of that wifi"),
               Text("net_iface", "The interface of the default route", false),
               Text("net_ip", "Its IPv4 address", false)}),
       ""},
      {"events.history.list", "mowbite_bridge", "The days with events", NoParams(),
       List("result", "Days as YYYYMMDD, newest first", Type::kString), ""},
      {"events.history.delete", "mowbite_bridge", "Deletes the events of one day, without a day of all",
       Object("params", "None for every day", {Text("date", "The day as YYYYMMDD", false)}, false), Deleted(), ""},
      {"position.history", "mowbite_bridge", "The track of a job of mowing",
       Object("params", "None for the current job", {Text("job_id", "The job", false)}, false),
       Object("result", "The track, or error for an unknown job",
              {Rows("segments", "A stretch with the same job, session and blade",
                    {Object("attributes", "What the stretch belongs to",
                            {Text("job_id", "The job"), Text("session_id", "The session of the logic"),
                             Bool("blades", "Whether the blade ran")}),
                     List("points", "Points [x, y] in m", Type::kArray)},
                    false),
               List("buffer", "Points [x, y] in m not yet in a segment", Type::kArray, false),
               Text("error", "not found for an unknown job", false, {"not found"})}),
       ""},
      {"position.history.list", "mowbite_bridge", "The jobs with a track", NoParams(),
       Rows("result", "A job with a track, newest first",
            {Text("job_id", "The job"), Integer("timestamp", "When its track began, unix s")}),
       ""},
      {"position.history.delete", "mowbite_bridge", "Deletes the track of a job, without a job all but the current one",
       Object("params", "None for all", {Text("job_id", "The job", false)}, false), Deleted(), ""},
      {"mowing.plan", "mowbite_logic", "Asks the planner for the paths of an area without mowing it",
       Object("params", "area_id, or outline with obstacles; the rest overrides the planner's values for this plan",
              {Text("area_id", "A mowing area of the map", false),
               List("outline", "Points {x, y} in m", Type::kObject, false),
               List("obstacles", "Rings of points {x, y} in m", Type::kArray, false),
               Number("angle", "rad", "Direction of the lanes, 0 east, counter-clockwise", false),
               Integer("outline_count", "Loops along the outline", false),
               Integer("outline_overlap_count", "Lanes that overlap the loops", false),
               Number("outline_offset", "m", "Distance of the first loop from the outline", false)}),
       Object("result", "The plan",
              {Text("area_id", "The area, empty for an outline"),
               Number("angle", "rad", "Direction of the lanes the planner used"),
               Rows("paths", "A path of the plan",
                    {Bool("is_outline", "A loop along the outline"),
                     List("points", "Points [x, y] in m, simplified to 1 cm", Type::kArray),
                     List("pose_index", "Index of each point in its path", Type::kInteger)}),
               Text("fingerprint", "The planner's fingerprint of the plan")}),
       ""},
      {"params.set", "mowbite_logic",
       "Changes settings of x-settings in IDLE, all or nothing; a value outside its range is set to the bound",
       Object("params", "The settings by full name",
              {Object("params", "\"/mower_logic/<name>\": number, for names of x-settings")}),
       Object("result", "The values taken by full name, those outside the range at the bound"), ""},
      {"robot.geometry.get", "mowbite_logic", "The dimensions of the mower, keys of x-geometry", NoParams(),
       GeometryAnswer(), ""},
      {"robot.geometry.set", "mowbite_logic", "Changes dimensions in IDLE, all or nothing; GEOMETRY_CHANGED follows",
       Object("params", "The change",
              {Integer("version", "version of the last robot.geometry.get"),
               Object("settings", "Values by key of x-geometry", {}, false),
               List("reset", "Keys back to the platform's value", Type::kString, false)}),
       GeometryAnswer(), ""},
      {"system.hold", "mowbite_logic",
       "Holds the mower for an update: granted only in IDLE with nothing on its way and no start of the schedule within "
       "1800 s, 0 lets go",
       Object("params", "The hold", {Integer("seconds", "0 to 1800")}),
       Object("result", "What the gate of the updater looks at",
              {Text("state", "current_state of robot_state"), Text("sub_state", "current_sub_state of robot_state"),
               Bool("is_charging", "On the charging contacts"),
               Integer("hold_until", "End of the hold in unix s, 0 without one"),
               Bool("idle", "Whether a hold can be granted now: also false within 1800 s before next_start"),
               Integer("emergency_reason", "The bits of the emergency, 0 without one"),
               Time("next_start", "The next start of the schedule, null without one")}),
       ""},
      {"calibration.check", "mowbite_logic", "Whether a calibration drive could start here, without a drive",
       NoParams(),
       Object("result", "The check",
              {Bool("ok", "No problem"),
               List("problems", "Each with reason, message and its fields", Type::kObject),
               Nullable(Object("space", "Room for the drive, null without one",
                               {Number("length", "m", "Length of the strip"), Number("width", "m", "Width of the strip"),
                                Number("lane_length", "m", "Length of a lane"),
                                Number("heading", "rad", "Direction of the lanes")}))}),
       "while the calibration program runs"},
      {"calibration.start", "mowbite_logic", "Starts the calibration drive; a refusal carries reason and message",
       NoParams(), Object("result", "The drive began", {Text("state", "Always CALIBRATING", true, {"CALIBRATING"})}),
       "while the calibration program runs"},
      {"calibration.abort", "mowbite_logic", "Stops the calibration drive where the mower is", NoParams(),
       Object("result", "Whether a drive ran", {Bool("ok", "A drive was stopped")}),
       "while the calibration program runs"},
      {"calibration.result", "mowbite_calibration", "The result of the last calibration, null before one",
       NoParams(), Nullable(Object("result", "As the calibration wrote it")), ""},
      {"schedule.get", "mowbite_schedule", "The schedule, whether it may start, the clock and the sun", NoParams(),
       ScheduleAnswer(), ""},
      {"schedule.set", "mowbite_schedule",
       "Changes keys of the schedule, all or nothing: -32602 with data {field, reason}, -32001 when version is not the "
       "one of the last schedule.get",
       Object("params", "The change",
              {Schedule(true), Integer("version", "version of the last schedule.get", false)}),
       ScheduleAnswer(), ""},
      {"schedule.log", "mowbite_schedule", "The log of the schedule, oldest first; words in x-schedule-log",
       Object("params", "How many", {Integer("limit", "The last so many, at most 100", false)}, false),
       Object("result", "The log",
              {Rows("entries", "Each with t (unix s), what, detail, more and at (HH:MM of the plan or null)",
                    {UnixTime("t", "When"), Text("what", "A word of x-schedule-log"),
                     Nullable(Text("detail", "As x-schedule-log has it")),
                     Nullable(Text("more", "As x-schedule-log has it")),
                     Nullable(Text("at", "HH:MM of the plan, null for none"))})}),
       ""},
      {"schedule.leftovers", "mowbite_schedule",
       "The leftovers after a job: enabled 1 is refused (-32602 unsupported) until the planner prepares them on the OS",
       Object("params", "The change, none to read",
              {Switch("enabled", "1: leftovers after a job"), Number("min_m2", "m2", "Less is not mowed", false)},
              false),
       Object("result", "The leftovers now",
              {Switch("enabled", "1: leftovers after a job"), Number("min_m2", "m2", "Less is not mowed")}),
       ""},
      {"stats.get", "mowbite_stats", "Mowing statistics per hour and the parts to see to",
       Object("params", "None: the hours since the turn of the year, less a day",
              {UnixTime("from", "The hours from then on")}, false),
       StatsAnswer(), ""},
      {"stats.part_done", "mowbite_stats",
       "The part was seen to now: its blade hours start from 0; as stats.get from now",
       Object("params", "The part", {Text("part", "Its key", true, {"blade", "clean"})}), StatsAnswer(), ""},
      {"stats.part_due", "mowbite_stats", "After how many blade hours the part is due again; as stats.get from now",
       Object("params", "The part and its hours",
              {Text("part", "Its key", true, {"blade", "clean"}),
               Whole("hours", "h", 1, kStatsMaxDueHours, "Blade hours")}),
       StatsAnswer(), ""},
      {"notify.get", "mowbite_net", "The settings of the pushes; of the token only whether there is one", NoParams(),
       NotifyAnswer(), ""},
      {"notify.set", "mowbite_net",
       "Changes settings, all or nothing: -32602 with data {field, reason}. A push says so to the targets of before",
       Object("params", "The change",
              [] {
                std::vector<Field> fields = NotifySettings(false);
                fields.push_back(Text("token", "New token for the ntfy server; empty drops it", false));
                return fields;
              }()),
       NotifyAnswer(), ""},
      {"notify.test", "mowbite_net", "Sends a test push to the topic and the phones, or to one address",
       Object("params", "None for all", {Text("address", "One UnifiedPush address of pushes", false)}, false),
       Object("result", "How it went",
              {Text("result", "sent: to at least one; no_target: no topic and no phone", true,
                    {"sent", "failed", "no_target"})}),
       ""},
      {"notify.push_add", "mowbite_net",
       "A phone's UnifiedPush address; past " + std::to_string(kNotifyMaxPushes) + " the oldest goes",
       Object("params", "The address", {Text("address", "https URL, at most 1000 characters")}), NotifyAnswer(), ""},
      {"notify.push_remove", "mowbite_net", "Drops a phone's UnifiedPush address",
       Object("params", "The address", {Text("address", "As in pushes")}), NotifyAnswer(), ""},
      {"notify.snooze", "mowbite_net", "No reminders for so long; 0 lifts it",
       Object("params", "How long", {Whole("seconds", "s", 0, kNotifyMaxSnooze, "Seconds from now")}), NotifyAnswer(),
       ""},
      {"notify.log", "mowbite_net", "The last pushes, newest first",
       Object("params", "How many", {Whole("limit", "", 1, kNotifyMaxLog, "The last so many", false)}, false),
       Rows("result", "A push",
            {UnixTime("t", "When"), Bool("sent", "Reached at least one target"), Text("kind", "Its event or test"),
             Text("title", "Its title")}),
       ""},
      {"light.get_profile", "mowbite_light",
       "The lights: the states and effects there are, the profile by state, its defaults, the fixed signal of an "
       "emergency and the limits",
       NoParams(),
       Object("result", "The lights now",
              {List("states", "The states of a profile in the order to show them", Type::kString),
               List("effects", "The effects there are", Type::kString),
               Object("profile", "An effect by state"), Object("defaults", "The effect by state after light.reset"),
               LightEffect("emergency", "What every emergency shows; no profile changes it"),
               Object("limits", "Bounds of an effect",
                      {Integer("colors", "Most colours of an effect"),
                       Number("min_period", "s", "Shortest period"), Number("max_period", "s", "Longest period"),
                       Number("max_preview", "s", "Longest light.preview")}),
               Integer("leds", "LEDs on the mower")}),
       ""},
      {"light.set_profile", "mowbite_light",
       "Stores the profile, all or nothing; a state left out gets its default",
       Object("params", "The profile", {Object("profile", "An effect by state of light.get_profile")}),
       LightProfile(), ""},
      {"light.preview", "mowbite_light",
       "Shows an effect for a while, then the state again; an emergency shows over it",
       Object("params", "What to show",
              {LightEffect("effect", "The effect"),
               Number("duration", "s", "How long, more than 0 and at most limits.max_preview; that without", false)}),
       Object("result", "The preview",
              {Number("duration", "s", "How long it shows"),
               Bool("emergency", "An emergency shows over it right now")}),
       ""},
      {"light.reset", "mowbite_light", "Back to the default profile; ends a preview", NoParams(), LightProfile(), ""},
      {"sound.list", "mowbite_sound", "The situations the OS has a sound for, the sound of each and the volume",
       NoParams(),
       Object("result", "The sounds",
              {Rows("situations", "A situation, in the order to show them",
                    {Text("name", "Name of the situation", true, SoundSituationNames()),
                     Text("event", "What makes the OS play it, in English"),
                     Nullable(Object("file", "Its sound, null without one",
                                     {Text("source", "vendor: of the maker of the mower; upload: of the user", true,
                                           {"vendor", "upload"}),
                                      Integer("size", "Bytes of the original"),
                                      Text("sha256", "SHA-256 of the original"),
                                      Number("seconds", "s", "How long it plays")}))}),
               Integer("volume", "0 to 100; 0 is silent")}),
       kSoundOffered},
      {"sound.play", "mowbite_sound",
       "Plays the sound of a situation once, to listen to it; refused (SOUND_REFUSED) in an emergency and without a "
       "sound (NO_FILE)",
       Object("params", "The situation", {Text("situation", "A name of sound.list", true, SoundSituationNames())}),
       Object("result", "What plays",
              {Text("situation", "The situation"), Number("seconds", "s", "How long it plays")}),
       kSoundOffered},
      {"sound.stop", "mowbite_sound", "Ends the sound that plays; refused (SOUND_REFUSED) in an emergency", NoParams(),
       Object("result", "Empty"), kSoundOffered},
      {"sound.volume", "mowbite_sound",
       "Reads the volume, or sets it with volume; 0 is silent, also in an emergency. A change while an update holds "
       "the mower is refused with -32004",
       Object("params", "None to read", {Integer("volume", "0 to 100", false)}, false),
       Object("result", "The volume now", {Integer("volume", "0 to 100")}), kSoundOffered},
      {"sound.upload", "mowbite_sound",
       "Uploads an MP3 of at most 2 MB and 30 s for a situation in parts, one upload at a time, also in an emergency; "
       "part 0 starts it, the same file from part 0 starts it again. The OS checks it after the last part and only "
       "then plays it instead of the vendor's sound. Refused (SOUND_REFUSED) for a wrong file, order or size and while "
       "another upload runs; an unfinished upload is dropped 60 s after its last part; -32004 while an update holds "
       "the mower",
       Object("params", "A part",
              {Text("situation", "A name of sound.list", true, SoundSituationNames()),
               Integer("part", "Number of this part, from 0"), Integer("parts", "How many parts the file has"),
               Text("data", "The bytes of this part in base64, at most 512 KB before base64"),
               Text("sha256", "SHA-256 of the whole file in lower-case hex, the same in every part")}),
       Object("result", "After a part before the last: received; after the last: the sound now",
              {Integer("received", "Number of the part taken", false),
               Text("situation", "The situation", false), Text("source", "Always upload", false, {"upload"}),
               Number("duration_s", "s", "How long it plays", false),
               Text("sha256", "SHA-256 of the file", false)}),
       kSoundOffered},
      {"sound.delete", "mowbite_sound",
       "Deletes the upload of a situation, so that the vendor's sound plays again, or none without one; nothing to "
       "delete is no error. -32004 while an update holds the mower",
       Object("params", "The situation", {Text("situation", "A name of sound.list", true, SoundSituationNames())}),
       Object("result", "The sound of the situation now, as sound.list has it",
              {Text("situation", "The situation"),
               Nullable(Text("source", "vendor, or null without a sound", true, {"vendor"})),
               Nullable(Object("file", "Its sound, null without one",
                               {Text("source", "Always vendor", true, {"vendor"}),
                                Integer("size", "Bytes of the original"), Text("sha256", "SHA-256 of the original"),
                                Number("seconds", "s", "How long it plays")}))}),
       kSoundOffered},
      {"camera.list", "mowbite_camera",
       "The cameras of the mower; CAMERA_CHANGED tells when one of them can stream again or no longer, or streams "
       "with other protocols or at another size", NoParams(),
       Rows("result", "A camera, in the order they first reported",
            {Text("id", "Its id for camera.stream"), Text("name", "Its name to show"),
             Text("kind", "What it is for: obstacle, stereo, rear or another kind of the platform"),
             Nullable(Text("position", "Where it looks, null when unknown or not fixed", true,
                           {"front", "rear", "left", "right"})),
             Integer("width", "Width of the picture in pixels"), Integer("height", "Height of the picture in pixels"),
             Bool("detections", "The boxes of the camera AI are drawn into the picture"),
             List("protocols", "How it streams, mjpeg or webrtc; camera.stream takes the first without a protocol",
                  Type::kString),
             Bool("available", "Whether it can stream now"),
             Text("reason", "Why not, only while it is not available: a reason of CAMERA_REFUSED in x-reasons", false),
             Text("detail", "A few English words on the reason, for the log", false)}),
       kCameraOffered},
      {"camera.stream", "mowbite_camera",
       "Keeps a place for one viewer and gives the address to watch from: for one connection, which has to begin "
       "before expires; the place is free again when that connection ends, and every new connection needs a new "
       "call. At most two viewers at once over all cameras (CAMERA_BUSY). mjpeg: a GET of url gives "
       "multipart/x-mixed-replace, every part one JPEG with its own Content-Length, and every answer to that GET, "
       "also a refusal, carries Access-Control-Allow-Origin: *; webrtc: url is a WHEP endpoint (H.264). Refused "
       "(CAMERA_REFUSED) while the camera is not available; -32602 for an id or a protocol it does not have",
       Object("params", "The camera",
              {Text("id", "An id of camera.list"),
               Text("protocol", "One of its protocols; its first without", false, {"mjpeg", "webrtc"})}),
       Object("result", "The stream",
              {Text("url", "http address with the token of this connection; its host is the IPv4 of the mower in the "
                    "network of its default route"),
               Text("protocol", "The protocol of url", true, {"mjpeg", "webrtc"}),
               UnixTime("expires", "The connection has to begin before this")}),
       kCameraOffered},
      {"map.replace", "mowbite_map", "Replaces the map with the one given, in the shape of map/json",
       List("params", "[map]", Type::kObject), Text("result", "Successfully stored map (n areas)"), ""},
      {"map.area_properties", "mowbite_map", "The properties an area of the map may have", NoParams(),
       List("result", "Property names", Type::kString), ""},
      {"ntrip.get", "mowbite_ntrip", "The access to the caster for corrections, without user and password",
       NoParams(), NtripAccess(), ""},
      {"ntrip.set", "mowbite_ntrip",
       "Stores the access in IDLE, all or nothing; a stored login stays only for the same host and port",
       Object("params", "The access",
              {Text("host", "Host of the caster"), Text("mountpoint", "Mountpoint"),
               Integer("port", "Port, 2101 without one", false), Integer("version", "1 or 2, 2 without one", false),
               Text("user", "User", false), Text("password", "Password", false)}),
       NtripAccess(), "only with access_over_rpc"},
      {"ntrip.delete", "mowbite_ntrip", "Removes the access in IDLE", NoParams(), NtripAccess(),
       "only with access_over_rpc"},
      {"update.status", "mowbite_host",
       "What the updater does and what is there; the same retained on mowbite/update/status", UpdateParams({}),
       UpdateStatus(), kHostOffered},
      {"update.start", "mowbite_host",
       "Installs the newest available revision of the component or the one given, one update at a time; os and "
       "planner wait for the gate (IDLE on the charging contacts, the hold granted) for up to 30 min, the planner also "
       "for the end of the mowing job, the app and the host for none. The host starts itself anew and gives the result "
       "once the new one confirmed itself after 60 s, else it goes back. Refused (UPDATE_REFUSED) when none is there "
       "or one runs",
       UpdateParams({Component(),
                     Text("revision", "A revision of available or found by update.check; the newest without", false),
                     ResetJobs()},
                    true),
       UpdateStatus(), kHostOffered},
      {"update.abort", "mowbite_host", "Stops the update before its switch; after it the health check decides",
       UpdateParams({}), UpdateStatus(), kHostOffered},
      {"update.rollback", "mowbite_host",
       "Goes back to the previous installation through the same gate, planner and app without an own one before to "
       "the bundled one, the host to its seed in the rootfs; that revision is then not installed by itself. "
       "NO_PREVIOUS on the bundled one or the seed",
       UpdateParams({Component(), ResetJobs()}, true), UpdateStatus(), kHostOffered},
      {"update.check", "mowbite_host", "Looks for bundles in the origins now (the inbox), as it does every 6 h",
       UpdateParams({}), UpdateStatus(), kHostOffered},
      {"update.set_auto", "mowbite_host",
       "Switches the automatic update on or off and sets its window; it needs an app as new as min_app. It takes "
       "the switch also without a sure clock, only the update in the window waits for one",
       UpdateParams(Auto(false)),
       UpdateStatus(), kHostOffered},
      {"host.restart_os", "mowbite_host",
       "Starts MowBite OS again, its restarts from 0: always when it is stopped, down or halted, else only in IDLE "
       "without an emergency or the restart hold; never during an update. The restart hold judges at the start as "
       "always",
       NoParams(), Object("result", "os of mowbite/host/status after the restart", HostOs()), kHostOffered},
  };
  return methods;
}

const std::vector<NotifyEvent>& NotifyEvents() {
  static const std::vector<NotifyEvent> events = {
      {"emergency", true, true}, {"dock_failed", true, true}, {"undock_failed", true, true},
      {"nav_error", true, true}, {"spinup", true, true},      {"standstill", true, true},
      {"refused", true, true},   {"done"},                    {"rain"},
      {"battery"},               {"dark"},                    {"maintenance", false, true},
      {"schedule_skipped"}};
  return events;
}

const std::vector<ScheduleLogWord>& ScheduleLogWords() {
  static const std::vector<ScheduleLogWord> words = {
      {"skip_missed", "The time of the plan passed while the OS was off, the clock jumped or the plan waited", "", "",
       false, true},
      {"skip_offline", "The logic was silent through the minute of the plan", "", "", true, true},
      {"skip_clock", "The clock was not safe", "clock_reason of schedule.get", "", true, true},
      {"skip_paused", "The pause covers every area of the plan", "", "", true},
      {"skip_no_area", "No area of the plan is a mowing area of the map", "", "", true, true},
      {"skip_end", "The end of the plan came while it waited", "", ""},
      {"skip_emergency", "An emergency, its latch or the restart hold", "", "", true, true},
      {"skip_busy", "The mower was busy", "The state of robot_state or the phase of a start of the schedule", "", true},
      {"skip_not_docked", "The mower was not on the contacts", "", "", true, true},
      {"skip_rain", "The rain sensor was wet with skip_rain 1 or rain_mode not 0", "", "", true},
      {"skip_dark", "The sun was down for a plan without dark", "", "", true},
      {"skip_no_position", "No datum for the sun, for a plan without dark", "", "", true, true},
      {"waiting_battery", "The plan waits up to 2 h for min_battery", "Battery %", "", true},
      {"skip_battery", "The battery stayed below min_battery for 2 h", "Battery %", "", false, true},
      {"waiting_hold", "The plan waits up to 2 h for the end of a hold", "", "", true},
      {"skip_hold", "A hold stayed for 2 h", "", "", false, true},
      {"skip_gone", "The plan was removed while it waited", "", ""},
      {"weather_unknown", "No usable weather; the plan goes on", "forecast or cold", ""},
      {"skip_forecast", "Rain now or in the next forecast_hours", "", ""},
      {"skip_cold", "Colder than min_temp", "deg C", ""},
      {"unknown_area", "An area of the plan is not on the map; the run leaves it out", "The area id", ""},
      {"paused_area", "An area of the plan is paused; the run leaves it out", "The area id", ""},
      {"start_refused", "The logic refused the start", "The reason of MOWING_REFUSED", "Its value, or null"},
      {"start_canceled", "The announcement was canceled", "Its source: emergency, rpc, undocked or hold", ""},
      {"start_failed", "The start did not reach mowing",
       "storage, service, timeout, or the end of the run (UNDOCKING_FAILED, EMERGENCY, ...)", "", false, true},
      {"started", "The mower mows", "Battery %", ""},
      {"stopped_end", "Home at the end of the plan", "", ""},
      {"stopped_dark", "Home at sunset", "", ""},
      {"stopped_rain", "Home for rain", "", ""},
      {"stopped_paused", "The pause covered every area left of the run", "", ""},
  };
  return words;
}

const std::vector<SoundSituation>& SoundSituations() {
  static const std::vector<SoundSituation> situations = {
      {"MOWING_START", "STATE MOWING, not after PAUSED"},
      {"MOWING_COMPLETE", "JOB_COMPLETE"},
      {"DOCKING_START", "STATE DOCKING"},
      {"LOW_BATTERY", "BATTERY_LOW, or DOCKING for BATTERY_LOW or BATTERY_CRITICAL"},
      {"RAIN_DETECTED", "DOCKING for RAIN"},
      {"MOWING_PAUSED", "STATE PAUSED"},
      {"AREA_RECORDING_MODE", "STATE AREA_RECORDING"},
      {"DOCKING_FAILED", "DOCKING_FAILED"},
      {"CHARGING_START", "is_charging of robot_state turns true"},
      {"CHARGING_DONE", "battery_percentage reaches 0.99 while charging"},
      {"EMERGENCY", "An emergency begins without LIFT or LIFT_MULTIPLE"},
      {"LIFT_TRIGGERED", "An emergency with LIFT or LIFT_MULTIPLE begins, or one of them joins it"},
      {"SCHEDULE_START", "SCHEDULE_START: a plan of the schedule starts in seconds"},
  };
  return situations;
}

std::string NodeOf(const std::string& method) {
  for (const auto& entry : Methods()) {
    if (entry.name == method) return entry.node;
  }
  if (method.rfind("system.", 0) == 0 && FindCommand(method)) return "mowbite_bridge";
  return FindCommand(method) ? "mowbite_logic" : "";
}

const std::vector<SensorEntry>& Sensors() {
  static const std::vector<SensorEntry> sensors = {
      {"om_v_battery", "V Battery", "V", false, "logic", "Voltage of the battery"},
      {"om_v_charge", "V Charge (dock contact)", "V", false, "logic",
       "The charge voltage while on the contacts, else 0; derived, the board reports none"},
      {"om_charge_current", "Charge Current", "A", false, "logic", "Current into the battery"},
      {"om_charge_state", "Charge State (derived)", "", true, "logic",
       "Not Charging, Fast Charge or Done, derived from contact, current and charge"},
      {"om_mow_motor_rpm", "Mow Motor Revolutions", "rpm", false, "logic", "Speed of the blade motor"},
      {"om_mow_motor_current", "Mow Motor Current", "A", false, "logic", "Current of the blade motor"},
      {"om_gps_accuracy", "GPS Accuracy", "m", false, "logic", "Expected error of the position"},
      {"om_mow_motor_temp", "Mow Motor Temp", "deg.C", false, "logic", "Temperature of the blade motor"},
      {"om_left_esc_temp", "Left ESC Temp", "deg.C", false, "logic", "Temperature of the left motor board"},
      {"om_right_esc_temp", "Right ESC Temp", "deg.C", false, "logic", "Temperature of the right motor board"},
      {"battery_temp", "Battery Temp", "deg.C", false, "logic", "Temperature of the battery from its BMS"},
      {"os_correction_age", "RTK Correction Age", "s", false, "gnss", "Age of the last RTK correction"},
      {"os_ntrip_state", "NTRIP", "", true, "ntrip", "State of the connection to the caster"},
  };
  return sensors;
}

const SensorEntry* FindSensor(const std::string& id) {
  for (const auto& sensor : Sensors()) {
    if (sensor.id == id) return &sensor;
  }
  return nullptr;
}

const std::vector<SettingEntry>& LogicSettings() {
  // Bounds of MowBite OS beyond MowerLogic.cfg: max_position_accuracy at most 0.3 m, gps_timeout at least 1 s,
  // docking_approach_distance and undock_distance 0.5 to 2 m, docking_retry_count at most 5.
  static const std::vector<SettingEntry> settings = {
      {"rain_mode", true, 0, 3, 0, "", "What rain does, as in OpenMower"},
      {"rain_delay_minutes", true, 30, 1440, 30, "min", "How long to wait after rain"},
      {"rain_check_seconds", true, 20, 300, 20, "s", "How long rain has to last to count"},
      {"automatic_mode", true, 0, 2, 0, "", "Automatic mowing, as in OpenMower"},
      {"gps_timeout", false, 1.0, 60.0, 10.0, "s", "Without a position this long the mower stops"},
      {"max_position_accuracy", false, 0.01, 0.3, 0.1, "m", "Worst expected error of the position to mow"},
      {"gps_wait_time", false, 0.0, 60.0, 10.0, "s", "How long the position has to be good again to go on"},
      {"docking_approach_distance", false, 0.5, 2.0, 1.5, "m", "Distance of the point in front of the station"},
      {"docking_distance", false, 0.0, 100.0, 2.0, "m", "Distance of the drive onto the station, as in OpenMower"},
      {"docking_retry_count", true, 0, 5, 4, "", "Tries of docking after the first"},
      {"undock_distance", false, 0.5, 2.0, 2.0, "m", "How far the mower backs off the station"},
      {"motor_hot_temperature", false, 20.0, 150.0, 70.0, "deg.C", "The blade motor is too hot from here"},
      {"motor_cold_temperature", false, 20.0, 150.0, 40.0, "deg.C",
       "The blade motor may run again below this; below motor_hot_temperature"},
      {"dock_at_sunset", true, 0, 1, 1, "", "Whether a run started by hand goes home at sunset"},
  };
  return settings;
}

const std::vector<GeometryEntry>& GeometryKeys() {
  // Ranges are against slips of the pen (cm for m, a wrong sign), not against the platform's values: a measured body
  // may be smaller than the one of the platform's file. The calibration drive sets track, the antenna, ticks_per_m and
  // the mounting of the IMU, which may differ from the platform's by 5 degrees at most.
  static const std::vector<GeometryEntry> keys = {
      {"robot_width", false, true, 0.1, 1.5, "m", false, "Width of the body"},
      {"robot_front", false, true, 0.0, 1.5, "m", false, "Front of the body ahead of the drive axle (base_link)"},
      {"robot_rear", false, true, 0.0, 1.5, "m", false, "Rear of the body behind the drive axle"},
      {"robot_outline", true, true, 0.0, 0.0, "m", false,
       "Outline of the body as points [x, y] from the drive axle, each at most 1.5 m away"},
      {"mower_width", false, true, 0.05, 1.5, "m", false, "Width the blade cuts"},
      {"blade_ahead", false, true, -1.5, 1.5, "m", false, "Centre of the blade ahead of the drive axle"},
      {"blade_offset", false, true, -0.75, 0.75, "m", false, "Centre of the blade left of the middle"},
      {"antenna_x", false, true, -0.6, 0.6, "m", false, "GNSS antenna ahead of the drive axle"},
      {"antenna_y", false, true, -0.3, 0.3, "m", false, "GNSS antenna left of the middle"},
      {"track", false, true, 0.15, 0.8, "m", false, "Distance between the wheels"},
      {"wheel_radius", false, false, 0.02, 0.5, "m", false, "Radius of the drive wheels"},
      {"dock_contact_x", false, false, -1.5, 1.5, "m", false, "Charging contacts ahead of the drive axle"},
      {"ticks_per_m", false, true, 100.0, 10000.0, "1/m", false, "Wheel ticks per metre"},
      {"imu_roll", false, true, -0.087, 0.087, "rad", true, "Roll of the IMU against the platform's value"},
      {"imu_pitch", false, true, -0.087, 0.087, "rad", true, "Pitch of the IMU against the platform's value"},
  };
  return keys;
}

const EventEntry* FindEvent(const std::string& type) {
  static const std::map<std::string, const EventEntry*> index = [] {
    std::map<std::string, const EventEntry*> out;
    for (const auto& event : Events()) out[event.type] = &event;
    return out;
  }();
  const auto found = index.find(type);
  return found == index.end() ? nullptr : found->second;
}

const EventEntry* FindReasonGroup(const std::string& type) {
  if (const EventEntry* event = FindEvent(type)) return event;
  for (const auto& group : Refusals()) {
    if (group.type == type) return &group;
  }
  return nullptr;
}

const Command* FindCommand(const std::string& name) {
  for (const auto& command : Commands()) {
    if (command.name == name) return &command;
  }
  return nullptr;
}

const Command* CommandOfAction(const std::string& action, Json* params) {
  for (const auto& command : Commands()) {
    for (const auto& entry : command.actions) {
      if (entry.action != action) continue;
      if (params) *params = entry.params;
      return &command;
    }
  }
  return nullptr;
}

bool ReasonFits(const ReasonEntry& reason, const std::string& code, const Json& details) {
  if (reason.code != code) return false;
  for (const auto& [key, value] : reason.when.items()) {
    if (!details.is_object() || !details.contains(key) || details[key] != value) return false;
  }
  return true;
}

Rating RateReason(const std::string& event, const std::string& code, const Json& details) {
  const EventEntry* entry = FindReasonGroup(event);
  if (!entry) return {"warning", "user"};
  for (const auto& reason : entry->reasons) {
    if (ReasonFits(reason, code, details)) return {reason.severity.c_str(), reason.resume.c_str()};
  }
  return {entry->other_severity.c_str(), entry->other_resume.c_str()};
}

std::vector<std::string> EmergencyNameList(int bits) {
  const uint32_t set = static_cast<uint32_t>(bits);
  uint32_t named = 0;
  std::vector<std::string> names;
  for (const auto& bit : EmergencyBits()) {
    named |= 1u << bit.bit;
    if ((set & (1u << bit.bit)) != 0) names.push_back(bit.name);
  }
  if ((set & ~named) != 0) names.push_back(kUnknownEmergency);
  return names;
}

std::string EmergencyNames(int bits) {
  std::string names;
  for (const auto& name : EmergencyNameList(bits)) names += (names.empty() ? "" : ", ") + name;
  return names;
}

}  // namespace mowbite_api
