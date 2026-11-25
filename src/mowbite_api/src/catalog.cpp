#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <map>
#include <string>

#include "fields.h"
#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"

namespace mowbite_api {

namespace {

using namespace fields;

// An optional whole number with a unit.
Field Count(std::string name, std::string unit, std::string text) {
  return {std::move(name), Type::kInteger, std::move(unit), false, std::move(text), {}, false, {}, ""};
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
              std::string refused, std::vector<CommandAction> actions, std::string offered = "") {
  params.insert(params.begin(), Client());
  return {std::move(name), std::move(text), std::move(params), std::move(result), std::move(refused),
          std::move(actions), std::move(offered)};
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
                        "(number, points or shape), unit, min and max of a number, settable, and for a settable key "
                        "confirmed (unix s the user set or confirmed its value, null before)"),
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

const char kSetupOffered[] = "where the platform has a step list (setup/setup.yaml)";

const char kObstaclesOffered[] = "where the platform has an obstacle sensor (obstacle_sources of hw/capabilities)";

Field SetupAnswer() {
  Field state = SetupState();
  state.name = "result";
  state.text = "The state of the program after the call, as setup/json";
  state.defined = "setup";
  return state;
}

Field SetupStep() {
  return Text("step", "id of the step");
}

Field SetupVersion() {
  return Integer("version", "version of the state read last");
}

// The steps of sensors.history from first on, as "60, 300 or 900".
std::string HistorySteps(size_t first) {
  std::string text;
  for (size_t i = first; i < std::size(kSensorsHistorySteps); ++i) {
    if (i > first) text += i + 1 == std::size(kSensorsHistorySteps) ? " or " : ", ";
    text += std::to_string(kSensorsHistorySteps[i]);
  }
  return text;
}

Field UnixTime(std::string name, std::string text) {
  return {std::move(name), Type::kInteger, "unix s", true, std::move(text), {}, false, {}, ""};
}

// An obstacle as vision.obstacles.list has it; x, y and r in the map frame as map/json.
Field ObstacleRow() {
  return Rows(
      "obstacles", "The obstacles seen, last seen first",
      {Integer("id", "Its id, stable while it is in the list"), UnixTime("t", "When it was first seen"),
       UnixTime("last_t", "When it was last seen"), Integer("count", "How often it was seen anew"),
       Number("x", "m", "Middle in the map frame"), Number("y", "m", "Middle in the map frame"),
       Number("r", "m", "Radius on the ground"),
       Text("kind", "What it is", true, ObstacleKinds()),
       Text("class", "As the model of the platform names it, for the app only; empty for UNKNOWN"),
       Number("confidence", "", "Best score of the model, 0 to 1; 1 for UNKNOWN"),
       Text("camera_id", "The sensor that saw it, an id of camera.list for a camera"),
       Text("action", "What the mower did", true, ObstacleActions()),
       Nullable(Number("keep_out_r", "m",
                       "While the mower keeps out of a circle around it (action avoided): its radius around x, y; "
                       "null without one")),
       Nullable(Number("left_out", "m", "With keep_out_r: m of the paths of the job left unmowed for it")),
       Nullable(Object("image", "Its photo, null without one",
                       {Text("name", "For vision.obstacles.image; stays while the entry stays"),
                        Text("sha256", "sha256 of the JPEG"), Integer("bytes", "Size of the JPEG"),
                        Integer("width", "Width in pixels"), Integer("height", "Height in pixels"),
                        Object("box", "Where it is in the photo, pixels from the top left",
                               {Number("x", "px", "Left edge"), Number("y", "px", "Upper edge"),
                                Number("w", "px", "Width"), Number("h", "px", "Height")})}))});
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

Field Required(Field field) {
  field.required = true;
  return field;
}

// The fields of an object all optional: what is left out stays as it is.
Field Partial(Field object) {
  for (auto& field : object.fields) field.required = false;
  return object;
}

Field ScheduleAnswer() {
  return Object(
      "result", "The schedule now",
      {Integer("schema", "Always 1"), Integer("version", "Goes up with every change; schedule.set may name it"),
       Bool("available", "Whether the schedule may start the mower now"),
       Nullable(Text("reason",
                     "Why not, else null: platform_off (not released for the platform), data_rollback (the data went "
                     "back to an older backup and the map waits for data.rollback.confirm; schedule.set still takes "
                     "changes)",
                     true, {"platform_off", "data_rollback"})),
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
       Nullable(Integer("api", "os: the api it speaks, null for a bundle that does not say; absent for the others",
                        false)),
       Nullable(Integer("api_break",
                        "os: the break of its api, null for a bundle from before it; absent for the others", false)),
       Nullable(Object("blocked",
                       "Not installed by itself: why, the result of the attempt that ended so and when; update.start "
                       "of this revision tries it anyway",
                       {Text("reason", "A reason of UPDATE in x-reasons, never BLOCKED"),
                        Text("result", "How the attempt ended", false, {"rolled_back", "failed"}),
                        Time("t", "When, null for a reason reckoned from the installed OS (kept nowhere)")})),
       Text("from", "Where it was found: the inbox on the mower or the net (the release index)", false,
            {"inbox", "net"})}));
}

// The release index on the net the updater asks every 6 h and on update.check.
Field UpdateOrigin() {
  return Object(
      "origin", "The release index on the net and the last time it was asked",
      {Text("channel", "stable, or preview for previews as well", true, {"stable", "preview"}),
       Nullable(Text("url", "The address of the index, null without one")),
       Bool("downloading", "Whether the index or a bundle is being downloaded"),
       Text("result", "How the last asking ended; absent before the first", false,
            {"ok", "unreachable", "bad_index", "too_large", "no_space", "bad_bundle", "failed"}),
       Nullable(Text("message", "English text for the log, null when ok", false)),
       Integer("checked_at", "When it was last asked, unix s; absent before the first", false),
       Integer("index_serial", "The serial of the last index taken", false),
       Nullable(Integer("index_date", "When that index was published, unix s", false))},
      false);
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
       Time("at", "With reason SCHEDULED_START: when the plan starts; else null"),
       Nullable(Integer("percent", "In verifying (the copy of the bundle) and unpacking: how far, 0 to 100; else "
                        "null")),
       Nullable(Integer("bytes", "In verifying and unpacking: the bytes copied or unpacked so far; else null")),
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
       Object("auto", "The automatic update", Auto(true)), UpdateOrigin(),
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
      Text("pause_reason",
           "In PAUSED the reason of STATE that holds the mower: an emergency, a fault, the user (MANUAL), an obstacle "
           "(OBSTACLE), then the position; otherwise empty"),
      Text("pause_kind", "With pause_reason OBSTACLE the kind that held it (PERSON, ANIMAL, OBJECT, UNKNOWN), "
                         "otherwise empty"),
      Switch("emergency", "1 while an emergency stops motion and blade"),
      Integer("emergency_reason", "The bits of the emergency (x-emergency-bits), 0 without one"),
      Switch("is_charging", "1 while the mower charges on the docking station"),
      Switch("rain_detected", "1 while the rain sensor reports rain"),
      Switch("hardware", "1 while the driver of the platform reports, 0 without a platform or while it is silent"),
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
      {22, "INPUTS_LEARNING",
       "The driver was started to learn its inputs (step inputs_assign of the setup): it watches its candidate lines, "
       "none of them stops anything, so it holds the mower until it starts without; no release ends it",
       {{"de",
         {"Eingänge werden zugeordnet",
          "Der Mäher beobachtet gerade seine Eingänge, um sie zuzuordnen. Solange stoppt kein Sensor, deshalb fährt "
          "und mäht er nicht.",
          "Die Zuordnung in der Einrichtung abschließen oder abbrechen. Danach startet MowBite OS neu."}},
        {"en",
         {"Inputs are being assigned",
          "The mower is watching its inputs to assign them. No sensor stops anything meanwhile, so it does not "
          "drive or mow.",
          "Finish or stop the assignment in the setup. MowBite OS restarts after it."}}}},
      {23, "BOARD_NOT_HEARD",
       "The driver opened its port and heard no frame of its board since: it sends nothing there until one comes, so "
       "a mower chosen wrong talks to no other board. The first frame ends it without a release",
       {{"de",
         {"Platine meldet sich nicht",
          "Am Anschluss des gewählten Mähers meldet sich keine passende Platine. MowBite OS sendet nichts dorthin.",
          "Prüfen, ob der richtige Mäher gewählt ist und die Platine Strom hat. Sonst in der Einrichtung einen "
          "anderen Mäher wählen."}},
        {"en",
         {"Board not answering",
          "No board of the chosen mower reports at its port. MowBite OS sends nothing there.",
          "Check that the right mower is chosen and the board has power. Else choose another mower in the setup."}}}},
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
        Integer("api_break", "Break of this interface, the x-api-break of the schema system.api gives"),
        Integer("mqtt", "MQTT version of the bridge at the broker: 5 sends each answer to the response topic of its "
                        "request, 3 sends all to rpc/response"),
        Nullable(Object("stop_release",
                        "Releasing the emergency with STOP at the mower: free for free_s, held for hold_s, let go "
                        "before max_hold_s; null or missing where the platform has no such release",
                        {Number("free_s", "s", "STOP not pressed for this long before"),
                         Number("hold_s", "s", "Held at least this long"),
                         Number("max_hold_s", "s", "Let go before this")},
                        false)),
        Nullable(Object("blade_height",
                        "Cutting heights the platform sets for /mower_logic/mow_height_mm; null or missing where it "
                        "sets none, then the setting has no effect",
                        {Integer("min_mm", "Lowest height"), Integer("max_mm", "Highest height"),
                         Integer("default_mm", "Height of the platform for 0")},
                        false)),
        Nullable(Text("update_path",
                      "How the OS is updated: native as a bundle over mowbite-host, compose as an image by hand; "
                      "null where it does not know",
                      false, {"native", "compose"})),
        Text("names",
             "openmower: sensor ids, /ll/board and the line of emergency reasons in the log as the app of OpenMower "
             "knows them, each sensor with its own id as key; own: only the own ids",
             true, {"openmower", "own"}),
        Nullable(Object("data_rollback",
                        "The last way back of the data to an older backup until data.rollback.confirm or the next "
                        "update; null or missing without one",
                        {Integer("from", "Schema of the data before"),
                         Integer("to", "Schema of the backup that came back"),
                         Text("backup", "Folder in data_dir that keeps the data before"),
                         Integer("backup_of", "When the backup that came back was made, unix s"),
                         Integer("t", "When the way back was, unix s"),
                         Bool("confirm", "The map changed since the update: nothing mows by itself until "
                                         "data.rollback.confirm")},
                        false))},
       "",
       {}},
      {"system.api",
       "This interface as JSON Schema with its version",
       {},
       {Integer("api", "Version of this interface"),
        Integer("api_break", "Break of this interface"),
        {"json_schema", Type::kObject, "", true, "This schema", {}, false, {}, ""}},
       "",
       {}},
      Logic("emergency.reset",
            "Asks the safety node to release the emergency; answers once the emergency ended, or with the refusal of "
            "the safety node or STILL_ACTIVE with the bits that stay, after at most 5 s",
            {}, Answer({Bool("released", "Always true: the emergency ended")}),
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
            "Drives home from IDLE to the docking station", {},
            Answer({}, {"DOCKING"}), "DRIVE_REFUSED", {}, "only where the OS docks by itself"),
      Logic("docking.test",
            "Docks by the dock camera alone from where the mower stands in front of the station, without map or "
            "position (the reason TEST at DOCKING)",
            {Integer("runs",
                     "1 to 10 (default 1): above 1 a series, after each DOCKED the mower drives out forwards to the "
                     "start of the line by the marker and docks again; DOCKING_TEST_RUN ends every run, the first "
                     "failure ends the series without a further try",
                     false)},
            Answer({}, {"DOCKING"}), "DRIVE_REFUSED", {}, "only where the platform has a dock marker"),
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
            "Records a point of the docking station: first in front of it, then on the contacts, where base_link of the "
            "docked mower becomes the position of the station; DOCK_RECORDED follows",
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
       Object("params", "None for all",
              {List("ids", "Sensor ids or keys of x-sensors", Type::kString, false)}, false),
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
             Text("key", "Own id of the sensor (key of x-sensors), the same as sensor_id without openmower_names"),
             Nullable({"value", Type::kAny, "", true, "The last value, a text for a text sensor; null before one came",
                       {}, false, {}, ""}),
             Nullable(Number("stamp", "unix s", "When the last value came, null before one came"))}),
       ""},
      {"logs.recent", "mowbite_bridge", "The last log lines from WARN on, oldest first, logins hidden",
       Object("params", "All optional",
              {Number("since", "unix s", "Only lines after this time", false),
               Text("level", "Lowest level: WARN, ERROR or FATAL; another, also DEBUG or INFO, counts as WARN", false),
               Integer("limit", "Only the newest lines", false)},
              false),
       Rows("result", "A log line",
            {Number("t", "unix s", "When it was logged"), Text("level", "Its level", true, {"WARN", "ERROR", "FATAL"}),
             Text("node", "The node that logged it"), Text("msg", "The text, cut after 2000 bytes")}),
       ""},
      {"events.history", "mowbite_bridge", "The events of one local day as events/json sent them",
       Object("params", "None for today; [\"YYYYMMDD\"] works as well",
              {Text("date", "The day as YYYYMMDD", false)}, false),
       List("result",
            "Events, oldest first; none for an unknown day; over " + std::to_string(kHistoryMaxAnswerBytes) +
                " bytes the oldest of the day are left out",
            Type::kObject),
       ""},
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
               Bool("thinned",
                    "true when only every n-th point is in the answer (first and last of each stretch stay), so it "
                    "stays below " + std::to_string(kHistoryMaxAnswerBytes) + " bytes",
                    false),
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
       "Changes settings of x-settings in IDLE, mow_height_mm alone at any time; all or nothing, a value outside its "
       "range is set to the bound",
       Object("params", "The settings by full name",
              {Object("params", "\"/mower_logic/<name>\": number, for names of x-settings")}),
       Object("result", "The values taken by full name, those outside the range at the bound"), ""},
      {"robot.geometry.get", "mowbite_logic", "The dimensions of the mower, keys of x-geometry", NoParams(),
       GeometryAnswer(), ""},
      {"robot.geometry.set", "mowbite_logic",
       "Changes dimensions in IDLE, all or nothing; GEOMETRY_CHANGED follows. A body that leaves out the drive wheels "
       "(track, wheel_width, wheel_radius) is refused with WHEELS_OUTSIDE; one that leaves out a part of robot_shape "
       "counts, and GEOMETRY_CHANGED warns with SHAPE_OUTSIDE",
       Object("params", "The change",
              {Integer("version", "version of the last robot.geometry.get"),
               Object("settings", "Values by key of x-geometry", {}, false),
               List("reset", "Keys back to the platform's value", Type::kString, false),
               List("confirm", "Keys the user confirms as they are; a key of settings counts as confirmed as well",
                    Type::kString, false)}),
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
               Time("next_start", "The next start of the schedule, null without one"),
               Bool("scheduled_start", "Whether next_start refuses a hold now: within 1800 s or past it")}),
       ""},
      {"data.rollback.confirm", "mowbite_logic",
       "The user checked the map after a way back of the data: the note goes and mowing by itself is free again "
       "(DATA_ROLLBACK confirmed); without a note nothing happens",
       NoParams(), Object("result", "What happened", {Bool("released", "A note was there and went")}), ""},
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
              {Partial(Schedule(true)), Integer("version", "version of the last schedule.get", false)}),
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
      {"journal.status", "mowbite_stats", "What the journal of the OS keeps, and its incidents", NoParams(),
       Object("result", "The journal and its incidents",
              {Bool("enabled", "Whether the OS records a journal (journal_enabled; a change counts from the next start)"),
               Text("problem", "Why no incident is kept right now, empty while they are"),
               Count("bytes", "B", "Size of the journal on the disk, incidents left out"),
               Count("max_bytes", "B", "Its limit: the oldest runs go first"),
               Count("max_days", "d", "Runs older than this go"),
               Count("free", "B", "Space left on its disk"),
               Count("reserve", "B", "Below this much free space nothing is written"),
               Count("runs", "", "Runs of processes kept"),
               Count("active", "", "Runs of processes going on now"),
               Object("incidents", "The incidents",
                      {Count("count", "", "Incidents kept"), Count("bytes", "B", "Their size, each file once"),
                       Count("max_bytes", "B", "Their limit: the oldest that are no safety go first"),
                       Count("max_days", "d", "Incidents older than this go"),
                       Count("max_days_safety", "d", "Safety incidents older than this go"),
                       Rows("throttled",
                            "Triggers that opened no incident since the start because their kind came too often",
                            {Text("kind", "The trigger"), Count("count", "", "How often")})})}),
       ""},
      {"incidents.list", "mowbite_stats", "The incidents kept, newest first", NoParams(),
       Rows("result", "An incident",
            {Text("id", "Its id"), UnixTime("created", "When its first trigger came"),
             Text("state", "collecting while its window is open, sealing until its files are complete, done", true,
                  {"collecting", "sealing", "done"}),
             Bool("safety", "Whether a trigger was about safety: kept longest"),
             List("triggers", "The kinds of its triggers in their order, each once", Type::kString),
             Text("reason", "The reason of its first trigger, empty for none"),
             Count("bytes", "B", "Size of its files"),
             Text("replay", "from_start when every run in it can be replayed, window when only its window is kept", true,
                  {"from_start", "window"})}),
       ""},
      {"incidents.get", "mowbite_stats",
       "The manifest of an incident and its timeline of at most " + std::to_string(kIncidentTimelineBytes) + " bytes",
       Object("params", "The incident", {Client(), Text("id", "Its id")}),
       Object("result", "The incident, or error for an unknown id",
              {Object("manifest", "Triggers, window, runs with versions and files, log lines of the window"),
               List("timeline", "Changes of state, events and calls of the cores in the window, oldest first",
                    Type::kObject)}),
       ""},
      {"setup.get", "mowbite_setup", "The state of the setup program: the steps of the platform with their texts",
       NoParams(), SetupAnswer(), kSetupOffered},
      {"setup.start", "mowbite_setup",
       "Begins a step whose start is true, or repeats one that ended; a step of the check confirm ends with it. "
       "SETUP_STEP follows when it ends",
       Object("params", "The step",
              {SetupStep(), SetupVersion(),
               List("confirm", "The points of confirm[] of the step the user confirmed", Type::kString, false)}),
       SetupAnswer(), kSetupOffered},
      {"setup.answer", "mowbite_setup",
       "Gives the inputs of a step: a choice, a number, the answer to a question, the delays of a step inputs_assign "
       "that is ok",
       Object("params", "The answer",
              {SetupStep(), SetupVersion(), Object("values", "Values by name of inputs[] of the step")}),
       SetupAnswer(), kSetupOffered},
      {"setup.skip", "mowbite_setup",
       "Skips a step that is skippable; it can be done later all the same. A skipped check never opens the lock of "
       "the safety measurement",
       Object("params", "The step and why",
              {SetupStep(), SetupVersion(), Text("reason", "One of skip_reasons[] of the step"),
               Text("note", "Text of the user, needed with other; at most " + std::to_string(kSetupMaxSkipText) +
                                " characters",
                    false)}),
       SetupAnswer(), kSetupOffered},
      {"setup.abort", "mowbite_setup", "Ends the running step with STOPPED; nothing happens without one",
       Object("params", "The step", {Text("step", "The running step; none for whichever runs", false)}, false),
       SetupAnswer(), kSetupOffered},
      {"setup.keep", "mowbite_setup",
       "Keeps a step that moves the wheels going: at least every " +
           std::to_string(static_cast<int>(kSetupKeepTimeout)) + " s" +
           " while it runs, else it stops with RELEASED. Not needed for other steps",
       Object("params", "The step", {SetupStep()}),
       Object("result", "Whether the step still runs", {Bool("running", "It runs; false once it ended")}),
       kSetupOffered},
      {"setup.reset", "mowbite_setup",
       "Takes every result back: all steps are open again, the record starts anew. It takes the date, the stand time "
       "and allow_blade out of the parameter file of the program; a lock the program opened (safety.source program) "
       "closes with a reset until a summary passes again, a date from the file kept by hand stays as it is. The "
       "causes of failed safety steps stay",
       Object("params", "The version", {SetupVersion()}), SetupAnswer(), kSetupOffered},
      {"setup.platforms", "mowbite_setup",
       "The mowers this image runs as, for the step platform: name, texts and image of each, the one chosen and "
       "whether the installation fixed it",
       NoParams(),
       Object("result", "The mowers",
              {Rows("platforms", "The platforms of the image, by name",
                    {Text("name", "Name for setup.platform.set"),
                     Object("texts", "name, text (what it is) and board (what the driver expects at which port) by "
                                     "language (de, en)"),
                     Nullable(SetupImage("Its drawing from above, null without one"))}),
               Text("current", "The platform the OS runs as, empty in the stage of the choice"),
               Bool("fixed", "The installation fixed it (MOWBITE_PLATFORM): setup.platform.set takes no other")}),
       ""},
      {"setup.platform.set", "mowbite_setup",
       "Chooses the mower and restarts the OS as it; refused while a step runs or the mower is not idle. On a mower "
       "that runs as another one every result goes back and the lock closes with a reset (SETUP platform); the same "
       "one again does nothing",
       Object("params", "The mower", {Text("platform", "A name of setup.platforms"), SetupVersion()}),
       Object("result", "What happens", {Bool("restart", "The OS restarts as the mower chosen")}), ""},
      {"setup.image", "mowbite_setup",
       "An image of a step in pieces of at most " + std::to_string(kSetupImageBytes) + " bytes",
       Object("params", "The piece",
              {Text("name", "name of image or image.accent of a step"),
               Count("offset", "B", "Where the piece begins, 0 without"),
               Whole("size", "B", 1, kSetupImageBytes, "Its size at most, the whole image without", false)}),
       Object("result", "The piece",
              {Required(Count("size", "B", "Size of the whole image")),
               Whole("offset", "B", 0, kSetupImageBytes, "Where the piece begins"),
               Text("data", "The piece in base64, empty at the end"), Text("sha256", "sha256 of the whole image"),
               Text("type", "Always image/svg+xml")}),
       kSetupOffered},
      {"incidents.download", "mowbite_stats",
       "An incident as a tar archive in pieces of at most " + std::to_string(kIncidentChunkBytes) +
           " bytes; without location the journal goes without place and with a shifted date",
       Object("params", "The piece",
              {Client(), Text("id", "The incident"), Count("offset", "B", "Where the piece begins in the archive"),
               Whole("size", "B", 1, kIncidentChunkBytes, "Its size at most"),
               Bool("location", "true for the archive with the place of the mower; none or false without", false)}),
       Object("result", "The piece, or error for an unknown id or an incident still collecting",
              {Bool("ready", "false while the archive is made (the first call starts it): no data yet, ask again"),
               Count("size", "B", "Size of the whole archive"), Count("offset", "B", "Where the piece begins"),
               Text("data", "The piece in base64, empty at the end"),
               Text("sha256", "sha256 of the piece"), Text("archive_sha256", "sha256 of the whole archive")}),
       ""},
      {"incidents.mark", "mowbite_stats",
       "Keeps the last 5 min as an incident with a note, as a report of the user",
       Object("params", "The note",
              {Client(), Text("note", "What happened, at most " + std::to_string(kIncidentMaxNote) + " characters")}),
       Object("result", "The incident", {Text("id", "Its id")}), ""},
      {"incidents.delete", "mowbite_stats", "Deletes one incident, without an id all",
       Object("params", "None for all", {Client(), Text("id", "The incident", false)}, false), Deleted(), ""},
      {"sensors.history", "mowbite_stats",
       "Min, mean and max of the number sensors per step over the last " + std::to_string(kSensorsHistoryHours) +
           " h; the answer stays below " + std::to_string(kHistoryMaxAnswerBytes) +
           " bytes: the step grows to " + HistorySteps(1) + ", then the oldest points go",
       Object("params", "None: all number sensors over the last " + std::to_string(kSensorsHistoryHours) + " h",
              {Client(), List("sensor_ids",
                              "Sensor ids or keys, none for all; ids the history does not know are left out",
                              Type::kString, false),
               Count("from", "unix s", "Points from the one that holds this time on; none for " +
                                           std::to_string(kSensorsHistoryHours) + " h before to"),
               Count("to", "unix s", "Points that begin up to then, the one going on included; none for now"),
               Whole("step", "s", 1, kSensorsHistoryMaxStep, "At least so long a point: " + HistorySteps(0), false)},
              false),
       Object("result", "The history",
              {Whole("step", "s", kSensorsHistorySteps[0], kSensorsHistoryMaxStep, "The step taken: " + HistorySteps(0)),
               UnixTime("from", "The points from then on: from rounded down to the step, later when truncated"),
               UnixTime("to", "The points that begin up to then"),
               Bool("truncated", "The oldest points were left out to keep the answer small"),
               Rows("sensors", "Each number sensor asked for that is recorded now or has values kept, by sensor_id",
                    {Text("sensor_id", "Id of x-sensors or of the platform"),
                     Text("key", "Own id of the sensor, the same as sensor_id without openmower_names"),
                     Text("name", "Name to show"),
                     Text("unit", "Unit of the values, empty for none"),
                     List("points",
                          "[t, min, avg, max] per step with values, oldest first: t its start in unix s, avg "
                          "weighted by the values in it; a step without values is left out",
                          Type::kArray)})}),
       ""},
      {"notify.get", "mowbite_net", "The settings of the pushes; of the token only whether there is one", NoParams(),
       NotifyAnswer(), ""},
      {"notify.set", "mowbite_net",
       "Changes settings, all or nothing: -32602 with data {field, reason}. A push says so to the targets of before",
       Object("params", "The change",
              [] {
                std::vector<Field> fields = NotifySettings(false);
                fields.push_back(Text("token",
                                      "New token for the ntfy server; empty drops it, so does a server of another "
                                      "scheme, host or port without a new token",
                                      false));
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
       "(CAMERA_REFUSED) while the camera is not available; -32602 for an id or a protocol it does not have and "
       "for detections true at a camera without",
       Object("params", "The camera",
              {Text("id", "An id of camera.list"),
               Text("protocol", "One of its protocols; its first without", false, {"mjpeg", "webrtc"}),
               Bool("detections",
                    "For this stream: true with the boxes of the camera AI, false without; without, the boxes are "
                    "there when camera.list has detections true",
                    false)}),
       Object("result", "The stream",
              {Text("url", "http address with the token of this connection; its host is the IPv4 of the mower in the "
                    "network of its default route"),
               Text("protocol", "The protocol of url", true, {"mjpeg", "webrtc"}),
               UnixTime("expires", "The connection has to begin before this"),
               Bool("detections", "Whether the boxes of the camera AI are in the picture")}),
       kCameraOffered},
      {"camera.unlock", "mowbite_camera",
       "Deletes the lock a camera keeps after a kernel crash while it ran (reason LOCKED_AFTER_CRASH in camera.list), "
       "so it may stream again. The camera may have caused the crash: unlock it only when it is clear it did not. "
       "Only at rest: IDLE without a sub state, docked or not, with the blade still. Refused (CAMERA_UNLOCK_REFUSED) "
       "for a camera without that lock, away from rest, without an unlock service at the camera, when the camera "
       "could not delete the lock, and after 5 s without an answer (TIMEOUT: camera.list tells whether it worked); "
       "-32602 for an id it does not have. CAMERA_CHANGED follows once the camera reports again",
       Object("params", "The camera", {Text("id", "An id of camera.list")}),
       Object("result", "The lock is deleted",
              {Bool("unlocked", "Always true"),
               Text("reason", "What the lock said of the crash, one line, may be empty")}),
       kCameraOffered},
      {"vision.obstacles.list", "mowbite_obstacles",
       "The obstacles the sensors of the mower saw, kept on the mower only: at most limit.entries, limit.bytes with "
       "the photos and limit.days since last seen, the least recently seen go first. Things of one place (objects "
       "within 0.5 m) or of one kind within 30 s are one entry; OBSTACLE_SEEN tells of a new one",
       Object("params", "Which",
              {Client(), Number("since", "s", "Only the entries last seen after this unix time; all without", false)}),
       Object("result", "The list",
              {Text("mode", "obstacle_mode now", true, ObstacleModes()),
               Text("mode_max",
                    "The highest obstacle_mode proven on this platform; a higher setting acts as this one", true,
                    ObstacleModes()),
               Bool("photos", "obstacle_photos now: whether new entries get a photo"),
               Object("sight", "Whether the sensors see now",
                      {Bool("ok", "All of them see"),
                       Nullable(Text("reason", "Why not, null with ok: a reason of OBSTACLE_SIGHT in x-reasons"))}),
               Object("limit", "What the mower keeps",
                      {Integer("entries", "Entries at most"), Integer("bytes", "Bytes with the photos at most"),
                       Number("days", "d", "Days since last seen at most")}),
               ObstacleRow()}),
       kObstaclesOffered},
      {"vision.obstacles.image", "mowbite_obstacles",
       "The photo of an entry in pieces of at most " + std::to_string(kObstacleImageBytes) +
           " bytes; -32602 for a name the list does not have",
       Object("params", "The piece",
              {Client(), Text("name", "image.name of an entry"), Count("offset", "B", "Where the piece begins, 0 without"),
               Whole("size", "B", 1, kObstacleImageBytes, "Its size at most, the most without", false)}),
       Object("result", "The piece",
              {Count("size", "B", "Size of the whole photo"), Count("offset", "B", "Where the piece begins"),
               Text("data", "The piece in base64, empty at the end"), Text("sha256", "sha256 of the whole photo"),
               Text("type", "Always image/jpeg", true, {"image/jpeg"})}),
       kObstaclesOffered},
      {"vision.obstacles.clear", "mowbite_obstacles",
       "Removes an entry with its photo, or all without id; OBSTACLES_CLEARED tells the other clients. -32602 for an "
       "id the list does not have",
       Object("params", "Which", {Client(), Integer("id", "The entry; all without", false)}),
       Object("result", "What went", {Integer("removed", "Entries removed")}), kObstaclesOffered},
      {"map.replace", "mowbite_map",
       "Replaces the map with the one given, in the shape of map/json. With a second parameter {\"expected_sha256\": "
       "sha256 of the map/json text the client last read} another map stored meanwhile is not replaced (-32001 map "
       "changed meanwhile)",
       List("params", "[map] or [map, {expected_sha256}]", Type::kObject),
       Text("result", "Successfully stored map (n areas)"), ""},
      {"map.area_properties", "mowbite_map",
       "The properties an area of the map may have. mow_height_mm: the cutting height of a mowing area in mm instead of "
       "/mower_logic/mow_height_mm while the blade cuts in it, a change at once; map.replace takes it as params.set takes "
       "that setting, rounded and within its range; no effect where system.info.blade_height is null",
       NoParams(),
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
      {"update.check", "mowbite_host",
       "Looks for bundles in the inbox now and asks the release index on the net in the background, as it does every "
       "6 h; a bundle newer than what runs is downloaded, never installed by that",
       UpdateParams({}), UpdateStatus(), kHostOffered},
      {"update.set_channel", "mowbite_host",
       "Chooses which releases of the index count: stable, or preview for previews as well; then asks the index",
       UpdateParams({Text("channel", "stable or preview", true, {"stable", "preview"})}, true), UpdateStatus(),
       kHostOffered},
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
      {"host.confirm_start", "mowbite_host",
       "Ends the trial period of a newly written system (trial in mowbite/host/status) once its start is good: the "
       "pack of the installer unpacked, a WLAN connection, MowBite OS running, the first installation done; else "
       "START_NOT_OK with what fails, or BUSY while the first installation runs. A call counts as reaching the "
       "mower only while the first installation runs or once the start is good, so a bad start still goes back into "
       "the USB mode on its own. Without a trial period nothing to do",
       Open(Object("params", "Optional",
                   {Text("challenge", "32 to 128 hex digits (0-9, a-f) for proof; INVALID otherwise", false)})),
       Object("result", "The trial period after the call",
              {Bool("trial", "Always false: it is over"),
               Text("proof",
                    "HMAC-SHA256 in hex of the challenge under the install id the installer put on this mower; "
                    "missing without a challenge or an id",
                    false),
               List("notes", "Other services that failed, as \"failed: <unit>\"; missing without", Type::kString,
                    false)}),
       kHostOffered},
      {"host.wifi_scan", "mowbite_host",
       "The WLANs the mower sees now, the one in use first, then by signal; UNSUPPORTED without NetworkManager",
       NoParams(),
       Object("result", "What the mower sees",
              {List("networks", "Each {ssid, signal 0 to 100, secure, in_use}, at most 64", Type::kObject)}),
       kHostOffered},
      {"host.wifi_set", "mowbite_host",
       "Tries the WLAN in the background and answers at once: the mower keeps it only once it connected and any "
       "request reached it over the new WLAN within 5 min, else the connection before comes back (wifi failed, "
       "WIFI_FAILED). The outcome is wifi in mowbite/host/status. The password is never sent back, logged or shown in "
       "a status; INVALID for a name or password out of range, BUSY while a change runs or waits to be reached",
       Object("params", "The WLAN",
              {Text("ssid", "Its name, 1 to 32 bytes", true),
               Text("password", "8 to 63 printable ASCII characters or 64 hex digits; none for an open WLAN", false),
               Bool("hidden", "A WLAN that does not send its name; false without", false)}),
       Object("result", "wifi of mowbite/host/status",
              {Text("state", "trying, connected or failed", true, {"trying", "connected", "failed"}),
               Text("ssid", "The WLAN", true), Nullable(Text("reason", "Why it failed, else null")),
               Nullable(Text("message", "English text for the log, else null")), Integer("t", "unix s", "When")}),
       kHostOffered},
      {"host.usb_mode", "mowbite_host",
       "Tron only: restarts the mower into the USB mode of its loader, where the installer on a PC takes over (back "
       "to the vendor system, or writing again). Only through the gate of a switch without the charging contacts: "
       "a running MowBite OS idles without an emergency and grants its hold, so wheels and blade stay off. An update "
       "that has not begun its switch ends for it (result aborted); during a switch HOLD. UNSUPPORTED on other "
       "mowers",
       NoParams(), Object("result", "Answered before the restart", {Bool("rebooting", "Always true")}), kHostOffered},
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
      {"battery_voltage", "Battery Voltage", "V", false, "logic", "Voltage of the battery", "om_v_battery",
       "V Battery"},
      {"charge_voltage", "Charge Voltage (derived)", "V", false, "logic",
       "The charge voltage while on the contacts, else 0; derived, the board reports none", "om_v_charge",
       "V Charge (dock contact)"},
      {"charge_current", "Charge Current", "A", false, "logic", "Current into the battery", "om_charge_current",
       "Charge Current"},
      {"charge_state", "Charge State (derived)", "", true, "logic",
       "idle, charging or done, derived from contact, current and charge; under om_charge_state Not Charging, Fast "
       "Charge or Done",
       "om_charge_state", "Charge State (derived)"},
      {"blade_rpm", "Blade Speed", "rpm", false, "logic", "Speed of the blade motor", "om_mow_motor_rpm",
       "Mow Motor Revolutions"},
      {"blade_current", "Blade Current", "A", false, "logic", "Current of the blade motor", "om_mow_motor_current",
       "Mow Motor Current"},
      {"position_accuracy", "Position Accuracy", "m", false, "logic", "Expected error of the position",
       "om_gps_accuracy", "GPS Accuracy"},
      {"blade_temp", "Blade Motor Temp", "deg.C", false, "logic", "Temperature of the blade motor",
       "om_mow_motor_temp", "Mow Motor Temp"},
      {"drive_left_temp", "Left Drive Temp", "deg.C", false, "logic", "Temperature of the left motor board",
       "om_left_esc_temp", "Left ESC Temp"},
      {"drive_right_temp", "Right Drive Temp", "deg.C", false, "logic", "Temperature of the right motor board",
       "om_right_esc_temp", "Right ESC Temp"},
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

const SensorEntry* FindOpenMowerSensor(const std::string& openmower_id) {
  for (const auto& sensor : Sensors()) {
    if (!sensor.openmower_id.empty() && sensor.openmower_id == openmower_id) return &sensor;
  }
  return nullptr;
}

const std::vector<std::string>& ObstacleModes() {
  static const std::vector<std::string> modes = {"off", "show", "stop", "avoid"};
  return modes;
}

const std::vector<std::string>& ObstacleKinds() {
  static const std::vector<std::string> kinds = {"PERSON", "ANIMAL", "OBJECT", "UNKNOWN"};
  return kinds;
}

const std::vector<std::string>& ObstacleActions() {
  static const std::vector<std::string> actions = {"seen", "slowed", "would_stop", "stopped", "avoided"};
  return actions;
}

int EffectiveObstacleMode(int setting, int mode_max) {
  return std::clamp(std::min({setting, mode_max, kObstacleModeBuilt}), 0, kObstacleModeBuilt);
}

bool ObstacleLiving(const std::string& kind) {
  return kind == "PERSON" || kind == "ANIMAL";
}

int ObstacleActionRank(const std::string& action) {
  const auto& actions = ObstacleActions();
  const auto it = std::find(actions.begin(), actions.end(), action);
  return it == actions.end() ? -1 : static_cast<int>(it - actions.begin());
}

const std::vector<SettingEntry>& LogicSettings() {
  // Bounds of MowBite OS beyond MowerLogic.cfg: max_position_accuracy at most 0.3 m, gps_timeout at least 1 s,
  // docking_approach_distance and undock_distance 0.5 to 2 m, docking_retry_count at most 5.
  static const std::vector<SettingEntry> settings = {
      {"rain_mode", true, 0, 3, 1, "", "What rain does, as in OpenMower; 1 by default: rain sends the mower home"},
      {"rain_delay_minutes", true, 30, 1440, 30, "min", "How long to wait after rain"},
      {"rain_check_seconds", true, 20, 300, 20, "s", "How long rain has to last to count"},
      {"automatic_mode", true, 0, 2, 0, "",
       "No effect on MowBite OS, kept for clients of OpenMower: only the schedule starts the mower by itself"},
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
      {"mow_height_mm", true, 0, 150, 0, "mm",
       "Cutting height, a change at once also while the blade runs, within blade_height of system.info; 0 = that of "
       "the platform"},
      {"journal_enabled", true, 0, 1, 1, "",
       "Whether the OS keeps a journal of its processes for incidents; a change counts from the next start"},
      {"obstacle_mode", true, 0, 3, 0, "",
       "What obstacles the sensors see do: off; show them in the list and on the map without any effect on driving; "
       "stop: the mower stands with the blade off while one is in the way; avoid: as stop, but an object that stands "
       "3 s is driven around within its area with the blade on, a person or animal never. Above mode_max of "
       "vision.obstacles.list it acts as that",
       ObstacleModes()},
      {"obstacle_photos", true, 0, 1, 1, "",
       "Whether a new entry of the obstacles gets a photo of the whole image, kept on the mower only"},
  };
  return settings;
}

std::optional<double> FitSetting(const std::string& name, double value) {
  for (const auto& setting : LogicSettings()) {
    if (setting.name == name) return std::clamp(setting.integer ? std::round(value) : value, setting.min, setting.max);
  }
  return std::nullopt;
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
      {"wheel_width", false, false, 0.01, 0.3, "m", false,
       "Width of the tread of the drive wheels; the body has to cover the wheels from above"},
      {"dock_contact_x", false, false, -1.5, 1.5, "m", false, "Charging contacts ahead of the drive axle"},
      {"dock_footprint", true, false, 0.0, 0.0, "m", false,
       "Footprint of the docking station as points [x, y] in base_link of the docked mower (x to its front, y left), "
       "each at most 1.5 m away; [] for the planner's own"},
      {"ticks_per_m", false, true, 100.0, 10000.0, "1/m", false, "Wheel ticks per metre"},
      {"imu_roll", false, true, -0.087, 0.087, "rad", true, "Roll of the IMU against the platform's value"},
      {"imu_pitch", false, true, -0.087, 0.087, "rad", true, "Pitch of the IMU against the platform's value"},
      {"robot_shape", false, false, 0.0, 0.0, "m", false,
       "How the mower looks from above, for the app to draw: parts {id, kind, points [x, y], closed} in the order to "
       "draw them, at most " +
           std::to_string(kShapeMaxParts) + " parts, " + std::to_string(kShapeMaxPartPoints) + " points a part and " +
           std::to_string(kShapeMaxPoints) +
           " in all; kind as x-setup.shape_kinds; [] for a platform without one. Shown only: robot_outline stays "
           "the body the mower drives and plans with",
       true},
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
