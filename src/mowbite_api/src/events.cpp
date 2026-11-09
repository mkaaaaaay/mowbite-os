#include <map>

#include "mowbite_api/catalog.h"

namespace mowbite_api {

namespace {

Field Number(std::string name, std::string unit, std::string text, bool required = false) {
  return {std::move(name), Type::kNumber, std::move(unit), required, std::move(text), {}, false, {}, ""};
}

Field Integer(std::string name, std::string unit, std::string text, bool required = false) {
  return {std::move(name), Type::kInteger, std::move(unit), required, std::move(text), {}, false, {}, ""};
}

Field Text(std::string name, std::string text, bool required = false, std::vector<std::string> values = {}) {
  return {std::move(name), Type::kString, "", required, std::move(text), std::move(values), false, {}, ""};
}

Field Bool(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kBoolean, "", required, std::move(text), {}, false, {}, ""};
}

Field List(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kArray, "", required, std::move(text), {}, false, {}, ""};
}

Field Limit(std::string unit) {
  return Number("limit", std::move(unit), "The limit the value went past, in the unit of the value");
}

Field Distance() {
  return Number("distance", "m", "The distance the reason is about");
}

Field Seconds() {
  return Number("seconds", "s", "How long it lasted");
}

Field Temperature() {
  return Number("temperature", "°C", "The temperature it measured");
}

// reason, message, severity and resume as PutReason of the logic writes them.
std::vector<Field> Rated(bool always, std::vector<Field> fields = {}) {
  fields.push_back(Text("reason", "Why: a fixed name in capitals, rated in x-reasons", always));
  fields.push_back(Text("message", "English text for the log and for apps that do not know the reason"));
  fields.push_back(Text("severity", "How bad it is", always, {"info", "warning", "error"}));
  fields.push_back(
      Text("resume", "auto when it goes on by itself, user when it waits for the user", always, {"auto", "user"}));
  return fields;
}

// The tries of docking in DOCKING_RETRY and DOCKING_FAILED, whatever the reason.
std::vector<Field> DockingTries() {
  return Rated(true, {Integer("attempts", "", "Tries of docking so far")});
}

Field Nullable(Field field) {
  field.nullable = true;
  return field;
}

struct ReasonText {
  std::string text;
  std::vector<Field> fields;
  UserTexts texts = {};
};

// What each reason means and the fields it brings, the same in every event that carries it.
const std::map<std::string, ReasonText>& ReasonTexts() {
  static const std::map<std::string, ReasonText> texts = {
      {"ALREADY_THERE", {"The mower is there already", {}}},
      {"APPROACH_FAILED", {"The approach to the docking station failed", {}}},
      {"BATTERY_CRITICAL",
       {"The battery voltage is critically low",
        {Number("battery_voltage", "V", "Voltage of the battery"), Limit("V")}}},
      {"BATTERY_HOT", {"The battery is too hot", {Temperature(), Limit("°C")}}},
      {"BATTERY_LOW",
       {"The battery is too low",
        {Number("battery_voltage", "V", "Voltage of the battery"),
         Integer("battery_percentage", "%", "Charge of the battery"), Limit("V")}}},
      {"BELOW_DEFAULT", {"A dimension of the body is now below the value of the platform: check the measurement", {}}},
      {"BLADE_REFUSED", {"The driver refused to start the blade", {}}},
      {"BLADE_SPINUP_FAILED", {"The blade did not come up to speed", {}}},
      {"BUSY", {"The mower is busy with something else", {Text("state", "What it is busy with")}}},
      {"CALIBRATION_MISSING", {"The calibration program is not running or gave no sign", {Seconds(), Limit("s")}}},
      {"CANCELED", {"The drive was canceled", {}}},
      {"CHANGED",
       {"The area or path is another one than the client saw: nothing was skipped",
        {Integer("area", "", "current_area now"), Integer("path", "", "current_path now")}}},
      {"CHECKSUM",
       {"The file that came has another sha256 than the one given",
        {},
        {{"de",
          {"Prüfsumme falsch", "Die angekommene Datei passt nicht zu ihrer Prüfsumme, unterwegs ging etwas verloren.",
           "Die Datei noch einmal hochladen."}},
         {"en",
          {"Checksum mismatch", "The file that arrived does not match its checksum, something got lost on the way.",
           "Upload the file again."}}}}},
      {"CORRELATION_DATA",
       {"The correlation data of the request is longer than 1024 bytes: nothing was done",
        {Limit("bytes")}}},
      {"CROSS_TRACK", {"The mower got too far off its path", {}}},
      {"DARK",
       {"The sun has set",
        {Number("sun_elevation", "deg", "Height of the sun over the horizon at the garden"), Limit("deg")}}},
      {"DOCK_TOO_CLOSE", {"The two points of the docking station are too close together", {Distance(), Limit("m")}}},
      {"DOCK_UNREACHABLE",
       {"The docking station is outside of the areas the mower may drive in", {Distance()}}},
      {"DONE", {"The drive arrived", {}}},
      {"DRIVER_MISSING", {"The driver of the platform does not answer", {}}},
      {"EMERGENCY", {"An emergency stops the mower", {}}},
      {"FOLLOWER_MISSING", {"The path follower does not answer", {Seconds(), Limit("s")}}},
      {"FOLLOWER_REFUSED", {"The path follower refused the path", {}}},
      {"FOREIGN_HOST", {"Another host talks to the board", {}}},
      {"GAP", {"Back on GPS after a gap", {Distance()}}},
      {"GNSS_RATE",
       {"The GNSS reports too seldom", {Number("rate", "1/s", "How often it reports"), Limit("1/s")}}},
      {"GYRO_SIGN", {"The gyro turns the other way than the wheels", {}}},
      {"HEADING", {"The heading is not good enough", {}}},
      {"HOLD",
       {"An update holds the mower: nothing starts until it is done",
        {Integer("until", "unix s", "When the hold of the logic ends; none for the hold of the driver")}}},
      {"IMU_GRAVITY",
       {"The IMU does not measure gravity as it should",
        {Number("gravity", "m/s2", "The gravity it measures"), Number("tilt", "deg", "The tilt it measures")}}},
      {"INVALID", {"The path or the request was not valid", {}}},
      {"KERNEL_CRASH",
       {"The kernel of the mower crashed; it holds the mower in every state",
        {},
        {{"de",
          {"Kernel-Absturz",
           "Das Betriebssystem des Mähers ist abgestürzt und neu gestartet. Der Mäher fährt und mäht nicht, bis er "
           "freigegeben wird, gleich was er vorher tat.",
           "Mäher und Umgebung ansehen, dann den Notaus zurücksetzen. Kommt es öfter vor, die Protokolle für eine "
           "Meldung sichern."}},
         {"en",
          {"Kernel crash",
           "The operating system of the mower crashed and started again. The mower does not drive or mow until it "
           "is released, whatever it did before.",
           "Look at the mower and around it, then reset the emergency. If it happens again, keep the logs for a "
           "report."}}}}},
      {"LATCH_CHANGED",
       {"Another reason latched after STOP was pressed: the driver kept the emergency",
        {},
        {{"de",
          {"Neuer Notaus-Grund", "Nach dem Drücken von STOP kam ein neuer Notaus-Grund dazu, der Notaus bleibt.",
           "Den Mäher prüfen und noch einmal lösen."}},
         {"en",
          {"New emergency reason", "Another emergency reason came up after STOP was pressed, the emergency stays.",
           "Check the mower and release again."}}}}},
      {"LEFT_LANE", {"The mower left its lane", {}}},
      {"MANUAL", {"By the user", {}}},
      {"MAP_MISMATCH",
       {"The map does not fit the datum in use",
        {Text("datum_state", "unbound when the map has no recorded datum", false, {"unbound"})}}},
      {"MAP_UNKNOWN", {"The datum of the GNSS is not known yet", {}}},
      {"MAP_UNREADABLE", {"The map cannot be read", {}}},
      {"MOTOR_HOT", {"The blade motor is too hot", {Temperature(), Limit("°C")}}},
      {"NOT_ALLOWED", {"Outside of the areas the mower may drive in", {}}},
      {"NOT_MOVING", {"The mower does not move", {}}},
      {"NOT_MP3",
       {"The file is no MP3 the mower can play",
        {},
        {{"de", {"Keine MP3", "Die Datei ist keine MP3, die der Mäher abspielen kann.", "Eine MP3-Datei wählen."}},
         {"en", {"Not an MP3", "The file is not an MP3 the mower can play.", "Choose an MP3 file."}}}}},
      {"NOT_TURNING", {"The mower does not turn", {}}},
      {"NO_ANSWER", {"No answer in time", {}}},
      {"OS_CRASH",
       {"MowBite OS ended without its own stop in this boot; it holds the mower after a state other than IDLE",
        {},
        {{"de",
          {"MowBite OS abgestürzt",
           "MowBite OS wurde ohne eigenen Stopp beendet und neu gestartet (Absturz, beendet, Speicher voll). Lief "
           "gerade eine Arbeit, wartet der Mäher auf die Freigabe; im Leerlauf geht es von selbst weiter.",
           "Mäher und Umgebung ansehen, dann den Notaus zurücksetzen. Kommt es öfter vor, das Protokoll für eine "
           "Meldung sichern."}},
         {"en",
          {"MowBite OS crashed",
           "MowBite OS ended without its own stop and started again (crash, killed, out of memory). If it was "
           "working, the mower waits for a release; in IDLE it goes on by itself.",
           "Look at the mower and around it, then reset the emergency. If it happens again, keep the log for a "
           "report."}}}}},
      {"NO_CHARGE_CURRENT",
       {"Contact with the docking station, but no charge current",
        {Number("current", "A", "The charge current seen; while docking the largest one of the check"), Limit("A")}}},
      {"NO_DOCK", {"The map has no docking station", {}}},
      {"NO_FILE",
       {"The situation has no sound",
        {},
        {{"de", {"Kein Ton", "Für diese Situation hat der Mäher keinen Ton.", "Eine Situation mit Ton wählen."}},
         {"en", {"No sound", "The mower has no sound for this situation.", "Choose a situation that has a sound."}}}}},
      {"NO_DOCK_CONTACT", {"No contact with the docking station", {}}},
      {"NO_HEADING", {"The heading is not known yet", {}}},
      {"NO_JOB", {"There is no unfinished job", {}}},
      {"NO_MOWING_AREA", {"The map has no active mowing area", {}}},
      {"NO_OUTLINE", {"There is no outline yet", {}}},
      {"NO_POSITION", {"No position from the localization", {}}},
      {"NO_ROOM",
       {"No room for a calibration drive here",
        {Number("free_length", "m", "The free length in front of the mower"),
         Number("length", "m", "The length the drive needs"), Number("strip_length", "m", "Length of the strip"),
         Number("strip_width", "m", "Width of the strip")}}},
      {"NO_RTK", {"No RTK fix", {Limit("m")}}},
      {"NO_SPACE",
       {"No room for the file: all sounds together have a limit, and the journal keeps its reserve free",
        {},
        {{"de",
          {"Kein Platz", "Auf dem Mäher ist kein Platz mehr für diesen Ton.",
           "Einen anderen eigenen Ton löschen oder eine kleinere Datei wählen."}},
         {"en",
          {"No space", "The mower has no space left for this sound.",
           "Delete another sound of your own or choose a smaller file."}}}}},
      {"NO_SAFETY_MEASUREMENT", {"This platform has no safety measurement yet, so it does not drive by itself", {}}},
      {"ON_DOCK", {"The mower is on the docking station", {Distance(), Limit("m")}}},
      {"OUTLIERS", {"Back on GPS after outliers", {Distance()}}},
      {"OUTSIDE", {"The mower left the allowed area", {}}},
      {"OVERSPEED", {"The mower drove faster than it should", {}}},
      {"PART_ORDER",
       {"A part of the upload came out of order",
        {Integer("expected", "", "The part that comes next")},
        {{"de",
          {"Teile durcheinander", "Ein Teil der Datei kam nicht in der richtigen Reihenfolge an.",
           "Die Datei noch einmal hochladen."}},
         {"en",
          {"Parts out of order", "A part of the file did not arrive in the right order.",
           "Upload the file again."}}}}},
      {"PLANNER_MISSING", {"The planner is not running", {}}},
      {"PLANNER_TIMEOUT", {"The planner took too long", {}}},
      {"PLANNING_FAILED", {"No plan for the area", {}}},
      {"POINT_TOO_CLOSE", {"The point is too close to the last one", {Distance(), Limit("m")}}},
      {"POSE_LOST", {"The position stopped coming", {}}},
      {"POSITION_ACCURACY",
       {"The position is not accurate enough",
        {Number("accuracy", "m", "Expected error of the position"), Limit("m")}}},
      {"POWER_LOSS",
       {"The mower started again while MowBite OS ran (power, reset, power key, hang); it holds the mower after a "
        "state other than IDLE",
        {},
        {{"de",
          {"Unterbrechung während der Arbeit",
           "Der Mäher ist neu gestartet, während MowBite OS lief: Strom weg, Akku raus, harter Reset, Ein-Taste "
           "gehalten oder ein Hänger. Jede solche Unterbrechung während der Arbeit sperrt den Mäher bis zur "
           "Freigabe. Das ist Absicht, kein Fehler.",
           "Mäher und Umgebung ansehen, dann den Notaus zurücksetzen. Im Leerlauf sperrt eine Unterbrechung "
           "nicht."}},
         {"en",
          {"Interrupted while working",
           "The mower started again while MowBite OS ran: power cut, battery out, hard reset, power key held or a "
           "hang. Any such interruption during work holds the mower until it is released. This is on purpose, not a "
           "fault.",
           "Look at the mower and around it, then reset the emergency. In IDLE an interruption holds nothing."}}}}},
      {"RAIN", {"It rains", {}}},
      {"REFERENCE_MISMATCH", {"The docking station is not where the map has it", {Distance(), Limit("m")}}},
      {"RESTART_HOLD_STAYS", {"The hold after an unexpected restart could not be removed from data_dir", {}}},
      {"RESPONSE_TOPIC",
       {"The response topic of the request is not rpc/response/<token> with a token of 1 to 64 of A-Z, a-z, 0-9, "
        "dot, underscore and hyphen: nothing was done",
        {}}},
      {"RING_CROSSES_ITSELF", {"The ring crosses itself", {}}},
      {"RING_FEW_POINTS", {"The ring has too few points", {Integer("points", "", "Points of the ring"), Limit("")}}},
      {"RING_TOO_SMALL",
       {"The ring encloses too small an area", {Number("area", "m2", "Area of the ring"), Limit("m2")}}},
      {"SAFETY_MISSING", {"The safety node does not answer", {}}},
      {"SAVE_FAILED",
       {"The map could not be saved",
        {Text("code", "What went wrong", false, {"INVALID", "DATUM", "TOO_LARGE", "STORAGE", "NO_ANSWER"})}}},
      {"SENSOR_MISSING",
       {"A sensor the calibration needs does not report",
        {Number("rate", "1/s", "How often it reports"), Limit("1/s")}}},
      {"SLOPE", {"The ground is too steep", {Number("slope", "deg", "Slope of the ground"), Limit("deg")}}},
      {"STILL_ACTIVE",
       {"The release was taken, but the emergency did not end within 2 s",
        {List("bits", "Names of the emergency bits that stay (x-emergency-bits), UNKNOWN for one without a name",
              false)},
        {{"de",
          {"Notaus besteht weiter",
           "Der Mäher hat das Lösen angenommen, aber der Notaus besteht weiter, etwa weil er angehoben ist oder STOP "
           "noch gedrückt ist.",
           "Den Mäher abstellen, STOP loslassen und noch einmal lösen."}},
         {"en",
          {"Emergency still active",
           "The mower took the release, but the emergency stays, for example because it is lifted or STOP is still "
           "pressed.",
           "Put the mower down, let go of STOP and release again."}}}}},
      {"STILL_SAVING", {"The last change of the map is still being saved", {}}},
      {"STILL_STOPPING", {"The last drive is still stopping", {}}},
      {"STILL_TILTED", {"The mower is still tilted", {Number("angle", "deg", "Its tilt"), Limit("deg")}}},
      {"STOPPED", {"Stopped by the app or by the logic", {}}},
      {"TILT_UNKNOWN", {"The tilt of the mower is not known", {Seconds()}}},
      {"TIMEOUT", {"It took too long", {}}},
      {"TOO_LARGE",
       {"The file or a part of it is too large",
        {Limit("bytes")},
        {{"de", {"Datei zu groß", "Die Datei ist zu groß, erlaubt sind 2 MB.", "Eine kleinere Datei wählen."}},
         {"en", {"File too large", "The file is too large, 2 MB are allowed.", "Choose a smaller file."}}}}},
      {"TOO_LONG",
       {"The sound is too long",
        {Number("seconds", "s", "How long the sound lasts"), Limit("s")},
        {{"de", {"Ton zu lang", "Der Ton ist länger als 30 Sekunden.", "Einen kürzeren Ton wählen."}},
         {"en", {"Sound too long", "The sound is longer than 30 seconds.", "Choose a shorter sound."}}}}},
      {"TOO_MANY_POINTS", {"The ring has too many points", {Limit("")}}},
      {"TRANSIT_FAILED", {"No way to the goal", {}}},
      {"WRONG_STATE",
       {"The command does not fit the state of the mower; its action would not be enabled",
        {Text("state", "current_state of robot_state"), Text("sub_state", "current_sub_state of robot_state")}}},
      {"approach_failed", {"The approach to the docking station failed (value of OpenMower)", {}}},
      {"dock_failed", {"The last part onto the contacts failed (value of OpenMower)", {}}},
      {"no_gps", {"No GPS for the way off the docking station (value of OpenMower)", {}}},
      {"path_failed", {"The way off the docking station failed (value of OpenMower)", {}}},
      {"LOST", {"No corrections from the NTRIP caster", {Limit("s")}}},
      {"RESTORED", {"Corrections from the NTRIP caster again", {}}},
      {"REJECTED", {"The NTRIP caster refused the login", {Integer("code", "", "The HTTP status of the caster")}}},
      {"SOURCE_CHANGED",
       {"Corrections from another caster or mountpoint than before",
        {Text("host", "The caster now in use"), Integer("port", "", "Its port"),
         Text("mountpoint", "Its mountpoint"), Text("previous_host", "The caster used before"),
         Integer("previous_port", "", "Its port"), Text("previous_mountpoint", "Its mountpoint")}}},
  };
  return texts;
}

ReasonEntry R(const std::string& code, std::string severity, std::string resume, Json when = Json::object()) {
  const ReasonText& text = ReasonTexts().at(code);
  return {code, std::move(severity), std::move(resume), text.text, text.fields, std::move(when), text.texts};
}

// A reason with texts of its own in one event, where the shared ones would not fit.
ReasonEntry WithTexts(ReasonEntry reason, UserTexts texts) {
  reason.texts = std::move(texts);
  return reason;
}

// A list of reasons that all wait for the user: codes with their severity.
std::vector<ReasonEntry> User(const std::vector<std::pair<std::string, std::string>>& codes) {
  std::vector<ReasonEntry> out;
  for (const auto& [code, severity] : codes) out.push_back(R(code, severity, "user"));
  return out;
}

EventEntry Event(std::string type, std::string emitter, std::string text, std::vector<Field> fields = {},
                 std::vector<ReasonEntry> reasons = {}, std::string other_severity = "warning",
                 std::string other_resume = "user") {
  return {std::move(type),   std::move(emitter),        std::move(text),        std::move(fields),
          std::move(reasons), std::move(other_severity), std::move(other_resume)};
}

const std::vector<std::string> kRecordingActions = {"start_area_recording",
                                                    "start_recording",
                                                    "stop_recording",
                                                    "finish_navigation_area",
                                                    "finish_mowing_area",
                                                    "finish_discard",
                                                    "record_dock",
                                                    "auto_point_collecting_enable",
                                                    "auto_point_collecting_disable",
                                                    "collect_point",
                                                    "start_manual_mowing",
                                                    "stop_manual_mowing",
                                                    "point"};

}  // namespace

const std::vector<EventEntry>& Events() {
  static const std::vector<EventEntry> events = {
      Event("BOOTED", "bridge", "The OS started; sent again with the same id after the first connect"),
      Event("SHUTDOWN", "bridge", "The OS stops"),
      Event("STATE", "logic", "The state changed; a pause carries its reason",
            Rated(false, {Text("state", "The new state", true,
                               {"IDLE", "AREA_RECORDING", "MOWING", "PAUSED", "UNDOCKING", "DOCKING"}),
                          Text("sub_state", "What IDLE does: a drive or a calibration drive", false,
                               {"DRIVING", "CALIBRATING"})}),
            {R("MANUAL", "info", "user"), R("NO_POSITION", "warning", "auto"), R("POSE_LOST", "warning", "auto"),
             R("POSITION_ACCURACY", "warning", "auto"), R("HEADING", "warning", "auto"),
             R("NO_HEADING", "warning", "auto"), R("EMERGENCY", "error", "user"),
             R("NO_SAFETY_MEASUREMENT", "error", "user"), R("MAP_UNKNOWN", "error", "user"),
             R("NO_MOWING_AREA", "error", "user"), R("MAP_MISMATCH", "error", "user"),
             R("MAP_UNREADABLE", "error", "user"), R("BLADE_SPINUP_FAILED", "error", "user"),
             R("BLADE_REFUSED", "error", "user"), R("PLANNING_FAILED", "error", "user"), R("INVALID", "error", "user"),
             R("NOT_ALLOWED", "error", "user"), R("CROSS_TRACK", "error", "user"), R("OUTSIDE", "error", "user"),
             R("TIMEOUT", "error", "user"), R("APPROACH_FAILED", "error", "user"),
             R("FOLLOWER_MISSING", "error", "user"), R("BATTERY_HOT", "warning", "user"),
             R("MOTOR_HOT", "warning", "user"), R("RAIN", "info", "user"), R("DARK", "info", "user")},
            "error", "user"),
      Event("MOWING_REFUSED", "logic", "A start of mowing was refused", Rated(true),
            User({{"BUSY", "info"},
                  {"HOLD", "info"},
                  {"RAIN", "info"},
                  {"ON_DOCK", "info"},
                  {"STILL_STOPPING", "info"},
                  {"EMERGENCY", "error"},
                  {"NO_SAFETY_MEASUREMENT", "error"},
                  {"MAP_UNKNOWN", "error"},
                  {"MAP_MISMATCH", "error"},
                  {"MAP_UNREADABLE", "error"},
                  {"PLANNER_MISSING", "error"},
                  {"FOLLOWER_MISSING", "error"},
                  {"BATTERY_LOW", "warning"},
                  {"NO_POSITION", "warning"},
                  {"POSITION_ACCURACY", "warning"},
                  {"NO_HEADING", "warning"},
                  {"NO_MOWING_AREA", "warning"},
                  {"NO_DOCK", "warning"},
                  {"DOCK_UNREACHABLE", "warning"},
                  {"NO_DOCK_CONTACT", "warning"},
                  {"REFERENCE_MISMATCH", "warning"},
                  {"BATTERY_HOT", "warning"},
                  {"MOTOR_HOT", "warning"}})),
      Event("DRIVE_REFUSED", "logic", "A drive (to a point or home) was refused", Rated(true),
            User({{"BUSY", "info"},
                  {"HOLD", "info"},
                  {"ON_DOCK", "info"},
                  {"STILL_STOPPING", "info"},
                  {"ALREADY_THERE", "info"},
                  {"EMERGENCY", "error"},
                  {"NO_SAFETY_MEASUREMENT", "error"},
                  {"MAP_UNKNOWN", "error"},
                  {"MAP_MISMATCH", "error"},
                  {"MAP_UNREADABLE", "error"},
                  {"PLANNER_MISSING", "error"},
                  {"FOLLOWER_MISSING", "error"},
                  {"NO_POSITION", "warning"},
                  {"POSITION_ACCURACY", "warning"},
                  {"NO_HEADING", "warning"},
                  {"NO_MOWING_AREA", "warning"},
                  {"NO_DOCK", "warning"},
                  {"DOCK_UNREACHABLE", "warning"},
                  {"NO_DOCK_CONTACT", "warning"},
                  {"REFERENCE_MISMATCH", "warning"}})),
      Event("RECORDING_REFUSED", "logic", "A step of the recording of an area was refused",
            Rated(true, {Text("action", "The step that was refused; point for an automatic point", true,
                              kRecordingActions)}),
            [] {
              // An automatic point is taken again by itself once the position is accurate.
              std::vector<ReasonEntry> reasons = {R("POSITION_ACCURACY", "warning", "auto", {{"action", "point"}})};
              for (auto& reason : User({{"HOLD", "info"},
                                        {"STILL_SAVING", "info"},
                                        {"EMERGENCY", "error"},
                                        {"MAP_UNKNOWN", "error"},
                                        {"MAP_MISMATCH", "error"},
                                        {"SAVE_FAILED", "error"},
                                        {"NO_POSITION", "warning"},
                                        {"POSITION_ACCURACY", "warning"},
                                        {"NO_HEADING", "warning"},
                                        {"NO_DOCK_CONTACT", "warning"},
                                        {"DOCK_TOO_CLOSE", "warning"},
                                        {"NO_OUTLINE", "warning"},
                                        {"RING_FEW_POINTS", "warning"},
                                        {"RING_CROSSES_ITSELF", "warning"},
                                        {"RING_TOO_SMALL", "warning"},
                                        {"TOO_MANY_POINTS", "warning"},
                                        {"POINT_TOO_CLOSE", "warning"}})) {
                reasons.push_back(std::move(reason));
              }
              return reasons;
            }()),
      Event("CALIBRATION_REFUSED", "logic", "A start of the calibration drive was refused", Rated(true),
            User({{"BUSY", "info"},
                  {"HOLD", "info"},
                  {"ON_DOCK", "info"},
                  {"STILL_STOPPING", "info"},
                  {"EMERGENCY", "error"},
                  {"NO_SAFETY_MEASUREMENT", "error"},
                  {"MAP_UNKNOWN", "error"},
                  {"MAP_MISMATCH", "error"},
                  {"MAP_UNREADABLE", "error"},
                  {"CALIBRATION_MISSING", "error"},
                  {"SENSOR_MISSING", "error"},
                  {"IMU_GRAVITY", "error"},
                  {"NO_POSITION", "warning"},
                  {"POSITION_ACCURACY", "warning"},
                  {"NO_HEADING", "warning"},
                  {"BATTERY_LOW", "warning"},
                  {"NO_MOWING_AREA", "warning"},
                  {"NO_ROOM", "warning"},
                  {"NO_RTK", "warning"},
                  {"GNSS_RATE", "warning"},
                  {"SLOPE", "warning"}})),
      Event("CALIBRATION", "logic", "A phase of the calibration drive began",
            {Text("phase", "The phase", true,
                  {"start", "align", "stand", "line", "spin_left", "spin_right", "evaluate", "end"}),
             Integer("index", "", "Which round of the phase", true)}),
      Event("CALIBRATED", "logic", "The calibration drive ended with its result",
            {Text("status", "How far the evaluation got", true, {"done", "not_evaluated", "failed"}),
             Text("summary", "English text of the result", true),
             Text("severity", "info when evaluated, else warning", true, {"info", "warning"})}),
      Event("CALIBRATION_FAILED", "logic", "The calibration drive stopped before its end", Rated(true),
            User({{"STOPPED", "info"},
                  {"POSE_LOST", "warning"},
                  {"BATTERY_LOW", "warning"},
                  {"NO_RTK", "warning"},
                  {"NO_ROOM", "warning"},
                  {"SLOPE", "warning"},
                  {"EMERGENCY", "error"},
                  {"NO_SAFETY_MEASUREMENT", "error"},
                  {"CALIBRATION_MISSING", "error"},
                  {"SENSOR_MISSING", "error"},
                  {"IMU_GRAVITY", "error"},
                  {"TIMEOUT", "error"},
                  {"GYRO_SIGN", "error"},
                  {"NOT_TURNING", "error"},
                  {"OVERSPEED", "error"},
                  {"LEFT_LANE", "error"},
                  {"NOT_MOVING", "error"},
                  {"BUSY", "error"}}),
            "error", "user"),
      Event("DRIVE", "logic", "A drive to a point began or ended; the end carries how",
            Rated(false, {Text("phase", "Begin or end of the drive", true, {"start", "end"}),
                          Number("goal_x", "m", "The goal, east of the datum of the map"),
                          Number("goal_y", "m", "The goal, north of the datum of the map"),
                          Bool("arrived", "Whether the mower got there", false)}),
            User({{"DONE", "info"},
                  {"ALREADY_THERE", "info"},
                  {"STOPPED", "info"},
                  {"CANCELED", "info"},
                  {"POSE_LOST", "warning"},
                  {"POSITION_ACCURACY", "warning"},
                  {"HEADING", "warning"},
                  {"TRANSIT_FAILED", "error"},
                  {"FOLLOWER_REFUSED", "error"},
                  {"FOLLOWER_MISSING", "error"},
                  {"PLANNER_TIMEOUT", "error"},
                  {"PLANNER_MISSING", "error"},
                  {"EMERGENCY", "error"},
                  {"NOT_ALLOWED", "error"},
                  {"INVALID", "error"},
                  {"CROSS_TRACK", "error"},
                  {"OUTSIDE", "error"},
                  {"TIMEOUT", "error"},
                  {"MAP_UNKNOWN", "error"},
                  {"NO_MOWING_AREA", "error"},
                  {"MAP_MISMATCH", "error"},
                  {"MAP_UNREADABLE", "error"}}),
            "error", "user"),
      Event("POSITION_JUMP", "logic", "The position jumped back onto GPS", Rated(true),
            {R("OUTLIERS", "warning", "auto"), R("GAP", "warning", "auto")}, "warning", "auto"),
      Event("EMERGENCY", "logic", "The emergency began or ended",
            {Bool("emergency", "Whether an emergency stops the mower now"),
             Integer("reason", "", "The bits of the emergency (x-emergency-bits), 0 once it ended", true),
             Text("severity", "error while it lasts, info once it ended", true, {"info", "error"}),
             Text("resume", "user while it lasts, auto once it ended", true, {"auto", "user"}),
             Text("source", "Who released it: rpc for a command over MQTT (action or RPC), button; none when not known",
                  false, {"rpc", "button"})}),
      Event("EMERGENCY_RESET_REFUSED", "logic", "A reset of the emergency was refused",
            Rated(true, {Text("source", "Who asked: rpc for a command over MQTT (action or RPC), button for STOP at "
                                        "the mower; none when not known",
                              false, {"rpc", "button"})}),
            User({{"STILL_ACTIVE", "warning"},
                  {"LATCH_CHANGED", "warning"},
                  {"STILL_TILTED", "warning"},
                  {"TILT_UNKNOWN", "error"},
                  {"FOREIGN_HOST", "error"},
                  {"DRIVER_MISSING", "error"},
                  {"NO_ANSWER", "error"},
                  {"SAFETY_MISSING", "error"},
                  {"RESTART_HOLD_STAYS", "error"}}),
            "error", "user"),
      Event("UNEXPECTED_RESTART", "logic",
            "MowBite OS started after an end that was not its own stop, or after a kernel crash; hold tells whether "
            "the mower waits for emergency.reset (emergency bit UNEXPECTED_RESTART)",
            Rated(true,
                  {Text("cause", "What ended the OS before, the same as reason", true,
                        {"KERNEL_CRASH", "OS_CRASH", "POWER_LOSS"}),
                   Text("detail", "English text of what is known: the line of the kernel or what the OS found", true),
                   Nullable(Integer("crashed_at", "unix s", "When the kernel crashed, null when not known", true)),
                   Text("state_before",
                        "The state before: current_state, DRIVING or CALIBRATING for IDLE with a drive; empty when "
                        "not known",
                        true),
                   Nullable(Text("job_id_before", "The job of mowing then, null without one", true)),
                   Bool("hold", "The mower is held until a release, also by a hold from before")}),
            {R("OS_CRASH", "warning", "auto", {{"hold", false}}), R("POWER_LOSS", "warning", "auto", {{"hold", false}}),
             R("KERNEL_CRASH", "error", "user"), R("OS_CRASH", "error", "user"), R("POWER_LOSS", "error", "user")},
            "error", "user"),
      Event("BLADES", "logic", "The blade started or stopped, as its speed shows",
            {Bool("enabled", "Whether the blade runs")}),
      Event("BATTERY_LOW", "logic", "The battery is low and the mower cannot go home: it stays where it is",
            {Number("battery_voltage", "V", "Voltage of the battery", true),
             Number("limit", "V", "The voltage it fell below", true)}),
      Event("AREA", "logic", "Mowing began in an area",
            {Text("area_id", "Id of the area", true), Text("area_name", "Name of the area", true)}),
      Event("AREA_SKIPPED", "logic", "An area was skipped",
            Rated(true, {Text("area_id", "Id of the area", true), Text("area_name", "Name of the area", true)}),
            {R("MANUAL", "info", "auto")}, "info", "auto"),
      Event("JOB_COMPLETE", "logic", "The job of mowing is done"),
      Event("JOB_RESET", "logic", "The job of mowing was dropped; job_id names the dropped one"),
      Event("MOW_MOTOR_SPINUP_FAILED", "logic", "The blade did not come up to speed; mowing pauses"),
      Event("UNDOCKED", "logic", "The mower left the docking station"),
      Event("UNDOCKING_FAILED", "logic", "Leaving the docking station failed; the mower stays where it stopped",
            Rated(true), User({{"path_failed", "error"}, {"no_gps", "error"}}), "error", "user"),
      Event("DOCKING", "logic", "The mower goes home to the docking station; the reason tells why", Rated(true),
            {R("MANUAL", "info", "user"), R("BATTERY_CRITICAL", "warning", "user"), R("BATTERY_HOT", "warning", "user"),
             R("BATTERY_LOW", "info", "user"), R("MOTOR_HOT", "warning", "user"), R("RAIN", "info", "user"),
             R("DARK", "info", "user")},
            "info", "user"),
      Event("DOCKED", "logic", "The mower is on the contacts of the docking station"),
      Event("DOCKING_RETRY", "logic", "Docking failed, the next try follows by itself", DockingTries(),
            {R("approach_failed", "warning", "auto"), R("dock_failed", "warning", "auto"),
             R("NO_CHARGE_CURRENT", "warning", "auto")},
            "warning", "auto"),
      Event("DOCKING_FAILED", "logic", "Docking failed for good; the mower waits", DockingTries(),
            User({{"approach_failed", "error"},
                  {"dock_failed", "error"},
                  {"NO_CHARGE_CURRENT", "error"},
                  {"NO_POSITION", "error"},
                  {"NO_HEADING", "error"},
                  {"POSITION_ACCURACY", "error"}}),
            "error", "user"),
      Event("CHARGING_LOST", "logic", "Charging stopped while the mower stood on the docking station; it stays there",
            Rated(true, {Integer("battery_percentage", "%", "Charge of the battery")}),
            User({{"NO_DOCK_CONTACT", "warning"}, {"NO_CHARGE_CURRENT", "warning"}}), "warning", "user"),
      Event("DOCK_RECORDED", "logic", "The docking station was recorded",
            {Text("dock_id", "Id of the docking station", true),
             Number("heading", "rad", "The heading it drives in with, counter-clockwise from east", true)}),
      Event("AREA_RECORDED", "logic", "An area was recorded and saved",
            {Text("area_id", "Id of the area", true),
             Text("area_type", "mow for a mowing area, nav for a navigation area", true, {"mow", "nav"}),
             Integer("obstacles", "", "Obstacles in it", true), Integer("points", "", "Points of its outline", true),
             Number("area_m2", "m2", "Its area", true)}),
      Event("GEOMETRY_CHANGED", "logic", "The dimensions of the mower changed",
            {List("changed", "The keys whose value is another now"),
             List("below_default", "Keys of the body now below the value of the platform"),
             Integer("version", "", "Version of the dimensions after the change", true),
             Text("reason", "BELOW_DEFAULT when below_default is not empty"),
             Text("severity", "warning with BELOW_DEFAULT, else info", true, {"info", "warning"}),
             Text("message", "English text naming the keys", true)},
            {R("BELOW_DEFAULT", "warning", "user")}),
      Event("MAP_DATUM", "map", "The map does not fit the datum of the GNSS: no mowing",
            Rated(true, {Text("state", "How it does not fit", true, {"mismatch", "unbound"})}),
            User({{"MAP_MISMATCH", "error"}}), "error", "user"),
      Event("CORRECTIONS", "ntrip", "The RTK corrections from the NTRIP caster changed",
            {Text("reason", "What changed (x-reasons)", true), Text("message", "English text", true),
             Text("severity", "How bad it is", true, {"info", "warning", "error"}),
             Text("resume", "auto when it goes on by itself, user when it waits for the user", false,
                  {"auto", "user"})},
            {R("LOST", "warning", "auto"), R("RESTORED", "info", "auto"), R("REJECTED", "error", "user"),
             R("SOURCE_CHANGED", "warning", "auto")}),
  };
  return events;
}

std::vector<ReasonEntry> SoundRefusals() {
  std::vector<ReasonEntry> reasons =
      User({{"NO_FILE", "info"}, {"EMERGENCY", "info"}, {"NOT_MP3", "info"}, {"TOO_LARGE", "info"},
            {"TOO_LONG", "info"}, {"CHECKSUM", "info"}, {"PART_ORDER", "info"}, {"NO_SPACE", "warning"}});
  reasons.push_back(WithTexts(
      R("BUSY", "info", "user"),
      {{"de",
        {"Upload läuft", "Gerade wird schon ein anderer Ton hochgeladen oder geprüft.",
         "Kurz warten und noch einmal versuchen."}},
       {"en",
        {"Upload in progress", "Another sound is being uploaded or checked right now.",
         "Wait a moment and try again."}}}));
  return reasons;
}

const std::vector<EventEntry>& Refusals() {
  static const std::vector<EventEntry> groups = {
      Event("COMMAND_REFUSED", "answer",
            "A command was refused for a reason no event tells: the data of its -32600, never an event",
            Rated(true, {Text("event", "The event whose texts the reason has: a *_REFUSED type or COMMAND_REFUSED",
                              true),
                         Text("state", "current_state of robot_state; BUSY brings its own", true),
                         Text("sub_state", "current_sub_state of robot_state", true),
                         Text("event_id", "id of the event the refusal also went out as, none without one")}),
            User({{"WRONG_STATE", "info"}, {"NO_JOB", "info"}, {"CHANGED", "info"}})),
      Event("REQUEST_REFUSED", "answer",
            "The bridge refused a request over MQTT 5 before any method saw it: the data of its -32600, never an event",
            Rated(true, {Text("event", "Always REQUEST_REFUSED", true),
                         Text("state", "current_state of robot_state, empty before the first", true),
                         Text("sub_state", "current_sub_state of robot_state", true)}),
            User({{"RESPONSE_TOPIC", "error"}, {"CORRELATION_DATA", "error"}})),
      Event("SOUND_REFUSED", "answer", "A method sound.* was refused: the data of its -32600, never an event",
            Rated(true, {Text("event", "Always SOUND_REFUSED", true)}), SoundRefusals(), "info", "user"),
  };
  return groups;
}

}  // namespace mowbite_api
