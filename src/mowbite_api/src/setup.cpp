#include "mowbite_api/setup.h"

#include <cstdio>

#include "fields.h"
#include "internal.h"

namespace mowbite_api {

namespace {

using namespace fields;

UserTexts Both(UserText de, UserText en) {
  return {{"de", std::move(de)}, {"en", std::move(en)}};
}

// Texts of a name that needs no help: a stage, a part, a point, a value.
UserTexts Named(std::string de, std::string en) {
  return Both({std::move(de), "", ""}, {std::move(en), "", ""});
}

std::string Num(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%g", value);
  return text;
}

SetupParam Seconds(std::string name, double min, double max, double def, std::string text) {
  return {std::move(name), Type::kNumber, "s", std::move(text), min, max, def};
}

SetupParam Amount(std::string name, std::string unit, double min, double max, double def, std::string text) {
  return {std::move(name), Type::kNumber, std::move(unit), std::move(text), min, max, def};
}

SetupParam Word(std::string name, std::string text, std::vector<std::string> values = {}, bool required = false,
                std::string word = {}) {
  return {std::move(name), Type::kString, "", std::move(text), 0, 0, 0, std::move(values), required, std::move(word)};
}

SetupParam Words(std::string name, std::string text, std::vector<std::string> values = {}, bool required = false) {
  return {std::move(name), Type::kArray, "", std::move(text), 0, 0, 0, std::move(values), required};
}

template <typename Name>
std::vector<std::string> Ids(const std::vector<Name>& names) {
  std::vector<std::string> ids;
  ids.reserve(names.size());
  for (const auto& name : names) ids.push_back(name.id);
  return ids;
}

// name, help and fix by language (de, en); en is the one to fall back to.
Field TextsField(std::string text = "name, help and fix by language (de, en)") {
  return Object("texts", std::move(text));
}

ReasonEntry Reason(std::string code, std::string severity, std::string resume, std::string text,
                   std::vector<Field> fields, UserTexts texts) {
  return {std::move(code), std::move(severity), std::move(resume), std::move(text), std::move(fields),
          Json::object(),  std::move(texts)};
}

Field Part() {
  return Text("part", "The part of the step it happened in", false);
}

ReasonEntry NotWritable(const char* severity) {
  return Reason("STORAGE_NOT_WRITABLE", severity, "user", "setup.json in data_dir cannot be written",
                {Text("file", "The file that cannot be written", false)},
                Both({"Speicher nicht beschreibbar",
                      "Der Mäher kann den Stand der Einrichtung nicht speichern.",
                      "Speicher des Mähers prüfen, dann MowBite OS neu starten."},
                     {"Storage not writable", "The mower can't save the state of the setup.",
                      "Check the storage of the mower, then restart MowBite OS."}));
}

ReasonEntry DriverMissing(const char* severity) {
  return Reason("DRIVER_MISSING", severity, "user", "The driver of the platform reports no status", {},
                Both({"Platine antwortet nicht",
                      "Der Mäher bekommt keine Daten von seiner Platine. Ohne sie lässt sich nichts prüfen.",
                      "Mäher einschalten, Kabel zur Platine prüfen, dann wiederholen."},
                     {"Board not answering", "The mower gets no data from its board, so nothing can be checked.",
                      "Switch the mower on, check the cable to the board, then try again."}));
}

ReasonEntry Busy(const char* severity) {
  return Reason("BUSY", severity, "user", "Another step runs, or the mower is not idle",
                {Text("step", "The step that runs", false), Text("state", "current_state of the mower", false)},
                Both({"Gerade nicht möglich",
                      "Ein anderer Schritt läuft noch, oder der Mäher ist gerade unterwegs.",
                      "Den laufenden Schritt beenden oder warten, bis der Mäher steht."},
                     {"Not possible right now", "Another step is still running, or the mower is on its way.",
                      "End the running step or wait until the mower stands."}));
}

}  // namespace

const std::vector<SetupName>& SetupStages() {
  static const std::vector<SetupName> stages = {
      {"device", "The board answers and fits", Named("Gerät", "Device")},
      {"inputs", "Every safety input by hand", Named("Eingänge", "Inputs")},
      {"position", "Correction data and a fix", Named("Position", "Position")},
      {"wheels", "Wheels and stops with the mower jacked up", Named("Räder aufgebockt", "Wheels off the ground")},
      {"ground", "Driving and stopping on the ground", Named("Fahren am Boden", "Driving on the ground")},
      {"blade", "The blade and its stops", Named("Messer", "Blade")},
      {"safety", "The summary of the safety checks", Named("Sicherheit", "Safety")},
      {"dimensions", "The dimensions of the mower", Named("Maße", "Dimensions")},
      {"station", "The docking station", Named("Ladestation", "Docking station")},
      {"map", "The areas to mow", Named("Karte", "Map")},
      {"calibration", "The calibration drive", Named("Kalibrierfahrt", "Calibration drive")},
      {"trial", "A first run", Named("Probelauf", "Trial run")},
  };
  return stages;
}

const std::vector<SetupRole>& SetupRoles() {
  static const std::vector<SetupRole> roles = {
      {{"stop", "A STOP button",
        Both({"STOP-Taste", "Jede STOP-Taste einmal drücken und wieder loslassen. Der Mäher muss dabei sofort den "
                           "Notaus melden.",
             "Kommt kein Haken: Stecker und Kabel der Taste prüfen, dann wiederholen."},
            {"STOP button", "Press every STOP button once and let go. The mower has to report the emergency stop at "
                            "once.",
             "No tick: check plug and cable of the button, then try again."})},
       "stop", "STOP"},
      {{"lift", "The mower is lifted",
        Both({"Anheben", "Den Mäher anheben, bis der Haken kommt, und wieder absetzen.",
             "Kommt kein Haken: die Hebesensoren und ihre Kabel prüfen, dann wiederholen."},
            {"Lifting", "Lift the mower until the tick comes, then put it down.",
             "No tick: check the lift sensors and their cables, then try again."})},
       "lift", "LIFT"},
      {{"bumper", "The bumper is pressed, on any side",
        Both({"Stoßfänger", "Den Stoßfänger eindrücken, bis der Haken kommt, und loslassen.",
             "Kommt kein Haken: die Schalter des Stoßfängers und ihre Kabel prüfen, dann wiederholen."},
            {"Bumper", "Press the bumper in until the tick comes, then let go.",
             "No tick: check the switches of the bumper and their cables, then try again."})},
       "bumper", "COLLISION"},
      {{"bumper_left", "The left side of the bumper, on platforms that tell the sides apart",
        Both({"Stoßfänger links", "Den Stoßfänger links eindrücken, bis der Haken kommt, und loslassen.",
             "Kommt kein Haken: den linken Schalter des Stoßfängers und sein Kabel prüfen, dann wiederholen."},
            {"Bumper left", "Press the bumper in on the left until the tick comes, then let go.",
             "No tick: check the left switch of the bumper and its cable, then try again."})},
       "bumper", "COLLISION"},
      {{"bumper_right", "The right side of the bumper, on platforms that tell the sides apart",
        Both({"Stoßfänger rechts", "Den Stoßfänger rechts eindrücken, bis der Haken kommt, und loslassen.",
             "Kommt kein Haken: den rechten Schalter des Stoßfängers und sein Kabel prüfen, dann wiederholen."},
            {"Bumper right", "Press the bumper in on the right until the tick comes, then let go.",
             "No tick: check the right switch of the bumper and its cable, then try again."})},
       "bumper", "COLLISION"},
  };
  return roles;
}

const std::vector<SetupName>& SetupModes() {
  static const std::vector<SetupName> modes = {
      {"press",
       "The input switches within the time of the step, the emergency of the step follows within its limit, then the "
       "input is let go"},
      {"short",
       "The input switches for less than short_max and no emergency follows: what a bump of the lift sensors must not "
       "stop",
       Both({"Kurz auslösen", "Nur kurz auslösen und sofort loslassen: der Mäher darf dabei nicht stoppen.", ""},
            {"Trigger briefly", "Trigger only briefly and let go at once: the mower must not stop.", ""})},
      {"assign",
       "A part of a step inputs_assign: one line alone changes, and the step asks whether it is the one of the role of "
       "the part. Help and fix of such a part; its name is the one of its role",
       Both({"Zuordnen",
             "Diesen Sensor einmal auslösen und wieder loslassen. Der Mäher zeigt, an welcher Leitung er ihn gesehen "
             "hat.",
             "Kommt kein Vorschlag: Stecker und Kabel des Sensors prüfen, dann wiederholen."},
            {"Assign", "Trigger this sensor once and let go. The mower shows the line it saw it on.",
             "No proposal: check plug and cable of the sensor, then try again."})},
  };
  return modes;
}

const std::vector<SetupName>& SetupMoves() {
  static const std::vector<SetupName> moves = {
      {"front_up", "The front goes up by the angle tilt", Named("Vorn anheben", "Lift the front")},
      {"left_up", "The left side goes up by the angle tilt", Named("Linke Seite anheben", "Lift the left side")},
      {"right_up", "The right side goes up by the angle tilt", Named("Rechte Seite anheben", "Lift the right side")},
      {"turn_left", "The mower turns left by the angle turn, seen from above",
       Named("Nach links drehen", "Turn to the left")},
      {"turn_right", "The mower turns right by the angle turn", Named("Nach rechts drehen", "Turn to the right")},
      {"tilt_stop", "The mower is tilted until the emergency TILT comes",
       Named("Weit kippen, bis der Notaus kommt", "Tilt far until the emergency stop comes")},
  };
  return moves;
}

const std::vector<SetupCondition>& SetupConditions() {
  static const std::vector<SetupCondition> conditions = {
      {{"ntrip", "Correction data arrives from the caster",
        Both({"Korrekturdaten", "Den Zugang zum Korrekturdienst eintragen. Der Haken kommt, sobald Korrekturdaten "
                               "ankommen.",
             "Zugangsdaten, Mountpoint und die Internetverbindung des Mähers prüfen."},
            {"Correction data", "Enter the access to the correction service. The tick comes once correction data "
                                "arrives.",
             "Check the access data, the mountpoint and the internet connection of the mower."})},
       "ntrip"},
      {{"station", "A docking station is in the map and the mower stands on its contacts",
        Both({"Ladestation aufnehmen", "Den Mäher auf die Ladestation fahren und die Station aufnehmen. Der Haken "
                                      "kommt, sobald sie in der Karte steht und der Mäher auf den Kontakten ist.",
             "Den Mäher gerade auf die Kontakte schieben, bis er lädt, dann die Station aufnehmen."},
            {"Record the docking station", "Drive the mower onto the docking station and record the station. The "
                                           "tick comes once it is in the map and the mower is on the contacts.",
             "Push the mower straight onto the contacts until it charges, then record the station."})},
       "dock"},
      {{"area", "The map has an area and allows mowing",
        Both({"Mähfläche aufnehmen", "Eine Mähfläche aufnehmen. Der Haken kommt, sobald die Karte eine Fläche hat, "
                                    "auf der gemäht werden kann.",
             "Die Fläche einmal ganz umfahren und speichern."},
            {"Record a mowing area", "Record an area to mow. The tick comes once the map has an area the mower can "
                                     "mow.",
             "Drive around the whole area once and save it."})},
       "recording"},
      {{"dock_test", "A docking test ended on the contacts",
        Both({"Andocken testen", "Den Andocktest starten. Der Haken kommt, wenn der Mäher von selbst wieder auf den "
                                "Kontakten steht.",
             "Schlägt der Test fehl, die Station neu aufnehmen und wiederholen."},
            {"Test docking", "Start the docking test. The tick comes when the mower is back on the contacts by "
                             "itself.",
             "If the test fails, record the station again and repeat."})},
       "dock_test"},
      {{"mow", "The mower mowed and came back onto the contacts",
        Both({"Probelauf", "Mähen starten und den Mäher zur Ladestation zurückkehren lassen. Der Haken kommt, wenn er "
                          "nach dem Mähen wieder auf den Kontakten steht.",
             "Bleibt der Mäher stehen, zeigt die App den Grund. Danach erneut starten."},
            {"Trial run", "Start mowing and let the mower return to the docking station. The tick comes when it is "
                          "back on the contacts after mowing.",
             "If the mower stops, the app shows why. Then start again."})},
       "mow"},
  };
  return conditions;
}

const std::vector<SetupName>& SetupPoints() {
  static const std::vector<SetupName> points = {
      {"raised", "The mower is jacked up, its wheels turn freely",
       Named("Mäher aufgebockt, Räder frei", "Mower jacked up, wheels free")},
      {"blade_removed", "The blade is taken off", Named("Messer abgenommen", "Blade removed")},
      {"clear", "Nobody is close to the mower", Named("Niemand in der Nähe", "Nobody close by")},
      {"outdoors", "The mower stands under the open sky",
       Named("Mäher steht im Freien", "Mower stands under the open sky")},
  };
  return points;
}

const std::vector<SetupName>& SetupSkipReasons() {
  static const std::vector<SetupName> reasons = {
      {"later", "To be done later", Named("Später", "Later")},
      {"not_fitted", "The mower does not have this part", Named("Nicht vorhanden", "Not fitted")},
      {"done_before", "Checked before, outside of the program", Named("Schon geprüft", "Checked before")},
      {"other", "Another reason, in the text of setup.skip", Named("Anderer Grund", "Other reason")},
  };
  return reasons;
}

const std::vector<std::string>& SetupOpens() {
  static const std::vector<std::string> opens = {"ntrip", "geometry", "recording", "dock", "map", "dock_test", "mow"};
  return opens;
}

const std::vector<std::string>& SetupSymbols() {
  static const std::vector<std::string> symbols = {"stop",  "lift",    "bumper", "wheels", "ground",
                                                   "blade", "antenna", "dock",   "board",  "map"};
  return symbols;
}

const std::vector<std::string>& SetupStates() {
  static const std::vector<std::string> states = {"locked", "open", "running", "ok", "failed", "skipped", "stale"};
  return states;
}

const std::vector<std::string>& SetupEmergencies() {
  static const std::vector<std::string> names = {"STOP", "LIFT", "COLLISION", "NONE"};
  return names;
}

const SetupName& SetupAccept() {
  static const SetupName accept = {
      "accept", "Whether the line of ask is the one of its part",
      Both({"Leitung übernehmen?", "Der Mäher hat den Handgriff an dieser Leitung gesehen.", ""},
           {"Take this line?", "The mower saw the action on this line.", ""})};
  return accept;
}

const std::vector<SetupName>& SetupAcceptValues() {
  static const std::vector<SetupName> values = {
      {"yes", "The line is taken for the role of the part", Named("Übernehmen", "Take it")},
      {"no", "The part waits for another line", Named("Nicht diese", "Not this one")},
  };
  return values;
}

const SetupName& SetupDelay() {
  static const SetupName delay = {
      "delay", "How long the sensor of a part has to stay switched before the hardware stops, at most what the "
               "platform gives",
      Both({"Verzögerung",
            "So lange muss der Sensor ausgelöst bleiben, bevor der Mäher stoppt. Kürzer als die Vorgabe geht, länger "
            "nicht. Nach einer Änderung startet MowBite OS neu, die Eingänge sind danach neu zu prüfen.",
            ""},
           {"Delay",
            "The sensor has to stay triggered this long before the mower stops. Shorter than the default works, "
            "longer does not. MowBite OS restarts after a change, the inputs are to be checked again after it.",
            ""})};
  return delay;
}

const std::vector<SetupName>& SetupRestarts() {
  static const std::vector<SetupName> restarts = {
      {"self", "The OS ends in order and whatever runs it starts it again"},
      {"host", "The OS asks its host service for the restart (host.restart_os)"},
  };
  return restarts;
}

const std::vector<std::string>& ShapeKinds() {
  static const std::vector<std::string> kinds = {"body", "wheel", "caster", "bumper", "button", "sensor", "handle"};
  return kinds;
}

const std::vector<SetupCheck>& SetupChecks() {
  static const std::vector<SetupCheck> checks = {
      {"confirm", "confirm", true, false,
       "The user confirms what the step says: setup.start with every point of confirm[] ends it"},
      {"choice", "input", false, false,
       "One of the values, given with setup.answer under the name of the input; kept in the record of the program",
       {Word("name", "Name of the input in setup.answer", {}, true),
        Words("values", "The values to choose from, their texts in value_texts of the step", {}, true)}},
      {"number", "input", false, false,
       "A number within min and max, given with setup.answer under the name of the input; kept in the record",
       {Word("name", "Name of the input in setup.answer", {}, true), Word("unit", "Its unit"),
        Amount("min", "", -1e6, 1e6, 0, "The least it takes"), Amount("max", "", -1e6, 1e6, 0, "The most it takes")}},
      {"board", "measure", true, true,
       "The driver reports for time: status at its rate, firmware known and compatible, no errors on the link, no "
       "input switched, motor controllers ok, battery read, and with imu gravity within " +
           Num(kSetupGravityMin) + " and " + Num(kSetupGravityMax) + " m/s². Nothing moves",
       {Seconds("time", 1, 30, 5, "How long it watches"),
        Amount("status_rate", "Hz", 1, 50, 8, "hw/status at least this often"),
        Amount("link_errors", "", 0, 100, 0, "Errors on the link to the board at most, in time"),
        Amount("battery", "%", 0, 100, 0, "Charge of the battery at least; 0 only asks for a reading"),
        Word("imu", "yes to check gravity on imu/data", {"yes", "no"}, false, "yes")},
       {{"status_rate", "Hz", "Rate of hw/status", Named("Meldungen der Platine", "Reports of the board")},
        {"firmware", "", "Firmware of every board, as hw/capabilities names it", Named("Firmware", "Firmware")},
        {"link_errors", "", "Errors on the link in time", Named("Fehler auf der Leitung", "Errors on the link")},
        {"inputs", "", "Inputs that are switched, none for ok", Named("Eingänge in Ruhe", "Inputs at rest")},
        {"motors", "", "Motor controllers that are not ok, none for ok", Named("Regler", "Motor controllers")},
        {"battery", "%", "Charge of the battery", Named("Akku", "Battery")},
        {"gravity", "m/s²", "Gravity the IMU reads at rest", Named("Lagesensor in Ruhe", "Motion sensor at rest")}},
       {"NO_DATA", "LIMIT", "STOPPED", "INTERRUPTED"},
       Both({"Platine und Sensoren",
             "Der Mäher prüft, ob seine Platine antwortet, die Firmware passt und Sensoren, Regler und Akku Werte "
             "liefern. Dabei bewegt sich nichts.",
             "Den Mäher einschalten und frei hinstellen, keine Taste drücken, dann wiederholen."},
            {"Board and sensors",
             "The mower checks that its board answers, the firmware fits and sensors, motor controllers and battery "
             "report. Nothing moves.",
             "Switch the mower on, let it stand free, press no button, then try again."})},
      {"input", "measure", true, true,
       "One input of hw/status by hand, part by part: each part of mode press or short, see modes. With hw/inputs line "
       "by line: parts of one role need different lines, and every line of the role has to switch in the step. The "
       "program releases the emergency a part of mode press brought by itself once the input is free, where the "
       "driver counts its latches and no other reason came up. Texts of the step by its role",
       {Word("role", "The input", Ids(SetupRoles()), true),
        Word("emergency", "The emergency a press brings; without, the one of the role", SetupEmergencies()),
        Seconds("within", 0.05, 10, 1, "The emergency follows the input within this time"),
        Seconds("time", 5, 300, 60, "Each part is done within this time"),
        Seconds("short_max", 0.1, 5, 0.5, "A part of mode short switches for less than this")},
       {{"delay", "s", "From the input to the emergency, one value by part of mode press with the id of the part"}},
       {"NOT_SEEN", "WRONG_INPUT", "NO_EMERGENCY", "STOPPED_EARLY", "LINE_SILENT", "NO_DATA", "STOPPED",
        "INTERRUPTED"}},
      {"inputs_assign", "input", true, false,
       "Finds the line of every sensor, on a platform whose driver learns its inputs: the OS restarts with the driver "
       "watching its candidate lines (emergency INPUTS_LEARNING, nothing moves), watches them at rest for quiet, then "
       "takes one part after the other, each of a role (stop, lift, bumper_left or bumper_right) and with the delay_ms "
       "its line gets: one line alone changes, ask of the step names it, and setup.answer with accept yes takes it. "
       "After the last part the table goes into the settings the program writes for the platform, the OS restarts "
       "without learning and the step holds hw/inputs against the table. setup.abort ends it with a restart as well. "
       "The steps input are open again after it. The step list names how the OS restarts (restart). Once the step "
       "is ok setup.answer takes delay_<part> for any of its parts, a whole number of ms from 0 to delay_ms of the "
       "part: the table is written with it, the OS restarts and the step holds hw/inputs against it again",
       {Word("node", "The node of the driver the table is written for", {}, true),
        Seconds("quiet", 3, 60, 10, "The lines are watched untouched for this time first"),
        Seconds("time", 5, 300, 60, "Each part is done within this time"),
        Seconds("restart_time", 20, 600, 120, "The OS is back within this time after a restart")},
       {{"line", "", "The line taken, one value by part with the id of the part"}},
       {"NO_LINE", "AMBIGUOUS", "FLUTTER", "NOT_APPLIED", "RESTART_FAILED", "NO_DATA", "STOPPED", "INTERRUPTED"},
       Both({"Eingänge zuordnen",
             "Der Mäher findet heraus, an welcher Leitung jeder Sensor hängt. Dafür startet MowBite OS neu; "
             "währenddessen stoppt kein Sensor, der Mäher fährt und mäht nicht.",
             "Jeden Sensor einzeln auslösen und die anderen in Ruhe lassen."},
            {"Assign the inputs",
             "The mower finds the line every sensor is on. MowBite OS restarts for it; no sensor stops anything "
             "meanwhile, the mower does not drive or mow.",
             "Trigger every sensor on its own and leave the others alone."})},
      {"imu", "measure", true, true,
       "The mower is moved by hand, part by part, see moves: the IMU has to show each move with its sign",
       {Amount("tilt", "deg", 5, 25, 10, "How far a side goes up"),
        Amount("turn", "deg", 10, 90, 30, "How far the mower turns"),
        Seconds("time", 5, 300, 60, "Each part is done within this time")},
       {{"angle", "deg", "The angle reached, one value by part with the id of the part"}},
       {"NOT_SEEN", "WRONG_SIGN", "NO_EMERGENCY", "NO_DATA", "STOPPED", "INTERRUPTED"},
       Both({"Lagesensor",
             "Den Mäher von Hand kippen und drehen, wie angezeigt. So zeigt sich, ob der Sensor richtig herum "
             "eingebaut ist.",
             "Stimmt eine Richtung nicht, ist der Sensor anders eingebaut als eingestellt: die Einbaulage in den "
             "Einstellungen der Plattform prüfen."},
            {"Motion sensor",
             "Tilt and turn the mower by hand as shown. This shows whether the sensor is mounted the right way "
             "round.",
             "If a direction is wrong, the sensor is mounted another way than set: check its mounting in the "
             "settings of the platform."})},
      {"position", "measure", true, true,
       "The position holds for time: solution, accuracy and rate of gps/absolute_pose",
       {Word("solution", "The solution it needs at least", {"fixed", "float"}, false, "fixed"),
        Amount("accuracy", "m", 0.005, 1, 0.05, "Accuracy at most"),
        Amount("rate", "Hz", 0.5, 20, 4, "Positions at least this often"),
        Seconds("time", 1, 60, 10, "How long all of it has to hold in a row"),
        Seconds("timeout", 10, 900, 180, "It gives up after this time")},
       {{"solution", "", "none, single, float or fixed", Named("Lösung", "Solution")},
        {"accuracy", "m", "Accuracy of the position", Named("Genauigkeit", "Accuracy")},
        {"rate", "Hz", "Rate of the positions", Named("Positionen je Sekunde", "Positions per second")}},
       {"NO_DATA", "NO_RTK", "LIMIT", "STOPPED", "INTERRUPTED"},
       Both({"Genaue Position",
             "Der Mäher wartet unter freiem Himmel auf eine genaue Position und hält sie einige Sekunden.",
             "Freie Sicht zum Himmel schaffen und die Korrekturdaten prüfen, dann wiederholen."},
            {"Exact position",
             "The mower waits under the open sky for an exact position and holds it for some seconds.",
             "Give it a free view of the sky and check the correction data, then try again."})},
      {"done_when", "confirm", false, true,
       "Done as soon as the OS sees its condition, see conditions; the app opens the screen open names, by default "
       "the one of the condition. Texts of the step by its condition",
       {Word("condition", "What it waits for", Ids(SetupConditions()), true)}},
      {"geometry", "input", false, true,
       "Done once every key of keys is confirmed in robot.geometry (robot.geometry.set with a value or with confirm); "
       "the app opens its screen of the dimensions",
       {Words("keys", "Keys of x-geometry the user can set; none for all of them")},
       {},
       {},
       Both({"Maße prüfen",
             "Die Maße des Mähers auf der Zeichnung prüfen und bestätigen oder ändern. Mit ihnen plant und fährt "
             "der Mäher.",
             "Nachmessen und den Wert eintragen; alle Maße gelten ab der Mitte der Antriebsachse."},
            {"Check the dimensions",
             "Check the dimensions of the mower on the drawing and confirm or change them. The mower plans and "
             "drives with them.",
             "Measure again and enter the value; all dimensions count from the middle of the drive axle."}),
       "geometry"},
  };
  return checks;
}

const SetupCheck* FindSetupCheck(const std::string& name) {
  for (const auto& check : SetupChecks()) {
    if (check.name == name) return &check;
  }
  return nullptr;
}

Json SetupCatalog() {
  const auto named = [](const auto& names) {
    Json out = Json::object();
    int order = 0;
    for (const SetupName& name : names) {
      Json entry = {{"order", order++}, {"description", name.text}};
      if (!name.texts.empty()) entry["texts"] = Texts(name.texts);
      out[name.id] = entry;
    }
    return out;
  };
  Json roles = named(SetupRoles());
  for (const auto& role : SetupRoles()) {
    roles[role.id]["group"] = role.group;
    roles[role.id]["emergency"] = role.emergency;
  }
  Json conditions = named(SetupConditions());
  for (const auto& condition : SetupConditions()) conditions[condition.id]["open"] = condition.open;
  Json accept = {{"description", SetupAccept().text},
                 {"texts", Texts(SetupAccept().texts)},
                 {"values", named(SetupAcceptValues())}};
  Json delay = {{"description", SetupDelay().text},
                {"texts", Texts(SetupDelay().texts)},
                {"prefix", kSetupDelayPrefix},
                {"unit", "ms"}};
  Json checks = Json::object();
  for (const auto& check : SetupChecks()) {
    Json params = Json::object();
    for (const auto& param : check.params) {
      Json entry = {{"description", param.text}, {"required", param.required}};
      if (param.type == Type::kNumber) {
        entry["type"] = "number";
        entry["unit"] = param.unit;
        entry["min"] = param.min;
        entry["max"] = param.max;
        if (!param.required) entry["default"] = param.def;
      } else {
        entry["type"] = param.type == Type::kArray ? "list" : "text";
        if (!param.values.empty()) entry["values"] = param.values;
        if (!param.word.empty()) entry["default"] = param.word;
      }
      params[param.name] = entry;
    }
    Json values = Json::object();
    for (const auto& value : check.values) {
      Json entry = {{"unit", value.unit}, {"description", value.text}};
      if (!value.texts.empty()) entry["texts"] = Texts(value.texts);
      values[value.id] = entry;
    }
    Json entry = {{"kind", check.kind},        {"start", check.start}, {"auto", check.automatic},
                  {"description", check.text}, {"params", params},     {"values", values},
                  {"reasons", check.reasons}};
    if (!check.texts.empty()) entry["texts"] = Texts(check.texts);
    if (!check.open.empty()) entry["open"] = check.open;
    checks[check.name] = entry;
  }
  return {{"description",
           "The setup program: stages in their order, the checks a step of setup/setup.yaml of a platform can be "
           "(kind, whether setup.start begins it, params with the bounds and defaults of the code, values, reasons of "
           "SETUP_STEP, the screen it opens), roles of the inputs (group, emergency of a press; a role is named as its "
           "flag in hw/status) and modes of a part, moves of a step imu, conditions of a step done_when with their "
           "screen, points to confirm, reasons to skip, the question of a step inputs_assign and the delays it takes "
           "once it is ok, the ways to restart, "
           "and the limits; texts by language where the OS has them. The steps themselves come with setup.get"},
          {"stages", named(SetupStages())},
          {"checks", checks},
          {"roles", roles},
          {"modes", named(SetupModes())},
          {"moves", named(SetupMoves())},
          {"conditions", conditions},
          {"points", named(SetupPoints())},
          {"skip_reasons", named(SetupSkipReasons())},
          {"accept", accept},
          {"delay", delay},
          {"restart", named(SetupRestarts())},
          {"open", SetupOpens()},
          {"symbols", SetupSymbols()},
          {"states", SetupStates()},
          {"limits",
           {{"steps", kSetupMaxSteps},
            {"parts", kSetupMaxParts},
            {"image_bytes", kSetupImageBytes},
            {"skip_text", kSetupMaxSkipText},
            {"shape_parts", kShapeMaxParts},
            {"shape_part_points", kShapeMaxPartPoints},
            {"shape_points", kShapeMaxPoints}}},
          {"shape_kinds", ShapeKinds()}};
}

Field SetupState() {
  const Field image_fields = Object(
      "image", "The image of the step, null for none: the app draws symbol then",
      {Text("name", "Its name for setup.image"), Text("sha256", "sha256 of the file, to keep it by"),
       Integer("bytes", "Its size"), Text("type", "Always image/svg+xml"),
       Nullable(Object("accent", "A second image of the same size to lay over it, for the spot to press; null for "
                                 "none",
                       {Text("name", "Its name for setup.image"), Text("sha256", "sha256 of the file"),
                        Integer("bytes", "Its size")}))});
  const Field step = Rows(
      "steps", "The steps in the order of the step list",
      {Text("id", "Name of the step in setup.*"),
       Text("stage", "Its stage"),
       Text("check", "Its check, a key of x-setup.checks"),
       Text("kind", "How to show it: confirm, input or measure; a kind the app does not know as text with a button"),
       Text("state", "locked: a step of needs is not ok; open; running; ok; failed; skipped; stale", true,
            SetupStates()),
       Integer("attempt", "How often it was started or answered"),
       List("needs", "Steps that have to be ok before this one", Type::kString),
       Rows("blocked", "What keeps it from going on right now, reasons of SETUP_BLOCKED in x-reasons",
            {Text("reason", "A fixed name in capitals"), Text("message", "English text")}),
       Nullable(Object("motion", "What moves in the step, null for nothing")),
       Rows("confirm", "Points setup.start has to name in confirm", {Text("id", "Name of the point"), TextsField()}),
       Bool("start", "setup.start begins it"),
       Bool("auto", "Its ticks come without a tap"),
       Bool("skippable", "setup.skip takes it"),
       Rows("skip_reasons", "Reasons setup.skip takes", {Text("id", "Name of the reason"), TextsField()}),
       Rows("inputs", "What setup.answer takes in values",
            {Text("name", "Key in values; in a step inputs_assign accept and delay_<part>, see x-setup.accept and "
                          "x-setup.delay"),
             Text("type", "choice or number", true, {"choice", "number"}),
             Text("unit", "Unit of a number", false), Number("min", "", "Least number", false),
             Number("max", "", "Largest number", false),
             Rows("values", "Values of a choice", {Text("id", "The value"), TextsField()}, false),
             Bool("ask", "A question of the running step that waits for its answer: accept while ask of the step is "
                         "set"),
             Bool("advanced", "A setting beside the check, to be shown apart: the delays of a step inputs_assign"),
             {"value", Type::kAny, "", true, "The value given last, null before", {}, true, {}, ""}, TextsField()}),
       Rows("values", "What it measures, live while it runs and as it ended",
            {Text("id", "Name of the value"),
             {"value", Type::kAny, "", true, "A number, a text, or null before the first reading", {}, true, {}, ""},
             Text("unit", "Unit of a number, empty for a text"), Nullable(Number("min", "", "Least allowed")),
             Nullable(Number("max", "", "Most allowed")), Nullable(Bool("ok", "Within its bounds; null before it "
                                                                              "is judged")),
             TextsField()}),
       Rows("parts", "Its parts, each with a tick of its own",
            {Text("id", "Name of the part"),
             Text("state", "waiting for the one to do now", true, {"open", "waiting", "ok", "failed"}), TextsField()}),
       Rows("lines", "The inputs of the platform one by one as hw/inputs has them, live, in a step input or "
                     "inputs_assign; empty in other steps and where the driver has no hw/inputs",
            {Text("name", "The line as the hardware names it"),
             Text("role", "A key of x-setup.roles, empty for a line without a role"),
             Text("reason", "The emergency the hardware latches with it: stop, lift or collision; empty for none"),
             Bool("active", "Switched now; while the driver learns: the level of the line is high"),
             Bool("assigned", "In the table of the mower; false for a line the driver only watches while it learns"),
             Bool("held", "The hardware keeps it switched after the sensor is free, until the emergency is released"),
             Integer("delay_ms", "How long it has to stand before the hardware latches")}),
       Nullable(Object("ask", "What a running step inputs_assign proposes and waits for setup.answer {accept} with; "
                              "null else",
                       {Text("part", "The part it is for"), Text("line", "The line that changed, a name of lines[]"),
                        Bool("active_high", "The line went high with the action")})),
       Nullable(Object("progress", "Where a running step is, null without parts or rounds",
                       {Integer("n", "The part or round now, from 1"), Integer("of", "How many there are")})),
       Nullable(Number("ends_at", "unix s", "When the part of now gives up, null for no limit")),
       Nullable(Text("open", "A screen of the app to open for it, null for none", true, SetupOpens())),
       Nullable(image_fields),
       Nullable(Text("symbol", "A sign to draw without an image", true, SetupSymbols())),
       List("mark", "ids of robot_shape or of the app's derived parts (wheel_left, wheel_right, blade, antenna, "
                    "contacts, outline) to highlight",
            Type::kString),
       TextsField("name, help and fix of the step by language (de, en); en is the one to fall back to"),
       Nullable(Object("result", "How it ended the last time, null before",
                       {Bool("ok", "Passed"), Nullable(Text("reason", "A reason of SETUP_STEP, null with ok")),
                        Nullable(Text("message", "English text, null with ok")),
                        Nullable(Text("part", "The part it failed in")),
                        Nullable(Text("skip", "The reason of setup.skip of a skipped step")),
                        Nullable(Text("note", "The text of setup.skip")), Number("t", "unix s", "When"),
                        Integer("attempt", "The attempt it ended"),
                        Text("firmware", "Firmware of the boards then, empty while unknown")})),
       Bool("restart", "Its result counts only after a restart of the OS, which the program asks for itself")});
  return Object(
      "setup", "The state of the setup program",
      {Integer("version", "Goes up with every change of a step; setup.start, answer, skip and reset need the one "
                          "read last"),
       Text("platform", "The platform of the step list, as its driver reports it (hw/capabilities)"),
       Integer("catalog", "Counter of the step list of the platform: another one means other steps or images"),
       Nullable(Text("stage", "The first stage with a step still to do")),
       Nullable(Text("current", "The step that runs, else the first one still to do; null once complete")),
       Bool("complete", "Every step is ok or skipped"),
       Object("safety", "The lock of autonomous driving and mowing",
              {Bool("open", "The platform has a safety measurement"),
               Nullable(Text("date", "Its date, YYYY-MM-DD")),
               Nullable(Text("firmware", "The firmware it was made with, null while unknown")),
               Nullable(Text("why", "NO_SAFETY_MEASUREMENT while closed"))}),
       Bool("restart_pending", "A step waits for a restart of the OS"),
       Rows("stages", "The stages with steps, in their order",
            {Text("id", "A key of x-setup.stages"), Text("state", "Of all its steps together", true, SetupStates()),
             TextsField()}),
       step});
}

EventEntry SetupStepEvent() {
  const auto failed = [](std::string code, std::string text, std::vector<Field> fields, UserTexts texts) {
    return Reason(std::move(code), "warning", "user", std::move(text), std::move(fields), std::move(texts));
  };
  return {"SETUP_STEP", "setup", "A step of the setup program ended",
          Rated(false, {Text("step", "The step"), Text("state", "How it ended", true, {"ok", "failed", "skipped"}),
                        Integer("attempt", "The attempt that ended"),
                        Text("skip", "With skipped: the reason of setup.skip", false),
                        Text("note", "With skipped: the text of setup.skip", false)}),
          {failed("NOT_SEEN", "The OS did not see what the part asked for within its time", {Part()},
                  Both({"Nichts gesehen",
                        "Der Mäher hat den Handgriff nicht bemerkt. Der Sensor oder sein Kabel meldet nichts.",
                        "Stecker und Kabel prüfen, den Handgriff deutlich ausführen und den Schritt wiederholen."},
                       {"Nothing seen", "The mower didn't notice the action. The sensor or its cable reports nothing.",
                        "Check plug and cable, do the action clearly and repeat the step."})),
           failed("WRONG_INPUT", "Another input switched than the one the part asked for",
                  {Part(), Text("seen", "The input that switched", false)},
                  Both({"Anderer Eingang",
                        "Statt des erwarteten Eingangs hat ein anderer geschaltet. Die Sensoren sind vertauscht "
                        "angeschlossen oder falsch zugeordnet.",
                        "Anschlüsse der Sensoren prüfen, dann den Schritt wiederholen."},
                       {"Another input",
                        "Another input switched than the one expected. The sensors are plugged in the wrong way "
                        "round or assigned wrongly.",
                        "Check where the sensors are plugged in, then repeat the step."})),
           failed("NO_EMERGENCY", "The input switched, but the emergency of the step did not follow within its limit",
                  {Part(), Number("limit", "s", "The limit", false)},
                  Both({"Kein Notaus",
                        "Der Eingang hat geschaltet, aber der Mäher hat nicht rechtzeitig den Notaus gemeldet. So "
                        "würde er im Betrieb nicht anhalten.",
                        "Den Mäher nicht fahren lassen. Zuordnung und Einstellung des Eingangs prüfen, dann "
                        "wiederholen."},
                       {"No emergency stop",
                        "The input switched, but the mower didn't report the emergency stop in time. It wouldn't "
                        "stop like this in use.",
                        "Don't let the mower drive. Check how the input is assigned and set, then repeat."})),
           failed("STOPPED_EARLY", "A short switch of the input brought the emergency already", {Part()},
                  Both({"Stoppt zu früh",
                        "Schon ein kurzes Auslösen hat den Notaus gebracht. Im Betrieb würde der Mäher bei jeder "
                        "Unebenheit stehen bleiben.",
                        "Die Verzögerung des Eingangs prüfen, dann wiederholen."},
                       {"Stops too early",
                        "A short trigger brought the emergency stop already. In use the mower would stop at every "
                        "bump.",
                        "Check the delay of the input, then repeat."})),
           failed("LINE_SILENT", "A line of the role did not switch in the whole step",
                  {Part(), List("lines", "The lines that did not switch", Type::kString, false)},
                  Both({"Leitung blieb stumm",
                        "Eine Leitung dieses Eingangs hat bei keinem Handgriff geschaltet. Ein Sensor oder ein "
                        "Kontakt eines Paars ist defekt oder nicht angeschlossen.",
                        "Jeden Sensor einzeln auslösen. Bleibt die Leitung stumm: Sensor, Stecker und Kabel prüfen."},
                       {"Line stayed silent",
                        "A line of this input did not switch with any action. A sensor or one contact of a pair is "
                        "broken or not connected.",
                        "Trigger every sensor on its own. If the line stays silent, check sensor, plug and cable."})),
           failed("NO_LINE", "No line changed within the time of the part", {Part()},
                  Both({"Keine Leitung gesehen",
                        "Beim Handgriff hat keine der beobachteten Leitungen gewechselt. Der Sensor ist nicht "
                        "angeschlossen, defekt oder hängt an einem Anschluss, den der Mäher nicht beobachtet.",
                        "Stecker und Kabel des Sensors prüfen, dann wiederholen."},
                       {"No line seen",
                        "None of the lines watched changed with the action. The sensor is not connected, broken, or "
                        "on a connector the mower does not watch.",
                        "Check plug and cable of the sensor, then repeat."})),
           failed("AMBIGUOUS", "More than one line changed with the action of the part",
                  {Part(), List("lines", "The lines that changed", Type::kString, false)},
                  Both({"Mehrere Leitungen",
                        "Beim Handgriff haben mehrere Leitungen zugleich gewechselt. So lässt sich der Sensor keiner "
                        "Leitung zuordnen.",
                        "Nur einen Sensor auslösen und die anderen in Ruhe lassen, dann wiederholen."},
                       {"More than one line",
                        "More than one line changed with the action, so the sensor can't be put to one line.",
                        "Trigger one sensor only and leave the others alone, then repeat."})),
           failed("FLUTTER", "A line changed while nothing was to be touched",
                  {List("lines", "The lines that changed", Type::kString, false)},
                  Both({"Leitung unruhig",
                        "Eine Leitung wechselt, ohne dass jemand etwas berührt. Ein Sensor oder Kabel hat einen "
                        "Wackelkontakt.",
                        "Den Mäher nicht berühren. Die genannte Leitung, ihren Stecker und Sensor prüfen, dann "
                        "wiederholen."},
                       {"Line unsteady",
                        "A line changes while nobody touches anything. A sensor or cable has a loose contact.",
                        "Don't touch the mower. Check the line named, its plug and sensor, then repeat."})),
           failed("NOT_APPLIED", "After the restart hw/inputs does not show the table the step wrote",
                  {Text("key", "A setting the file kept by hand sets as well", false)},
                  Both({"Zuordnung nicht übernommen",
                        "Nach dem Neustart arbeitet der Mäher nicht mit der neuen Zuordnung. Ein Eintrag in der von "
                        "Hand gepflegten Gerätedatei hat Vorrang.",
                        "Die Einträge zu den Eingängen aus der Gerätedatei des Mähers entfernen, dann den Schritt "
                        "wiederholen."},
                       {"Assignment not applied",
                        "After the restart the mower does not work with the new assignment. An entry in the device "
                        "file kept by hand comes first.",
                        "Remove the entries of the inputs from the device file of the mower, then repeat the "
                        "step."})),
           failed("RESTART_FAILED", "The OS was not back within the time after the restart the step asked for",
                  {Number("limit", "s", "The time it had", false)},
                  Both({"Neustart blieb aus", "MowBite OS ist nicht rechtzeitig neu gestartet.",
                        "MowBite OS von Hand neu starten, dann den Schritt wiederholen."},
                       {"No restart", "MowBite OS did not restart in time.",
                        "Restart MowBite OS by hand, then repeat the step."})),
           failed("WRONG_SIGN", "The IMU showed the move the other way round", {Part()},
                  Both({"Falsche Richtung",
                        "Der Lagesensor hat die Bewegung in die andere Richtung gemeldet. Er ist anders eingebaut "
                        "als eingestellt.",
                        "Die Einbaulage des Sensors in den Einstellungen der Plattform ändern, dann wiederholen."},
                       {"Wrong direction",
                        "The motion sensor reported the move the other way round. It is mounted another way than "
                        "set.",
                        "Change the mounting of the sensor in the settings of the platform, then repeat."})),
           failed("LIMIT", "A value is outside of its bounds",
                  {Text("value_id", "id of the value in values[] of the step", false),
                   {"value", Type::kAny, "", false, "What was measured", {}, true, {}, ""},
                   Number("limit", "", "The bound it broke", false), Text("unit", "Unit of both", false)},
                  Both({"Wert außerhalb",
                        "Ein Messwert liegt außerhalb seiner Grenze. Die Liste der Werte zeigt, welcher.",
                        "Die Ursache am genannten Wert beheben, dann den Schritt wiederholen."},
                       {"Value out of bounds", "A value is outside of its bound. The list of values shows which.",
                        "Fix the cause at the value named, then repeat the step."})),
           failed("NO_DATA", "A source the step needs sent nothing",
                  {Text("source", "hw/status, hw/capabilities, imu/data or gps/absolute_pose", false)},
                  Both({"Keine Daten",
                        "Der Mäher bekommt für diesen Schritt keine Daten von Platine, Lagesensor oder GPS.",
                        "Mäher einschalten, Kabel prüfen, dann wiederholen."},
                       {"No data", "The mower gets no data from board, motion sensor or GPS for this step.",
                        "Switch the mower on, check the cables, then repeat."})),
           failed("NO_RTK", "No position of the solution, accuracy and rate asked for within the timeout", {},
                  Both({"Keine genaue Position",
                        "Der Mäher hat in der Zeit keine genaue Position bekommen.",
                        "Freie Sicht zum Himmel schaffen, Korrekturdaten prüfen, dann wiederholen."},
                       {"No exact position", "The mower didn't get an exact position in time.",
                        "Give it a free view of the sky, check the correction data, then repeat."})),
           Reason("STOPPED", "info", "user", "Ended with setup.abort", {},
                  Both({"Abgebrochen", "Der Schritt wurde abgebrochen.", "Den Schritt erneut starten."},
                       {"Stopped", "The step was stopped.", "Start the step again."})),
           Reason("INTERRUPTED", "info", "user", "The OS restarted while the step ran", {},
                  Both({"Unterbrochen", "MowBite OS wurde neu gestartet, während der Schritt lief.",
                        "Den Schritt erneut starten."},
                       {"Interrupted", "MowBite OS restarted while the step ran.", "Start the step again."}))},
          "warning", "user"};
}

EventEntry SetupProgramEvent() {
  return {"SETUP", "setup", "The setup program reached a mark",
          {Text("what", "stage_done: every step of stage is ok or skipped; complete: every step is; reset: "
                        "setup.reset took all results back",
                true, {"stage_done", "complete", "reset"}),
           Text("stage", "With stage_done: the stage", false)},
          {}};
}

EventEntry SetupRefused() {
  const auto refused = [](std::string code, std::string text, std::vector<Field> fields, UserTexts texts) {
    return Reason(std::move(code), "info", "user", std::move(text), std::move(fields), std::move(texts));
  };
  return {"SETUP_REFUSED", "answer", "A method setup.* was refused: the data of its -32600, never an event",
          Rated(true, {Text("event", "Always SETUP_REFUSED")}),
          {refused("UNKNOWN_STEP", "The step list has no step of this name", {Text("step", "The name", false)},
                   Both({"Schritt unbekannt", "Diesen Schritt gibt es auf diesem Mäher nicht.",
                         "Die Einrichtung neu öffnen."},
                        {"Unknown step", "This mower has no such step.", "Open the setup again."})),
           refused("VERSION", "The state changed since the version the call names",
                   {Integer("version", "The version now", false)},
                   Both({"Stand veraltet", "Die Einrichtung hat sich inzwischen geändert.",
                         "Die Ansicht aktualisieren und es noch einmal versuchen."},
                        {"State out of date", "The setup changed in the meantime.",
                         "Refresh the view and try again."})),
           refused("LOCKED", "A step this one needs is not ok",
                   {List("needs", "The steps to do first", Type::kString, false)},
                   Both({"Noch gesperrt", "Dieser Schritt braucht zuerst einen anderen bestandenen Schritt.",
                         "Zuerst den genannten Schritt abschließen."},
                        {"Still locked", "This step needs another step passed first.",
                         "Finish the step named first."})),
           Busy("info"),
           refused("CONFIRM_MISSING", "setup.start does not name every point of confirm[] of the step",
                   {List("missing", "The points that are missing", Type::kString, false)},
                   Both({"Bestätigung fehlt", "Vor dem Start müssen alle Punkte bestätigt sein.",
                         "Alle Punkte prüfen und bestätigen."},
                        {"Confirmation missing", "Every point has to be confirmed before the start.",
                         "Check and confirm every point."})),
           refused("NOT_SKIPPABLE", "The step cannot be skipped", {},
                   Both({"Nicht überspringbar", "Dieser Schritt lässt sich nicht überspringen.",
                         "Den Schritt ausführen."},
                        {"Can't be skipped", "This step can't be skipped.", "Do the step."})),
           refused("INVALID_VALUE", "A value of setup.answer or the reason of setup.skip is not one the step takes",
                   {Text("name", "The input or field", false)},
                   Both({"Wert ungültig", "Der eingegebene Wert passt nicht zu diesem Schritt.",
                         "Einen Wert aus dem angezeigten Bereich wählen."},
                        {"Invalid value", "The value entered doesn't fit this step.",
                         "Pick a value from the range shown."})),
           refused("WRONG_METHOD", "The step is not done with this method: see start and inputs of the step", {},
                   Both({"So nicht möglich", "Dieser Schritt wird auf andere Weise erledigt.",
                         "Die Einrichtung neu öffnen und der Anzeige folgen."},
                        {"Not possible this way", "This step is done another way.",
                         "Open the setup again and follow what it shows."})),
           refused("EMERGENCY", "An emergency stands and the step restarts the OS", {},
                   Both({"Notaus steht", "Für diesen Schritt startet MowBite OS neu. Das geht nur ohne Notaus.",
                         "Den Notaus aufheben, dann den Schritt starten."},
                        {"Emergency stop is on",
                         "MowBite OS restarts for this step. That takes a mower without an emergency stop.",
                         "Release the emergency stop, then start the step."})),
           DriverMissing("info"), NotWritable("error")},
          "info", "user"};
}

EventEntry SetupBlocked() {
  return {"SETUP_BLOCKED", "answer",
          "The reasons of blocked[] of a step of setup.get and setup/json: what keeps it from going on right now; "
          "never an event",
          {Text("reason", "Why: a fixed name in capitals"), Text("message", "English text")},
          {NotWritable("error"), DriverMissing("warning"), Busy("info"),
           Reason("EMERGENCY", "info", "user", "An emergency is latched that the program does not release itself", {},
                  Both({"Notaus aufheben", "Ein Notaus steht noch. Erst danach geht es weiter.",
                        "Den Notaus in der App aufheben."},
                       {"Release the emergency stop", "An emergency stop is still on. It goes on only after it.",
                        "Release the emergency stop in the app."})),
           Reason("INPUT_ACTIVE", "info", "user",
                  "The input of the part is still switched: before its action, or after it while its emergency waits "
                  "for the input to be free",
                  {},
                  Both({"Noch ausgelöst", "Der Eingang ist noch ausgelöst. Es geht weiter, sobald er in Ruhe ist.",
                        "Taste, Stoßfänger oder Mäher loslassen bzw. absetzen."},
                       {"Still switched", "The input is still switched. It goes on once it is at rest.",
                        "Let go of the button or bumper, or put the mower down."})),
           Reason("NO_DATA", "warning", "user", "A source the step needs sends nothing",
                  {Text("source", "imu/data or gps/absolute_pose", false)},
                  Both({"Keine Daten", "Lagesensor oder GPS liefern gerade keine Daten.",
                        "Mäher einschalten und die Kabel prüfen."},
                       {"No data", "Motion sensor or GPS send no data right now.",
                        "Switch the mower on and check the cables."}))},
          "info", "user"};
}

}  // namespace mowbite_api
