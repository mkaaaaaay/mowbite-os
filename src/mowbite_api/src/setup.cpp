#include "mowbite_api/setup.h"

#include <cstdio>
#include <set>

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

// What keeps a step of kind drive from starting; the same in SETUP_REFUSED and SETUP_BLOCKED.
std::vector<ReasonEntry> DriveBlocks() {
  return {Reason("NOT_OFFERED", "info", "user", "The driver does not offer what the step needs", {},
                 Both({"Nicht angeboten", "Die Platine dieses Mähers bietet diese Prüfung nicht an.",
                       "Den Schritt überspringen."},
                      {"Not offered", "The board of this mower doesn't offer this check.", "Skip the step."})),
          Reason("RESTARTS_LOW", "info", "user", "The launch has fewer than two restarts of the driver left",
                 {Integer("left", "The restarts left", false)},
                 Both({"Zu wenige Neustarts übrig",
                       "Der Treiber ist zuletzt mehrmals neu gestartet. Für diese Prüfung braucht es noch Reserve.",
                       "MowBite OS neu starten, dann den Schritt wiederholen."},
                      {"Too few restarts left",
                       "The driver restarted several times lately. This check needs some to spare.",
                       "Restart MowBite OS, then repeat the step."})),
          Reason("SAFETY_INPUTS", "info", "user", "A step input of the roles stop, lift or bumper is not ok",
                 {List("steps", "The steps to pass first", Type::kString, false)},
                 Both({"Erst die Sensoren prüfen",
                       "Bevor sich Räder drehen, müssen STOP-Taste, Anheben und Stoßfänger im Stand geprüft sein.",
                       "Die Prüfung der Eingänge abschließen; sie lässt sich hierfür nicht überspringen."},
                      {"Check the sensors first",
                       "Before any wheel turns, STOP button, lifting and bumper have to be checked standing.",
                       "Finish the check of the inputs; it can't be skipped for this."})),
          Reason("ON_DOCK", "info", "user", "The mower is on the charging contacts", {},
                 Both({"Auf der Ladestation", "Auf der Ladestation dreht der Mäher keine Räder.",
                       "Den Mäher von der Station nehmen und aufbocken."},
                      {"On the docking station", "The mower turns no wheels on the docking station.",
                       "Take the mower off the station and jack it up."})),
          Reason("NO_RTK", "info", "user", "A step on the ground needs an RTK fix: it compares its way with the GNSS",
                 {},
                 Both({"Keine genaue Position",
                       "Für die Fahrt am Boden braucht der Mäher eine genaue Position (RTK fix).",
                       "Den Mäher unter freien Himmel stellen und warten, bis die Position genau ist."},
                      {"No exact position", "Driving on the ground needs an exact position (RTK fix).",
                       "Put the mower under the open sky and wait until the position is exact."})),
          Reason("SAFETY_AIR", "info", "user",
                 "The lock is closed: a step on the ground needs every safety step jacked up passed first",
                 {List("steps", "The steps to pass first", Type::kString, false)},
                 Both({"Erst aufgebockt prüfen",
                       "Bevor der Mäher am Boden fährt, müssen die Prüfungen mit aufgebocktem Mäher bestanden sein.",
                       "Die genannten Schritte mit aufgebocktem Mäher abschließen."},
                      {"Check jacked up first",
                       "Before the mower drives on the ground, the checks with the mower jacked up have to pass.",
                       "Finish the steps named with the mower jacked up."})),
          Reason("BLADE_NOT_ALLOWED", "info", "user",
                 "hw/capabilities has blade_allowed false although the parameter file of the program sets allow_blade: "
                 "the file kept by hand or the driver overrules it",
                 {Text("key", "<node>.allow_blade where the file kept by hand sets it", false)},
                 Both({"Messer gesperrt",
                       "Der Treiber lässt das Messer nicht drehen. Die von Hand gepflegte Einstellungsdatei oder der "
                       "Treiber selbst sperrt es.",
                       "allow_blade in der Einstellungsdatei von Hand prüfen, dann MowBite OS neu starten."},
                      {"Blade locked",
                       "The driver doesn't let the blade turn. The settings file kept by hand or the driver itself "
                       "locks it.",
                       "Check allow_blade in the settings file kept by hand, then restart MowBite OS."}))};
}

// What holds for every check of kind drive.
std::string DriveRules() {
  return "Every step that moves the wheels: jacked up (motion.raised) setup.start confirms raised and hand_at_stop, on "
         "the ground area_clear; the mower is in IDLE, off the charging contacts, without emergency or hold; every "
         "step input of the roles stop, lift and bumper of the list is ok (SAFETY_INPUTS, not to be skipped). The "
         "mower is in the sub state SETUP meanwhile. setup.keep at least every " +
         Num(kSetupKeepTimeout) + " s keeps it going (else RELEASED), setup.abort and any emergency it did not ask "
         "for stop it; a single move lasts " + Num(kSetupMoveTime) + " s at most; motion of the step names its bounds";
}

SetupParam Speed() {
  return Amount("speed", "m/s", 0.02, kSetupRaisedSpeed, 0.05, "Speed of each wheel");
}

// Wheels stand still within this time of the trigger and go at most this far after it (m).
SetupParam StandTime(std::string name, double def, std::string text) {
  return Seconds(std::move(name), 0.1, 3, def, std::move(text));
}

SetupParam StandWay(std::string name, double def, std::string text) {
  return Amount(std::move(name), "m", 0.05, 1, def, std::move(text));
}

SetupParam StopMax() {
  return Seconds("stop_max", 0.1, 3, 1, "The wheels stand within this time of the trigger");
}

SetupParam Hold() {
  return Seconds("hold", 0.5, 5, 1, "No wheel starts again for this time, after the stop and after the release");
}

SetupParam Emergencies() {
  std::vector<std::string> names;
  for (const auto& bit : EmergencyBits()) {
    if (bit.name != "LATCH") names.push_back(bit.name);
  }
  return Words("emergency", "The emergencies, by the names of x-emergency-bits, of which the driver shows at least one "
                            "after it, besides LATCH and no other",
               names, true);
}

// The reasons a check of kind drive can fail with: its own, then those of all of them.
std::vector<std::string> DriveReasons(std::vector<std::string> own) {
  own.insert(own.begin(), "NO_START");
  for (const char* reason : {"LOGIC_REFUSED", "RELEASED", "EMERGENCY", "NO_DATA", "STOPPED", "INTERRUPTED"}) {
    own.push_back(reason);
  }
  return own;
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
      {"hand_at_stop", "A hand is at a STOP button, to press it at any time",
       Named("Hand an der STOP-Taste", "Hand at the STOP button")},
      {"blade_removed", "The blade is taken off", Named("Messer abgenommen", "Blade removed")},
      {"clear", "Nobody is close to the mower", Named("Niemand in der Nähe", "Nobody close by")},
      {"area_clear", "3 m are clear in front of and behind the mower, and the user stands next to it at a STOP button",
       Named("3 m frei vor und hinter dem Mäher, ich stehe daneben an der STOP-Taste",
             "3 m clear in front of and behind the mower, I stand next to it at the STOP button")},
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

const std::vector<SetupName>& SetupWheelParts() {
  static const std::vector<SetupName> parts = {
      {"forward", "Both wheels forwards at speed", Named("Vorwärts", "Forwards")},
      {"backward", "Both wheels backwards at speed", Named("Rückwärts", "Backwards")},
      {"left", "A turn to the left on the spot, each wheel at speed", Named("Nach links drehen", "Turn to the left")},
      {"right", "A turn to the right on the spot, each wheel at speed",
       Named("Nach rechts drehen", "Turn to the right")},
      {"one_wheel", "Only the right wheel turns forwards at speed, the left one stands; the step asks which one turns",
       Named("Nur ein Rad", "One wheel only")},
  };
  return parts;
}

const std::vector<SetupName>& SetupGroundParts() {
  static const std::vector<SetupName> parts = {
      {"straight", "About distance m straight on at speed, then no command: it stands by itself",
       Both({"Geradeaus", "Der Mäher fährt etwa 2 m geradeaus und bleibt von selbst stehen.", ""},
            {"Straight on", "The mower drives about 2 m straight on and stops by itself.", ""})},
      {"turn", "A turn on the spot by angle to the left, then back to the right",
       Both({"Drehen", "Der Mäher dreht sich auf der Stelle nach links und wieder zurück.", ""},
            {"Turn", "The mower turns on the spot to the left and back again.", ""})},
      {"stop", "Back towards the start at speed until the user presses STOP",
       Both({"STOP drücken", "Der Mäher fährt zurück zum Start. Jetzt die STOP-Taste drücken.", ""},
            {"Press STOP", "The mower drives back towards the start. Press the STOP button now.", ""})},
      {"bumper", "Slowly in bumper_direction against a soft obstacle the user holds, until the bumper stops it",
       Both({"Stoßfänger", "Der Mäher fährt langsam an. Ein weiches Hindernis vor den Stoßfänger halten und "
                           "andrücken lassen.", ""},
            {"Bumper", "The mower drives slowly. Hold a soft obstacle against the bumper and let it press.", ""})},
  };
  return parts;
}

const std::vector<SetupName>& SetupBladeParts() {
  static const std::vector<SetupName> parts = {
      {"on_off", "On at percent until rpm, then off: it stands within off_max s and stays for hold s",
       Both({"An und aus", "Das Messer läuft an und wird wieder abgeschaltet.", ""},
            {"On and off", "The blade starts and is switched off again.", ""})},
      {"lease", "On with one lease that is not renewed: the driver switches it off, it stands within lease plus "
                "lease_extra s",
       Both({"Abschalten ohne Verlängerung", "Das Messer läuft, ohne dass es weiter angefordert wird.", ""},
            {"Off without renewal", "The blade runs without being asked for any more.", ""})},
      {"silent", "On, then hw/silence with the blade: the board stops it without its host within silent_max s",
       Both({"Abschalten ohne Steuerung", "Das Messer läuft, dann schweigt die Steuerung.", ""},
            {"Off without control", "The blade runs, then the control stays silent.", ""})},
      {"stop", "On until the user presses STOP: it stands within stop_max s of the input",
       Both({"STOP drücken", "Das Messer läuft. Jetzt die STOP-Taste drücken.", ""},
            {"Press STOP", "The blade runs. Press the STOP button now.", ""})},
      {"lift", "On until the user lifts the mower: it stands within stop_max s of the input",
       Both({"Anheben", "Das Messer läuft. Jetzt den Mäher vorn anheben, bis der Haken kommt, und absetzen.", ""},
            {"Lift", "The blade runs. Lift the mower at the front until the tick comes, then put it down.", ""})},
      {"bumper", "On until the user presses the bumper: it stands within stop_max s of the input",
       Both({"Stoßfänger", "Das Messer läuft. Jetzt den Stoßfänger eindrücken, bis der Haken kommt.", ""},
            {"Bumper", "The blade runs. Press the bumper in until the tick comes.", ""})},
  };
  return parts;
}

const SetupName& SetupWheelQuestion() {
  static const SetupName question = {
      "wheel", "Which wheel turns while the part one_wheel runs",
      Both({"Welches Rad dreht?", "Jetzt sollte sich nur ein Rad drehen. Angeben, welches sich dreht.", ""},
           {"Which wheel turns?", "Only one wheel should turn now. Tell which one turns.", ""})};
  return question;
}

const std::vector<SetupName>& SetupWheelValues() {
  static const std::vector<SetupName> values = {
      {"left", "The left wheel, seen from behind", Named("Linkes Rad", "Left wheel")},
      {"right", "The right wheel, seen from behind", Named("Rechtes Rad", "Right wheel")},
      {"both", "Both wheels", Named("Beide", "Both")},
      {"none", "No wheel", Named("Keines", "None")},
  };
  return values;
}

const SetupName& SetupPressed() {
  static const SetupName question = {
      "pressed", "Whether the user did the action of the part the step saw nothing of",
      Both({"Hast du es gemacht?",
            "Der Mäher hat in der Zeit nichts bemerkt. Hast du gedrückt, angehoben oder gekippt, wie angezeigt?", ""},
           {"Did you do it?",
            "The mower noticed nothing in time. Did you press, lift or tilt as shown?", ""})};
  return question;
}

const std::vector<SetupName>& SetupPressedValues() {
  static const std::vector<SetupName> values = {
      {"yes", "The action was done: the sensor or its line is dead, the step fails with NOT_SEEN",
       Named("Ja, gemacht", "Yes, I did")},
      {"no", "The action was not done: the step ends open with NOT_DONE", Named("Nein, noch nicht", "No, not yet")},
  };
  return values;
}

const std::vector<std::string>& ShapeKinds() {
  static const std::vector<std::string> kinds = {"body", "wheel", "caster", "bumper", "button", "sensor", "handle"};
  return kinds;
}

namespace {

// The checks of the safety of the mower: the inputs, the IMU and everything that moves the wheels or the blade.
const std::set<std::string>& SafetyChecks() {
  static const std::set<std::string> names = {"input",   "imu",        "wheels", "stop_moving", "no_command",
                                              "silence", "driver_end", "ground", "blade"};
  return names;
}

// Reasons of SETUP_STEP that close the lock when a safety step fails with them: what was measured broke a bound or
// the hardware did not do what it has to. NOT_SEEN only comes after the user said yes to the question pressed.
const std::set<std::string>& ClosingReasons() {
  static const std::set<std::string> codes = {"LIMIT",       "STARTED_AGAIN", "NO_EMERGENCY", "WRONG_DIRECTION",
                                              "WRONG_WHEEL", "NO_STANDSTILL", "NOT_SEEN"};
  return codes;
}

std::vector<SetupCheck> WithSafety(std::vector<SetupCheck> list) {
  for (auto& check : list) check.safety = SafetyChecks().count(check.name) > 0;
  return list;
}

}  // namespace

const std::vector<SetupCheck>& SetupChecks() {
  static const std::vector<SetupCheck> checks = WithSafety({
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
        "INTERRUPTED", "NOT_DONE"}},
      {"inputs_assign", "input", true, false,
       "Finds the line of every sensor, on a platform whose driver learns its inputs: the OS restarts with the driver "
       "watching its candidate lines (emergency INPUTS_LEARNING, nothing moves), watches them at rest for quiet, then "
       "takes one part after the other, each of a role (stop, lift, bumper_left or bumper_right) and with the delay_ms "
       "its line gets: one line alone changes, ask of the step names it, and setup.answer with accept yes takes it. "
       "After the last part the table goes into the settings the program writes for the platform, the OS restarts "
       "without learning and the step holds hw/inputs against the table. A board the learning driver held stays "
       "held after that restart (HIGH_LEVEL) until emergency.reset. setup.abort ends it with a restart as well. "
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
       {"NOT_SEEN", "WRONG_SIGN", "NO_EMERGENCY", "NO_DATA", "STOPPED", "INTERRUPTED", "NOT_DONE"},
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
      {"wheels", "drive", true, false,
       "The mower jacked up turns its wheels part by part, see wheel_parts: each part drives time s after a run-up of "
       "0.5 s, and the odometry has to show it with its sign and 0.5 to 1.5 times speed. In the part one_wheel only "
       "the right wheel turns, and inputs[] asks which one turns. " + DriveRules(),
       {Speed(), Seconds("time", 1, 3, 2, "Each part drives this long"),
        Seconds("answer_time", 3, kSetupMoveTime, kSetupMoveTime, "The part one_wheel waits this long for the answer")},
       {{"speed", "m/s", "Speed of the wheels along the part, one value by part with the id of the part",
         Named("Geschwindigkeit", "Speed")}},
       DriveReasons({"WRONG_DIRECTION", "WRONG_WHEEL", "LIMIT", "NOT_SEEN", "NOT_DONE"}),
       Both({"Räder", "Der Mäher ist aufgebockt und dreht seine Räder langsam vor, zurück und zu beiden Seiten. Zum "
                      "Schluss dreht nur ein Rad: angeben, welches.",
             "Dreht ein Rad falsch herum oder das falsche: die Anschlüsse der Motoren und die Einstellungen der "
             "Plattform prüfen."},
            {"Wheels", "The mower is jacked up and turns its wheels slowly forwards, backwards and to both sides. At "
                       "the end only one wheel turns: tell which one.",
             "If a wheel turns the wrong way or the wrong one turns, check the plugs of the motors and the settings of "
             "the platform."})},
      {"stop_moving", "drive", true, false,
       "While the wheels turn, the user triggers the input of a part (role as in a step input): the wheels stand "
       "within stop_max, the emergency of the role follows, nothing starts again for hold s, the program releases the "
       "emergency itself as in a step input, nothing starts for hold s, and a new command turns the wheels again. "
       + DriveRules(),
       {Speed(), Seconds("time", 3, kSetupMoveTime, 8, "The wheels turn at most this long for the action of a part"),
        StopMax(), Hold()},
       {{"stand", "s", "From the input to standing wheels, one value by part with the id of the part",
         Named("Standzeit", "Time to stand")}},
       DriveReasons({"NOT_SEEN", "LIMIT", "STARTED_AGAIN", "NO_EMERGENCY", "NOT_RELEASED", "NO_RESTART", "NOT_DONE"}),
       Both({"Stopp bei drehenden Rädern",
             "Die Räder drehen langsam, der Mäher ist aufgebockt. Den angezeigten Sensor auslösen: die Räder müssen "
             "sofort stehen und stehen bleiben.",
             "Bleiben die Räder nicht stehen: den Mäher nicht fahren lassen, Sensor, Kabel und Platine prüfen."},
            {"Stop with turning wheels",
             "The wheels turn slowly with the mower jacked up. Trigger the sensor shown: the wheels have to stand at "
             "once and stay so.",
             "If the wheels do not stand, don't let the mower drive; check sensor, cable and board."})},
      {"no_command", "drive", true, false,
       "While the wheels turn, the program sends no commands any more: the wheels stand within stop_max of the last "
       "one by the watchdogs alone and do not start for hold s; then a new command turns them again. No emergency. "
       + DriveRules(),
       {Speed(), StopMax(), Hold()},
       {{"stand", "s", "From the last command to standing wheels", Named("Standzeit", "Time to stand")}},
       DriveReasons({"LIMIT", "STARTED_AGAIN", "NO_RESTART"}),
       Both({"Stopp ohne Fahrbefehl",
             "Die Räder drehen langsam, dann kommen keine Fahrbefehle mehr. Die Räder müssen von selbst stehen "
             "bleiben.",
             "Drehen die Räder weiter: den Mäher nicht fahren lassen und die Wächter der Plattform prüfen."},
            {"Stop without commands",
             "The wheels turn slowly, then no commands come any more. The wheels have to stand by themselves.",
             "If the wheels keep turning, don't let the mower drive; check the watchdogs of the platform."})},
      {"silence", "drive", true, false,
       "Round by round, while the wheels turn: hw/silence for silence_max_s of the driver, commands go on. The board "
       "stops the wheels by itself within stop_max of the answer, nothing starts until hold s after the silence; then "
       "the driver shows emergencies of the list and LATCH only, the program releases them itself, nothing starts for "
       "hold s and a new command turns the wheels. The stand times of the rounds differ by spread_max at most. "
       "Offered where hw/capabilities has silence_max_s. " + DriveRules(),
       {Speed(), StopMax(), Hold(), Amount("rounds", "", 1, 10, 5, "How many rounds"),
        Seconds("spread_max", 0.05, 1, 0.3, "Stand times of the rounds differ this much at most"), Emergencies()},
       {{"stand", "s", "From the answer of hw/silence to standing wheels, one value by round, round_1 and on",
         Named("Runde", "Round")},
        {"spread", "s", "Longest less shortest stand of the rounds", Named("Streuung", "Spread")}},
       DriveReasons({"DRIVER_REFUSED", "LIMIT", "STARTED_AGAIN", "NO_EMERGENCY", "NOT_RELEASED", "NO_RESTART"}),
       Both({"Stopp ohne Steuerung",
             "Die Räder drehen langsam, dann schweigt die Steuerung einige Sekunden, mehrmals hintereinander. Die "
             "Platine muss die Räder jedes Mal von selbst anhalten.",
             "Hält die Platine nicht an: den Mäher nicht fahren lassen und die Einstellungen der Platine prüfen."},
            {"Stop without control",
             "The wheels turn slowly, then the control stays silent for some seconds, several times. The board has to "
             "stop the wheels by itself each time.",
             "If the board does not stop, don't let the mower drive; check the settings of the board."})},
      {"driver_end", "drive", true, false,
       "While the wheels turn: hw/end, commands go on. The driver is back within return_max (hw/status fresh after a "
       "gap), the wheels stand and do not start for hold s, the driver shows emergencies of the list and LATCH only, "
       "the program releases them itself, nothing starts for hold s and a new command turns the wheels. Offered where "
       "hw/capabilities has driver_end and the launch has at least two restarts of the driver left. " + DriveRules(),
       {Speed(), Hold(), Seconds("return_max", 5, 60, 20, "The driver is back within this time"), Emergencies()},
       {{"return", "s", "From the answer of hw/end to the first hw/status of the new driver",
         Named("Treiber zurück nach", "Driver back after")}},
       DriveReasons({"DRIVER_REFUSED", "NO_RETURN", "STARTED_AGAIN", "NO_EMERGENCY", "NOT_RELEASED", "NO_RESTART"}),
       Both({"Stopp bei Treiberende",
             "Die Räder drehen langsam, dann beendet sich der Treiber der Platine und startet neu. Die Räder müssen "
             "stehen und gesperrt bleiben, bis das Programm sie freigibt.",
             "Bleiben die Räder nicht stehen oder kommt der Treiber nicht zurück: den Mäher nicht fahren lassen und "
             "MowBite OS neu starten."},
            {"Stop when the driver ends",
             "The wheels turn slowly, then the driver of the board ends and starts again. The wheels have to stand and "
             "stay locked until the program releases them.",
             "If the wheels do not stand or the driver does not come back, don't let the mower drive and restart "
             "MowBite OS."})},
      {"ground", "drive", true, false,
       "On the ground, the parts of ground_parts in one run. straight: about distance m forwards at speed, then the "
       "program sends nothing: the mower stands within quiet_max s and goes quiet_way m at most after the last "
       "command, and the way by odometry and by gps/absolute_pose differ by the share ratio at most. turn: angle deg "
       "to the left on the spot at turn_rate, then back to the right: the IMU turns with the sign of each command and "
       "by the odometry within the share ratio. stop: back towards the start at speed, at most as far as straight "
       "went, until the user presses STOP: the mower stands within stop_max s and goes stop_way m at most after the "
       "press, the emergency follows, nothing starts for hold s, the program releases it itself and nothing starts "
       "for hold s. bumper: at bumper_speed in bumper_direction against a soft obstacle the user holds, the same as "
       "stop for the bumper. No new command after a stop on the ground. A stop or bumper the step saw nothing of "
       "within time asks pressed. Needs an RTK fix (NO_RTK) and, while the lock is closed, every safety step jacked up "
       "of the list passed (SAFETY_AIR). Not jacked up: mowbite_safety holds it to its bounds on the ground, without "
       "pose and fence. " + DriveRules(),
       {Amount("speed", "m/s", 0.05, kSetupGroundSpeed, kSetupGroundSpeed, "Speed of straight and stop"),
        Amount("distance", "m", 1, 2.5, 2, "How far straight goes before the program falls silent"),
        Amount("turn_rate", "rad/s", 0.25, kSetupGroundTurn, 0.3, "Turn rate of turn"),
        Amount("angle", "deg", 45, 120, 90, "How far turn goes to each side"),
        Amount("ratio", "", 0.05, 0.5, 0.2,
               "Share by which the way by GNSS may differ from the odometry, and the turn of the IMU from it"),
        Amount("bumper_speed", "m/s", 0.03, 0.1, 0.1, "Speed of bumper"),
        Word("bumper_direction", "The way bumper drives: where the bumper is", {"forward", "backward"}, false,
             "forward"),
        Seconds("time", 3, kSetupMoveTime, 8, "stop and bumper drive at most this long for the action"),
        StandTime("quiet_max", 1, "straight: the mower stands within this time of the last command"),
        StandWay("quiet_way", 0.6, "straight: the mower goes at most this far after the last command"),
        StandTime("stop_max", 0.5, "stop and bumper: the mower stands within this time of the trigger"),
        StandWay("stop_way", 0.3, "stop and bumper: the mower goes at most this far after the trigger"), Hold()},
       {{"stand", "s", "From the last command (straight) or the trigger to standing wheels, by part: <part>_stand",
         Named("Standzeit", "Time to stand")},
        {"way", "m", "The way by odometry from the last command or the trigger until the wheels stand, by part: "
                     "<part>_way",
         Named("Weg bis zum Stand", "Way until standing")},
        {"gnss", "", "straight: the way by GNSS divided by the way by odometry", Named("Weg nach GPS", "Way by GPS")},
        {"turn", "", "turn: the turn of the IMU divided by the one of the odometry, by side: turn_left, turn_right",
         Named("Drehung Lagesensor", "Turn of the motion sensor")}},
       DriveReasons({"WRONG_DIRECTION", "LIMIT", "NO_STANDSTILL", "STARTED_AGAIN", "NO_EMERGENCY", "NOT_RELEASED",
                     "NOT_SEEN", "NOT_DONE", "NO_RTK"}),
       Both({"Fahren am Boden",
             "Der Mäher fährt etwa 2 m geradeaus und bleibt von selbst stehen, dreht sich nach links und zurück, fährt "
             "zurück, bis du STOP drückst, und zum Schluss langsam gegen ein weiches Hindernis, das du hältst.",
             "Bleibt der Mäher nicht rechtzeitig stehen: nicht mähen lassen. Die Werte zeigen, wo es hakt; Räder, "
             "Bremsweg und Platine prüfen."},
            {"Driving on the ground",
             "The mower drives about 2 m straight on and stops by itself, turns to the left and back, drives back "
             "until you press STOP, and at the end slowly against a soft obstacle you hold.",
             "If the mower does not stop in time, don't let it mow. The values show where it fails; check wheels, "
             "braking distance and board."})},
      {"blade", "blade", true, false,
       "The parts of blade_parts in one run, the blade taken off and the wheels without a command. Where "
       "hw/capabilities has blade_allowed false, setup.start writes allow_blade for node into the parameter file of "
       "the program and restarts the OS (restart of the step): after it a second setup.start runs the step, never "
       "the restart itself; a blade the driver still does not allow blocks with BLADE_NOT_ALLOWED. Each part starts "
       "the blade at percent and height through the logic (setup/motion blade) until it turns at rpm within spin_up "
       "s. on_off: off, it stands (below still_rpm) within off_max s. lease: one request on lease s that is not "
       "renewed, the driver switches it off: it stands within lease plus lease_extra s of that request. silent: "
       "hw/silence with blade: the board stops it within silent_max s of the answer, then the driver shows "
       "emergencies of the list and LATCH only and the program releases them itself. stop, lift and bumper: the user "
       "triggers the input of the role within time s: it stands within stop_max s of the input, the emergency of the "
       "role follows and the program releases it itself. After every stand nothing turns for hold s, after a release "
       "again. A part the step saw nothing of within time asks pressed. Needs the inputs checked standing "
       "(SAFETY_INPUTS) and a driver that offers hw/silence; the mower is in IDLE off the charging contacts, without "
       "emergency or hold, in the sub state SETUP meanwhile, the blade turns only there and goes off when it ends. "
       "setup.start confirms blade_removed and hand_at_stop; setup.keep at least every " +
           Num(kSetupKeepTimeout) + " s keeps it going (else RELEASED).",
       {Word("node", "The node of the driver allow_blade is written for", {}, true),
        Amount("percent", "%", 1, 100, 100, "Speed of the blade in percent of the driver"),
        Amount("height", "mm", 0, 120, 0, "Cutting height while it turns, 0 for the default of the driver"),
        Amount("rpm", "rpm", 100, 10000, 2000, "The blade counts as turning from this speed on"),
        Seconds("spin_up", 1, 10, 3, "The blade turns at rpm within this time of the request"),
        Amount("still_rpm", "rpm", 10, 1000, 100, "Below this speed the blade stands"),
        Seconds("off_max", 0.2, 6, 1, "on_off: the blade stands within this time of the off"),
        Seconds("lease", 0.5, 5, 1, "Lease of the requests of the step, at most blade_lease_max_s of the driver"),
        Seconds("lease_extra", 0.2, 6, 1, "lease: the blade stands within lease and this time of the request"),
        Seconds("silent_max", 0.2, 6, 1, "silent: the blade stands within this time of the answer of hw/silence"),
        Seconds("stop_max", 0.2, 6, 1, "stop, lift and bumper: the blade stands within this time of the input"),
        Seconds("time", 3, kSetupMoveTime, 8, "stop, lift and bumper: the blade turns at most this long for the "
                                               "action"),
        Seconds("hold", 0.5, 5, 1, "The blade does not turn for this time, after it stood and after the release"),
        Emergencies()},
       {{"rpm", "rpm", "on_off: the speed the blade reached", Named("Drehzahl", "Speed")},
        {"stand", "s", "From the off, the request not renewed, the answer of hw/silence or the input to the standing "
                       "blade, by part with the id of the part",
         Named("Standzeit", "Time to stand")}},
       {"NO_START", "LIMIT", "STARTED_AGAIN", "NO_EMERGENCY", "NOT_RELEASED", "DRIVER_REFUSED", "NOT_SEEN",
        "NOT_DONE", "LOGIC_REFUSED", "RELEASED", "EMERGENCY", "NO_DATA", "STOPPED", "INTERRUPTED"},
       Both({"Messer",
             "Die Messerscheibe ist ab. Der Motor läuft an und wird auf verschiedene Weise gestoppt: abschalten, ohne "
             "Verlängerung, ohne Steuerung, dann mit STOP-Taste, Anheben und Stoßfänger. Er muss jedes Mal "
             "rechtzeitig stehen und darf nicht wieder anlaufen.",
             "Steht der Motor nicht rechtzeitig oder läuft er wieder an: nicht mähen lassen, Platine und Motor "
             "prüfen."},
            {"Blade",
             "The blade disc is off. The motor starts and is stopped in several ways: switched off, without renewal, "
             "without control, then with the STOP button, lifting and the bumper. It has to stand in time each time "
             "and must not start again.",
             "If the motor does not stand in time or starts again, don't let the mower mow; check board and motor."})},
  });
  return checks;
}

bool SetupClosesLock(const std::string& check, const std::string& reason) {
  return SafetyChecks().count(check) > 0 && ClosingReasons().count(reason) > 0;
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
  Json wheel = {{"description", SetupWheelQuestion().text},
                {"texts", Texts(SetupWheelQuestion().texts)},
                {"values", named(SetupWheelValues())}};
  Json pressed = {{"description", SetupPressed().text},
                  {"texts", Texts(SetupPressed().texts)},
                  {"values", named(SetupPressedValues())},
                  {"time", kSetupPressedTime}};
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
                  {"reasons", check.reasons},  {"safety", check.safety}};
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
           "once it is ok, the ways to restart, the parts of a step wheels and its question, the parts of the steps "
           "ground and blade, the question pressed of a safety step that saw nothing (a check with safety closes the lock when "
           "it fails with a reason that has closes in x-reasons of SETUP_STEP), and the limits; texts by language "
           "where the OS has them. The steps themselves come with setup.get"},
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
          {"wheel_parts", named(SetupWheelParts())},
          {"ground_parts", named(SetupGroundParts())},
          {"blade_parts", named(SetupBladeParts())},
          {"wheel", wheel},
          {"pressed", pressed},
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
            {"shape_points", kShapeMaxPoints},
            {"raised_speed", kSetupRaisedSpeed},
            {"raised_turn", kSetupRaisedTurn},
            {"ground_speed", kSetupGroundSpeed},
            {"ground_turn", kSetupGroundTurn},
            {"move_time", kSetupMoveTime},
            {"keep", kSetupKeepTimeout},
            {"closed", kSetupMaxClosed}}},
          {"shape_kinds", ShapeKinds()}};
}

namespace {

Field Closed() {
  Field closed = Rows("closed", "What keeps the lock closed: a failed safety step until it passes again, or a reset",
                      {Nullable(Text("step", "The step, null for a reset")), Nullable(Text("part", "The part")),
                       Text("reason", "A reason of SETUP_STEP, or for a reset RESET or UNREADABLE"),
                       Number("t", "unix s", "When")});
  closed.max = kSetupMaxClosed;
  return closed;
}

}  // namespace

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
       Text("kind", "How to show it: confirm, input, measure, drive or blade; a kind the app does not know as text "
                    "with a button"),
       Text("state", "locked: a step of needs is not ok; open; running; ok; failed; skipped; stale", true,
            SetupStates()),
       Bool("safety", "A safety step: failing it with a reason that closes closes the lock, passing it opens it again"),
       Integer("attempt", "How often it was started or answered"),
       List("needs", "Steps that have to be ok before this one", Type::kString),
       Rows("blocked", "What keeps it from going on right now, reasons of SETUP_BLOCKED in x-reasons",
            {Text("reason", "A fixed name in capitals"), Text("message", "English text")}),
       Nullable(Object("motion", "What moves in the step, null for nothing",
                       {Bool("wheels", "The program turns the wheels"), Bool("blade", "The program turns the blade"),
                        Bool("raised", "The mower is jacked up"),
                        Number("max_speed", "m/s", "The fastest a wheel turns"),
                        Nullable(Number("max_distance", "m", "The longest way on the ground, null while jacked up")),
                        Number("max_time", "s", "The longest single move")})),
       Rows("confirm", "Points setup.start has to name in confirm", {Text("id", "Name of the point"), TextsField()}),
       Bool("start", "setup.start begins it"),
       Bool("auto", "Its ticks come without a tap"),
       Bool("skippable", "setup.skip takes it"),
       Rows("skip_reasons", "Reasons setup.skip takes", {Text("id", "Name of the reason"), TextsField()}),
       Rows("inputs", "What setup.answer takes in values",
            {Text("name", "Key in values; in a step inputs_assign accept and delay_<part>, see x-setup.accept and "
                          "x-setup.delay; in a step wheels wheel, see x-setup.wheel"),
             Text("type", "choice or number", true, {"choice", "number"}),
             Text("unit", "Unit of a number", false), Number("min", "", "Least number", false),
             Number("max", "", "Largest number", false),
             Rows("values", "Values of a choice", {Text("id", "The value"), TextsField()}, false),
             Bool("ask", "A question of the running step that waits for its answer: accept while ask of the step is "
                         "set, wheel while the part one_wheel of a step wheels turns one wheel, pressed while a "
                         "safety step saw nothing of the action of a part"),
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
       Bool("restart", "The program restarts the OS for it by itself: a step inputs_assign for its result, a step "
                       "blade at its first start while the driver does not allow the blade")});
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
              {Bool("open", "The platform has a safety measurement and no safety step closed the lock"),
               Nullable(Text("date", "Its date, YYYY-MM-DD")),
               Nullable(Text("firmware", "The firmware it was made with, null while unknown")),
               Nullable(Text("why", "While closed: NO_SAFETY_MEASUREMENT (no date), CHECK_FAILED (a safety step "
                                    "failed) or RESET (the program was reset or its file could not be read)",
                             true, {"NO_SAFETY_MEASUREMENT", "CHECK_FAILED", "RESET"})),
               Nullable(Text("source", "Who gave the date: hand (the file kept by hand or the platform) or program "
                                       "(the summary of the setup program); null without one",
                             true, {"hand", "program"})),
               Closed()}),
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
  EventEntry entry = {"SETUP_STEP", "setup", "A step of the setup program ended",
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
           failed("NO_EMERGENCY",
                  "The input switched, or the silence or end of the driver came, but the emergency of the step did "
                  "not follow within its limit",
                  {Part(), Number("limit", "s", "The limit", false)},
                  Both({"Kein Notaus",
                        "Der Auslöser kam, aber der Mäher hat nicht rechtzeitig den Notaus gemeldet. So würde er im "
                        "Betrieb nicht anhalten.",
                        "Den Mäher nicht fahren lassen. Zuordnung und Einstellung des Auslösers prüfen, dann "
                        "wiederholen."},
                       {"No emergency stop",
                        "The trigger came, but the mower didn't report the emergency stop in time. It wouldn't stop "
                        "like this in use.",
                        "Don't let the mower drive. Check how the trigger is assigned and set, then repeat."})),
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
           failed("NO_START", "The wheels or the blade did not turn within the time after the first command",
                  {Part(), Number("rpm", "rpm", "A blade: the fastest it turned", false)},
                  Both({"Läuft nicht an",
                        "Räder oder Messer haben sich auf den Befehl nicht oder zu langsam gedreht.",
                        "Prüfen, ob der Mäher frei aufgebockt ist und die Motoren angeschlossen sind, dann "
                        "wiederholen."},
                       {"Not starting", "The wheels or the blade didn't turn on the command, or too slowly.",
                        "Check that the mower is jacked up freely and the motors are connected, then repeat."})),
           failed("STARTED_AGAIN", "A wheel or the blade turned while nothing was to turn: after the stop or the "
                  "release",
                  {Part(), Number("speed", "m/s", "The fastest wheel then", false),
                   Number("rpm", "rpm", "The blade then", false)},
                  Both({"Wieder angelaufen",
                        "Ein Rad oder das Messer hat sich gedreht, obwohl es stehen musste. So würde der Mäher nach "
                        "einem Stopp von selbst weiterlaufen.",
                        "Den Mäher nicht fahren lassen. Platine und Treiber prüfen, dann wiederholen."},
                       {"Started again",
                        "A wheel or the blade turned while it had to stand. Like this the mower would go on by itself "
                        "after a stop.",
                        "Don't let the mower drive. Check board and driver, then repeat."})),
           failed("NO_RESTART", "After the release a new command did not turn the wheels within its time", {Part()},
                  Both({"Kein Anlauf nach Freigabe",
                        "Nach der Freigabe hat ein neuer Fahrbefehl die Räder nicht gedreht.",
                        "Den Notaus in der App prüfen und aufheben, dann wiederholen."},
                       {"No start after the release", "After the release a new command didn't turn the wheels.",
                        "Check the emergency stop in the app and release it, then repeat."})),
           failed("WRONG_DIRECTION", "A part turned the wheels the other way than asked", {Part()},
                  Both({"Falsche Drehrichtung",
                        "Ein Rad hat sich anders herum gedreht als verlangt.",
                        "Anschlüsse der Motoren und die Drehrichtung in den Einstellungen der Plattform prüfen."},
                       {"Wrong direction", "A wheel turned the other way round than asked.",
                        "Check the plugs of the motors and their direction in the settings of the platform."})),
           failed("WRONG_WHEEL", "In the part one_wheel the answer or the odometry named another wheel than the right",
                  {Part(), Text("answer", "The wheel the user named", false)},
                  Both({"Falsches Rad",
                        "Es hat sich ein anderes Rad gedreht als das rechte. Die Motoren sind vertauscht.",
                        "Die Anschlüsse der Motoren tauschen oder die Seiten in den Einstellungen der Plattform "
                        "ändern, dann wiederholen."},
                       {"Wrong wheel", "Another wheel than the right one turned. The motors are swapped.",
                        "Swap the plugs of the motors or the sides in the settings of the platform, then repeat."})),
           failed("NOT_RELEASED", "The emergency the step brought did not end with the release of the program",
                  {Part()},
                  Both({"Freigabe gescheitert",
                        "Der Notaus des Schritts ließ sich nicht aufheben.",
                        "Den Notaus in der App aufheben, dann wiederholen."},
                       {"Release failed", "The emergency stop of the step couldn't be released.",
                        "Release the emergency stop in the app, then repeat."})),
           failed("NO_RETURN", "The driver was not back within return_max after hw/end",
                  {Number("limit", "s", "The limit", false)},
                  Both({"Treiber nicht zurück",
                        "Der Treiber der Platine ist nach seinem Ende nicht rechtzeitig neu gestartet.",
                        "MowBite OS neu starten, dann wiederholen."},
                       {"Driver not back", "The driver of the board didn't start again in time after its end.",
                        "Restart MowBite OS, then repeat."})),
           failed("DRIVER_REFUSED", "The driver refused hw/silence or hw/end",
                  {Text("code", "Its code: READ_ONLY, OUT_OF_RANGE, CHARGING, BLADE or BUSY", false), Part()},
                  Both({"Platine lehnt ab",
                        "Der Treiber der Platine hat die Prüfung abgelehnt, etwa weil der Mäher lädt oder das "
                        "Messer läuft.",
                        "Den Mäher von der Ladestation nehmen, das Messer stehen lassen, dann wiederholen."},
                       {"Board refused", "The driver of the board refused the check, for instance because the mower "
                                         "charges or the blade runs.",
                        "Take the mower off the docking station, let the blade stand, then repeat."})),
           failed("LOGIC_REFUSED", "The logic did not take the step into SETUP within its time",
                  {Text("mower_state", "current_state of the mower then", false), Part()},
                  Both({"Mäher nicht bereit",
                        "Der Mäher hat den Prüfschritt nicht angenommen: er ist unterwegs, lädt oder steht im "
                        "Notaus.",
                        "Den Mäher von der Station nehmen, Notaus aufheben, dann wiederholen."},
                       {"Mower not ready",
                        "The mower didn't take the step: it is on its way, charges or has an emergency stop.",
                        "Take the mower off the station, release the emergency stop, then repeat."})),
           failed("RELEASED", "setup.keep stayed away: the app lost its connection or let go of the step", {Part()},
                  Both({"Verbindung weg",
                        "Die App hat den Schritt nicht mehr begleitet, der Mäher hat angehalten.",
                        "Die App offen lassen und die Verbindung prüfen, dann wiederholen."},
                       {"Connection lost", "The app stopped following the step, so the mower stopped.",
                        "Keep the app open and check the connection, then repeat."})),
           failed("EMERGENCY", "An emergency the step did not ask for came up",
                  {Text("emergency", "The emergencies, by the names of x-emergency-bits", false), Part()},
                  Both({"Notaus",
                        "Ein Notaus hat den Schritt beendet.",
                        "Den Grund in der App ansehen, den Notaus aufheben und den Schritt wiederholen."},
                       {"Emergency stop", "An emergency stop ended the step.",
                        "See the reason in the app, release the emergency stop and repeat the step."})),
           failed("LIMIT", "A value is outside of its bounds",
                  {Part(), Text("value_id", "id of the value in values[] of the step", false),
                   {"value", Type::kAny, "", false, "What was measured", {}, true, {}, ""},
                   Number("limit", "", "The bound it broke", false), Text("unit", "Unit of both", false)},
                  Both({"Wert außerhalb",
                        "Ein Messwert liegt außerhalb seiner Grenze. Die Liste der Werte zeigt, welcher.",
                        "Die Ursache am genannten Wert beheben, dann den Schritt wiederholen."},
                       {"Value out of bounds", "A value is outside of its bound. The list of values shows which.",
                        "Fix the cause at the value named, then repeat the step."})),
           failed("NO_STANDSTILL", "The mower did not stand still within 3 s of the last command or of the trigger",
                  {Part(), {"value", Type::kAny, "", false, "What was measured", {}, true, {}, ""}},
                  Both({"Bleibt nicht stehen",
                        "Der Mäher ist nach dem Stopp nicht zum Stillstand gekommen.",
                        "Den Mäher nicht fahren lassen. Platine, Antrieb und Einstellungen der Plattform prüfen."},
                       {"Does not stop", "The mower did not come to a standstill after the stop.",
                        "Don't let the mower drive. Check board, drive and the settings of the platform."})),
           failed("NO_DATA", "A source the step needs sent nothing",
                  {Text("source", "hw/status, hw/inputs, hw/capabilities, imu/data or gps/absolute_pose", false),
                   Part()},
                  Both({"Keine Daten",
                        "Der Mäher bekommt für diesen Schritt keine Daten von Platine, Lagesensor oder GPS.",
                        "Mäher einschalten, Kabel prüfen, dann wiederholen."},
                       {"No data", "The mower gets no data from board, motion sensor or GPS for this step.",
                        "Switch the mower on, check the cables, then repeat."})),
           failed("NO_RTK", "No position of the solution, accuracy and rate asked for within the timeout, or the RTK "
                  "fix was lost in a step on the ground",
                  {Part()},
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
                       {"Interrupted", "MowBite OS restarted while the step ran.", "Start the step again."})),
           Reason("NOT_DONE", "info", "user",
                  "The step saw nothing of the action of a part, and the answer to pressed was no or did not come: "
                  "the step is open again and the lock stays as it was",
                  {Part()},
                  Both({"Nicht gemacht", "Der Handgriff wurde nicht gemacht. Die Prüfung zählt nicht.",
                        "Den Schritt wiederholen und den Handgriff in der angezeigten Zeit machen."},
                       {"Not done", "The action wasn't done. The check doesn't count.",
                        "Repeat the step and do the action within the time shown."}))},
          "warning", "user"};
  for (auto& reason : entry.reasons) reason.closes = ClosingReasons().count(reason.code) > 0;
  return entry;
}

EventEntry SetupProgramEvent() {
  return {"SETUP", "setup", "The setup program reached a mark",
          {Text("what", "stage_done: every step of stage is ok or skipped; complete: every step is; reset: "
                        "setup.reset took all results back; safety_closed: a safety step failed and closed the lock "
                        "of autonomous driving and mowing; safety_opened: the lock is open again; blade_allowed: the "
                        "first start of a step blade allowed the blade, the OS restarts and the step waits for its "
                        "next start",
                true, {"stage_done", "complete", "reset", "safety_closed", "safety_opened", "blade_allowed"}),
           Text("stage", "With stage_done: the stage", false),
           Text("step", "With safety_closed: the step that failed; with blade_allowed: the step blade", false),
           Text("cause", "With safety_closed: its reason of SETUP_STEP", false)},
          {}};
}

EventEntry SetupRefused() {
  const auto refused = [](std::string code, std::string text, std::vector<Field> fields, UserTexts texts) {
    return Reason(std::move(code), "info", "user", std::move(text), std::move(fields), std::move(texts));
  };
  EventEntry entry = {"SETUP_REFUSED", "answer",
                      "A method setup.* was refused: the data of its -32600, never an event",
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
           refused("EMERGENCY", "An emergency stands: a step that restarts the OS or moves the wheels needs none", {},
                   Both({"Notaus steht",
                         "Dieser Schritt startet MowBite OS neu oder dreht die Räder. Das geht nur ohne Notaus.",
                         "Den Notaus aufheben, dann den Schritt starten."},
                        {"Emergency stop is on",
                         "This step restarts MowBite OS or turns the wheels. That takes a mower without an emergency "
                         "stop.",
                         "Release the emergency stop, then start the step."})),
           DriverMissing("info"), NotWritable("error")},
          "info", "user"};
  for (auto& reason : DriveBlocks()) entry.reasons.push_back(std::move(reason));
  return entry;
}

EventEntry SetupBlocked() {
  EventEntry entry = {"SETUP_BLOCKED", "answer",
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
  for (auto& reason : DriveBlocks()) entry.reasons.push_back(std::move(reason));
  return entry;
}

}  // namespace mowbite_api
