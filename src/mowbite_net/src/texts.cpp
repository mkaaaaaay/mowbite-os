#include "mowbite_net/texts.h"

#include <algorithm>
#include <cctype>
#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_net {

namespace {

// id -> {de, en}.
const std::map<std::string, std::pair<std::string, std::string>>& Table() {
  static const std::map<std::string, std::pair<std::string, std::string>> table = {
      {"mower", {"Der Mäher", "The mower"}},
      {"emergency.title", {"Notaus", "Emergency stop"}},
      {"emergency.new", {"{n} steht im Notaus.", "{n} is in emergency stop."}},
      {"emergency.again",
       {"{n} steht immer noch im Notaus (seit {min} min).", "{n} is still in emergency stop (for {min} min)."}},
      {"emergency.spinup", {" Der Mähmotor lief nicht an.", " The mow motor didn't start."}},
      {"emergency.clear.title", {"Notaus aufgehoben", "Emergency stop cleared"}},
      {"emergency.clear", {"{n} ist wieder bereit.", "{n} is ready again."}},
      {"not_charging.title", {"Lädt nicht", "Not charging"}},
      {"not_charging.new",
       {"{n} steht im Dock, lädt aber nicht. Hat die Ladestation Strom?",
        "{n} is in the dock but not charging. Does the docking station have power?"}},
      {"charging_lost.contact",
       {"{n} hat keinen Kontakt mehr zur Ladestation und lädt nicht ({pct} %).",
        "{n} lost contact with the docking station and isn't charging ({pct} %)."}},
      {"charging_lost.power",
       {"{n} steht im Dock, lädt aber nicht ({pct} %). Hat die Ladestation Strom?",
        "{n} is in the dock but not charging ({pct} %). Does the docking station have power?"}},
      {"cant_go_home.title", {"Kann nicht heim", "Can't go home"}},
      {"cant_go_home.new",
       {"{n} konnte nicht zum Dock losfahren ({reason}) und wartet draußen.",
        "{n} couldn't set off for the dock ({reason}) and is waiting outside."}},
      {"dock_failed.title", {"Andocken fehlgeschlagen", "Docking failed"}},
      {"dock_failed.new",
       {"{n} hat das Andocken nach mehreren Versuchen aufgegeben und steht außerhalb des Docks.",
        "{n} gave up docking after several tries and is standing outside the dock."}},
      {"dock_failed.again",
       {"{n} steht immer noch außerhalb des Docks (seit {min} min).",
        "{n} is still outside the dock (for {min} min)."}},
      {"charging_again.title", {"Lädt wieder", "Charging again"}},
      {"back_in_dock.title", {"Wieder im Dock", "Back in the dock"}},
      {"charging_again", {"{n} lädt wieder.", "{n} is charging again."}},
      {"no_gps.title", {"Kein GPS nach dem Abdocken", "No GPS after undocking"}},
      {"no_gps.new",
       {"{n} ist aus dem Dock gekommen, hat aber kein GPS und steht vor dem Dock.",
        "{n} got out of the dock but has no GPS, it stands in front of the dock."}},
      {"no_gps.again",
       {"{n} steht immer noch ohne GPS vor dem Dock (seit {min} min).",
        "{n} still stands in front of the dock without GPS (for {min} min)."}},
      {"undock_failed.title", {"Abdocken fehlgeschlagen", "Undocking failed"}},
      {"undock_failed.new",
       {"{n} ist nicht aus dem Dock gekommen und hat angehalten.", "{n} didn't get out of the dock and stopped."}},
      {"undock_failed.again",
       {"{n} ist immer noch nicht aus dem Dock gekommen (seit {min} min).",
        "{n} still hasn't got out of the dock (for {min} min)."}},
      {"on_its_way.title", {"Wieder unterwegs", "On its way again"}},
      {"undocked", {"{n} ist aus dem Dock gekommen.", "{n} got out of the dock."}},
      {"moving_again", {"{n} bewegt sich wieder.", "{n} is moving again."}},
      {"standstill.title", {"Steht still", "Standing still"}},
      {"standstill.new",
       {"{n} steht seit {min} min still und lädt nicht.",
        "{n} has been standing still for {min} min and isn't charging."}},
      {"standstill.paused", {" Pausiert: {reason}.", " Paused: {reason}."}},
      {"standstill.idle",
       {" Wartet auf dem Rasen, oder steht im Dock und das hat keinen Strom.",
        " Waiting on the lawn, or in the dock and that has no power."}},
      {"standstill.stuck", {" Kommt nicht weiter, steckt vielleicht fest.", " Not getting on, maybe stuck."}},
      {"paused.title", {"Pausiert, wartet", "Paused, waiting"}},
      {"paused",
       {"{n} hat pausiert: {reason}. Wartet, bis du fortsetzt.",
        "{n} paused: {reason}. It waits until you continue it."}},
      {"spinup.title", {"Mähmotor lief nicht an", "Mow motor didn't start"}},
      {"spinup",
       {"{n} konnte das Messer nicht starten und hat angehalten.", "{n} couldn't start the blade and stopped."}},
      {"done.title", {"Fertig gemäht", "Done mowing"}},
      {"done", {"{n} hat den Mähvorgang abgeschlossen.", "{n} has finished the job."}},
      {"rain.title", {"Regen", "Rain"}},
      {"rain.home", {"{n} fährt wegen Regen nach Hause.", "{n} is going home, it's raining."}},
      {"rain.stopped",
       {"{n} hat wegen Regen angehalten und wartet an Ort und Stelle, allein kann er nicht heim.",
        "{n} stopped for rain and waits where it is, it can't go home by itself."}},
      {"battery.title", {"Akku niedrig", "Battery low"}},
      {"battery.home", {"{n} fährt zum Laden nach Hause.", "{n} is going home to charge."}},
      {"battery.stays",
       {"{n} hat wenig Akku ({volt} V) und kann nicht heim, er bleibt stehen.",
        "{n} is low on battery ({volt} V) and can't go home, it stays where it is."}},
      {"dark.title", {"Sonnenuntergang", "Sunset"}},
      {"dark.home", {"{n} fährt nach Hause, die Sonne ist untergegangen.", "{n} is going home, the sun has set."}},
      {"dark.stopped",
       {"{n} hat angehalten, die Sonne ist untergegangen. Er wartet an Ort und Stelle, allein kann er nicht heim.",
        "{n} stopped, the sun has set. It waits where it is, it can't go home by itself."}},
      {"refused.title", {"Mähen nicht gestartet", "Mowing not started"}},
      {"refused", {"{n} hat nicht mit dem Mähen begonnen: {reason}.", "{n} didn't start mowing: {reason}."}},
      {"map_datum.title", {"Karte passt nicht", "Map doesn't fit"}},
      {"map_datum", {"{n} mäht nicht: {reason}.", "{n} won't mow: {reason}."}},
      {"maintenance_blade.title", {"Messer wechseln", "Change the blades"}},
      {"maintenance_blade",
       {"{n} hat seit dem letzten Messerwechsel {hours} h gemäht.",
        "{n} has mowed {hours} h since the blades were last changed."}},
      {"maintenance_clean.title", {"Zeit zum Reinigen", "Time for a clean"}},
      {"maintenance_clean",
       {"{n} hat seit der letzten Reinigung {hours} h gemäht: Unterseite, Räder und Ladekontakte.",
        "{n} has mowed {hours} h since it was last cleaned: underside, wheels and charging contacts."}},
      {"schedule_skipped.title", {"Geplanter Start ausgefallen", "Planned start skipped"}},
      {"schedule_skipped", {"{n} ist um {at} nicht losgefahren: {reason}.", "{n} didn't set off at {at}: {reason}."}},
      {"skip_missed", {"die Zeit verstrich, während er aus war", "the time passed while it was off"}},
      {"skip_offline", {"die Steuerung antwortete nicht", "its control didn't answer"}},
      {"skip_clock", {"die Uhr war nicht sicher", "the clock wasn't sure"}},
      {"skip_no_area", {"keine Fläche des Plans ist auf der Karte", "no area of the plan is on the map"}},
      {"skip_emergency", {"Notaus", "emergency stop"}},
      {"skip_not_docked", {"er stand nicht auf der Ladestation", "it wasn't on the docking station"}},
      {"skip_no_position", {"ohne Ort der Karte keine Sonnenzeiten", "no sun times without the place of the map"}},
      {"skip_battery", {"der Akku blieb 2 h zu leer", "the battery stayed too low for 2 h"}},
      {"skip_hold", {"ein Update hielt ihn 2 h fest", "an update held it for 2 h"}},
      {"start_failed", {"der Start kam nicht bis zum Mähen", "the start didn't get to mowing"}},
      {"schedule_changed.title", {"Zeitplan geändert", "Schedule changed"}},
      {"schedule_changed", {"Der Zeitplan von {n} wurde geändert.", "The schedule of {n} was changed."}},
      {"notify_changed.title", {"Benachrichtigungen geändert", "Notifications changed"}},
      {"notify_changed",
       {"Die Benachrichtigungen von {n} wurden geändert. Warst du das nicht, sieh in MowBite nach.",
        "The notifications of {n} were changed. If that wasn't you, have a look in MowBite."}},
      {"test.title", {"Test von MowBite", "Test from MowBite"}},
      {"test", {"{n} meldet sich hier, wenn er Hilfe braucht.", "{n} will tell you here when it needs help."}},
      {"open", {"MowBite öffnen", "Open MowBite"}},
  };
  return table;
}

std::string Small(std::string code) {
  std::transform(code.begin(), code.end(), code.begin(), [](unsigned char c) {
    return c == '_' ? ' ' : static_cast<char>(std::tolower(c));
  });
  return code;
}

}  // namespace

std::string Say(const std::string& lang, const std::string& id, const Vars& vars) {
  const auto it = Table().find(id);
  if (it == Table().end()) return id;
  std::string text = lang == "de" ? it->second.first : it->second.second;
  for (const auto& [key, value] : vars) {
    const std::string mark = "{" + key + "}";
    for (size_t at = text.find(mark); at != std::string::npos; at = text.find(mark, at + value.size())) {
      text.replace(at, mark.size(), value);
    }
  }
  return text;
}

std::string ReasonName(const std::string& lang, const std::string& event, const std::string& code,
                       const std::string& message) {
  if (const mowbite_api::EventEntry* entry = mowbite_api::FindReasonGroup(event)) {
    for (const auto& reason : entry->reasons) {
      if (reason.code != code) continue;
      const auto text = reason.texts.find(lang);
      if (text != reason.texts.end() && !text->second.name.empty()) return text->second.name;
    }
  }
  if (!message.empty()) return message;
  return Small(code);
}

}  // namespace mowbite_net
