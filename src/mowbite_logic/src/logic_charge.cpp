#include "mowbite_logic/logic_core.h"

#include <cstdio>

namespace mowbite_logic {

void LogicCore::WatchCharge() {
  if (state_ != State::kIdle || !hw_.telemetry_ok) {
    charge_watch_.OnBusy(hw_.telemetry_ok && !hw_.dock_contact);
    return;
  }
  const ChargeSample now = ChargeSampleNow();
  const ChargeConfig& c = config_.charge;
  char text[200];
  switch (charge_watch_.OnSample(now, emergency_)) {
    case ChargeWatch::Change::kConfirmed:
      AddEvent("DOCKED", json::object());
      Notify("Docked: charging confirmed on the docking station");
      return;
    case ChargeWatch::Change::kContactLost:
      ChargingLost(Reason("NO_DOCK_CONTACT", "The mower lost the contact with the docking station"));
      return;
    case ChargeWatch::Change::kCurrentLost:
      std::snprintf(text, sizeof(text),
                    "The charge current on the docking station stopped for %.0f s (below %.2f A, the battery at "
                    "%d %%)",
                    c.lost_time, c.current, now.percentage);
      ChargingLost(Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNoCurrent:
      std::snprintf(text, sizeof(text),
                    "On the docking station, but no charge current for %.0f s (at least %.2f A, the battery at %d %%)",
                    c.timeout, c.current, now.percentage);
      ChargingLost(Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNone:
    default: return;
  }
}

void LogicCore::ChargingLost(const Reason& why) {
  json details = json::object();
  PutReason(&details, "CHARGING_LOST", why);
  if (hw_.battery_ok) details["battery_percentage"] = hw_.battery_percentage;
  AddEvent("CHARGING_LOST", std::move(details));
  Notify("Charging lost: " + why.message);
}

}  // namespace mowbite_logic
