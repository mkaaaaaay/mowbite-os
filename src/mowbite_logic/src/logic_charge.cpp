#include "mowbite_logic/docking_part.h"

#include <cstdio>

namespace mowbite_logic {

void DockingPart::WatchCharge(CoreAccess core) {
  if (core.state() != State::kIdle || !core.hw().telemetry_ok) {
    charge_watch_.OnBusy(core.hw().telemetry_ok && !core.hw().dock_contact);
    return;
  }
  const ChargeSample now = ChargeSampleNow(core);
  const ChargeConfig& c = core.config().charge;
  char text[200];
  switch (charge_watch_.OnSample(now, core.emergency())) {
    case ChargeWatch::Change::kConfirmed:
      core.AddEvent("DOCKED", json::object());
      core.Notify("Docked: charging confirmed on the docking station");
      return;
    case ChargeWatch::Change::kContactLost:
      ChargingLost(core, Reason("NO_DOCK_CONTACT", "The mower lost the contact with the docking station"));
      return;
    case ChargeWatch::Change::kCurrentLost:
      std::snprintf(text, sizeof(text),
                    "The charge current on the docking station stopped for %.0f s (below %.2f A, the battery at "
                    "%d %%)",
                    c.lost_time, c.current, now.percentage);
      ChargingLost(core,
                   Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNoCurrent:
      std::snprintf(text, sizeof(text),
                    "On the docking station, but no charge current for %.0f s (at least %.2f A, the battery at %d %%)",
                    c.timeout, c.current, now.percentage);
      ChargingLost(core,
                   Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNone:
    default: return;
  }
}

void DockingPart::ChargingLost(CoreAccess core, const Reason& why) {
  json details = json::object();
  PutReason(&details, "CHARGING_LOST", why);
  if (core.hw().battery_ok) details["battery_percentage"] = core.hw().battery_percentage;
  core.AddEvent("CHARGING_LOST", std::move(details));
  core.Notify("Charging lost: " + why.message);
}

}  // namespace mowbite_logic
