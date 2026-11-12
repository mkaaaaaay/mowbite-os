#include "mowbite_logic/logic_core.h"

#include <string>
#include <utility>

namespace mowbite_logic {

void LogicCore::OnMapEditResult(bool ok, const std::string& id, const std::string& code,
                                const std::string& message) {
  recorder_.OnMapEditResult(ok, id, code, message);
  PullRecorder();
}

void LogicCore::PullRecorder() {
  for (auto& event : recorder_.TakeEvents()) AddEvent(event.type, std::move(event.details));
  if (recorder_.TakeActionsChanged()) SetDue<ActionsChanged>(true);
}

json LogicCore::MapOverlay() const {
  // The recorder is reset when the state changes, so outside of AREA_RECORDING it has no lines.
  return recorder_.Overlay();
}

bool LogicCore::TakeOverlayChanged() {
  return recorder_.TakeOverlayChanged();
}

void LogicCore::RecordedBodyKept(int areas, const std::string& code, const std::string& message) {
  json details = json::object();
  if (areas >= 0) details["areas"] = areas;
  PutReason(&details, "RECORDED_BODY_KEPT",
            Reason("SAVE_FAILED", "The areas recorded here keep the old body: " + message).With("code", code));
  AddEvent("RECORDED_BODY_KEPT", std::move(details));
}

}  // namespace mowbite_logic
