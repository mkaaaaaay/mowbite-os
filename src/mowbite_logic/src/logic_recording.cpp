#include "mowbite_logic/logic_core.h"

#include <cmath>
#include <string>
#include <utility>

namespace mowbite_logic {

bool LogicCore::AreaRecording() const {
  return state_ == State::kAreaRecording;
}

void LogicCore::OnMapStatus(MapDatum datum, const std::string& reason) {
  const bool writable = datum == MapDatum::kMatch || datum == MapDatum::kEmpty;
  if (writable != map_writable_) SetDue<ActionsChanged>(true);
  map_writable_ = writable;
  map_reason_ = writable ? Reason() : MapReason(datum, reason);
  recorder_.SetMapWritable(writable, map_reason_);
}

void LogicCore::OnWheelOdom(double vx, double now_s) {
  if (state_ != State::kAreaRecording) return;
  // A value that is no number is no measurement.
  if (std::isfinite(vx) && last_wheel_s_ && now_s - *last_wheel_s_ <= RecorderConfig::kWheelGap) {
    recorder_.OnWheelTravel(vx * (now_s - *last_wheel_s_));
  } else {
    recorder_.OnWheelsLost();
  }
  last_wheel_s_ = now_s;
  PullRecorder();
}

void LogicCore::TickRecording(double now_s) {
  if (last_wheel_s_ && now_s - *last_wheel_s_ <= RecorderConfig::kWheelGap) return;
  recorder_.OnWheelsLost();
  PullRecorder();
}

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
