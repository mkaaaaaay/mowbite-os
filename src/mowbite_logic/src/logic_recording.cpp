#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "mowbite_logic/recording_part.h"

namespace mowbite_logic {

void RecordingPart::OnMapStatus(CoreAccess core, MapDatum datum, const std::string& reason) {
  const bool writable = datum == MapDatum::kMatch || datum == MapDatum::kEmpty;
  if (writable != map_writable_) core.effects().SetDue<ActionsChanged>(true);
  map_writable_ = writable;
  map_reason_ = writable ? Reason() : MapReason(datum, reason);
  recorder_.SetMapWritable(writable, map_reason_);
}

void RecordingPart::OnWheelOdom(CoreAccess core, double vx, double now_s) {
  if (core.state() != State::kAreaRecording) return;
  // A value that is no number is no measurement.
  if (std::isfinite(vx) && last_wheel_s_ && now_s - *last_wheel_s_ <= RecorderConfig::kWheelGap) {
    const double seconds = now_s - *last_wheel_s_;
    recorder_.OnWheelTravel(vx * seconds, seconds);
  } else {
    recorder_.OnWheelsLost();
  }
  last_wheel_s_ = now_s;
  PullRecorder(core);
}

void RecordingPart::TickRecording(CoreAccess core, double now_s) {
  if (last_wheel_s_ && now_s - *last_wheel_s_ <= RecorderConfig::kWheelGap) return;
  recorder_.OnWheelsLost();
  PullRecorder(core);
}

void RecordingPart::OnMapEditResult(CoreAccess core, bool ok, const std::string& id, const std::string& code,
                                    const std::string& message) {
  recorder_.OnMapEditResult(ok, id, code, message);
  PullRecorder(core);
}

void RecordingPart::PullRecorder(CoreAccess core) {
  for (auto& event : recorder_.TakeEvents()) core.AddEvent(event.type, std::move(event.details));
  if (recorder_.TakeActionsChanged()) core.effects().SetDue<ActionsChanged>(true);
}

json RecordingPart::MapOverlay() const {
  // The recorder is reset when the state changes, so outside of AREA_RECORDING it has no lines.
  return recorder_.Overlay();
}

bool RecordingPart::TakeOverlayChanged() {
  return recorder_.TakeOverlayChanged();
}

void RecordingPart::RecordedBodyKept(CoreAccess core, int areas, const std::string& code, const std::string& message) {
  json details = json::object();
  if (areas >= 0) details["areas"] = areas;
  PutReason(&details, "RECORDED_BODY_KEPT",
            Reason("SAVE_FAILED", "The areas recorded here keep the old body: " + message).With("code", code));
  core.AddEvent("RECORDED_BODY_KEPT", std::move(details));
}

std::optional<std::pair<double, double>> RecordingPart::OnTeleop(CoreFacts core, double vx, double vz, double now_s) {
  const double quiet_s = now_s - last_teleop_s_;
  last_teleop_s_ = now_s;
  if (core.state() != State::kAreaRecording || core.emergency()) {
    teleop_armed_ = false;
    return std::nullopt;
  }
  if (!teleop_armed_) {
    const bool zero = vx == 0.0 && vz == 0.0;
    if (!zero && quiet_s < kTeleopQuietS) return std::nullopt;
    teleop_armed_ = true;
  }
  return std::make_pair(vx, vz);
}

void RecordingPart::DisarmTeleop() {
  teleop_armed_ = false;
}

}  // namespace mowbite_logic
