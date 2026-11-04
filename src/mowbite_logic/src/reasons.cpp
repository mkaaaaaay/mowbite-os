#include "mowbite_logic/reasons.h"

#include <cmath>
#include <set>

namespace mowbite_logic {

namespace {

constexpr const char* kInfo = "info";
constexpr const char* kWarning = "warning";
constexpr const char* kError = "error";
constexpr const char* kAuto = "auto";
constexpr const char* kUser = "user";

bool In(const std::set<std::string>& names, const std::string& code) {
  return names.count(code) != 0;
}

// The position or the heading is not good enough; it may get better by itself.
const std::set<std::string> kPositionCodes = {"NO_POSITION", "POSE_LOST", "POSITION_ACCURACY", "HEADING",
                                              "NO_HEADING"};

}  // namespace

Rating RateReason(const std::string& event, const std::string& code, const std::string& action) {
  if (event == "MOWING_REFUSED" || event == "DRIVE_REFUSED" || event == "RECORDING_REFUSED") {
    // Points are taken again by themselves once the position is accurate.
    if (event == "RECORDING_REFUSED" && action == "point" && code == "POSITION_ACCURACY") return {kWarning, kAuto};
    static const std::set<std::string> info = {"BUSY", "STILL_STOPPING", "STILL_SAVING", "ON_DOCK", "RAIN",
                                               "ALREADY_THERE"};
    static const std::set<std::string> error = {"EMERGENCY",       "NO_SAFETY_MEASUREMENT", "MAP_MISMATCH",
                                                "MAP_UNREADABLE",  "MAP_UNKNOWN",           "PLANNER_MISSING",
                                                "FOLLOWER_MISSING", "SAVE_FAILED"};
    if (In(info, code)) return {kInfo, kUser};
    if (In(error, code)) return {kError, kUser};
    return {kWarning, kUser};
  }
  if (event == "STATE") {
    if (code == "MANUAL") return {kInfo, kUser};
    if (In(kPositionCodes, code)) return {kWarning, kAuto};
    return {kError, kUser};
  }
  if (event == "DRIVE") {
    static const std::set<std::string> info = {"DONE", "ALREADY_THERE", "STOPPED", "CANCELED"};
    if (In(info, code)) return {kInfo, kUser};
    if (In(kPositionCodes, code)) return {kWarning, kUser};
    return {kError, kUser};
  }
  if (event == "POSITION_JUMP") return {kWarning, kAuto};
  if (event == "AREA_SKIPPED") return {kInfo, kAuto};
  // With OpenMower's reason values path_failed and no_gps; the mower stays where it stopped.
  if (event == "UNDOCKING_FAILED") return {kError, kUser};
  // approach_failed or dock_failed: the next try follows by itself, after the last one the mower waits.
  if (event == "DOCKING_RETRY") return {kWarning, kAuto};
  if (event == "DOCKING_FAILED") return {kError, kUser};
  // Tilted: level the mower and reset again. Everything else needs a look at the mower or its software.
  if (event == "EMERGENCY_RESET_REFUSED") return {code == "STILL_TILTED" ? kWarning : kError, kUser};
  return {kWarning, kUser};
}

void PutReason(json* details, const std::string& event, const Reason& reason) {
  (*details)["reason"] = reason.code;
  for (const auto& [key, value] : reason.fields.items()) (*details)[key] = value;
  if (!reason.message.empty()) (*details)["message"] = reason.message;
  const std::string action = details->contains("action") ? (*details)["action"].get<std::string>() : "";
  const Rating rating = RateReason(event, reason.code, action);
  (*details)["severity"] = rating.severity;
  (*details)["resume"] = rating.resume;
}

Reason ResetRefusal(const std::string& code, const std::string& message, double value, double limit) {
  Reason why(code.empty() ? "NO_ANSWER" : code, message);
  if (code == "STILL_TILTED" && std::isfinite(value) && std::isfinite(limit)) {
    why.With("angle", value).With("limit", limit);
  }
  if (code == "TILT_UNKNOWN" && std::isfinite(value)) why.With("seconds", value);
  return why;
}

Reason MapReason(MapDatum datum, const std::string& message) {
  switch (datum) {
    case MapDatum::kUnknown:
      return Reason("MAP_UNKNOWN", message.empty() ? "The datum of the GNSS is not known yet" : message);
    case MapDatum::kEmpty: return Reason("NO_MOWING_AREA", message.empty() ? "The map is empty" : message);
    case MapDatum::kUnbound:
      return Reason("MAP_MISMATCH", message.empty() ? "The map has no recorded datum" : message)
          .With("datum_state", "unbound");
    case MapDatum::kMismatch:
    case MapDatum::kMatch:
    default:
      return Reason("MAP_MISMATCH", message.empty() ? "The map does not fit the datum in use" : message);
  }
}

}  // namespace mowbite_logic
