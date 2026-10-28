#include "mowbite_logic/reasons.h"

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
    static const std::set<std::string> info = {"BUSY", "STILL_STOPPING", "STILL_SAVING", "ON_DOCK", "RAIN"};
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
