#include "mowbite_logic/reasons.h"

#include <cmath>

#include "mowbite_api/catalog.h"

namespace mowbite_logic {

// The one table of severity and resume is the catalog of the interface.
Rating RateReason(const std::string& event, const std::string& code, const json& details) {
  const mowbite_api::Rating rating = mowbite_api::RateReason(event, code, details);
  return {rating.severity, rating.resume};
}

void PutReason(json* details, const std::string& event, const Reason& reason) {
  (*details)["reason"] = reason.code;
  for (const auto& [key, value] : reason.fields.items()) (*details)[key] = value;
  if (!reason.message.empty()) (*details)["message"] = reason.message;
  const Rating rating = RateReason(event, reason.code, *details);
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

Reason FollowerEndReason(const std::string& code, const std::string& message, const std::optional<Reason>& cancel) {
  const bool same_emergency = code == "EMERGENCY" && cancel && cancel->code == "EMERGENCY";
  if (cancel && (code == "CANCELED" || same_emergency)) {
    return Reason(*cancel).WithMessage(cancel->message + " (" + message + ")");
  }
  return Reason(code, message);
}

}  // namespace mowbite_logic
