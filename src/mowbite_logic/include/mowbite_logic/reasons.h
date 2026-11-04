#ifndef MOWBITE_LOGIC_REASONS_H
#define MOWBITE_LOGIC_REASONS_H

#include <nlohmann/json.hpp>
#include <string>
#include <utility>

namespace mowbite_logic {

using json = nlohmann::ordered_json;

// Why something was refused, paused or ended (masterplan 3.1a): a fixed name in capitals, numbers as own fields with
// limit in the unit of the value, and an English message for the log and for apps that do not know the name.
struct Reason {
  std::string code;
  std::string message;
  json fields = json::object();

  Reason() = default;
  Reason(std::string code_in, std::string message_in) : code(std::move(code_in)), message(std::move(message_in)) {}
  bool empty() const {
    return code.empty();
  }
  Reason& WithMessage(std::string text) {
    message = std::move(text);
    return *this;
  }
  template <typename T>
  Reason& With(const std::string& key, T value) {
    fields[key] = std::move(value);
    return *this;
  }
};

// How bad it is and whether it goes on by itself (auto) or waits for the user (user); App and HA read both.
struct Rating {
  const char* severity;
  const char* resume;
};

// severity and resume per event and reason from the catalog (mowbite_api). action only for RECORDING_REFUSED.
Rating RateReason(const std::string& event, const std::string& code, const std::string& action = "");

// Puts reason, its fields, message, severity and resume into the details of an event of this type.
void PutReason(json* details, const std::string& event, const Reason& reason);

// map/status of mowbite_map: whether the map fits the datum of the GNSS.
enum class MapDatum { kUnknown, kEmpty, kMatch, kMismatch, kUnbound };

// A release refused by the driver or by S2 (SetEmergency): angle and limit in degrees for STILL_TILTED, seconds for
// TILT_UNKNOWN when the driver knows them.
Reason ResetRefusal(const std::string& code, const std::string& message, double value, double limit);

// Why the map cannot be used in this datum state; message is the text of mowbite_map.
Reason MapReason(MapDatum datum, const std::string& message);

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_REASONS_H
