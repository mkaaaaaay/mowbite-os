#ifndef XCORE_DRIVER_INPUT_CONFIG_H
#define XCORE_DRIVER_INPUT_CONFIG_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "xcore_driver/wire.h"
#include "xcore_driver/xcore_control.h"

namespace xcore {

// A GPIO input of the board and what it means.
struct GpioInput {
  // Line name as the board knows it, e.g. "GPIO10".
  std::string line;
  bool active_high = true;
  // "stop", "lift" or "collision" make the board latch its emergency; empty for none.
  std::string reason;
  uint16_t delay_ms = 0;
  // What the driver reports it as: "stop", "lift", "bumper_left", "bumper_right" or empty.
  std::string role;
};

// A setting of the cover UI driver, such as hall_mux = om; it takes an index of its own.
struct CoverSetting {
  std::string id;
  std::string value;
};

struct InputConfig {
  std::vector<GpioInput> gpio;
  std::vector<CoverSetting> cover;
  // gpio holds the candidate lines of the platform, none with a reason or a role: the driver learns its inputs.
  bool learning = false;
};

// The candidate lines as the table of a driver that learns: active high, without reason, delay and role.
std::vector<GpioInput> CandidateInputs(const std::vector<std::string>& lines);

// nullopt with a message if a value is out of what the board accepts.
std::optional<std::string> CheckInputConfig(const InputConfig& config);
// The JSON the input service expects: GPIO inputs first, then the cover settings, each taking the next index.
std::string InputConfigJson(const InputConfig& config);
// Which index of the service's mask is which role.
InputMap InputMapOf(const InputConfig& config);

// A heatshrink stream (window 9, lookahead 5) of literals only, which every decoder of these parameters reads.
Bytes HeatshrinkLiterals(const std::string& text);
// Decodes literals and back references; nullopt on a reference before the start.
std::optional<std::string> HeatshrinkDecode(const Bytes& stream);

}  // namespace xcore

#endif  // XCORE_DRIVER_INPUT_CONFIG_H
