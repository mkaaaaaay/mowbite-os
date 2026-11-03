#include "xcore_driver/input_config.h"

namespace xcore {

namespace {

constexpr size_t kMaxInputs = 30;

// Names and values here are short ASCII; quotes and backslashes are refused by the check.
std::string Quote(const std::string& text) {
  return "\"" + text + "\"";
}

bool PlainText(const std::string& text, size_t max) {
  if (text.empty() || text.size() > max) return false;
  for (char c : text) {
    if (c < 0x20 || c > 0x7E || c == '"' || c == '\\') return false;
  }
  return true;
}

class BitWriter {
 public:
  void Put(uint32_t value, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
      current_ = static_cast<uint8_t>((current_ << 1) | ((value >> i) & 1u));
      if (++used_ == 8) {
        out_.push_back(current_);
        current_ = 0;
        used_ = 0;
      }
    }
  }
  Bytes Finish() {
    // The last byte is filled up with zero bits.
    if (used_) out_.push_back(static_cast<uint8_t>(current_ << (8 - used_)));
    used_ = 0;
    return out_;
  }

 private:
  Bytes out_;
  uint8_t current_ = 0;
  int used_ = 0;
};

class BitReader {
 public:
  explicit BitReader(const Bytes& in) : in_(in) {
  }
  bool Get(int bits, uint32_t& value) {
    if (in_.size() * 8 - at_ < static_cast<size_t>(bits)) return false;
    value = 0;
    for (int i = 0; i < bits; i++, at_++) value = (value << 1) | ((in_[at_ / 8] >> (7 - at_ % 8)) & 1u);
    return true;
  }

 private:
  const Bytes& in_;
  size_t at_ = 0;
};

constexpr int kWindowBits = 9;
constexpr int kLookaheadBits = 5;

}  // namespace

std::optional<std::string> CheckInputConfig(const InputConfig& config) {
  if (config.gpio.size() + config.cover.size() > kMaxInputs) return "more than 30 inputs";
  for (const auto& in : config.gpio) {
    if (!PlainText(in.line, 32)) return "bad line name '" + in.line + "'";
    if (!in.reason.empty() && in.reason != "stop" && in.reason != "lift" && in.reason != "collision") {
      return "unknown emergency reason '" + in.reason + "' of " + in.line;
    }
    if (!in.role.empty() && in.role != "stop" && in.role != "lift" && in.role != "bumper_left" &&
        in.role != "bumper_right") {
      return "unknown role '" + in.role + "' of " + in.line;
    }
  }
  for (const auto& c : config.cover) {
    if (!PlainText(c.id, 32) || !PlainText(c.value, 32)) return "bad cover setting '" + c.id + "'";
  }
  return std::nullopt;
}

std::string InputConfigJson(const InputConfig& config) {
  std::string json = "{";
  if (!config.gpio.empty()) {
    json += "\"gpio\":[";
    for (size_t i = 0; i < config.gpio.size(); i++) {
      const auto& in = config.gpio[i];
      if (i) json += ",";
      json += "{\"line\":" + Quote(in.line) + ",\"active\":" + Quote(in.active_high ? "high" : "low");
      if (!in.reason.empty()) {
        json += ",\"emergency\":{\"reason\":" + Quote(in.reason) + ",\"delay\":" + std::to_string(in.delay_ms) + "}";
      }
      json += "}";
    }
    json += "]";
  }
  if (!config.cover.empty()) {
    if (!config.gpio.empty()) json += ",";
    json += "\"yf_cover_ui\":[";
    for (size_t i = 0; i < config.cover.size(); i++) {
      if (i) json += ",";
      // The board wants the id before the value.
      json += "{\"id\":" + Quote(config.cover[i].id) + ",\"value\":" + Quote(config.cover[i].value) + "}";
    }
    json += "]";
  }
  return json + "}";
}

InputMap InputMapOf(const InputConfig& config) {
  InputMap map;
  for (size_t i = 0; i < config.gpio.size(); i++) {
    const std::string& role = config.gpio[i].role;
    int index = static_cast<int>(i);
    if (role == "stop") map.stop.push_back(index);
    if (role == "lift") map.lift.push_back(index);
    if (role == "bumper_left") map.bumper_left.push_back(index);
    if (role == "bumper_right") map.bumper_right.push_back(index);
  }
  return map;
}

Bytes HeatshrinkLiterals(const std::string& text) {
  BitWriter w;
  for (unsigned char c : text) {
    w.Put(1, 1);
    w.Put(c, 8);
  }
  return w.Finish();
}

std::optional<std::string> HeatshrinkDecode(const Bytes& stream) {
  BitReader r(stream);
  std::string out;
  uint32_t tag;
  while (r.Get(1, tag)) {
    uint32_t value;
    if (tag) {
      if (!r.Get(8, value)) break;
      out.push_back(static_cast<char>(value));
      continue;
    }
    uint32_t count;
    if (!r.Get(kWindowBits, value) || !r.Get(kLookaheadBits, count)) break;
    size_t distance = value + 1;
    if (distance > out.size()) return std::nullopt;
    for (uint32_t i = 0; i <= count; i++) out.push_back(out[out.size() - distance]);
  }
  return out;
}

}  // namespace xcore
