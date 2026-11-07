#include "tron_mcu/capture_dump.h"

#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "tron_mcu/mcu_codec.h"

namespace tron {

namespace {

bool ParseHex(const std::string& text, std::vector<uint8_t>& out) {
  if (text.size() % 2 != 0) return false;
  out.clear();
  for (size_t i = 0; i < text.size(); i += 2) {
    unsigned value = 0;
    for (size_t k = i; k < i + 2; ++k) {
      const char c = text[k];
      value <<= 4;
      if (c >= '0' && c <= '9') {
        value |= static_cast<unsigned>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        value |= static_cast<unsigned>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        value |= static_cast<unsigned>(c - 'A' + 10);
      } else {
        return false;
      }
    }
    out.push_back(static_cast<uint8_t>(value));
  }
  return true;
}

std::string Number(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", value);
  return text;
}

std::string Time(double t) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6f", t);
  return text;
}

std::string PacketLine(double t, const SubPacket& packet) {
  std::ostringstream line;
  line << "{\"t\":" << Time(t);
  if (packet.type != static_cast<uint8_t>(PacketType::kMcuToHost)) {
    line << ",\"type\":" << int{packet.type} << ",\"module\":" << int{packet.module} << "}";
    return line.str();
  }
  switch (static_cast<Module>(packet.module)) {
    case Module::kSpeed:
      if (auto speed = DecodeSpeed(packet.payload)) {
        line << ",\"speed\":[" << Number(speed->linear) << "," << Number(speed->angular) << "]}";
        return line.str();
      }
      break;
    case Module::kSensor:
      if (auto s = DecodeSensorInfo(packet.payload)) {
        line << ",\"sensor\":{\"bumper\":" << s->bumper << ",\"rain\":" << s->rain << ",\"lift\":" << s->lift
             << ",\"stop\":" << s->stop << ",\"bumper_left\":" << s->bumper_left << ",\"bumper_right\":"
             << s->bumper_right << "}}";
        return line.str();
      }
      break;
    case Module::kMotors:
      if (auto m = DecodeMotors(packet.payload)) {
        line << ",\"motors\":{\"cutter\":" << m->cutter.rpm << ",\"left\":" << m->left.rpm << ",\"right\":"
             << m->right.rpm << "}}";
        return line.str();
      }
      break;
    default:
      line << ",\"module\":" << int{packet.module} << "}";
      return line.str();
  }
  line << ",\"module\":" << int{packet.module} << ",\"invalid\":true}";
  return line.str();
}

}  // namespace

bool DumpCapture(std::istream& in, std::ostream& out) {
  FrameParser parser;
  uint64_t lines = 0;
  uint64_t bad_lines = 0;
  uint64_t bytes = 0;
  std::string text;
  std::vector<uint8_t> chunk;
  while (std::getline(in, text)) {
    if (text.empty()) continue;
    ++lines;
    std::istringstream fields(text);
    double t = 0;
    std::string hex;
    std::string rest;
    if (!(fields >> t >> hex) || (fields >> rest) || !ParseHex(hex, chunk)) {
      ++bad_lines;
      continue;
    }
    bytes += chunk.size();
    for (const auto& frame : parser.Push(chunk.data(), chunk.size())) {
      for (const auto& packet : frame) out << PacketLine(t, packet) << "\n";
    }
  }
  const ParserStats& s = parser.Stats();
  out << "{\"summary\":{\"lines\":" << lines << ",\"bad_lines\":" << bad_lines << ",\"bytes\":" << bytes
      << ",\"frames\":" << s.frames << ",\"checksum_errors\":" << s.checksum_errors << ",\"length_errors\":"
      << s.length_errors << ",\"escape_errors\":" << s.escape_errors << ",\"overflows\":" << s.overflows
      << ",\"dropped_bytes\":" << s.dropped_bytes << "}}\n";
  return bad_lines == 0;
}

}  // namespace tron
