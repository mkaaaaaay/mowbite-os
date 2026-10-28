#ifndef TRON_IMU_WIT_PARSER_H
#define TRON_IMU_WIT_PARSER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace tron {

// Packet types of the WIT standard protocol (11 bytes: 0x55, type, 8 data bytes, sum of the first 10).
enum class WitType : uint8_t {
  kAcceleration = 0x51,
  kAngularVelocity = 0x52,
  kAngle = 0x53,
};

struct WitPacket {
  WitType type;
  std::array<int16_t, 4> raw;  // little endian pairs; the fourth is temperature (0x51/0x52) or version (0x53)
};

class WitParser {
 public:
  // Appends every complete, valid packet. Resynchronises on the next 0x55 after a bad sum or unknown type.
  void Feed(const uint8_t* data, size_t size, std::vector<WitPacket>& out);

  uint64_t checksum_errors() const {
    return checksum_errors_;
  }
  uint64_t unknown_types() const {
    return unknown_types_;
  }
  uint64_t skipped_bytes() const {
    return skipped_bytes_;
  }

 private:
  static constexpr size_t kFrameSize = 11;
  std::vector<uint8_t> buffer_;
  uint64_t checksum_errors_ = 0;
  uint64_t unknown_types_ = 0;
  uint64_t skipped_bytes_ = 0;
};

}  // namespace tron

#endif  // TRON_IMU_WIT_PARSER_H
