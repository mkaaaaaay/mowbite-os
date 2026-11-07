#include "tron_imu/wit_parser.h"

namespace tron {

void WitParser::Feed(const uint8_t* data, size_t size, std::vector<WitPacket>& out) {
  buffer_.insert(buffer_.end(), data, data + size);
  size_t pos = 0;
  while (buffer_.size() - pos >= kFrameSize) {
    const uint8_t* f = &buffer_[pos];
    if (f[0] != 0x55) {
      pos++;
      skipped_bytes_++;
      continue;
    }
    uint8_t sum = 0;
    for (size_t i = 0; i < kFrameSize - 1; i++) sum = static_cast<uint8_t>(sum + f[i]);
    if (sum != f[kFrameSize - 1]) {
      checksum_errors_++;
      pos++;
      skipped_bytes_++;
      continue;
    }
    if (f[1] != 0x51 && f[1] != 0x52 && f[1] != 0x53) {
      unknown_types_++;
      last_unknown_type_ = f[1];
      pos += kFrameSize;
      continue;
    }
    WitPacket packet;
    packet.type = static_cast<WitType>(f[1]);
    for (size_t i = 0; i < 4; i++) {
      packet.raw[i] = static_cast<int16_t>(static_cast<uint16_t>(f[2 + 2 * i] | (f[3 + 2 * i] << 8)));
    }
    out.push_back(packet);
    pos += kFrameSize;
  }
  // Keep the incomplete tail (less than one frame).
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
}

}  // namespace tron
