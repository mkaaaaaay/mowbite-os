#include "mowbite_ntrip/rtcm.h"

namespace mowbite_ntrip {

namespace {

constexpr uint8_t kPreamble = 0xD3;
constexpr size_t kHeader = 3;
constexpr size_t kCrc = 3;

}  // namespace

uint32_t Crc24q(const uint8_t* data, size_t size) {
  uint32_t crc = 0;
  for (size_t i = 0; i < size; i++) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int bit = 0; bit < 8; bit++) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

uint16_t RtcmMessageType(const std::vector<uint8_t>& frame) {
  if (frame.size() < kHeader + 2 + kCrc) return 0;
  return static_cast<uint16_t>((frame[3] << 4) | (frame[4] >> 4));
}

void RtcmFramer::Reset() {
  buffer_.clear();
}

void RtcmFramer::Feed(const uint8_t* data, size_t size, std::vector<std::vector<uint8_t>>& frames) {
  buffer_.insert(buffer_.end(), data, data + size);
  size_t pos = 0;
  while (pos < buffer_.size()) {
    if (buffer_[pos] != kPreamble) {
      pos++;
      skipped_++;
      continue;
    }
    if (buffer_.size() - pos < kHeader) break;
    // The 6 bits after the preamble are reserved and zero, otherwise this 0xD3 is part of something else.
    if (buffer_[pos + 1] & 0xFC) {
      pos++;
      skipped_++;
      continue;
    }
    const size_t length = (static_cast<size_t>(buffer_[pos + 1] & 0x03) << 8) | buffer_[pos + 2];
    const size_t total = kHeader + length + kCrc;
    if (buffer_.size() - pos < total) break;
    const uint8_t* frame = buffer_.data() + pos;
    const uint32_t expected = (static_cast<uint32_t>(frame[kHeader + length]) << 16) |
                              (static_cast<uint32_t>(frame[kHeader + length + 1]) << 8) | frame[kHeader + length + 2];
    if (Crc24q(frame, kHeader + length) != expected) {
      crc_errors_++;
      pos++;
      skipped_++;
      continue;
    }
    frames.emplace_back(frame, frame + total);
    frames_++;
    pos += total;
  }
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
}

}  // namespace mowbite_ntrip
