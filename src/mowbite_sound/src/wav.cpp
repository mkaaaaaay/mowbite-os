#include "mowbite_sound/wav.h"

#include <cmath>

namespace mowbite_sound {

namespace {

constexpr uint16_t kPcm = 1;
constexpr uint16_t kBits = 16;

void Put16(std::string* out, uint16_t value) {
  out->push_back(static_cast<char>(value & 0xff));
  out->push_back(static_cast<char>(value >> 8));
}

void Put32(std::string* out, uint32_t value) {
  for (int i = 0; i < 4; ++i) out->push_back(static_cast<char>((value >> (8 * i)) & 0xff));
}

uint32_t Get(const std::string& bytes, size_t at, int size) {
  uint32_t value = 0;
  for (int i = size - 1; i >= 0; --i) value = (value << 8) | static_cast<uint8_t>(bytes[at + i]);
  return value;
}

}  // namespace

Wav AlarmTone() {
  constexpr uint32_t kRate = 16000;
  constexpr double kPitch = 2000.0, kBeep = 0.15, kPause = 0.1;
  Wav wav;
  wav.rate = kRate;
  for (int beep = 0; beep < 3; ++beep) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(kBeep * kRate); ++i) {
      wav.samples.push_back(static_cast<int16_t>(20000.0 * std::sin(2.0 * M_PI * kPitch * i / kRate)));
    }
    wav.samples.insert(wav.samples.end(), static_cast<size_t>(kPause * kRate), 0);
  }
  return wav;
}

std::string RateRange() {
  return std::to_string(kMinRate) + " to " + std::to_string(kMaxRate) + " Hz";
}

std::string EncodeWav(const Wav& wav) {
  const uint32_t data_bytes = static_cast<uint32_t>(wav.samples.size() * 2);
  std::string out = "RIFF";
  Put32(&out, 36 + data_bytes);
  out += "WAVEfmt ";
  Put32(&out, 16);
  Put16(&out, kPcm);
  Put16(&out, 1);
  Put32(&out, wav.rate);
  Put32(&out, wav.rate * 2);
  Put16(&out, 2);
  Put16(&out, kBits);
  out += "data";
  Put32(&out, data_bytes);
  for (const int16_t sample : wav.samples) Put16(&out, static_cast<uint16_t>(sample));
  return out;
}

std::optional<Wav> DecodeWav(const std::string& bytes, std::string* error) {
  if (bytes.size() < 12 || bytes.compare(0, 4, "RIFF") != 0 || bytes.compare(8, 4, "WAVE") != 0) {
    *error = "not a WAVE file";
    return std::nullopt;
  }
  bool format = false;
  size_t at = 12;
  Wav wav;
  while (at + 8 <= bytes.size()) {
    const std::string id = bytes.substr(at, 4);
    const uint32_t size = Get(bytes, at + 4, 4);
    const size_t body = at + 8;
    if (size > bytes.size() - body) {
      *error = "chunk " + id + " runs past the end";
      return std::nullopt;
    }
    if (id == "fmt ") {
      if (size < 16 || Get(bytes, body, 2) != kPcm || Get(bytes, body + 2, 2) != 1 ||
          Get(bytes, body + 14, 2) != kBits) {
        *error = "not 16-bit PCM with one channel";
        return std::nullopt;
      }
      wav.rate = Get(bytes, body + 4, 4);
      if (wav.rate < kMinRate || wav.rate > kMaxRate) {
        *error = "rate " + std::to_string(wav.rate) + " Hz is outside of " + RateRange();
        return std::nullopt;
      }
      format = true;
    } else if (id == "data") {
      if (!format) {
        *error = "data before fmt";
        return std::nullopt;
      }
      wav.samples.resize(size / 2);
      for (size_t i = 0; i < wav.samples.size(); ++i) {
        wav.samples[i] = static_cast<int16_t>(Get(bytes, body + 2 * i, 2));
      }
      return wav;
    }
    // Chunks are padded to an even size.
    at = body + size + (size & 1);
  }
  *error = "no data chunk";
  return std::nullopt;
}

}  // namespace mowbite_sound
