#include <cstdint>

#include "mowbite_sound/convert.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include "minimp3.h"

namespace mowbite_sound {

namespace {

// The bytes of an ID3v2 tag at the start, 0 without one.
size_t Id3Size(const std::string& bytes) {
  if (bytes.size() < 10 || bytes.compare(0, 3, "ID3") != 0) return 0;
  size_t size = 0;
  for (int i = 6; i < 10; ++i) size = (size << 7) | (static_cast<uint8_t>(bytes[i]) & 0x7f);
  // A footer of 10 bytes follows when flag 0x10 is set.
  return 10 + size + ((static_cast<uint8_t>(bytes[5]) & 0x10) ? 10 : 0);
}

}  // namespace

std::optional<Wav> DecodeMp3(const std::string& bytes, std::string* error) {
  mp3dec_t decoder;
  mp3dec_init(&decoder);
  Wav wav;
  size_t at = Id3Size(bytes);
  int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  while (at < bytes.size()) {
    mp3dec_frame_info_t info{};
    const int frames = mp3dec_decode_frame(&decoder, reinterpret_cast<const uint8_t*>(bytes.data()) + at,
                                           static_cast<int>(bytes.size() - at), pcm, &info);
    if (info.frame_bytes == 0) break;
    at += static_cast<size_t>(info.frame_bytes);
    if (frames == 0) continue;
    const uint32_t rate = static_cast<uint32_t>(info.hz);
    if (wav.rate == 0) {
      if (rate < kMinRate || rate > kMaxRate) {
        *error = "rate " + std::to_string(rate) + " Hz is outside of " + RateRange();
        return std::nullopt;
      }
      wav.rate = rate;
    } else if (rate != wav.rate) {
      *error = "the rate changes within the file";
      return std::nullopt;
    }
    if (static_cast<double>(wav.samples.size() + frames) > kMaxSeconds * wav.rate) {
      *error = "longer than " + std::to_string(static_cast<int>(kMaxSeconds)) + " s";
      return std::nullopt;
    }
    for (int i = 0; i < frames; ++i) {
      int sum = 0;
      for (int c = 0; c < info.channels; ++c) sum += pcm[i * info.channels + c];
      wav.samples.push_back(static_cast<int16_t>(sum / info.channels));
    }
  }
  if (wav.samples.empty()) {
    *error = "no MP3 frames";
    return std::nullopt;
  }
  return wav;
}

}  // namespace mowbite_sound
