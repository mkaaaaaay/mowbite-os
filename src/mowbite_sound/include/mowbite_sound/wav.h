#ifndef MOWBITE_SOUND_WAV_H
#define MOWBITE_SOUND_WAV_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_sound {

// Rates a sound may have, in Hz.
constexpr uint32_t kMinRate = 8000;
constexpr uint32_t kMaxRate = 48000;
// Longest sound in s, and the shortest one worth playing.
constexpr double kMaxSeconds = 30.0;
constexpr double kMinSeconds = 0.2;
// Largest WAV the OS reads: kMaxSeconds at kMaxRate and its header.
constexpr size_t kMaxWavBytes = 44 + 2 * kMaxRate * static_cast<size_t>(kMaxSeconds);

// "8000 to 48000 Hz", for messages.
std::string RateRange();

// A mono sound of 16-bit samples.
struct Wav {
  uint32_t rate = 0;
  std::vector<int16_t> samples;

  double Seconds() const { return rate ? static_cast<double>(samples.size()) / rate : 0; }
};

// A RIFF WAVE file of 16-bit PCM, one channel.
std::string EncodeWav(const Wav& wav);

// The sound of a WAVE file of 16-bit PCM, one channel, kMinRate to kMaxRate; nullopt and why for anything else.
std::optional<Wav> DecodeWav(const std::string& bytes, std::string* error);

}  // namespace mowbite_sound

#endif  // MOWBITE_SOUND_WAV_H
