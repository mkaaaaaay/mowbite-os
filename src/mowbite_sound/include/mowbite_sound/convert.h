#ifndef MOWBITE_SOUND_CONVERT_H
#define MOWBITE_SOUND_CONVERT_H

#include <optional>
#include <string>
#include <vector>

#include "mowbite_sound/wav.h"

namespace mowbite_sound {

// Longest sound in s, and the shortest one worth playing.
constexpr double kMaxSeconds = 30.0;
constexpr double kMinSeconds = 0.2;

// The sound of an MP3 (MPEG-1 or 2 Layer III) with its channels mixed to one, at most kMaxSeconds; nullopt and why
// for anything else. A leading ID3v2 tag is skipped.
std::optional<Wav> DecodeMp3(const std::string& bytes, std::string* error);

// Runs argv[0] with argv and waits at most timeout s, then kills it; false and why for another end than exit 0, with
// the last line it wrote.
bool RunProgram(const std::vector<std::string>& argv, double timeout, std::string* error);

}  // namespace mowbite_sound

#endif  // MOWBITE_SOUND_CONVERT_H
