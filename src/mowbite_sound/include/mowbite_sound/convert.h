#ifndef MOWBITE_SOUND_CONVERT_H
#define MOWBITE_SOUND_CONVERT_H

#include <optional>
#include <string>
#include <vector>

#include "mowbite_sound/wav.h"

namespace mowbite_sound {

// Largest MP3 the converter reads.
constexpr size_t kMaxMp3Bytes = 2 << 20;
// Exit codes of mowbite_sound_convert besides 0 and 1 (any other failure).
constexpr int kExitNotMp3 = 2;
constexpr int kExitTooLong = 3;

// The sound of an MP3 (MPEG-1 or 2 Layer III) with its channels mixed to one, at most kMaxSeconds; nullopt and why
// for anything else, too_long set when it is longer. A leading ID3v2 tag is skipped.
std::optional<Wav> DecodeMp3(const std::string& bytes, std::string* error, bool* too_long = nullptr);

// Runs argv[0] with argv and waits at most timeout s, then kills it; false and why for another end than exit 0, with
// the last line it wrote. exit_code gets the code it exited with, -1 when it did not exit.
bool RunProgram(const std::vector<std::string>& argv, double timeout, std::string* error, int* exit_code = nullptr);

}  // namespace mowbite_sound

#endif  // MOWBITE_SOUND_CONVERT_H
