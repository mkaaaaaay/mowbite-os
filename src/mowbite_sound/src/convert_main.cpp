// mowbite_sound_convert <in.mp3> <out.wav>: an MP3 to a WAV of one channel at its own rate. A program of its own with
// tight limits, so that a broken or hostile file never reaches the node that plays and decides.
#include <sys/resource.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <fstream>
#include <string>

#include "mowbite_hold/file_io.h"
#include "mowbite_sound/convert.h"

namespace {

constexpr rlim_t kCpuSeconds = 10;
constexpr rlim_t kMemoryBytes = 128u << 20;
constexpr rlim_t kFileBytes = 4u << 20;

bool Limit(int resource, rlim_t value) {
#ifdef __SANITIZE_ADDRESS__
  // The address sanitizer reserves terabytes of address space before main.
  if (resource == RLIMIT_AS) return true;
#endif
  const rlimit limit{value, value};
  return ::setrlimit(resource, &limit) == 0;
}

int Fail(const std::string& why) {
  std::fprintf(stderr, "%s\n", why.c_str());
  return 1;
}

}  // namespace

int main(int argc, char** argv) {
  if (!Limit(RLIMIT_CPU, kCpuSeconds) || !Limit(RLIMIT_AS, kMemoryBytes) || !Limit(RLIMIT_FSIZE, kFileBytes) ||
      !Limit(RLIMIT_CORE, 0)) {
    return Fail("cannot set the limits");
  }
  errno = 0;
  if (::nice(19) == -1 && errno != 0) return Fail("cannot lower the priority");
  if (argc != 3) return Fail("usage: mowbite_sound_convert <in.mp3> <out.wav>");

  std::string mp3;
  std::string error;
  switch (mowbite_hold::ReadSmallFile(argv[1], mowbite_sound::kMaxMp3Bytes, &mp3, &error)) {
    case mowbite_hold::FileRead::kText: break;
    case mowbite_hold::FileRead::kMissing: return Fail(std::string("cannot open ") + argv[1]);
    case mowbite_hold::FileRead::kTooLarge:
      return Fail("larger than " + std::to_string(mowbite_sound::kMaxMp3Bytes >> 20) + " MB");
    case mowbite_hold::FileRead::kFailed: return Fail(error);
  }
  const auto wav = mowbite_sound::DecodeMp3(mp3, &error);
  if (!wav) return Fail(error);
  std::ofstream out(argv[2], std::ios::binary | std::ios::trunc);
  const std::string bytes = mowbite_sound::EncodeWav(*wav);
  out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  out.close();
  if (!out) return Fail(std::string("cannot write ") + argv[2]);
  return 0;
}
