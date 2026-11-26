#ifndef MOWBITE_SERIAL_RECOGNIZE_H
#define MOWBITE_SERIAL_RECOGNIZE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_serial {

// What a stream of bytes looks like: nothing, NMEA lines (GNSS), WIT frames (IMU), RTCM 3 frames, or none of these.
enum class Kind { kSilent, kUnknown, kNmea, kWit, kRtcm };

const char* KindName(Kind kind);

struct Heard {
  Kind kind = Kind::kSilent;
  uint64_t bytes = 0;
  double seconds = 0;
  // Lines with a valid checksum; talkers ("GN", "GP", "P" for proprietary) and sentence types, sorted.
  int nmea_lines = 0;
  std::vector<std::string> talkers;
  std::vector<std::string> sentences;
  // GGA lines per second and the quality field of the last one (0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float),
  // -1 without a GGA.
  double gga_hz = 0;
  int fix = -1;
  // Frames with a valid checksum and their types (0x50..0x5F), sorted.
  int wit_frames = 0;
  std::vector<int> wit_types;
  // Frames with a valid CRC and their message numbers, sorted.
  int rtcm_frames = 0;
  std::vector<int> rtcm_types;
};

// Collects bytes of one port at one rate and tells what they are. Keeps at most kKeep bytes for the judgement and
// counts the rest.
class Recognizer {
 public:
  static constexpr size_t kKeep = 64 * 1024;

  void Feed(const uint8_t* data, size_t size);
  // A kind counts when enough frames of it came and the frames recognized make up at least half of the bytes kept;
  // of several, the one with the most bytes.
  Heard Result(double seconds) const;

 private:
  std::vector<uint8_t> kept_;
  uint64_t bytes_ = 0;
};

}  // namespace mowbite_serial

#endif  // MOWBITE_SERIAL_RECOGNIZE_H
