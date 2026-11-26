#include <gtest/gtest.h>

#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "mowbite_gnss/gga.h"
#include "mowbite_ntrip/rtcm.h"
#include "mowbite_serial/recognize.h"

namespace mowbite_serial {
namespace {

// The checksums come from the libraries of the GNSS and the NTRIP client, which test them on their own.
std::string Nmea(const std::string& body) {
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\r\n", mowbite_gnss::NmeaChecksum(body));
  return "$" + body + tail;
}

std::vector<uint8_t> Wit(uint8_t type) {
  std::vector<uint8_t> frame = {0x55, type, 1, 2, 3, 4, 5, 6, 7, 8};
  uint8_t sum = 0;
  for (uint8_t b : frame) sum = static_cast<uint8_t>(sum + b);
  frame.push_back(sum);
  return frame;
}

std::vector<uint8_t> Rtcm(int type, size_t length) {
  std::vector<uint8_t> frame = {0xd3, static_cast<uint8_t>(length >> 8), static_cast<uint8_t>(length & 0xff),
                                static_cast<uint8_t>(type >> 4), static_cast<uint8_t>((type & 0xf) << 4)};
  frame.resize(3 + length, 0x5a);
  const uint32_t crc = mowbite_ntrip::Crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc >> 16));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(static_cast<uint8_t>(crc));
  return frame;
}

Heard Hear(const std::vector<uint8_t>& bytes, double seconds = 2) {
  Recognizer recognizer;
  recognizer.Feed(bytes.data(), bytes.size());
  return recognizer.Result(seconds);
}

Heard Hear(const std::string& text, double seconds = 2) {
  return Hear(std::vector<uint8_t>(text.begin(), text.end()), seconds);
}

std::vector<uint8_t> Noise(size_t size, unsigned seed) {
  std::mt19937 random(seed);
  std::vector<uint8_t> bytes(size);
  for (auto& b : bytes) b = static_cast<uint8_t>(random());
  return bytes;
}

TEST(Recognize, NothingIsSilent) {
  const Heard heard = Hear(std::vector<uint8_t>{});
  EXPECT_EQ(heard.kind, Kind::kSilent);
  EXPECT_STREQ(KindName(heard.kind), "silent");
}

TEST(Recognize, NmeaWithTalkersRateAndFix) {
  std::string text;
  for (int i = 0; i < 4; i++) {
    text += Nmea("GNGGA,120000.00,5200.0,N,00800.0,E,4,12,0.8,60.0,M,47.0,M,1.0,0000");
    text += Nmea("GPRMC,120000.00,A,5200.0,N,00800.0,E,0.0,0.0,261126,,,A");
  }
  text += Nmea("PQTMVER,1.0");
  const Heard heard = Hear(text, 4);
  EXPECT_EQ(heard.kind, Kind::kNmea);
  EXPECT_EQ(heard.nmea_lines, 9);
  EXPECT_EQ(heard.talkers, (std::vector<std::string>{"GN", "GP", "P"}));
  EXPECT_EQ(heard.sentences, (std::vector<std::string>{"GGA", "QTMVER", "RMC"}));
  EXPECT_DOUBLE_EQ(heard.gga_hz, 1.0);
  EXPECT_EQ(heard.fix, 4);
}

TEST(Recognize, NmeaWithAWrongChecksumIsNotCounted) {
  std::string line = Nmea("GNGGA,1,2,3,4,5,1,7,8,9");
  line[line.size() - 3] = line[line.size() - 3] == '0' ? '1' : '0';
  const Heard heard = Hear(line + line + line);
  EXPECT_EQ(heard.nmea_lines, 0);
  EXPECT_EQ(heard.kind, Kind::kUnknown);
}

TEST(Recognize, WitFrames) {
  std::vector<uint8_t> bytes;
  for (int i = 0; i < 10; i++) {
    for (uint8_t type : {0x51, 0x52, 0x53}) {
      const auto frame = Wit(type);
      bytes.insert(bytes.end(), frame.begin(), frame.end());
    }
  }
  const Heard heard = Hear(bytes);
  EXPECT_EQ(heard.kind, Kind::kWit);
  EXPECT_EQ(heard.wit_frames, 30);
  EXPECT_EQ(heard.wit_types, (std::vector<int>{0x51, 0x52, 0x53}));
}

TEST(Recognize, RtcmFrames) {
  std::vector<uint8_t> bytes;
  for (int type : {1005, 1074, 1084, 1074}) {
    const auto frame = Rtcm(type, 40);
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  const Heard heard = Hear(bytes);
  EXPECT_EQ(heard.kind, Kind::kRtcm);
  EXPECT_EQ(heard.rtcm_frames, 4);
  EXPECT_EQ(heard.rtcm_types, (std::vector<int>{1005, 1074, 1084}));
}

TEST(Recognize, NmeaAndRtcmTogetherByTheLargerPart) {
  std::string text;
  for (int i = 0; i < 3; i++) text += Nmea("GNGGA,1,2,3,4,5,1,7,8,9");
  std::vector<uint8_t> bytes(text.begin(), text.end());
  for (int i = 0; i < 3; i++) {
    const auto frame = Rtcm(1077, 300);
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  const Heard heard = Hear(bytes);
  EXPECT_EQ(heard.kind, Kind::kRtcm);
  EXPECT_EQ(heard.nmea_lines, 3);
  EXPECT_EQ(heard.rtcm_frames, 3);
}

// Bytes at a wrong rate: random, no kind, however often a lone frame turns up by chance.
TEST(Recognize, NoiseIsUnknown) {
  for (unsigned seed = 1; seed <= 20; seed++) {
    const Heard heard = Hear(Noise(8000, seed));
    EXPECT_EQ(heard.kind, Kind::kUnknown) << seed;
    EXPECT_EQ(heard.bytes, 8000u);
  }
}

TEST(Recognize, FramesInNoiseBelowHalfAreUnknown) {
  std::vector<uint8_t> bytes = Noise(4000, 7);
  const std::string line = Nmea("GNGGA,1,2,3,4,5,1,7,8,9");
  for (int i = 0; i < 5; i++) bytes.insert(bytes.end(), line.begin(), line.end());
  EXPECT_EQ(Hear(bytes).kind, Kind::kUnknown);
}

TEST(Recognize, KeepsAtMostItsLimitAndCountsAll) {
  Recognizer recognizer;
  const std::vector<uint8_t> chunk(Recognizer::kKeep, 'x');
  recognizer.Feed(chunk.data(), chunk.size());
  recognizer.Feed(chunk.data(), chunk.size());
  EXPECT_EQ(recognizer.Result(1).bytes, 2 * Recognizer::kKeep);
}

}  // namespace
}  // namespace mowbite_serial
