#include <gtest/gtest.h>

#include "tron_imu/wit_parser.h"

namespace {

std::vector<uint8_t> Frame(uint8_t type, int16_t a, int16_t b, int16_t c, int16_t d = 0) {
  std::vector<uint8_t> f = {0x55, type};
  for (int16_t v : {a, b, c, d}) {
    f.push_back(static_cast<uint8_t>(v & 0xFF));
    f.push_back(static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF));
  }
  uint8_t sum = 0;
  for (uint8_t byte : f) sum = static_cast<uint8_t>(sum + byte);
  f.push_back(sum);
  return f;
}

}  // namespace

TEST(WitParser, ParsesOneFrame) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  auto f = Frame(0x51, 1000, -2000, 3000, 2512);
  parser.Feed(f.data(), f.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, tron::WitType::kAcceleration);
  EXPECT_EQ(out[0].raw[0], 1000);
  EXPECT_EQ(out[0].raw[1], -2000);
  EXPECT_EQ(out[0].raw[2], 3000);
  EXPECT_EQ(out[0].raw[3], 2512);
}

TEST(WitParser, FramesSplitAcrossReads) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  auto f = Frame(0x53, 1, 2, 3);
  for (uint8_t byte : f) parser.Feed(&byte, 1, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, tron::WitType::kAngle);
}

TEST(WitParser, ThreeFramesInOneRead) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  std::vector<uint8_t> all;
  for (uint8_t t : {0x51, 0x52, 0x53}) {
    auto f = Frame(t, 1, 2, 3);
    all.insert(all.end(), f.begin(), f.end());
  }
  parser.Feed(all.data(), all.size(), out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[2].type, tron::WitType::kAngle);
}

TEST(WitParser, ResyncsAfterGarbage) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  std::vector<uint8_t> data = {0x00, 0x12, 0x55, 0x99};
  auto f = Frame(0x52, 5, 6, 7);
  data.insert(data.end(), f.begin(), f.end());
  parser.Feed(data.data(), data.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].raw[0], 5);
  EXPECT_GT(parser.skipped_bytes(), 0u);
}

TEST(WitParser, SyncByteInsideDataDoesNotBreakNextFrame) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  auto f = Frame(0x51, 0x5555, 0x5551, 0x5552);
  auto g = Frame(0x53, 9, 8, 7);
  f.insert(f.end(), g.begin(), g.end());
  parser.Feed(f.data(), f.size(), out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[1].raw[0], 9);
}

TEST(WitParser, BadChecksumIsCountedAndDropped) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  auto bad = Frame(0x51, 1, 2, 3);
  bad.back() ^= 0xFF;
  auto good = Frame(0x52, 4, 5, 6);
  bad.insert(bad.end(), good.begin(), good.end());
  parser.Feed(bad.data(), bad.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, tron::WitType::kAngularVelocity);
  EXPECT_EQ(parser.checksum_errors(), 1u);
}

TEST(WitParser, UnknownTypeWithValidSumIsCountedAndSkipped) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  auto other = Frame(0x54, 1, 2, 3);  // magnetometer, not used
  auto good = Frame(0x53, 4, 5, 6);
  other.insert(other.end(), good.begin(), good.end());
  parser.Feed(other.data(), other.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(parser.unknown_types(), 1u);
}

TEST(WitParser, ShortFrameStaysBufferedUntilComplete) {
  tron::WitParser parser;
  std::vector<tron::WitPacket> out;
  auto f = Frame(0x51, 1, 2, 3);
  parser.Feed(f.data(), 10, out);
  EXPECT_TRUE(out.empty());
  parser.Feed(f.data() + 10, 1, out);
  EXPECT_EQ(out.size(), 1u);
}
