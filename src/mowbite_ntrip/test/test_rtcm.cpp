#include <gtest/gtest.h>

#include <string>

#include "mowbite_ntrip/rtcm.h"
#include "rtcm_frames.h"

namespace mowbite_ntrip {
namespace {

std::vector<std::vector<uint8_t>> Feed(RtcmFramer& framer, const std::vector<uint8_t>& bytes) {
  std::vector<std::vector<uint8_t>> frames;
  framer.Feed(bytes.data(), bytes.size(), frames);
  return frames;
}

TEST(Rtcm, Crc24qOfTheCheckString) {
  // The usual check value of CRC-24Q (as used by RTCM 3) for "123456789".
  const std::string check = "123456789";
  EXPECT_EQ(Crc24q(reinterpret_cast<const uint8_t*>(check.data()), check.size()), 0xCDE703u);
}

TEST(Rtcm, Crc24qOfNothingIsZero) {
  EXPECT_EQ(Crc24q(nullptr, 0), 0u);
}

TEST(Rtcm, OneWholeFrame) {
  RtcmFramer framer;
  auto frame = MakeFrame(1005, 19);
  auto frames = Feed(framer, frame);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], frame);
  EXPECT_EQ(RtcmMessageType(frames[0]), 1005);
  EXPECT_EQ(framer.frames(), 1u);
  EXPECT_EQ(framer.crc_errors(), 0u);
}

TEST(Rtcm, FrameSplitIntoSingleBytes) {
  RtcmFramer framer;
  auto frame = MakeFrame(1074, 120);
  std::vector<std::vector<uint8_t>> frames;
  for (uint8_t b : frame) framer.Feed(&b, 1, frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], frame);
}

TEST(Rtcm, SeveralFramesAndGarbageBetween) {
  RtcmFramer framer;
  std::vector<uint8_t> bytes = {'I', 'C', 'Y', 0x00, 0xFF};
  auto a = MakeFrame(1005, 19), b = MakeFrame(1230, 0), c = MakeFrame(1084, 300);
  bytes.insert(bytes.end(), a.begin(), a.end());
  bytes.insert(bytes.end(), {'\r', '\n'});
  bytes.insert(bytes.end(), b.begin(), b.end());
  bytes.insert(bytes.end(), c.begin(), c.end());
  auto frames = Feed(framer, bytes);
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0], a);
  EXPECT_EQ(frames[1], b);
  EXPECT_EQ(frames[2], c);
  EXPECT_EQ(framer.skipped(), 7u);
}

TEST(Rtcm, LargestFrame) {
  RtcmFramer framer;
  auto frame = MakeFrame(4094, RtcmFramer::kMaxPayload);
  auto frames = Feed(framer, frame);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].size(), RtcmFramer::kMaxPayload + 6);
}

TEST(Rtcm, BadCrcIsDroppedAndTheNextFrameFound) {
  RtcmFramer framer;
  auto bad = MakeFrame(1005, 19);
  bad[10] ^= 0x01;
  auto good = MakeFrame(1077, 50);
  std::vector<uint8_t> bytes = bad;
  bytes.insert(bytes.end(), good.begin(), good.end());
  auto frames = Feed(framer, bytes);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], good);
  EXPECT_EQ(framer.crc_errors(), 1u);
}

TEST(Rtcm, PreambleWithReservedBitsIsNoFrame) {
  RtcmFramer framer;
  auto good = MakeFrame(1005, 19);
  std::vector<uint8_t> bytes = {0xD3, 0x40, 0x10};
  bytes.insert(bytes.end(), good.begin(), good.end());
  auto frames = Feed(framer, bytes);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], good);
}

TEST(Rtcm, IncompleteFrameWaits) {
  RtcmFramer framer;
  auto frame = MakeFrame(1005, 19);
  std::vector<uint8_t> first(frame.begin(), frame.begin() + 10), rest(frame.begin() + 10, frame.end());
  EXPECT_TRUE(Feed(framer, first).empty());
  EXPECT_EQ(Feed(framer, rest).size(), 1u);
}

TEST(Rtcm, ResetDropsAHalfFrame) {
  RtcmFramer framer;
  auto frame = MakeFrame(1005, 19);
  std::vector<uint8_t> first(frame.begin(), frame.begin() + 10);
  Feed(framer, first);
  framer.Reset();
  EXPECT_EQ(Feed(framer, frame).size(), 1u);
}

}  // namespace
}  // namespace mowbite_ntrip
