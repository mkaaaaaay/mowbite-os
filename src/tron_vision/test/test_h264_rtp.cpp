#include <gtest/gtest.h>

#include <numeric>

#include "tron_vision/h264_rtp.h"

namespace tron_vision {
namespace {

std::vector<uint8_t> Nal(uint8_t header, size_t size) {
  std::vector<uint8_t> nal(size);
  std::iota(nal.begin(), nal.end(), static_cast<uint8_t>(1));
  nal[0] = header;
  return nal;
}

uint16_t Sequence(const std::vector<uint8_t>& p) {
  return static_cast<uint16_t>(p[2] << 8 | p[3]);
}

uint32_t Timestamp(const std::vector<uint8_t>& p) {
  return static_cast<uint32_t>(p[4]) << 24 | static_cast<uint32_t>(p[5]) << 16 | static_cast<uint32_t>(p[6]) << 8 | p[7];
}

TEST(SplitAnnexB, BothStartCodesAndPadding) {
  const std::vector<uint8_t> stream = {0, 0, 0, 1, 0x67, 0x42, 0xe0, 0x1f, 0, 0, 1, 0x68, 0xce, 0, 0, 0, 0, 1, 0x65, 0x88, 0, 0};
  const std::vector<NalUnit> nals = SplitAnnexB(stream.data(), stream.size());
  ASSERT_EQ(nals.size(), 3U);
  EXPECT_EQ(nals[0].Type(), kNalSps);
  EXPECT_EQ(nals[0].size, 4U);
  EXPECT_EQ(nals[1].Type(), kNalPps);
  EXPECT_EQ(nals[1].size, 2U);
  EXPECT_EQ(nals[2].Type(), kNalIdr);
  EXPECT_EQ(nals[2].size, 2U);
  EXPECT_EQ(nals[2].data, stream.data() + 18);
}

TEST(SplitAnnexB, NothingWithoutAStartCode) {
  const std::vector<uint8_t> junk = {1, 2, 3, 0, 0, 2, 9};
  EXPECT_TRUE(SplitAnnexB(junk.data(), junk.size()).empty());
  EXPECT_TRUE(SplitAnnexB(junk.data(), 0).empty());
  const std::vector<uint8_t> only_codes = {0, 0, 1, 0, 0, 0, 1};
  EXPECT_TRUE(SplitAnnexB(only_codes.data(), only_codes.size()).empty());
}

TEST(ParameterSets, FoundAndNamed) {
  const std::vector<uint8_t> header = {0, 0, 0, 1, 0x67, 0x42, 0xe0, 0x1f, 0xaa, 0, 0, 0, 1, 0x68, 0xce, 0x3c, 0x80};
  ParameterSets sets;
  ASSERT_TRUE(FindParameterSets(header.data(), header.size(), sets));
  EXPECT_EQ(sets.sps, (std::vector<uint8_t>{0x67, 0x42, 0xe0, 0x1f, 0xaa}));
  EXPECT_EQ(sets.pps, (std::vector<uint8_t>{0x68, 0xce, 0x3c, 0x80}));
  EXPECT_EQ(ProfileLevelId(sets.sps), "42e01f");
  EXPECT_EQ(ProfileLevelId({0x67, 0x42}), "");
  // An SPS alone is not enough.
  EXPECT_FALSE(FindParameterSets(header.data(), 9, sets));
}

TEST(Base64, Rfc4648Vectors) {
  const auto encode = [](const std::string& s) { return Base64(reinterpret_cast<const uint8_t*>(s.data()), s.size()); };
  EXPECT_EQ(encode(""), "");
  EXPECT_EQ(encode("f"), "Zg==");
  EXPECT_EQ(encode("fo"), "Zm8=");
  EXPECT_EQ(encode("foo"), "Zm9v");
  EXPECT_EQ(encode("foobar"), "Zm9vYmFy");
  const uint8_t high[] = {0xfb, 0xff, 0xfe};
  EXPECT_EQ(Base64(high, 3), "+//+");
}

TEST(RtpClock, NinetyKilohertzAndAlwaysRising) {
  RtpClock clock(1000);
  EXPECT_EQ(clock.Next(5000000), 1000U);
  EXPECT_EQ(clock.Next(5040000), 1000U + 3600);
  // Standing still and stepping back still rise by one tick.
  EXPECT_EQ(clock.Next(5040000), 1000U + 3601);
  EXPECT_EQ(clock.Next(5000000), 1000U + 3602);
  EXPECT_EQ(clock.Next(5080000), 1000U + 7200);
}

TEST(RtpClock, WrapsWithTheCounter) {
  RtpClock clock(0xFFFFFF00U);
  EXPECT_EQ(clock.Next(0), 0xFFFFFF00U);
  EXPECT_EQ(clock.Next(10000), 0xFFFFFF00U + 900);
}

TEST(RtpPacketizer, SmallUnitsGoOutAlone) {
  RtpPacketizer packetizer(0x11223344, 65535);
  const std::vector<uint8_t> sps = Nal(0x67, 20);
  const std::vector<uint8_t> pps = Nal(0x68, 5);
  const std::vector<uint8_t> idr = Nal(0x65, 1188);
  std::vector<std::vector<uint8_t>> packets;
  packetizer.Packetize({{sps.data(), sps.size()}, {pps.data(), pps.size()}, {idr.data(), idr.size()}}, 90000, packets);
  ASSERT_EQ(packets.size(), 3U);
  for (size_t i = 0; i < packets.size(); i++) {
    const std::vector<uint8_t>& p = packets[i];
    EXPECT_EQ(p[0], 0x80);
    EXPECT_EQ(p[1] & 0x7F, 96);
    EXPECT_EQ((p[1] & 0x80) != 0, i == 2);
    EXPECT_EQ(Timestamp(p), 90000U);
    EXPECT_EQ((std::vector<uint8_t>{p[8], p[9], p[10], p[11]}), (std::vector<uint8_t>{0x11, 0x22, 0x33, 0x44}));
  }
  // The sequence number wraps.
  EXPECT_EQ(Sequence(packets[0]), 65535);
  EXPECT_EQ(Sequence(packets[1]), 0);
  EXPECT_EQ(Sequence(packets[2]), 1);
  EXPECT_EQ(packetizer.Sequence(), 2);
  EXPECT_EQ(std::vector<uint8_t>(packets[0].begin() + 12, packets[0].end()), sps);
  EXPECT_EQ(packets[2].size(), 12U + 1188);
}

TEST(RtpPacketizer, LargeUnitsAreFragmented) {
  for (size_t size : {1189U, 1187U + 1186U, 1187U + 1186U + 1U, 40000U}) {
    RtpPacketizer packetizer(1, 0);
    const std::vector<uint8_t> idr = Nal(0x65, size);
    std::vector<std::vector<uint8_t>> packets;
    packetizer.Packetize({{idr.data(), idr.size()}}, 7, packets);
    ASSERT_EQ(packets.size(), (size - 1 + 1185) / 1186) << size;
    std::vector<uint8_t> joined = {0};
    for (size_t i = 0; i < packets.size(); i++) {
      const std::vector<uint8_t>& p = packets[i];
      ASSERT_GE(p.size(), 15U);
      EXPECT_LE(p.size(), 12U + 1188);
      // NRI of the unit, type 28; then start and end bits and the type of the unit.
      EXPECT_EQ(p[12], 0x60 | 28);
      EXPECT_EQ(p[13], (i == 0 ? 0x80 : 0) | (i + 1 == packets.size() ? 0x40 : 0) | 5);
      EXPECT_EQ((p[1] & 0x80) != 0, i + 1 == packets.size());
      EXPECT_EQ(Sequence(p), i);
      joined.insert(joined.end(), p.begin() + 14, p.end());
    }
    joined[0] = static_cast<uint8_t>((packets[0][12] & 0xE0) | (packets[0][13] & 0x1F));
    EXPECT_EQ(joined, idr) << size;
  }
}

TEST(RtpPacketizer, PacketsOfAnEarlierUnitAreGone) {
  RtpPacketizer packetizer(1, 0);
  const std::vector<uint8_t> big = Nal(0x41, 5000);
  const std::vector<uint8_t> small = Nal(0x41, 100);
  std::vector<std::vector<uint8_t>> packets;
  packetizer.Packetize({{big.data(), big.size()}}, 1, packets);
  EXPECT_EQ(packets.size(), 5U);
  packetizer.Packetize({{small.data(), small.size()}}, 2, packets);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].size(), 112U);
  EXPECT_EQ(packets[0][1], 0x80 | 96);
  packetizer.Packetize({}, 3, packets);
  EXPECT_TRUE(packets.empty());
}

}  // namespace
}  // namespace tron_vision
