#include <gtest/gtest.h>

#include <cstdio>
#include <string>

#include "tron_gnss/nmea.h"

namespace tron {
namespace {

std::string Sentence(const std::string& body) {
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\r\n", NmeaChecksum(body));
  return "$" + body + tail;
}

std::vector<NmeaMessage> Parse(NmeaParser& parser, const std::string& text) {
  std::vector<NmeaMessage> out;
  parser.Feed(reinterpret_cast<const uint8_t*>(text.data()), text.size(), out);
  return out;
}

TEST(Nmea, ChecksumOfTheTextbookGga) {
  EXPECT_EQ(NmeaChecksum("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"), 0x47);
}

TEST(Nmea, DecodesTextbookGga) {
  NmeaParser parser;
  auto out = Parse(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
  ASSERT_EQ(out.size(), 1u);
  ASSERT_EQ(out[0].type, NmeaMessage::Type::kGga);
  const GgaData& g = out[0].gga;
  EXPECT_NEAR(g.utc_s, 12 * 3600 + 35 * 60 + 19, 1e-9);
  EXPECT_NEAR(*g.lat_deg, 48.0 + 7.038 / 60.0, 1e-9);
  EXPECT_NEAR(*g.lon_deg, 11.0 + 31.0 / 60.0, 1e-9);
  EXPECT_EQ(g.quality, 1);
  EXPECT_EQ(g.satellites, 8);
  EXPECT_NEAR(*g.hdop, 0.9, 1e-9);
  EXPECT_NEAR(*g.altitude_m, 545.4, 1e-9);
  EXPECT_EQ(parser.decoded(), 1u);
}

TEST(Nmea, DecodesTextbookRmcAndVtg) {
  NmeaParser parser;
  auto out = Parse(parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].rmc.valid);
  EXPECT_NEAR(*out[0].rmc.speed_mps, 22.4 * 0.514444444, 1e-6);
  EXPECT_NEAR(*out[0].rmc.course_deg, 84.4, 1e-9);

  out = Parse(parser, "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\r\n");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].vtg.valid);
  EXPECT_NEAR(*out[0].vtg.speed_mps, 5.5 * 0.514444444, 1e-6);
  EXPECT_NEAR(*out[0].vtg.course_deg, 54.7, 1e-9);
}

TEST(Nmea, SouthWestIsNegative) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNGGA,000001.00,3351.5000,S,15112.0000,W,4,20,0.6,10.0,M,0.0,M,1.0,0000"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(*out[0].gga.lat_deg, -(33.0 + 51.5 / 60.0), 1e-9);
  EXPECT_NEAR(*out[0].gga.lon_deg, -(151.0 + 12.0 / 60.0), 1e-9);
  EXPECT_EQ(out[0].gga.quality, 4);
}

TEST(Nmea, GgaCarriesTheAgeOfTheCorrections) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNGGA,120000.00,5200.0,N,00900.0,E,4,20,0.6,50.0,M,0.0,M,2.5,0000"));
  ASSERT_EQ(out.size(), 1u);
  ASSERT_TRUE(out[0].gga.correction_age_s);
  EXPECT_DOUBLE_EQ(*out[0].gga.correction_age_s, 2.5);
  // Empty without corrections, a negative age is none either.
  out = Parse(parser, Sentence("GNGGA,120001.00,5200.0,N,00900.0,E,4,20,0.6,50.0,M,0.0,M,,"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].gga.correction_age_s);
  out = Parse(parser, Sentence("GNGGA,120002.00,5200.0,N,00900.0,E,4,20,0.6,50.0,M,0.0,M,-1.0,0000"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].gga.correction_age_s);
  // Cut off before field 13.
  out = Parse(parser, Sentence("GNGGA,120003.00,5200.0,N,00900.0,E,4,20,0.6,50.0"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].gga.correction_age_s);
}

TEST(Nmea, AnyTalkerIsAccepted) {
  NmeaParser parser;
  EXPECT_EQ(Parse(parser, Sentence("GLGGA,120000,5200.0,N,01000.0,E,1,05,1.0,50.0,M,0,M,,")).size(), 1u);
  EXPECT_EQ(Parse(parser, Sentence("BDVTG,10.0,T,,M,0.0,N,0.0,K,A")).size(), 1u);
}

TEST(Nmea, EmptyFieldsWithoutFix) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNGGA,120000.00,,,,,0,00,99.99,,,,,,"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].gga.lat_deg.has_value());
  EXPECT_FALSE(out[0].gga.lon_deg.has_value());
  EXPECT_EQ(out[0].gga.quality, 0);
  EXPECT_FALSE(out[0].gga.altitude_m.has_value());
}

TEST(Nmea, RmcWithoutCourseAndNotValid) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNRMC,120000.00,V,,,,,,,281026,,,N,V"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].rmc.valid);
  EXPECT_FALSE(out[0].rmc.speed_mps.has_value());
  EXPECT_FALSE(out[0].rmc.course_deg.has_value());
}

TEST(Nmea, RmcModeNIsInvalidEvenWithStatusA) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNRMC,120000.00,A,5200.0,N,01000.0,E,0.5,10.0,281026,,,N,V"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].rmc.valid);
}

TEST(Nmea, VtgFallsBackToKmh) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNVTG,90.0,T,,M,,N,3.6,K,A"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(*out[0].vtg.speed_mps, 1.0, 1e-9);
}

TEST(Nmea, BadChecksumIsCountedAndDropped) {
  NmeaParser parser;
  auto out = Parse(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n");
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(parser.checksum_errors(), 1u);
}

TEST(Nmea, MalformedLinesAreCounted) {
  NmeaParser parser;
  EXPECT_TRUE(Parse(parser, "$GPGGA,1,2\r\n").empty());                // no checksum
  EXPECT_TRUE(Parse(parser, "$GPGGA,1,2*ZZ\r\n").empty());             // not hex
  EXPECT_TRUE(Parse(parser, Sentence("GPGGA,12,,,,,1,,,,")).empty());  // time too short
  EXPECT_GE(parser.malformed(), 3u);
}

TEST(Nmea, OutOfRangeCoordinatesAreDropped) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GPGGA,120000,9100.0,N,18100.0,E,1,05,1.0,50.0,M,0,M,,"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].gga.lat_deg.has_value());
  EXPECT_FALSE(out[0].gga.lon_deg.has_value());
}

TEST(Nmea, OtherSentencesAreIgnored) {
  NmeaParser parser;
  EXPECT_TRUE(Parse(parser, Sentence("GNGSA,A,3,01,02,03,,,,,,,,,,1.0,0.5,0.8,1")).empty());
  EXPECT_TRUE(Parse(parser, Sentence("GNHPR,120000.00,0.0000,0.0000,0.0000,0,0,0,0")).empty());
  EXPECT_EQ(parser.ignored(), 2u);
  EXPECT_EQ(parser.checksum_errors(), 0u);
}

TEST(Nmea, EachIgnoredKindIsNamedOnceWithoutNumbers) {
  NmeaParser parser;
  Parse(parser, Sentence("GNGLL,4807.038,N,01131.000,E,123519,A,A"));
  Parse(parser, Sentence("GNGLL,4807.039,N,01131.000,E,123520,A,A"));
  Parse(parser, Sentence("command,GNRTK,ON,response: OK"));
  Parse(parser, Sentence("GNRTK,1"));
  Parse(parser, Sentence("GNTXT,,x"));
  ASSERT_EQ(parser.ignored_kinds().size(), 3u);
  EXPECT_EQ(parser.ignored_kinds()[0], "$GNGLL (7 fields)");
  EXPECT_EQ(parser.ignored_kinds()[1], "$command (3 fields, first GNRTK)");
  EXPECT_EQ(parser.ignored_kinds()[2], "$GNTXT (2 fields)");
  EXPECT_EQ(parser.ignored(), 4u);
  EXPECT_EQ(parser.malformed(), 1u);
  for (int i = 0; i < 40; ++i) Parse(parser, Sentence("GNX" + std::to_string(i) + ",A"));
  EXPECT_EQ(parser.ignored_kinds().size(), NmeaParser::kMaxIgnoredKinds);
}

TEST(Nmea, VersionAndHardwareLinesAreNamedInFull) {
  NmeaParser parser;
  Parse(parser, Sentence("GNVER,1.2.3"));
  Parse(parser, Sentence("GHARDM,7,LG290P"));
  Parse(parser, Sentence("GNVER," + std::string(200, 'v')));
  ASSERT_EQ(parser.ignored_kinds().size(), 2u);
  EXPECT_EQ(parser.ignored_kinds()[0], "$GNVER (1 fields: 1.2.3)");
  EXPECT_EQ(parser.ignored_kinds()[1], "$GHARDM (2 fields: 7,LG290P)");
}

TEST(Nmea, SentenceSplitAcrossReads) {
  NmeaParser parser;
  std::string s = Sentence("GNGGA,120000.00,5200.0,N,01000.0,E,4,20,0.6,10.0,M,0.0,M,1.0,0000");
  std::vector<NmeaMessage> out;
  for (char c : s) {
    uint8_t b = static_cast<uint8_t>(c);
    parser.Feed(&b, 1, out);
  }
  EXPECT_EQ(out.size(), 1u);
}

TEST(Nmea, BinaryFramesBetweenSentencesAreSkipped) {
  NmeaParser parser;
  std::string text = std::string("\xAA\x44\xB5\x01\x02\x03", 6) + Sentence("GNVTG,90.0,T,,M,1.0,N,1.8,K,A") +
                     std::string("\xAA\x44\xB5\x00\xFF", 5) + Sentence("GNVTG,91.0,T,,M,1.0,N,1.8,K,A");
  auto out = Parse(parser, text);
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(parser.checksum_errors(), 0u);
}

TEST(Nmea, CutOffSentenceDoesNotSwallowTheNext) {
  NmeaParser parser;
  std::string text = "$GNGGA,1200" + Sentence("GNVTG,90.0,T,,M,1.0,N,1.8,K,A");
  auto out = Parse(parser, text);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, NmeaMessage::Type::kVtg);
  EXPECT_EQ(parser.malformed(), 1u);
}

TEST(Nmea, OverlongLineIsDroppedAndParsingRecovers) {
  NmeaParser parser;
  std::string text = "$" + std::string(1000, 'A') + "\r\n" + Sentence("GNVTG,90.0,T,,M,1.0,N,1.8,K,A");
  auto out = Parse(parser, text);
  EXPECT_EQ(out.size(), 1u);
  EXPECT_GE(parser.malformed(), 1u);
}

TEST(Nmea, GarbageNeverProducesMessages) {
  NmeaParser parser;
  std::string noise;
  for (int i = 0; i < 4096; i++) noise.push_back(static_cast<char>((i * 131 + 7) & 0xFF));
  auto out = Parse(parser, noise);
  EXPECT_TRUE(out.empty());
}

TEST(Nmea, TheBoardCommandHasItsChecksum) {
  // The board compares "$GNRTK,ON*69" as text; the checksum in it is the NMEA one.
  EXPECT_EQ(NmeaChecksum("GNRTK,ON"), 0x69);
}

TEST(Nmea, BoardReports) {
  NmeaParser parser;
  auto out = Parse(parser, Sentence("GNRTK,ON") + Sentence("GNRTK,OFF") + Sentence("GNRTK,MAYBE"));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].type, NmeaMessage::Type::kBoardRtk);
  EXPECT_TRUE(out[0].board.on);
  EXPECT_EQ(out[0].sentence, "$GNRTK,ON*69");
  EXPECT_EQ(out[1].type, NmeaMessage::Type::kBoardRtk);
  EXPECT_FALSE(out[1].board.on);
  EXPECT_EQ(parser.malformed(), 1u);
}

TEST(Nmea, SentenceIsKeptWithoutLineEnd) {
  NmeaParser parser;
  auto out = Parse(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sentence, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
}

}  // namespace
}  // namespace tron
