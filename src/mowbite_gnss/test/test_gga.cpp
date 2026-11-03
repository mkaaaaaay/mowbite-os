#include <gtest/gtest.h>

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "mowbite_gnss/gga.h"

namespace mowbite_gnss {
namespace {

std::vector<std::string> Fields(const std::string& sentence) {
  std::vector<std::string> out;
  std::stringstream in(sentence.substr(1, sentence.find('*') - 1));
  std::string field;
  while (std::getline(in, field, ',')) out.push_back(field);
  if (!sentence.empty() && sentence[sentence.find('*') - 1] == ',') out.push_back("");
  return out;
}

GgaFields Garden() {
  GgaFields f;
  f.utc_s = 12 * 3600 + 34 * 60 + 56.78;
  f.lat_deg = 52.5;
  f.lon_deg = 9.75;
  f.altitude_m = 61.234;
  f.quality = GgaQuality::kRtkFixed;
  f.satellites = 10;
  f.hdop = 1.0;
  return f;
}

TEST(Gga, FieldsInOrder) {
  const std::string s = BuildGga(Garden());
  auto f = Fields(s);
  ASSERT_EQ(f.size(), 15u) << s;
  EXPECT_EQ(f[0], "GPGGA");
  EXPECT_EQ(f[1], "123456.78");
  EXPECT_EQ(f[2], "5230.0000000");
  EXPECT_EQ(f[3], "N");
  EXPECT_EQ(f[4], "00945.0000000");
  EXPECT_EQ(f[5], "E");
  EXPECT_EQ(f[6], "4");
  EXPECT_EQ(f[7], "10");
  EXPECT_EQ(f[8], "1.0");
  EXPECT_EQ(f[9], "61.234");
  EXPECT_EQ(f[10], "M");
  EXPECT_EQ(f[11], "");
  EXPECT_EQ(f[12], "M");
  EXPECT_EQ(f[13], "");
  EXPECT_EQ(f[14], "");
}

TEST(Gga, ChecksumCoversTheBody) {
  const std::string s = BuildGga(Garden());
  const size_t star = s.find('*');
  ASSERT_NE(star, std::string::npos);
  ASSERT_EQ(s.size(), star + 3);
  EXPECT_EQ(std::strtoul(s.substr(star + 1).c_str(), nullptr, 16), NmeaChecksum(s.substr(1, star - 1)));
  EXPECT_EQ(s.find('\r'), std::string::npos);
}

TEST(Gga, SouthAndWest) {
  GgaFields g = Garden();
  g.lat_deg = -33.25;
  g.lon_deg = -70.125;
  auto f = Fields(BuildGga(g));
  EXPECT_EQ(f[2], "3315.0000000");
  EXPECT_EQ(f[3], "S");
  EXPECT_EQ(f[4], "07007.5000000");
  EXPECT_EQ(f[5], "W");
}

TEST(Gga, RoundingCarriesIntoTheDegrees) {
  GgaFields g = Garden();
  g.lat_deg = 52.0 - 1e-12;
  g.utc_s = 86399.999;
  auto f = Fields(BuildGga(g));
  EXPECT_EQ(f[2], "5200.0000000");
  EXPECT_EQ(f[1], "000000.00");
}

TEST(Gga, MinutesKeepAboutTwoMillimetres) {
  GgaFields g = Garden();
  g.lat_deg = 52.5 + 0.0001 / 60.0;
  EXPECT_EQ(Fields(BuildGga(g))[2], "5230.0001000");
}

TEST(Gga, Quality) {
  GgaFields g = Garden();
  g.quality = GgaQuality::kRtkFloat;
  EXPECT_EQ(Fields(BuildGga(g))[6], "5");
  g.quality = GgaQuality::kSingle;
  EXPECT_EQ(Fields(BuildGga(g))[6], "1");
}

}  // namespace
}  // namespace mowbite_gnss
