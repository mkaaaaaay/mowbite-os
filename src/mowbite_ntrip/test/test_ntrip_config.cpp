#include <gtest/gtest.h>

#include "mowbite_ntrip/ntrip_config.h"

namespace mowbite_ntrip {
namespace {

TEST(NtripConfig, FullFile) {
  std::string error;
  auto config = ParseNtripConfig(
      "# access to the caster\n"
      "host: caster.example.org\n"
      "port: 2102\n"
      "mountpoint: /VRS_TEST   # with a slash\n"
      "user: 'rover one'\n"
      "password: \"p#ss: 'x'\"\n"
      "version: 1\n",
      error);
  ASSERT_TRUE(config) << error;
  EXPECT_EQ(config->host, "caster.example.org");
  EXPECT_EQ(config->port, 2102);
  EXPECT_EQ(config->mountpoint, "VRS_TEST");
  EXPECT_EQ(config->user, "rover one");
  EXPECT_EQ(config->password, "p#ss: 'x'");
  EXPECT_EQ(config->version, 1);
}

TEST(NtripConfig, Defaults) {
  std::string error;
  auto config = ParseNtripConfig("host: h\nmountpoint: M\n", error);
  ASSERT_TRUE(config) << error;
  EXPECT_EQ(config->port, 2101);
  EXPECT_EQ(config->version, 2);
  EXPECT_TRUE(config->user.empty());
  EXPECT_TRUE(config->password.empty());
}

TEST(NtripConfig, EmptyQuotedValues) {
  std::string error;
  auto config = ParseNtripConfig("host: h\r\nmountpoint: M\r\nuser: \"\"\r\npassword: ''\r\n", error);
  ASSERT_TRUE(config) << error;
  EXPECT_TRUE(config->user.empty());
}

TEST(NtripConfig, Errors) {
  struct Case {
    const char* text;
    const char* error;
  } cases[] = {
      {"mountpoint: M\n", "host missing"},
      {"host: h\n", "mountpoint missing"},
      {"host: h\nmountpoint: M\nport: 0\n", "line 3, port: not a port number"},
      {"host: h\nmountpoint: M\nport: 70000\n", "line 3, port: not a port number"},
      {"host: h\nmountpoint: M\nport: 21x\n", "line 3, port: not a port number"},
      {"host: h\nmountpoint: M\nversion: 3\n", "line 3, version: 1 or 2"},
      {"host: a b\nmountpoint: M\n", "line 1, host: empty or with spaces"},
      {"host: h\nmountpoint: \n", "line 2, mountpoint: empty or with spaces"},
      {"host: h\nmountpoint: M\npassword: \"open\n", "line 3, password: unclosed quote or text after it"},
      {"host: h\nmountpoint: M\nsecret123\n", "line 3: expected key: value"},
  };
  for (const auto& c : cases) {
    std::string error;
    EXPECT_FALSE(ParseNtripConfig(c.text, error)) << c.text;
    EXPECT_EQ(error, c.error) << c.text;
  }
}

TEST(NtripConfig, UnknownKeyIsNotNamed) {
  std::string error;
  EXPECT_FALSE(ParseNtripConfig("host: h\nmountpoint: M\nhunter2pass: x\n", error));
  EXPECT_EQ(error, "line 3: unknown key");
}

TEST(NtripConfig, ErrorsNeverShowTheValue) {
  std::string error;
  EXPECT_FALSE(ParseNtripConfig("host: h\nmountpoint: M\npassword: 'abc\x01xyz'\n", error));
  EXPECT_EQ(error.find("abc"), std::string::npos);
}

}  // namespace
}  // namespace mowbite_ntrip
