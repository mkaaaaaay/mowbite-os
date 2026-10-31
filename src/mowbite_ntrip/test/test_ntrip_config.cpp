#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>

#include "mowbite_ntrip/ntrip_config.h"

namespace mowbite_ntrip {
namespace {

namespace fs = std::filesystem;

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
      {"host: a b\nmountpoint: M\n", "line 1, host: empty, with spaces or not printable ASCII"},
      {"host: h\nmountpoint: \n", "line 2, mountpoint: empty, with spaces or not printable ASCII"},
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
  // Also with an unclosed quote.
  EXPECT_FALSE(ParseNtripConfig("host: h\nmountpoint: M\nhunter2pass: 'x\n", error));
  EXPECT_EQ(error, "line 3: unknown key");
}

TEST(NtripConfig, ErrorsNeverShowTheValue) {
  std::string error;
  EXPECT_FALSE(ParseNtripConfig("host: h\nmountpoint: M\npassword: 'abc\x01xyz'\n", error));
  EXPECT_EQ(error.find("abc"), std::string::npos);
  EXPECT_FALSE(ParseNtripConfig("host: h\nmountpoint: M\npassword: '" + std::string(256, 'q') + "'\n", error));
  EXPECT_EQ(error, "line 3, password: longer than 255 bytes");
}

TEST(NtripConfig, DoubledQuoteInSingleQuotes) {
  std::string error;
  auto config = ParseNtripConfig("host: h\nmountpoint: M\nuser: 'it''s'\npassword: 'end'''  # comment\n", error);
  ASSERT_TRUE(config) << error;
  EXPECT_EQ(config->user, "it's");
  EXPECT_EQ(config->password, "end'");
  // Two quoted parts are text after the first.
  EXPECT_FALSE(ParseNtripConfig("host: h\nmountpoint: M\npassword: 'a' 'b'\n", error));
  EXPECT_EQ(error, "line 3, password: unclosed quote or text after it");
}

TEST(NtripConfig, CheckValue) {
  EXPECT_EQ(CheckValue("host", "sapos-ni-ntrip.de"), "");
  EXPECT_EQ(CheckValue("host", ""), "empty, with spaces or not printable ASCII");
  EXPECT_EQ(CheckValue("mountpoint", "VRS 3"), "empty, with spaces or not printable ASCII");
  EXPECT_EQ(CheckValue("user", ""), "");
  EXPECT_EQ(CheckValue("password", "with space and 'quote'"), "");
  EXPECT_EQ(CheckValue("password", "line\nbreak"), "control characters");
  EXPECT_EQ(CheckValue("password", std::string(255, 'a')), "");
  EXPECT_EQ(CheckValue("host", std::string(256, 'a')), "longer than 255 bytes");
  // Host and mountpoint go into JSON: no byte above 0x7E, also where char has no sign (arm64).
  const std::string not_ascii = "empty, with spaces or not printable ASCII";
  EXPECT_EQ(CheckValue("host", "caster\xe4.example"), not_ascii);
  EXPECT_EQ(CheckValue("host", "caster\xc3\xa4.example"), not_ascii);
  EXPECT_EQ(CheckValue("mountpoint", "VRS\x7f"), not_ascii);
  EXPECT_EQ(CheckValue("password", "p\xe4ss"), "");
}

NtripConfig Tricky() {
  NtripConfig config;
  config.host = "caster.example.org";
  config.port = 2102;
  config.mountpoint = "VRS'3#x";
  config.user = " it's me ";
  config.password = "p'#'' \"x\": \xc3\xa4";
  config.version = 1;
  return config;
}

TEST(NtripConfig, FormatReadsBackTheSame) {
  std::string error;
  auto back = ParseNtripConfig(FormatNtripConfig(Tricky()), error);
  ASSERT_TRUE(back) << error;
  EXPECT_EQ(*back, Tricky());
  NtripConfig open;
  open.host = "h";
  open.mountpoint = "M";
  back = ParseNtripConfig(FormatNtripConfig(open), error);
  ASSERT_TRUE(back) << error;
  EXPECT_EQ(*back, open);
}

// A folder of its own per test, removed afterwards.
class AccessFileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("mowbite_ntrip_" + std::to_string(::getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    path_ = (dir_ / "ntrip.yaml").string();
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }
  static unsigned Mode(const std::string& path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0 ? st.st_mode & 0777 : 0;
  }
  static std::string Text(const std::string& path) {
    std::ifstream in(path);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
  }

  fs::path dir_;
  std::string path_;
};

TEST_F(AccessFileTest, WriteMode600ReadBack) {
  const mode_t umask_before = ::umask(0);
  std::string error;
  const bool written = WriteAccessFile(path_, Tricky(), error);
  ::umask(umask_before);
  ASSERT_TRUE(written) << error;
  EXPECT_EQ(Mode(path_), 0600u);
  EXPECT_FALSE(fs::exists(path_ + ".new"));
  const AccessFile file = ReadAccessFile(path_);
  EXPECT_TRUE(file.exists);
  ASSERT_TRUE(file.config) << file.error;
  EXPECT_EQ(*file.config, Tricky());
  EXPECT_FALSE(file.readable_by_others);
}

TEST_F(AccessFileTest, ReplacesAFileOthersCouldRead) {
  std::ofstream(path_) << "host: old\nmountpoint: OLD\npassword: old\n";
  fs::permissions(path_, fs::perms(0644));
  EXPECT_TRUE(ReadAccessFile(path_).readable_by_others);
  // A leftover of an attempt that stopped half way does not stand in the way.
  std::ofstream(path_ + ".new") << "half";
  std::string error;
  ASSERT_TRUE(WriteAccessFile(path_, Tricky(), error)) << error;
  EXPECT_EQ(Mode(path_), 0600u);
  EXPECT_EQ(Text(path_).find("old"), std::string::npos);
  EXPECT_FALSE(fs::exists(path_ + ".new"));
}

TEST_F(AccessFileTest, NoFileInvalidFileAndRemove) {
  AccessFile file = ReadAccessFile(path_);
  EXPECT_FALSE(file.exists);
  EXPECT_FALSE(file.config);
  EXPECT_EQ(file.error, "");
  std::ofstream(path_) << "host: h\npassword: hunter2\n";
  file = ReadAccessFile(path_);
  EXPECT_TRUE(file.exists);
  EXPECT_FALSE(file.config);
  EXPECT_EQ(file.error, "mountpoint missing");
  std::string error;
  EXPECT_TRUE(RemoveAccessFile(path_, error)) << error;
  EXPECT_FALSE(fs::exists(path_));
  // Gone already: still done.
  EXPECT_TRUE(RemoveAccessFile(path_, error)) << error;
}

TEST_F(AccessFileTest, WriteFailsWithoutTheFolder) {
  std::string error;
  EXPECT_FALSE(WriteAccessFile((dir_ / "missing" / "ntrip.yaml").string(), Tricky(), error));
  EXPECT_NE(error.find("cannot create"), std::string::npos) << error;
  EXPECT_EQ(error.find(Tricky().password), std::string::npos);
}

TEST_F(AccessFileTest, NeverWritesWhatReadsBackOtherwise) {
  NtripConfig config = Tricky();
  config.password = "two\nlines";
  std::string error;
  EXPECT_FALSE(WriteAccessFile(path_, config, error));
  EXPECT_FALSE(fs::exists(path_));
  EXPECT_EQ(error.find("lines"), std::string::npos) << error;
}

}  // namespace
}  // namespace mowbite_ntrip
