#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>

#include "mowbite_ntrip/ntrip_source.h"

namespace mowbite_ntrip {
namespace {

namespace fs = std::filesystem;

const char kUser[] = "user.of.the.test";
const char kPassword[] = "pw.of#the.test";

NtripConfig Access(const std::string& host, int port, const std::string& mountpoint) {
  NtripConfig config;
  config.host = host;
  config.port = port;
  config.mountpoint = mountpoint;
  config.user = kUser;
  config.password = kPassword;
  return config;
}

TEST(CorrectionSource, OfAnAccessWithoutTheLogin) {
  const CorrectionSource source = SourceOf(Access("sapos-ni-ntrip.de", 2101, "/VRS_3_4G_NI"));
  EXPECT_EQ(source, (CorrectionSource{"sapos-ni-ntrip.de", 2101, "VRS_3_4G_NI"}));
}

TEST(CorrectionSource, SameInAnyCaseOfTheHostVersionAndLogin) {
  NtripConfig a = Access("SAPOS-NI-NTRIP.de", 2101, "VRS_3_4G_NI");
  NtripConfig b = Access("sapos-ni-ntrip.DE", 2101, "/VRS_3_4G_NI");
  b.version = 1;
  b.user = "";
  b.password = "";
  EXPECT_TRUE(SameSource(SourceOf(a), SourceOf(b)));
  EXPECT_TRUE(SameSource({"h", 2101, "/M"}, {"H", 2101, "M"}));
}

TEST(CorrectionSource, AnotherHostPortOrMountpointIsAnother) {
  const CorrectionSource base{"sapos-ni-ntrip.de", 2101, "VRS_3_4G_NI"};
  EXPECT_FALSE(SameSource(base, {"caster.example", 2101, "VRS_3_4G_NI"}));
  EXPECT_FALSE(SameSource(base, {"sapos-ni-ntrip.de", 2102, "VRS_3_4G_NI"}));
  EXPECT_FALSE(SameSource(base, {"sapos-ni-ntrip.de", 2101, "VRS_3_4G_NI_2"}));
  // The mountpoint is exact, also in case.
  EXPECT_FALSE(SameSource(base, {"sapos-ni-ntrip.de", 2101, "vrs_3_4g_ni"}));
  // No name lookup.
  EXPECT_FALSE(SameSource({"localhost", 2101, "M"}, {"127.0.0.1", 2101, "M"}));
}

// A folder of its own per test, removed afterwards.
class SourceFileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("mowbite_ntrip_source_" + std::to_string(::getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    path_ = (dir_ / "ntrip_source.json").string();
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }
  void Put(const std::string& text) const {
    std::ofstream(path_) << text;
  }
  std::string Text() const {
    std::ifstream in(path_);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
  }

  fs::path dir_;
  std::string path_;
};

TEST_F(SourceFileTest, HoldsHostPortAndMountpointOnly) {
  const mode_t umask_before = ::umask(0);
  std::string error;
  const bool written = WriteSourceFile(path_, SourceOf(Access("sapos-ni-ntrip.de", 2101, "VRS_3_4G_NI")), error);
  ::umask(umask_before);
  ASSERT_TRUE(written) << error;
  EXPECT_EQ(Text(), "{\"host\":\"sapos-ni-ntrip.de\",\"port\":2101,\"mountpoint\":\"VRS_3_4G_NI\"}\n");
  struct stat st;
  ASSERT_EQ(::stat(path_.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0644u);
  EXPECT_FALSE(fs::exists(path_ + ".new"));
  const SourceFile file = ReadSourceFile(path_);
  EXPECT_TRUE(file.exists);
  ASSERT_TRUE(file.source) << file.error;
  EXPECT_EQ(*file.source, (CorrectionSource{"sapos-ni-ntrip.de", 2101, "VRS_3_4G_NI"}));
  // Replaced as a whole.
  ASSERT_TRUE(WriteSourceFile(path_, {"caster.example", 2102, "M"}, error)) << error;
  EXPECT_EQ(*ReadSourceFile(path_).source, (CorrectionSource{"caster.example", 2102, "M"}));
}

TEST_F(SourceFileTest, MissingIsNoSource) {
  const SourceFile file = ReadSourceFile(path_);
  EXPECT_FALSE(file.exists);
  EXPECT_FALSE(file.source);
  EXPECT_EQ(file.error, "");
}

TEST_F(SourceFileTest, ByHandWithASlashAndOtherKeys) {
  Put("{\"mountpoint\": \"/V2\", \"port\": 2102, \"host\": \"Caster.Example\", \"since\": 1}");
  const SourceFile file = ReadSourceFile(path_);
  ASSERT_TRUE(file.source) << file.error;
  EXPECT_EQ(*file.source, (CorrectionSource{"Caster.Example", 2102, "V2"}));
}

TEST_F(SourceFileTest, InvalidFiles) {
  struct Case {
    const char* text;
    const char* error;
  } cases[] = {
      {"", "not a JSON object"},
      {"[\"h\", 2101, \"M\"]", "not a JSON object"},
      {"{\"host\": \"h\", \"port\": 2101", "not a JSON object"},
      {"{\"port\": 2101, \"mountpoint\": \"M\"}", "host: missing or not a text"},
      {"{\"host\": \"h\", \"port\": 2101, \"mountpoint\": 7}", "mountpoint: missing or not a text"},
      {"{\"host\": \"h b\", \"port\": 2101, \"mountpoint\": \"M\"}", "host: empty, with spaces or not printable ASCII"},
      {"{\"host\": \"h\", \"port\": 2101, \"mountpoint\": \"/\"}",
       "mountpoint: empty, with spaces or not printable ASCII"},
      {"{\"host\": \"h\", \"port\": \"2101\", \"mountpoint\": \"M\"}", "port: not a port number"},
      {"{\"host\": \"h\", \"port\": 65536, \"mountpoint\": \"M\"}", "port: not a port number"},
      {"{\"host\": \"h\", \"mountpoint\": \"M\"}", "port: not a port number"},
  };
  for (const auto& c : cases) {
    Put(c.text);
    const SourceFile file = ReadSourceFile(path_);
    EXPECT_TRUE(file.exists) << c.text;
    EXPECT_FALSE(file.source) << c.text;
    EXPECT_EQ(file.error, c.error) << c.text;
  }
  fs::remove(path_);
  fs::create_directories(path_);
  EXPECT_EQ(ReadSourceFile(path_).error, "not a regular file of at most 4 KB");
}

TEST_F(SourceFileTest, NeverWritesWhatReadsBackOtherwise) {
  std::string error;
  // Not UTF-8: replaced in the text, so it would not read back; nothing throws.
  EXPECT_FALSE(WriteSourceFile(path_, {"caster\xe4.example", 2101, "M"}, error));
  EXPECT_NE(error.find("would not read back the same"), std::string::npos) << error;
  EXPECT_FALSE(WriteSourceFile(path_, {"h", 2101, "/M"}, error));
  EXPECT_FALSE(WriteSourceFile(path_, {"h", 0, "M"}, error));
  EXPECT_FALSE(fs::exists(path_));
  EXPECT_FALSE(WriteSourceFile((dir_ / "missing" / "ntrip_source.json").string(), {"h", 2101, "M"}, error));
  EXPECT_NE(error.find("cannot create"), std::string::npos) << error;
}

}  // namespace
}  // namespace mowbite_ntrip
