#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "mowbite_camera/stream_config.h"
#include "test_util.h"

namespace mowbite_camera {
namespace {

namespace fs = std::filesystem;

std::string ReadAll(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

mode_t ModeOf(const fs::path& path) {
  struct stat info {};
  return ::lstat(path.c_str(), &info) == 0 ? info.st_mode & 07777 : 0;
}

class StreamConfigTest : public ::testing::Test {
 protected:
  StreamConfigTest() {
    ::unsetenv("RUNTIME_DIRECTORY");
  }
  ~StreamConfigTest() override {
    ::unsetenv("RUNTIME_DIRECTORY");
  }

  TempDir temp_;
  const fs::path& base_ = temp_.path();
  std::string error_;
};

TEST(StreamConfigText, IsTheFileTheStreamServerLoads) {
  StreamConfig config;
  config.auth_port = 40123;
  config.sources = {{"stereo", "rtsp://127.0.0.1:8555/stereo"}, {"rear", "rtsp://127.0.0.1:8556/rear"}};
  EXPECT_EQ(StreamConfigText(config),
            "logLevel: warn\n"
            "logDestinations: [stdout]\n"
            "authMethod: http\n"
            "authHTTPAddress: \"http://127.0.0.1:40123/auth\"\n"
            "authHTTPExclude: []\n"
            "api: true\n"
            "apiAddress: \"127.0.0.1:9997\"\n"
            "metrics: false\n"
            "pprof: false\n"
            "playback: false\n"
            "rtsp: false\n"
            "rtmp: false\n"
            "hls: false\n"
            "srt: false\n"
            "moq: false\n"
            "webrtc: true\n"
            "webrtcAddress: \":8889\"\n"
            "webrtcLocalUDPAddress: \":8189\"\n"
            "webrtcLocalTCPAddress: \"\"\n"
            "paths:\n"
            "  \"rear\":\n"
            "    source: \"rtsp://127.0.0.1:8556/rear\"\n"
            "    rtspTransport: tcp\n"
            "    sourceOnDemand: true\n"
            "    maxReaders: 2\n"
            "  \"stereo\":\n"
            "    source: \"rtsp://127.0.0.1:8555/stereo\"\n"
            "    rtspTransport: tcp\n"
            "    sourceOnDemand: true\n"
            "    maxReaders: 2\n");
}

TEST(StreamConfigText, LeavesOutWhatWouldBreakTheFile) {
  StreamConfig config;
  config.sources = {{"a\"b", "rtsp://127.0.0.1:1/a"},
                    {"lines", "rtsp://127.0.0.1:1/lines\nrunOnInit: x"},
                    {"back", "rtsp://127.0.0.1:1/back\\"},
                    {"space", "rtsp://127.0.0.1:1/ space"},
                    {"", "rtsp://127.0.0.1:1/"}};
  const std::string text = StreamConfigText(config);
  EXPECT_NE(text.find("\npaths: {}\n"), std::string::npos) << text;
  EXPECT_EQ(text.find("runOnInit"), std::string::npos);
}

TEST_F(StreamConfigTest, TheRuntimeFolderIsMadeForThisUserAlone) {
  const std::string dir = RuntimeDir(base_.string(), &error_);
  EXPECT_EQ(dir, (base_ / "mowbite_camera").string()) << error_;
  EXPECT_EQ(ModeOf(dir), 0700u);
  // There already, and opened up by someone: taken and closed again.
  ASSERT_EQ(::chmod(dir.c_str(), 0755), 0);
  EXPECT_EQ(RuntimeDir(base_.string(), &error_), dir);
  EXPECT_EQ(ModeOf(dir), 0700u);
}

TEST_F(StreamConfigTest, TheRuntimeFolderComesFromTheEnvironmentOrTmp) {
  const fs::path second = base_ / "second";
  fs::create_directory(second);
  ::setenv("RUNTIME_DIRECTORY", (base_.string() + ":" + second.string()).c_str(), 1);
  EXPECT_EQ(RuntimeDir("", &error_), (base_ / "mowbite_camera").string()) << error_;
  EXPECT_EQ(RuntimeDir(second.string(), &error_), (second / "mowbite_camera").string()) << error_;
  ::setenv("RUNTIME_DIRECTORY", "", 1);
  // Without both it is the one in /tmp, unless another user has that already.
  const std::string dir = RuntimeDir("", &error_);
  EXPECT_TRUE(dir == "/tmp/mowbite_camera" || dir.empty()) << dir;
}

TEST_F(StreamConfigTest, WhatIsNoFolderOfItsOwnIsRefused) {
  // A file and a link to a folder where the folder would be, and a base that is not there.
  const fs::path file_base = base_ / "file";
  fs::create_directory(file_base);
  std::ofstream(file_base / "mowbite_camera") << "x";
  EXPECT_EQ(RuntimeDir(file_base.string(), &error_), "");
  EXPECT_NE(error_, "");
  const fs::path link_base = base_ / "link";
  fs::create_directory(link_base);
  fs::create_directory(base_ / "elsewhere");
  fs::create_directory_symlink(base_ / "elsewhere", link_base / "mowbite_camera");
  EXPECT_EQ(RuntimeDir(link_base.string(), &error_), "");
  EXPECT_EQ(RuntimeDir((base_ / "missing").string(), &error_), "");
}

TEST_F(StreamConfigTest, AFileIsReplacedAsAWhole) {
  const std::string dir = RuntimeDir(base_.string(), &error_);
  ASSERT_NE(dir, "");
  const fs::path path = fs::path(dir) / kStreamConfigFile;
  ASSERT_TRUE(ReplaceFile(dir, kStreamConfigFile, "first\n", &error_)) << error_;
  EXPECT_EQ(ReadAll(path), "first\n");
  EXPECT_EQ(ModeOf(path), 0600u);
  // What an earlier run left beside it does not get in the way, whatever its mode.
  std::ofstream(path.string() + ".new") << "left over";
  ASSERT_EQ(::chmod((path.string() + ".new").c_str(), 0644), 0);
  ASSERT_TRUE(ReplaceFile(dir, kStreamConfigFile, "second\n", &error_)) << error_;
  EXPECT_EQ(ReadAll(path), "second\n");
  EXPECT_EQ(ModeOf(path), 0600u);
  EXPECT_FALSE(fs::exists(path.string() + ".new"));
  EXPECT_FALSE(ReplaceFile(dir + "/missing", kStreamConfigFile, "x", &error_));
  EXPECT_NE(error_, "");
}

}  // namespace
}  // namespace mowbite_camera
