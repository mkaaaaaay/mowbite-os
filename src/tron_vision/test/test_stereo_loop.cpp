#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "fake_encoder.h"
#include "rtsp_test_client.h"
#include "test_util.h"
#include "tron_vision/stereo_loop.h"
#include "tron_vision/text_file.h"

namespace tron_vision {
namespace {

using test::Client;

constexpr char kBoot[] = "6b0a57c2-0000-4000-8000-00000000b007";

// Camera at 23.8 fps on the clock of the test; every call is noted.
class FakeSource : public FrameSource {
 public:
  explicit FakeSource(double& clock) : clock_(clock) {}

  bool Open(std::string& error) override {
    calls += "open ";
    if (open_failures > 0) {
      open_failures--;
      error = "/dev/video22 is open in process 4711";
      return false;
    }
    open = true;
    return true;
  }
  bool Start(std::string& error) override {
    calls += "start ";
    starts++;
    clock_ += start_s;
    if (!watched.empty()) during += std::filesystem::exists(watched) ? "S" : "s";
    if (fail_start) {
      error = "stream on: Input/output error";
      return false;
    }
    streaming = true;
    return true;
  }
  void Stop() override {
    if (streaming) calls += "stop ";
    if (streaming && !watched.empty()) during += std::filesystem::exists(watched) ? "T" : "t";
    streaming = false;
  }
  void Close() override {
    Stop();
    if (open) calls += "close ";
    open = false;
  }
  int Next(int timeout_ms, CapturedFrame& frame, std::string& error) override {
    if (!streaming || fail_next) {
      error = "dequeue buffer: Input/output error";
      return -1;
    }
    if (frames_left == 0) {
      clock_ += timeout_ms / 1000.0;
      return 0;
    }
    if (frames_left > 0) frames_left--;
    clock_ += 0.042;
    sequence += 1 + skip;
    skip = 0;
    frame.index = sequence % 4;
    frame.bytes = 2560 * 480;
    frame.stride = 2560;
    frame.sequence = sequence;
    frame.timestamp_us = static_cast<int64_t>(clock_ * 1e6);
    frame.dmabuf = 200 + static_cast<int>(frame.index);
    frame.data = pixels.empty() ? nullptr : pixels.data();
    return 1;
  }

  std::string calls;
  int starts = 0;
  bool open = false;
  bool streaming = false;
  int open_failures = 0;
  bool fail_start = false;
  bool fail_next = false;
  // -1: frames without end.
  int frames_left = -1;
  // Frames the driver drops before the next one.
  uint32_t skip = 0;
  uint32_t sequence = 0;
  // What the mapped buffers hold; empty: frames without a mapping.
  std::vector<uint8_t> pixels;
  // What STREAMON costs on the clock of the test.
  double start_s = 0.0;
  // Whether this file existed in Start (S, else s) and in Stop of a stream (T, else t).
  std::string watched;
  std::string during;

 private:
  double& clock_;
};

using test::FakeEncoder;

struct Rig {
  Rig() : camera(clock), encoder(clock), server(AnyPort(), [this] { return clock; }) {
    std::filesystem::create_directories(dir.Path() + "/markers");
    dir.Write("boot_id", std::string(kBoot) + "\n");
    config.setup_file = dir.Path() + "/stereo";
    config.marker_dir = dir.Path() + "/markers";
    config.boot_id_file = dir.Path() + "/boot_id";
    config.statm = dir.Write("statm", "30000 5000 900 1 0 20000 0\n");
    config.temp_file = dir.Write("temp", "45000\n");
    config.idle_wait_ms = 1;
    std::string error;
    listening = server.Listen(error);
  }
  static RtspConfig AnyPort() {
    RtspConfig c;
    c.port = 0;
    return c;
  }
  void Make() {
    loop.reset();
    guard = std::make_unique<CrashGuard>(config.marker_dir, dir.Path() + "/crash", config.boot_id_file);
    loop = std::make_unique<StereoLoop>(config, camera, encoder, server, *guard, [this] { return clock; }, sink);
  }
  void Configured() {
    dir.Write("stereo", "configured\n");
  }
  // The mower stands with the blade off, said again at each step like hw/status at 10 Hz.
  void Steps(int n, bool blade_on = false, bool moving = false) {
    for (int i = 0; i < n; i++) {
      loop->SetMowerStatus({true, clock, blade_on, moving});
      loop->Step();
    }
  }
  StereoState State() const {
    return loop->Status().state;
  }
  // Configured, standing, self-test passed.
  void Ready() {
    Configured();
    Make();
    Steps(40);
    ASSERT_EQ(State(), StereoState::kReady) << loop->Status().detail;
  }
  std::string Url(const std::string& rest = "") const {
    return "rtsp://127.0.0.1:" + std::to_string(server.Port()) + "/stereo" + rest;
  }
  std::string Ask(const Client& client, const std::string& request, bool blade_on = false) {
    client.Send(request);
    Steps(3, blade_on);
    return client.Receive();
  }
  // DESCRIBE, SETUP and PLAY; true once the stream has started.
  bool Play(const Client& client) {
    Steps(2);
    const std::string describe = Ask(client, "DESCRIBE " + Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n");
    if (describe.find("200 OK") == std::string::npos) return false;
    const std::string setup =
        Ask(client, "SETUP " + Url("/trackID=0") +
                        " RTSP/1.0\r\nCSeq: 2\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n\r\n");
    const std::string session = test::SessionOf(setup);
    if (session.empty()) return false;
    client.Send("PLAY " + Url("/") + " RTSP/1.0\r\nCSeq: 3\r\nSession: " + session + "\r\n\r\n");
    Steps(1);
    return State() == StereoState::kStreaming;
  }
  std::string Marker(const std::string& name) const {
    return FirstLine(config.marker_dir + "/" + name);
  }
  bool Describes() {
    Client probe(server.Port());
    Steps(2, last_blade);
    const std::string answer = Ask(probe, "DESCRIBE " + Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n", last_blade);
    return answer.find("200 OK") != std::string::npos;
  }

  test::TempDir dir;
  double clock = 1000.0;
  StereoConfig config;
  FakeSource camera;
  FakeEncoder encoder;
  RtspServer server;
  std::unique_ptr<CrashGuard> guard;
  std::unique_ptr<StereoLoop> loop;
  FrameSink* sink = nullptr;
  bool listening = false;
  // Blade state that Describes() keeps reporting.
  bool last_blade = false;
};

// Everything a client got: the RTP packets, the rest as text.
struct Received {
  explicit Received(const Client& client) {
    std::string data;
    for (std::string chunk = client.Receive(); !chunk.empty() && chunk != "<closed>"; chunk = client.Receive()) {
      data += chunk;
    }
    size_t at = 0;
    if (!data.empty() && data[0] != '$') {
      // An answer that was still on its way.
      at = std::min(data.find("\r\n\r\n"), data.size() - 4) + 4;
      rest = data.substr(0, at);
    }
    while (at + 4 <= data.size() && data[at] == '$') {
      const size_t length = static_cast<uint8_t>(data[at + 2]) << 8 | static_cast<uint8_t>(data[at + 3]);
      if (at + 4 + length > data.size()) break;
      packets.emplace_back(data.begin() + at + 4, data.begin() + at + 4 + length);
      at += 4 + length;
    }
    rest += data.substr(at);
  }
  std::vector<std::vector<uint8_t>> packets;
  std::string rest;
};

TEST(StereoLoop, WaitsForTheSetupResult) {
  Rig rig;
  rig.Make();
  rig.Steps(5);
  EXPECT_EQ(rig.State(), StereoState::kWaitingSetup);
  rig.clock += 119.0;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kWaitingSetup);
  rig.clock += 2.0;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kNotConfigured);
  EXPECT_EQ(rig.loop->Status().detail, "no result of the camera setup");
  EXPECT_FALSE(rig.Describes());

  rig.dir.Write("stereo", "failed asicrw\n");
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kNotConfigured);
  EXPECT_EQ(rig.loop->Status().detail, "failed asicrw");
  // Anything but the one word keeps the camera closed.
  rig.dir.Write("stereo", "configured?\n");
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kNotConfigured);
  EXPECT_EQ(rig.camera.calls, "");

  rig.Configured();
  rig.Steps(40);
  EXPECT_EQ(rig.State(), StereoState::kReady);
}

TEST(StereoLoop, AFailedSetupIsKnownAtOnce) {
  Rig rig;
  rig.dir.Write("stereo", "failed kernel-setup\n");
  rig.Make();
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kNotConfigured);
  EXPECT_EQ(rig.camera.calls, "");
}

TEST(StereoLoop, LockedAfterAKernelCrashDuringTheStream) {
  Rig rig;
  rig.Configured();
  rig.dir.Write("markers/stream-stereo", "another-boot\n");
  rig.dir.Write("crash", "oops in rkcif\n");
  rig.Make();
  rig.Steps(50);
  EXPECT_EQ(rig.State(), StereoState::kLocked);
  EXPECT_NE(rig.loop->Status().detail.find("oops in rkcif"), std::string::npos);
  EXPECT_EQ(rig.camera.calls, "");
  EXPECT_FALSE(rig.Describes());
}

TEST(StereoLoop, UnlockStartsAgainFromTheFirstCheck) {
  Rig rig;
  rig.Configured();
  rig.dir.Write("markers/stream-stereo", "another-boot\n");
  rig.dir.Write("crash", "oops in rkcif\n");
  rig.Make();
  rig.Steps(50);
  ASSERT_EQ(rig.State(), StereoState::kLocked);
  std::string text;
  ASSERT_TRUE(rig.loop->Unlock(text)) << text;
  EXPECT_EQ(text, "oops in rkcif");
  rig.Steps(50);
  EXPECT_NE(rig.State(), StereoState::kLocked);
  EXPECT_NE(rig.camera.calls, "");
}

TEST(StereoLoop, WithoutTheEncoderNothingStreams) {
  Rig rig;
  rig.Configured();
  rig.encoder.present = false;
  rig.Make();
  rig.Steps(50);
  EXPECT_EQ(rig.State(), StereoState::kEncoderMissing);
  EXPECT_EQ(rig.camera.calls, "");
}

TEST(StereoLoop, SelftestOnlyWhileTheMowerStands) {
  Rig rig;
  rig.Configured();
  rig.Make();
  for (int i = 0; i < 5; i++) rig.loop->Step();
  EXPECT_EQ(rig.State(), StereoState::kPausedDriver);
  EXPECT_EQ(rig.loop->Status().detail, "no hw/status yet");
  rig.Steps(5, true, false);
  EXPECT_EQ(rig.State(), StereoState::kPausedBlade);
  EXPECT_EQ(rig.loop->Status().detail, "blade running");
  rig.Steps(5, false, true);
  EXPECT_EQ(rig.State(), StereoState::kWaitingIdle);
  EXPECT_EQ(rig.loop->Status().detail, "mower moving");
  // A state that is too old counts as unknown.
  rig.loop->SetMowerStatus({true, rig.clock, false, false});
  rig.clock += 1.5;
  rig.loop->Step();
  EXPECT_EQ(rig.State(), StereoState::kPausedDriver);
  EXPECT_EQ(rig.loop->Status().detail, "hw/status older than 1.0 s");
  EXPECT_EQ(rig.camera.calls, "");

  rig.Steps(1);
  EXPECT_EQ(rig.State(), StereoState::kSelftest);
  EXPECT_EQ(rig.Marker("stream-stereo"), kBoot);
  rig.Steps(20);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.calls, "open start stop close ");
  EXPECT_EQ(rig.camera.sequence, 10U);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
  EXPECT_EQ(rig.Marker("verified-stereo"), std::string(kBoot) + " 23.81");
  EXPECT_NEAR(rig.loop->Status().camera_fps, 23.81, 0.01);
  EXPECT_EQ(rig.encoder.settings.fps, 24);
  EXPECT_FALSE(rig.encoder.is_open);
  EXPECT_TRUE(rig.Describes());
}

TEST(StereoLoop, SelftestOncePerBoot) {
  Rig rig;
  rig.Ready();
  rig.camera.calls.clear();
  // The process starts again in the same boot.
  rig.Make();
  rig.Steps(10);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.calls, "");
  EXPECT_NEAR(rig.loop->Status().camera_fps, 23.81, 0.01);
  // After a reboot the test runs again.
  rig.dir.Write("boot_id", "11111111-2222-4333-8444-555555555555\n");
  rig.Make();
  rig.Steps(40);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.calls, "open start stop close ");
}

TEST(StereoLoop, SelftestWithoutEnoughFramesEndsStreamingForThisBoot) {
  for (int frames : {0, 9}) {
    Rig rig;
    rig.Configured();
    rig.camera.frames_left = frames;
    rig.Make();
    rig.Steps(60);
    EXPECT_EQ(rig.State(), StereoState::kNoFrames);
    EXPECT_EQ(rig.loop->Status().detail, "self-test: " + std::to_string(frames) + " frames in 2.0 s");
    EXPECT_EQ(rig.camera.calls, "open start stop close ");
    EXPECT_EQ(rig.Marker("failed-stereo"), std::string(kBoot) + " " + rig.loop->Status().detail);
    EXPECT_EQ(rig.Marker("stream-stereo"), "");
    EXPECT_FALSE(rig.Describes());

    // Neither more steps nor a new process stream again.
    rig.camera.frames_left = -1;
    rig.Steps(100);
    rig.Make();
    rig.Steps(100);
    EXPECT_EQ(rig.State(), StereoState::kFailed);
    EXPECT_NE(rig.loop->Status().detail.find("no picture earlier in this boot"), std::string::npos);
    EXPECT_EQ(rig.camera.starts, 1);
  }
}

// The deadline of the self-test counts from the running stream and only after asking for the frames the driver holds.
TEST(StereoLoop, ASlowStreamOnOrALongStepLeavesTheSelftestItsFrames) {
  {
    Rig rig;
    rig.camera.start_s = 2.5;
    rig.Ready();
  }
  Rig rig;
  rig.Configured();
  rig.Make();
  while (rig.State() != StereoState::kSelftest) rig.Steps(1);
  rig.Steps(3);
  rig.clock += 2.5;
  rig.Steps(40);
  EXPECT_EQ(rig.State(), StereoState::kReady) << rig.loop->Status().detail;
}

// STREAMON, STREAMOFF and close run under the marker "calling"; one left by a process that ended in such a call ends
// the camera for this boot without a try.
TEST(StereoLoop, ACallIntoTheDriverThatDidNotReturnEndsTheCameraForThisBoot) {
  {
    Rig rig;
    rig.camera.watched = rig.config.marker_dir + "/calling-stereo";
    rig.Ready();
    EXPECT_EQ(rig.camera.during, "ST");
    EXPECT_EQ(rig.Marker("calling-stereo"), "");
  }
  Rig rig;
  rig.Configured();
  rig.dir.Write("markers/calling-stereo", std::string(kBoot) + " stream on\n");
  rig.Make();
  rig.Steps(20);
  EXPECT_EQ(rig.State(), StereoState::kFailed);
  EXPECT_EQ(rig.loop->Status().detail, "a call into the driver did not return earlier in this boot: stream on");
  EXPECT_EQ(rig.camera.calls, "");
  rig.loop.reset();
  EXPECT_EQ(rig.Marker("calling-stereo"), std::string(kBoot) + " stream on");
  // After a reboot the camera is tried again.
  rig.dir.Write("boot_id", "11111111-2222-4333-8444-555555555555\n");
  rig.Make();
  rig.Steps(40);
  EXPECT_EQ(rig.State(), StereoState::kReady);
}

TEST(StereoLoop, StreamOnThatFailsCountsAsNoPicture) {
  Rig rig;
  rig.Configured();
  rig.camera.fail_start = true;
  rig.Make();
  rig.Steps(20);
  EXPECT_EQ(rig.State(), StereoState::kNoFrames);
  EXPECT_NE(rig.Marker("failed-stereo").find("stream on"), std::string::npos);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
  rig.camera.fail_start = false;
  rig.Steps(50);
  EXPECT_EQ(rig.camera.starts, 1);
}

TEST(StereoLoop, BusyNodeIsTriedAgainAndThenGivenUp) {
  Rig rig;
  rig.Configured();
  rig.camera.open_failures = 2;
  rig.Make();
  rig.Steps(5);
  EXPECT_EQ(rig.State(), StereoState::kWaitingIdle);
  EXPECT_EQ(rig.loop->Status().detail, "/dev/video22 is open in process 4711");
  EXPECT_EQ(rig.camera.calls, "open ");
  rig.clock += 5.1;
  rig.Steps(3);
  rig.clock += 5.1;
  rig.Steps(40);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.Marker("failed-stereo"), "");

  Rig stuck;
  stuck.Configured();
  stuck.camera.open_failures = 99;
  stuck.Make();
  for (int i = 0; i < 4; i++) {
    stuck.Steps(3);
    stuck.clock += 5.1;
  }
  EXPECT_EQ(stuck.State(), StereoState::kFailed);
  EXPECT_EQ(stuck.camera.calls, "open open open ");
  // Nothing streamed, so a new process may try again.
  EXPECT_EQ(stuck.Marker("failed-stereo"), "");
}

TEST(StereoLoop, BladeCutsTheSelftestOff) {
  Rig rig;
  rig.Configured();
  rig.Make();
  rig.Steps(4);
  ASSERT_EQ(rig.State(), StereoState::kSelftest);
  rig.Steps(3);
  rig.Steps(1, true);
  EXPECT_EQ(rig.State(), StereoState::kPausedBlade);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
  EXPECT_EQ(rig.Marker("verified-stereo"), "");
  // Frames had come, so the test may run again once the mower stands.
  rig.Steps(5, true);
  EXPECT_EQ(rig.camera.starts, 1);
  rig.Steps(30);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.starts, 2);

  // Without a frame so far nothing is known about the camera, and it stays off for this boot.
  Rig blind;
  blind.Configured();
  blind.camera.frames_left = 0;
  blind.Make();
  blind.Steps(4);
  ASSERT_EQ(blind.State(), StereoState::kSelftest);
  blind.Steps(1, true);
  EXPECT_EQ(blind.State(), StereoState::kNoFrames);
  EXPECT_FALSE(blind.camera.streaming);
  EXPECT_NE(blind.Marker("failed-stereo").find("cut off before its first frame: blade running"), std::string::npos);
}

TEST(StereoLoop, StreamsToTheViewerAndStopsWhenItLeaves) {
  Rig rig;
  rig.Ready();
  rig.camera.calls.clear();
  const int opens = rig.encoder.opens;
  {
    Client client(rig.server.Port());
    ASSERT_TRUE(rig.Play(client));
    EXPECT_EQ(rig.camera.calls, "open start ");
    EXPECT_EQ(rig.Marker("stream-stereo"), kBoot);
    EXPECT_EQ(rig.encoder.opens, opens + 1);
    rig.Steps(30);
    EXPECT_EQ(rig.State(), StereoState::kStreaming);
    EXPECT_EQ(rig.loop->Status().frames, 30U);
    // On the clock of the test a frame takes 42 ms and its encoding 2 ms.
    EXPECT_NEAR(rig.loop->Status().fps, 22.7, 0.2);
    EXPECT_GT(rig.loop->Status().kbps, 30.0);
    EXPECT_NEAR(rig.loop->Status().encode_ms, 2.0, 0.01);

    const Received got(client);
    // The answer to PLAY, then the first access unit: SPS, PPS and the IDR picture in two fragments.
    EXPECT_NE(got.rest.find("RTSP/1.0 200 OK"), std::string::npos);
    ASSERT_GE(got.packets.size(), 4U + 29U);
    EXPECT_EQ(got.packets[0][12] & 0x1F, 7);
    EXPECT_EQ(got.packets[1][12] & 0x1F, 8);
    EXPECT_EQ(got.packets[2][12] & 0x1F, 28);
    EXPECT_EQ(got.packets[2][13], 0x80 | 5);
    EXPECT_EQ(got.packets[3][13], 0x40 | 5);
    EXPECT_EQ(got.packets[3][1], 0x80 | 96);
    EXPECT_EQ(got.packets[4][12], 0x41);
    uint32_t last = 0;
    for (size_t i = 3; i < got.packets.size(); i++) {
      const std::vector<uint8_t>& p = got.packets[i];
      const uint32_t ts = static_cast<uint32_t>(p[4]) << 24 | static_cast<uint32_t>(p[5]) << 16 |
                          static_cast<uint32_t>(p[6]) << 8 | p[7];
      if (i > 3) {
        EXPECT_NEAR(ts - last, 3960.0, 1.0) << i;
      }
      last = ts;
    }
  }
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.calls, "open start stop close ");
  EXPECT_FALSE(rig.encoder.is_open);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
  EXPECT_EQ(rig.loop->Status().fps, 0.0);

  // The next viewer gets a new stream that opens with an IDR picture again.
  Client next(rig.server.Port());
  ASSERT_TRUE(rig.Play(next));
  rig.Steps(2);
  const Received again(next);
  ASSERT_GE(again.packets.size(), 4U);
  EXPECT_EQ(again.packets[0][12] & 0x1F, 7);
  EXPECT_EQ(rig.camera.starts, 3);
}

TEST(StereoLoop, BladeEndsTheStreamAtOnce) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(5);
  rig.Steps(1, true);
  EXPECT_EQ(rig.State(), StereoState::kPausedBlade);
  EXPECT_EQ(rig.loop->Status().detail, "blade running");
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_FALSE(rig.camera.open);
  EXPECT_FALSE(rig.encoder.is_open);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
  const Received rest(client);
  EXPECT_EQ(client.Receive(), "<closed>");

  // While the blade runs there is nothing to play.
  rig.last_blade = true;
  EXPECT_FALSE(rig.Describes());
  rig.Steps(20, true);
  EXPECT_EQ(rig.camera.starts, 2);

  // Blade off: available again without another self-test.
  rig.last_blade = false;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_TRUE(rig.Describes());
  EXPECT_EQ(rig.camera.starts, 2);
  EXPECT_EQ(rig.Marker("failed-stereo"), "");
}

TEST(StereoLoop, UnknownMowerStateEndsTheStream) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(5);
  // hw/status stops coming while frames go on.
  for (int i = 0; i < 30 && rig.State() == StereoState::kStreaming; i++) rig.loop->Step();
  EXPECT_EQ(rig.State(), StereoState::kPausedDriver);
  EXPECT_EQ(rig.loop->Status().detail, "hw/status older than 1.0 s");
  EXPECT_FALSE(rig.camera.streaming);
  // 5 steps with a fresh state, then at most 1 s of frames.
  EXPECT_LE(rig.loop->Status().frames, 5U + 25U);
}

TEST(StereoLoop, MowerGateInItsOrder) {
  const double now = 50.0;
  EXPECT_EQ(CheckMower({}, now, 1.0, false), MowerGate::kUnknown);
  EXPECT_EQ(CheckMower({false, now, false, false}, now, 1.0, false), MowerGate::kUnknown);
  EXPECT_EQ(CheckMower({true, now - 1.0, false, false}, now, 1.0, false), MowerGate::kOpen);
  EXPECT_EQ(CheckMower({true, now - 1.001, false, false}, now, 1.0, false), MowerGate::kUnknown);
  // An old message counts as unknown before anything it says.
  EXPECT_EQ(CheckMower({true, now - 2.0, true, true}, now, 1.0, true), MowerGate::kUnknown);
  EXPECT_EQ(CheckMower({true, now, true, false}, now, 1.0, false), MowerGate::kBlade);
  EXPECT_EQ(CheckMower({true, now, true, true}, now, 1.0, true), MowerGate::kBlade);
  EXPECT_EQ(CheckMower({true, now, false, true}, now, 1.0, true), MowerGate::kMoving);
  EXPECT_EQ(CheckMower({true, now, false, true}, now, 1.0, false), MowerGate::kOpen);
  EXPECT_EQ(CheckMower({true, now, false, false}, now, 1.0, true), MowerGate::kOpen);
}

TEST(StereoLoop, SameCopyOfTheMowerAtEachStepGetsOld) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  // The node hands over its last copy before each step; the time it came with makes it old.
  const MowerView last{true, rig.clock, false, false};
  const double until = rig.clock + 1.5;
  while (rig.clock < until && rig.State() == StereoState::kStreaming) {
    rig.loop->SetMowerStatus(last);
    rig.loop->Step();
  }
  EXPECT_EQ(rig.State(), StereoState::kPausedDriver);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_LE(rig.clock - last.received_at, 1.0 + 0.2);
}

TEST(StereoLoop, UnknownMowerStateCutsTheSelftestOff) {
  Rig rig;
  // A test that takes longer than the age limit.
  rig.config.selftest_frames = 40;
  rig.Configured();
  rig.Make();
  rig.Steps(4);
  ASSERT_EQ(rig.State(), StereoState::kSelftest);
  rig.Steps(3);
  ASSERT_GT(rig.camera.sequence, 0U);
  // hw/status stops coming: the test ends within the age limit, and runs again once the state is known.
  const double last = rig.clock;
  while (rig.State() == StereoState::kSelftest) rig.loop->Step();
  EXPECT_EQ(rig.State(), StereoState::kPausedDriver);
  EXPECT_LE(rig.clock - last, 1.0 + 0.2);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.Marker("verified-stereo"), "");
  EXPECT_EQ(rig.Marker("failed-stereo"), "");
  rig.Steps(60);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.starts, 2);

  // Without a frame so far it stays off for this boot.
  Rig blind;
  blind.Configured();
  blind.camera.frames_left = 0;
  blind.Make();
  blind.Steps(4);
  ASSERT_EQ(blind.State(), StereoState::kSelftest);
  while (blind.State() == StereoState::kSelftest) blind.loop->Step();
  EXPECT_EQ(blind.State(), StereoState::kNoFrames);
  EXPECT_FALSE(blind.camera.streaming);
  EXPECT_NE(blind.Marker("failed-stereo").find("cut off before its first frame: hw/status older"), std::string::npos);
}

TEST(StereoLoop, DrivingWithoutTheBladeKeepsTheStream) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(10, false, true);
  EXPECT_EQ(rig.State(), StereoState::kStreaming);
}

TEST(StereoLoop, StreamWhileMowingOnlyWhenSetSo) {
  Rig rig;
  rig.config.stream_while_mowing = true;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(10, true, true);
  EXPECT_EQ(rig.State(), StereoState::kStreaming);
  // The self-test still waits for a standing mower.
  Rig fresh;
  fresh.config.stream_while_mowing = true;
  fresh.Configured();
  fresh.Make();
  fresh.Steps(10, true, false);
  EXPECT_EQ(fresh.State(), StereoState::kPausedBlade);
  EXPECT_EQ(fresh.camera.calls, "");
}

TEST(StereoLoop, GapEndsStreamingForThisBoot) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(5);
  rig.camera.frames_left = 0;
  rig.Steps(9);
  EXPECT_EQ(rig.State(), StereoState::kStreaming);
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kStalled);
  EXPECT_EQ(rig.loop->Status().detail, "no frame within 1.0 s");
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.Marker("failed-stereo"), std::string(kBoot) + " no frame within 1.0 s");
  EXPECT_EQ(rig.Marker("verified-stereo"), "");
  const Received rest(client);
  EXPECT_EQ(client.Receive(), "<closed>");

  rig.camera.frames_left = -1;
  EXPECT_FALSE(rig.Describes());
  rig.Steps(50);
  EXPECT_EQ(rig.camera.starts, 2);
  rig.Make();
  rig.Steps(50);
  EXPECT_EQ(rig.State(), StereoState::kFailed);
  EXPECT_EQ(rig.camera.starts, 2);
}

// A step that took longer than the gap (encoder, analysis) is no stall while the driver holds frames.
TEST(StereoLoop, ALongStepIsNoStallWhileFramesWait) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(5);
  rig.clock += 1.5;
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kStreaming);
  EXPECT_EQ(rig.Marker("failed-stereo"), "");
}

TEST(StereoLoop, StreamWithoutAFirstFrameEndsStreamingForThisBoot) {
  Rig rig;
  rig.Ready();
  rig.camera.frames_left = 0;
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(19);
  EXPECT_EQ(rig.State(), StereoState::kStreaming);
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kNoFrames);
  EXPECT_EQ(rig.Marker("failed-stereo"), std::string(kBoot) + " no frame within 2.0 s");
}

TEST(StereoLoop, DequeueErrorEndsStreamingForThisBoot) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.camera.fail_next = true;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kStalled);
  EXPECT_NE(rig.Marker("failed-stereo").find("dequeue buffer"), std::string::npos);
}

TEST(StereoLoop, HeatPausesAndCoolingResumes) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.dir.Write("temp", "80000\n");
  rig.clock += 5.1;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kPausedHot);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_FALSE(rig.Describes());
  rig.dir.Write("temp", "76000\n");
  rig.clock += 5.1;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kPausedHot);
  rig.dir.Write("temp", "74900\n");
  rig.clock += 5.1;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_TRUE(rig.Describes());
}

TEST(StereoLoop, EncoderFailuresEndInEncoderError) {
  Rig rig;
  rig.Ready();
  rig.encoder.fail_encode = true;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Client client(rig.server.Port());
    ASSERT_TRUE(rig.Play(client)) << attempt;
    rig.Steps(2);
    EXPECT_FALSE(rig.camera.streaming);
    EXPECT_EQ(client.Receive().find("$"), std::string::npos);
    if (attempt < 3) {
      EXPECT_EQ(rig.State(), StereoState::kReady);
      EXPECT_EQ(rig.loop->Status().detail, "encoder: put frame -1");
      // A viewer that comes back before the pause is over is turned away.
      Client early(rig.server.Port());
      EXPECT_FALSE(rig.Play(early));
      EXPECT_EQ(rig.camera.starts, 1 + attempt);
      rig.clock += 5.1;
    }
  }
  EXPECT_EQ(rig.State(), StereoState::kEncoderError);
  EXPECT_FALSE(rig.Describes());
  // The camera delivered, so nothing is noted against it.
  EXPECT_EQ(rig.Marker("failed-stereo"), "");
}

TEST(StereoLoop, SlowEncodeCountsAsFailure) {
  Rig rig;
  rig.Ready();
  rig.encoder.encode_s = 0.6;
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.loop->Status().detail, "encoder: encode took 0.6 s");
}

TEST(StereoLoop, EncoderThatDoesNotOpenAfterTheSelftest) {
  Rig rig;
  rig.Configured();
  rig.encoder.fail_open = true;
  rig.Make();
  rig.Steps(40);
  EXPECT_EQ(rig.State(), StereoState::kEncoderError);
  EXPECT_EQ(rig.loop->Status().detail, "mpp_init -1002");
  EXPECT_EQ(rig.encoder.opens, 1);
  rig.encoder.fail_open = false;
  rig.clock += 5.1;
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kReady);
}

TEST(StereoLoop, CountsFramesTheDriverDropped) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(3);
  rig.camera.skip = 2;
  rig.Steps(3);
  EXPECT_EQ(rig.loop->Status().lost, 2U);
  EXPECT_EQ(rig.loop->Status().frames, 6U);
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// Names of the files with that ending in a directory, sorted.
std::vector<std::string> Recorded(const std::string& dir, const std::string& ending) {
  std::vector<std::string> names;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
    if (entry.path().extension() == ending) names.push_back(entry.path().filename().string());
  }
  std::sort(names.begin(), names.end());
  return names;
}

// Both eyes side by side, each line a ramp of its own.
std::vector<uint8_t> YuyvPicture() {
  std::vector<uint8_t> yuyv(2560 * 480, 0x80);
  for (uint32_t y = 0; y < 480; y++) {
    for (uint32_t x = 0; x < 1280; x++) yuyv[y * 2560 + 2 * x] = static_cast<uint8_t>(16 + (x / 8 + y / 4) % 200);
  }
  return yuyv;
}

TEST(StereoLoop, RecordsBothEyesOfEverySecondFrameOfAStream) {
  Rig rig;
  rig.config.record.dir = rig.dir.Path() + "/record";
  rig.config.record.max_bytes = 64UL << 20;
  rig.config.stream_while_mowing = true;
  rig.camera.pixels = YuyvPicture();
  rig.Ready();
  // The self-test keeps nothing.
  EXPECT_TRUE(Recorded(rig.config.record.dir, ".jpg").empty());
  {
    Client client(rig.server.Port());
    ASSERT_TRUE(rig.Play(client));
    rig.Steps(30, true, true);
    EXPECT_EQ(rig.loop->Status().frames, 30U);
  }
  rig.Steps(3);
  // Frames come every 42 ms, one per 70 ms is kept.
  const std::vector<std::string> images = Recorded(rig.config.record.dir, ".jpg");
  ASSERT_EQ(images.size(), 15U);
  EXPECT_EQ(Recorded(rig.config.record.dir, ".txt").size(), 15U);
  EXPECT_TRUE(Recorded(rig.config.record.dir, ".part").empty());
  EXPECT_EQ(rig.loop->Status().record_error, "");

  const std::string stem = rig.config.record.dir + "/" + images[1].substr(0, images[1].size() - 4);
  const std::string jpeg = ReadFile(stem + ".jpg");
  JpegLuma decoder;
  GrayView luma;
  std::string error;
  ASSERT_TRUE(decoder.Decode(reinterpret_cast<const uint8_t*>(jpeg.data()), jpeg.size(), 1280, 480, luma, error))
      << error;
  EXPECT_NEAR(luma.data[100 * 1280 + 800], 16 + (800 / 8 + 100 / 4) % 200, 6);
  const std::string text = ReadFile(stem + ".txt");
  EXPECT_NE(text.find("taken " + images[1].substr(0, 4) + "."), std::string::npos) << text;
  EXPECT_NE(text.find("\nunix 1"), std::string::npos) << text;
  EXPECT_NE(text.find("\nsequence "), std::string::npos) << text;
  EXPECT_NE(text.find("\nwidth 1280\nheight 480\nblade_on 1\nmoving 1\n"), std::string::npos) << text;
}

TEST(StereoLoop, NoRecordingUnlessSetSo) {
  Rig rig;
  rig.config.record.dir = rig.dir.Path() + "/record";
  rig.camera.pixels = YuyvPicture();
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(10);
  EXPECT_FALSE(std::filesystem::exists(rig.config.record.dir));
  EXPECT_EQ(rig.loop->Status().record_error, "");
}

// A frame the recording cannot read ends the recording; the stream goes on.
TEST(StereoLoop, AFrameWithoutItsPictureEndsTheRecordingOnly) {
  Rig rig;
  rig.config.record.dir = rig.dir.Path() + "/record";
  rig.config.record.max_bytes = 64UL << 20;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(4);
  EXPECT_EQ(rig.loop->Status().record_error, "recording ended: frame of 1228800 bytes without its picture");
  rig.camera.pixels = YuyvPicture();
  rig.Steps(6);
  EXPECT_EQ(rig.State(), StereoState::kStreaming);
  EXPECT_EQ(rig.loop->Status().frames, 10U);
  EXPECT_TRUE(Recorded(rig.config.record.dir, ".jpg").empty());
}

TEST(StereoLoop, MemoryLimitEndsTheProcess) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  // 40000 pages of 4 KiB are over the 150 MB.
  rig.dir.Write("statm", "90000 40000 900 1 0 20000 0\n");
  rig.clock += 5.1;
  EXPECT_FALSE(rig.loop->Step());
  EXPECT_EQ(rig.loop->Status().exit_code, kExitMemory);
  EXPECT_EQ(rig.State(), StereoState::kFailed);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
  EXPECT_FALSE(rig.loop->Step());
}

TEST(StereoLoop, ShutdownEndsTheStream) {
  Rig rig;
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.loop.reset();
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_FALSE(rig.camera.open);
  EXPECT_EQ(rig.Marker("stream-stereo"), "");
}

// A picture of one gray value.
std::vector<uint8_t> FlatPicture(uint8_t luma) {
  std::vector<uint8_t> yuyv(2560 * 480, 128);
  for (size_t i = 0; i < yuyv.size(); i += 2) yuyv[i] = luma;
  return yuyv;
}

// The rig with the gain wish on, its files in the directory of the test.
void WithGainWish(Rig& rig) {
  rig.config.gain_wish_file = rig.dir.Path() + "/gain/wish";
  rig.config.gain_level_file = rig.dir.Path() + "/stereo-level";
  rig.config.gain_lock_file = rig.dir.Path() + "/stereo.lock";
  rig.config.gain_lock_wait_s = 0.03;
  rig.config.stream_while_mowing = true;
}

// Holds the lock the way the setup of the platform does, until it goes.
struct HeldLock {
  explicit HeldLock(const std::string& path) : fd(open(path.c_str(), O_RDWR | O_CREAT, 0644)) {
    flock(fd, LOCK_EX);
  }
  ~HeldLock() {
    close(fd);
  }
  int fd;
};

TEST(StereoLoop, NoGainWishUnlessSetSo) {
  Rig rig;
  rig.camera.pixels = FlatPicture(40);
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(600);
  EXPECT_EQ(rig.loop->Status().gain_level, "");
  EXPECT_FALSE(std::filesystem::exists(rig.dir.Path() + "/gain"));
}

TEST(StereoLoop, WishesForDayFromTheStartAndWritesItAnew) {
  Rig rig;
  WithGainWish(rig);
  rig.Ready();
  EXPECT_EQ(rig.loop->Status().gain_level, "day");
  EXPECT_EQ(ReadFile(rig.config.gain_wish_file).rfind("day 10", 0), 0U);
  rig.clock = 2000.0;
  rig.Steps(1);
  const std::string first = ReadFile(rig.config.gain_wish_file);
  EXPECT_EQ(first.rfind("day 2000.000 ", 0), 0U) << first;
  rig.clock += 9.0;
  rig.Steps(1);
  EXPECT_EQ(ReadFile(rig.config.gain_wish_file), first);
  rig.clock += 1.5;
  rig.Steps(1);
  EXPECT_EQ(ReadFile(rig.config.gain_wish_file).rfind("day 2010.500 ", 0), 0U);
  EXPECT_FALSE(std::filesystem::exists(rig.config.gain_wish_file + ".new"));
  EXPECT_EQ(rig.loop->Status().gain_error, "");
}

TEST(StereoLoop, StartsFromTheLevelTheSetupHolds) {
  Rig rig;
  WithGainWish(rig);
  rig.dir.Write("stereo-level", "night 12.000 1794800000.000\n");
  rig.Ready();
  EXPECT_EQ(rig.loop->Status().gain_level, "night");
  EXPECT_EQ(ReadFile(rig.config.gain_wish_file).rfind("night ", 0), 0U);
}

TEST(StereoLoop, ADarkStreamWishesForTheNextLevel) {
  Rig rig;
  WithGainWish(rig);
  rig.camera.pixels = FlatPicture(40);
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  // Frames come every 42 ms: 19 s of them.
  rig.Steps(452, true, true);
  EXPECT_EQ(rig.loop->Status().gain_level, "day");
  EXPECT_NEAR(rig.loop->Status().luma, 40.0F, 0.01F);
  rig.Steps(120, true, true);
  EXPECT_EQ(rig.loop->Status().gain_level, "dusk");
  EXPECT_EQ(ReadFile(rig.config.gain_wish_file).rfind("dusk ", 0), 0U);
  // The level is not there yet: the picture stays dark and the wish goes on to night.
  rig.Steps(650, true, true);
  EXPECT_EQ(rig.loop->Status().gain_level, "night");
}

TEST(StereoLoop, ABrightStreamKeepsDay) {
  Rig rig;
  WithGainWish(rig);
  rig.camera.pixels = FlatPicture(145);
  rig.Ready();
  Client client(rig.server.Port());
  ASSERT_TRUE(rig.Play(client));
  rig.Steps(1500, true, true);
  EXPECT_EQ(rig.loop->Status().gain_level, "day");
  EXPECT_NEAR(rig.loop->Status().luma, 145.0F, 0.01F);
}

TEST(StereoLoop, NoStreamStartsWhileTheGainLevelIsBeingSet) {
  Rig rig;
  WithGainWish(rig);
  rig.Configured();
  rig.Make();
  {
    const HeldLock held(rig.config.gain_lock_file);
    rig.Steps(40);
    EXPECT_EQ(rig.State(), StereoState::kWaitingIdle);
    EXPECT_EQ(rig.loop->Status().detail, "the gain level is being set");
    EXPECT_EQ(rig.camera.calls, "");
  }
  rig.Steps(40);
  ASSERT_EQ(rig.State(), StereoState::kReady) << rig.loop->Status().detail;
  {
    const HeldLock held(rig.config.gain_lock_file);
    Client client(rig.server.Port());
    EXPECT_FALSE(rig.Play(client));
    EXPECT_EQ(rig.State(), StereoState::kReady);
    EXPECT_EQ(rig.loop->Status().detail, "the gain level is being set");
    EXPECT_EQ(rig.camera.starts, 1);
  }
  Client client(rig.server.Port());
  EXPECT_TRUE(rig.Play(client));
}

TEST(StereoLoop, TheStreamStopsEvenWithTheLockHeld) {
  Rig rig;
  WithGainWish(rig);
  rig.Ready();
  {
    Client client(rig.server.Port());
    ASSERT_TRUE(rig.Play(client));
    rig.Steps(5);
  }
  const HeldLock held(rig.config.gain_lock_file);
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_FALSE(rig.camera.streaming);
}

TEST(StereoLoop, AWishThatCannotBeWrittenIsSaidAndTheStreamGoesOn) {
  Rig rig;
  WithGainWish(rig);
  rig.dir.Write("gain", "a file where the directory would be\n");
  rig.Ready();
  EXPECT_NE(rig.loop->Status().gain_error, "");
  Client client(rig.server.Port());
  EXPECT_TRUE(rig.Play(client));
}

// Counts what the camera hands on for the obstacles.
class CountingSink : public FrameSink {
 public:
  void OnFrame(const CapturedFrame& frame, double, const FrameContext& context) override {
    frames++;
    last = frame.sequence;
    moving = context.moving;
    at_max_gain = context.at_max_gain;
  }
  void OnStop() override {
    stops++;
  }
  int frames = 0;
  int stops = 0;
  uint32_t last = 0;
  bool moving = false;
  bool at_max_gain = false;
};

struct ObstacleRig : Rig {
  ObstacleRig() {
    config.obstacles = true;
    config.stream_while_mowing = true;
    sink = &counted;
  }
  // The loop tells the sink of its last stop, so it goes first.
  ~ObstacleRig() {
    loop.reset();
  }
  // Configured, standing, self-test passed, the camera running for the obstacles.
  void Running() {
    Configured();
    Make();
    Steps(40);
    ASSERT_TRUE(loop->Status().capturing) << StateName(State()) << ": " << loop->Status().detail;
  }
  CountingSink counted;
};

TEST(StereoLoopObstacles, TheCameraRunsWithoutAViewer) {
  ObstacleRig rig;
  rig.Running();
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_TRUE(rig.camera.streaming);
  // Self-test, then one start for good.
  EXPECT_EQ(rig.camera.starts, 2);
  const int before = rig.counted.frames;
  rig.Steps(50);
  EXPECT_EQ(rig.counted.frames, before + 50);
  EXPECT_EQ(rig.counted.stops, 0);
  // Nothing encoded without a viewer, the stream offered.
  EXPECT_FALSE(rig.encoder.is_open);
  EXPECT_TRUE(rig.Describes());
  EXPECT_EQ(rig.Marker("stream-stereo"), kBoot);
}

TEST(StereoLoopObstacles, OffWithoutTheSwitchOrTheSink) {
  Rig plain;
  CountingSink counted;
  plain.sink = &counted;
  plain.Ready();
  plain.Steps(30);
  EXPECT_FALSE(plain.camera.streaming);
  EXPECT_EQ(counted.frames, 0);
  Rig no_sink;
  no_sink.config.obstacles = true;
  no_sink.Ready();
  no_sink.Steps(30);
  EXPECT_FALSE(no_sink.camera.streaming);
}

TEST(StereoLoopObstacles, TheViewerComesAndGoesBesideTheCamera) {
  ObstacleRig rig;
  rig.Running();
  {
    Client client(rig.server.Port());
    ASSERT_TRUE(rig.Play(client));
    EXPECT_TRUE(rig.encoder.is_open);
    const int before = rig.counted.frames;
    rig.Steps(20);
    EXPECT_EQ(rig.State(), StereoState::kStreaming);
    EXPECT_EQ(rig.counted.frames, before + 20);
    const Received got(client);
    EXPECT_GE(got.packets.size(), 4U + 19U);
  }
  rig.Steps(3);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_FALSE(rig.encoder.is_open);
  EXPECT_TRUE(rig.camera.streaming);
  EXPECT_EQ(rig.camera.starts, 2);
  EXPECT_EQ(rig.counted.stops, 0);
}

TEST(StereoLoopObstacles, WithoutTheEncoderOnlyTheStreamIsMissing) {
  ObstacleRig rig;
  rig.encoder.present = false;
  rig.Running();
  EXPECT_EQ(rig.State(), StereoState::kEncoderMissing);
  EXPECT_TRUE(rig.camera.streaming);
  rig.Steps(10);
  EXPECT_GT(rig.counted.frames, 10);
  EXPECT_FALSE(rig.Describes());
}

TEST(StereoLoopObstacles, EncoderFailuresEndOnlyTheStream) {
  ObstacleRig rig;
  rig.Running();
  rig.encoder.fail_encode = true;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Client client(rig.server.Port());
    // The stream begins and its first frame fails in the same step.
    rig.Play(client);
    rig.Steps(2);
    EXPECT_EQ(rig.encoder.opens, 1 + attempt) << attempt;
    EXPECT_EQ(client.Receive().find("$"), std::string::npos);
    EXPECT_TRUE(rig.camera.streaming);
    if (attempt < 3) {
      EXPECT_EQ(rig.State(), StereoState::kReady);
      EXPECT_EQ(rig.loop->Status().detail, "encoder: put frame -1");
    }
    // The camera runs on: the pause before the next try passes with frames.
    rig.Steps(125);
  }
  EXPECT_EQ(rig.State(), StereoState::kEncoderError);
  EXPECT_FALSE(rig.Describes());
  const int before = rig.counted.frames;
  rig.Steps(10);
  EXPECT_EQ(rig.counted.frames, before + 10);
  EXPECT_EQ(rig.counted.stops, 0);
  EXPECT_EQ(rig.Marker("failed-stereo"), "");
}

TEST(StereoLoopObstacles, HeatStopsTheCameraAndCoolingStartsItAgain) {
  ObstacleRig rig;
  rig.Running();
  rig.dir.Write("temp", "80000\n");
  rig.clock += 5.1;
  rig.Steps(2);
  EXPECT_EQ(rig.State(), StereoState::kPausedHot);
  EXPECT_FALSE(rig.loop->Status().capturing);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.counted.stops, 1);
  rig.dir.Write("temp", "74900\n");
  rig.clock += 5.1;
  rig.Steps(3);
  EXPECT_TRUE(rig.loop->Status().capturing);
  EXPECT_EQ(rig.State(), StereoState::kReady);
  EXPECT_EQ(rig.camera.starts, 3);
}

TEST(StereoLoopObstacles, TheBladeStopsTheCameraUnlessStreamWhileMowing) {
  ObstacleRig rig;
  rig.config.stream_while_mowing = false;
  rig.Running();
  rig.Steps(2, true);
  EXPECT_EQ(rig.State(), StereoState::kPausedBlade);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.counted.stops, 1);
  rig.Steps(3);
  EXPECT_TRUE(rig.loop->Status().capturing);

  ObstacleRig mowing;
  mowing.Running();
  const int before = mowing.counted.frames;
  mowing.Steps(10, true, true);
  EXPECT_TRUE(mowing.camera.streaming);
  EXPECT_EQ(mowing.counted.frames, before + 10);
  EXPECT_TRUE(mowing.counted.moving);
}

TEST(StereoLoopObstacles, AGapEndsTheCameraForThisBoot) {
  ObstacleRig rig;
  rig.Running();
  rig.camera.frames_left = 0;
  rig.Steps(20);
  EXPECT_EQ(rig.State(), StereoState::kStalled);
  EXPECT_FALSE(rig.loop->Status().capturing);
  EXPECT_EQ(rig.counted.stops, 1);
  EXPECT_NE(rig.Marker("failed-stereo"), "");
}

TEST(StereoLoopObstacles, TheGainLevelGoesWithTheFrames) {
  ObstacleRig rig;
  WithGainWish(rig);
  rig.dir.Write("stereo-level", "night 12.000 1794800000.000\n");
  rig.Running();
  rig.Steps(2);
  EXPECT_TRUE(rig.counted.at_max_gain);
  ObstacleRig day;
  WithGainWish(day);
  day.Running();
  day.Steps(2);
  EXPECT_FALSE(day.counted.at_max_gain);
}

TEST(StereoLoopObstacles, ShutdownTellsTheSink) {
  ObstacleRig rig;
  rig.Running();
  rig.loop->Shutdown();
  EXPECT_EQ(rig.counted.stops, 1);
  EXPECT_FALSE(rig.camera.open);
}

}  // namespace
}  // namespace tron_vision
