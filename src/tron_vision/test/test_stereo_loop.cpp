#include <gtest/gtest.h>

#include <filesystem>

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
    if (fail_start) {
      error = "stream on: Input/output error";
      return false;
    }
    streaming = true;
    return true;
  }
  void Stop() override {
    if (streaming) calls += "stop ";
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

 private:
  double& clock_;
};

class FakeEncoder : public H264Encoder {
 public:
  explicit FakeEncoder(double& clock) : clock_(clock) {}

  bool Probe(std::string& error) override {
    if (!present) error = "cannot load librockchip_mpp.so.1";
    return present;
  }
  bool Open(const EncoderSettings& s, std::string& error) override {
    opens++;
    settings = s;
    if (fail_open) {
      error = "mpp_init -1002";
      return false;
    }
    is_open = true;
    first = true;
    return true;
  }
  void Close() override {
    is_open = false;
  }
  bool Header(std::vector<uint8_t>& header, std::string&) override {
    header = {0, 0, 0, 1, 0x67, 0x42, 0xe0, 0x1f, 0xaa, 0, 0, 0, 1, 0x68, 0xce, 0x3c, 0x80};
    return true;
  }
  bool Encode(const CapturedFrame& frame, const uint8_t*& data, size_t& size, std::string& error) override {
    clock_ += encode_s;
    if (fail_encode || !is_open || frame.dmabuf < 0) {
      error = "put frame -1";
      return false;
    }
    Header(output_, error);
    if (!first) output_.clear();
    output_.insert(output_.end(), {0, 0, 0, 1, static_cast<uint8_t>(first ? 0x65 : 0x41)});
    output_.insert(output_.end(), first ? 2000 : 200, 0x11);
    first = false;
    data = output_.data();
    size = output_.size();
    return true;
  }

  bool present = true;
  bool fail_open = false;
  bool fail_encode = false;
  double encode_s = 0.002;
  int opens = 0;
  bool is_open = false;
  bool first = true;
  EncoderSettings settings;

 private:
  double& clock_;
  std::vector<uint8_t> output_;
};

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
    loop = std::make_unique<StereoLoop>(config, camera, encoder, server, *guard, [this] { return clock; });
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
    if (Ask(client, "DESCRIBE " + Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n").find("200 OK") == std::string::npos) return false;
    const std::string setup =
        Ask(client, "SETUP " + Url("/trackID=0") + " RTSP/1.0\r\nCSeq: 2\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n\r\n");
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

}  // namespace
}  // namespace tron_vision
