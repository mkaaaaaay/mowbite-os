// The real stream server (MediaMTX) with the configuration, the supervision and the answers of this package, and a
// real WebRTC session on 127.0.0.1: a second instance stands in for the camera (RTSP with a test picture), a third
// one for the viewer (it reads over WHEP). Runs only with the program in MOWBITE_MEDIAMTX.
#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

#include "http_test_client.h"
#include "mowbite_camera/camera_service.h"
#include "mowbite_camera/stream_config.h"
#include "mowbite_camera/stream_server.h"
#include "test_util.h"

namespace mowbite_camera {
namespace {

namespace fs = std::filesystem;

constexpr uint16_t kCameraPort = 18554;
constexpr uint16_t kWebrtcPort = 18889;
constexpr uint16_t kIcePort = 18189;
constexpr uint16_t kApiPort = 18997;
constexpr char kOthersOff[] = "logLevel: warn\nlogDestinations: [stdout]\napi: false\nrtmp: false\nhls: false\n"
                              "srt: false\nmoq: false\nwebrtc: false\n";

double Now() {
  using std::chrono::steady_clock;
  return std::chrono::duration<double>(steady_clock::now().time_since_epoch()).count();
}

bool Until(const std::function<bool()>& done, double seconds) {
  for (const double end = Now() + seconds; Now() < end; ::usleep(50000)) {
    if (done()) return true;
  }
  return done();
}

// What a request without a body gets from the WHEP port: the head of the answer.
std::string Ask(const std::string& method, const std::string& target, const std::string& fields = "") {
  TestClient client(kWebrtcPort);
  client.Send(method + " " + target + " HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n" + fields + "\r\n");
  Until([&] {
    client.Take();
    return client.closed() || client.in().find("\r\n\r\n") != std::string::npos;
  }, 5.0);
  return client.in().substr(0, client.in().find("\r\n\r\n"));
}

TEST(StreamSmoke, ARealSessionTakesAPlaceUntilItIsKicked) {
  const char* program = std::getenv("MOWBITE_MEDIAMTX");
  if (!program || !*program) GTEST_SKIP() << "MOWBITE_MEDIAMTX names no stream server";
  const TempDir temp;
  const fs::path& dir = temp.path();
  std::string error;

  // The node without ROS: the service, and the listener the stream server asks, in a thread of its own.
  ServiceConfig config;
  config.public_host = "127.0.0.1";
  config.webrtc_port = kWebrtcPort;
  CameraService service(config, Clocks{Now, Now});
  HttpServer http([&](const HttpRequest& request) { return service.OnAuth(request); }, [](uint64_t) {}, Now);
  ListenerRules asking;
  asking.max_body = 4096;
  const int listener = http.Listen("127.0.0.1", 0, asking, &error);
  ASSERT_GE(listener, 0) << error;
  std::atomic<bool> serving{true};
  std::thread server([&] {
    while (serving) http.Poll(50);
  });

  // The camera: H.264 over RTSP on this host, always there.
  std::ofstream(dir / "camera.yml") << kOthersOff << "rtsp: true\nrtspAddress: 127.0.0.1:" << kCameraPort
                                    << "\nrtspTransports: [tcp]\npaths:\n  stereo:\n    alwaysAvailable: true\n"
                                    << "    alwaysAvailableTracks:\n      - codec: H264\n";
  ChildSpec camera_spec;
  camera_spec.program = program;
  camera_spec.args = {(dir / "camera.yml").string()};
  camera_spec.directory = dir.string();
  Child camera;
  ASSERT_TRUE(camera.Start(camera_spec, &error)) << error;

  CameraEntry stereo;
  stereo.id = "stereo";
  stereo.process = "stereo";
  stereo.protocols = {kWebrtc};
  stereo.source = "rtsp://127.0.0.1:" + std::to_string(kCameraPort) + "/stereo";
  stereo.available = true;
  ASSERT_EQ(service.OnCamera(stereo), "");

  // The stream server with the file of this package, started by its supervisor.
  StreamConfig stream;
  stream.auth_port = http.Port(listener);
  stream.api_port = kApiPort;
  stream.webrtc_port = kWebrtcPort;
  stream.ice_port = kIcePort;
  stream.sources = service.WebrtcSources();
  const std::string runtime = RuntimeDir(dir.string(), &error);
  ASSERT_NE(runtime, "") << error;
  ASSERT_TRUE(ReplaceFile(runtime, kStreamConfigFile, StreamConfigText(stream), &error)) << error;
  MediaServer media(program, runtime);
  ASSERT_TRUE(media.Installed());
  StreamSupervisor supervisor(&media, Now);
  const LocalServer api{kApiPort, service.api_user(), service.api_password()};
  std::vector<std::string> sessions;
  EXPECT_FALSE(supervisor.Tick(true));
  ASSERT_TRUE(Until([&] { return StreamSessions(api, &sessions); }, 15.0)) << "the configuration does not load";
  supervisor.Answered();
  service.SetWebrtc(supervisor.reason(), supervisor.detail());
  EXPECT_TRUE(supervisor.Up());
  EXPECT_EQ(sessions, std::vector<std::string>());

  // The API is closed to everyone but the node; the server holds a wrong password back for up to 4 s.
  EXPECT_EQ(LocalRequest({kApiPort, "", ""}, "GET", "/v3/webrtcsessions/list", 2.0).status, 401);
  EXPECT_EQ(LocalRequest({kApiPort, service.api_user(), "guess"}, "GET", "/v3/paths/list", 8.0).status, 401);

  // A page of another origin may ask, a token decides who gets in.
  const std::string preflight = Ask("OPTIONS", "/stereo/whep?t=none",
                                    "Origin: http://app.example\r\nAccess-Control-Request-Method: POST\r\n");
  EXPECT_TRUE(Has(preflight, " 204 ")) << preflight;
  EXPECT_TRUE(Has(preflight, "Access-Control-Allow-Origin: ")) << preflight;
  EXPECT_TRUE(Has(Ask("OPTIONS", "/stereo/whep?t=none"), " 401 "));
  const std::optional<RpcOutcome> granted = service.Call("camera.stream", {{"id", "stereo"}});
  ASSERT_TRUE(granted && granted->ok) << (granted ? granted->message : "no answer");
  const std::string url = granted->result["url"];
  const std::string target = url.substr(url.find("/stereo/"));
  EXPECT_EQ(url, "http://127.0.0.1:" + std::to_string(kWebrtcPort) + target);
  EXPECT_TRUE(Has(Ask("OPTIONS", target), " 204 "));
  EXPECT_EQ(service.WebrtcSessions().size(), 0u);

  // The viewer reads the camera through the stream server: its session takes the place of the token.
  std::ofstream(dir / "viewer.yml") << kOthersOff << "rtsp: false\npaths:\n  pull:\n    source: whep://127.0.0.1:"
                                    << kWebrtcPort << target << "\n";
  ChildSpec viewer_spec = camera_spec;
  viewer_spec.args = {(dir / "viewer.yml").string()};
  Child viewer;
  ASSERT_TRUE(viewer.Start(viewer_spec, &error)) << error;
  ASSERT_TRUE(Until([&] { return service.WebrtcSessions().size() == 1; }, 20.0)) << "no session took the place";
  const std::string session = service.WebrtcSessions()[0];
  ASSERT_TRUE(Until([&] { return StreamSessions(api, &sessions) && sessions == service.WebrtcSessions(); }, 10.0));
  service.OnSessions(sessions);
  service.OnSessions(sessions);
  EXPECT_EQ(service.WebrtcSessions(), std::vector<std::string>{session});

  // Kicked, the session is gone at once, and its token opens nothing any more.
  EXPECT_TRUE(KickStreamSession(api, session));
  ASSERT_TRUE(Until([&] { return StreamSessions(api, &sessions) && sessions.empty(); }, 5.0));
  service.OnSessions(sessions);
  service.OnSessions(sessions);
  EXPECT_EQ(service.WebrtcSessions().size(), 0u);
  EXPECT_TRUE(Has(Ask("OPTIONS", target), " 401 "));
  EXPECT_TRUE(supervisor.Up());
  EXPECT_FALSE(supervisor.Tick(true));

  viewer.Stop();
  const double before = Now();
  media.Stop();
  EXPECT_LT(Now() - before, 1.5) << "the stream server did not end on SIGINT";
  EXPECT_TRUE(supervisor.Tick(true));
  camera.Stop();
  serving = false;
  server.join();
}

}  // namespace
}  // namespace mowbite_camera
