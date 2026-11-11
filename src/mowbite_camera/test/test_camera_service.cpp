#include <gtest/gtest.h>

#include <memory>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/event_builder.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_camera/camera_service.h"

namespace mowbite_camera {
namespace {

// The first 16 bytes make the password for the stream server, the next ones the tokens.
constexpr char kApiPassword[] = "000102030405060708090a0b0c0d0e0f";
constexpr char kFirstToken[] = "101112131415161718191a1b1c1d1e1f";

CameraEntry Obstacle() {
  CameraEntry entry;
  entry.id = "oa_left";
  entry.name = "Obstacle camera";
  entry.process = "vision";
  entry.kind = "obstacle";
  entry.position = "front";
  entry.width = 640;
  entry.height = 480;
  entry.detections = true;
  entry.protocols = {"mjpeg"};
  entry.available = true;
  return entry;
}

CameraEntry Stereo() {
  CameraEntry entry;
  entry.id = "stereo";
  entry.name = "Stereo camera";
  entry.process = "stereo";
  entry.kind = "stereo";
  entry.width = 1280;
  entry.height = 720;
  entry.protocols = {"webrtc"};
  entry.source = "rtsp://127.0.0.1:8555/stereo";
  entry.available = true;
  return entry;
}

std::string TokenOf(const RpcOutcome& outcome) {
  const std::string url = outcome.result.value("url", "");
  const size_t mark = url.find("?t=");
  return mark == std::string::npos ? "" : url.substr(mark + 3);
}

class CameraServiceTest : public ::testing::Test {
 protected:
  CameraServiceTest()
      : service_(ServiceConfig(), Clocks{[this] { return steady_; }, [this] { return wall_; }}, Counting(),
                 [this] { return route_; }) {}

  static RandomBytes Counting() {
    auto next = std::make_shared<uint8_t>(0);
    return [next](uint8_t* out, size_t size) {
      for (size_t i = 0; i < size; ++i) out[i] = (*next)++;
      return true;
    };
  }

  RpcOutcome Call(const std::string& method, const Json& params = Json::object()) {
    const std::optional<RpcOutcome> outcome = service_.Call(method, params);
    EXPECT_TRUE(outcome.has_value());
    return outcome.value_or(RpcOutcome());
  }

  RpcOutcome Stream(const std::string& id) {
    return Call("camera.stream", {{"id", id}});
  }

  HttpAnswer View(const std::string& id, const std::string& token, uint64_t connection) {
    return service_.OnViewer({0, connection, "GET", "/camera/" + id + ".mjpg", "t=" + token, ""});
  }

  HttpAnswer Auth(const Json& asked) {
    return service_.OnAuth({1, 1, "POST", "/auth", "", asked.dump()});
  }

  // What the stream server asks for a reader of a camera; no session: the request before the session.
  HttpAnswer Reader(const std::string& camera, const std::string& token, const std::string& session) {
    return Auth({{"ip", "198.51.100.30"}, {"user", ""}, {"password", ""}, {"token", ""}, {"action", "read"},
                 {"path", camera}, {"protocol", "webrtc"}, {"id", session.empty() ? Json() : Json(session)},
                 {"query", "t=" + token}, {"userAgent", "test"}});
  }

  // The refusal of a call with this reason, as the catalog has it.
  void ExpectRefused(const RpcOutcome& outcome, const std::string& reason) {
    EXPECT_FALSE(outcome.ok);
    EXPECT_EQ(outcome.code, mowbite_bridge::kInvalidRequest);
    ASSERT_TRUE(outcome.data.is_object()) << outcome.message;
    EXPECT_EQ(outcome.data.value("event", ""), "CAMERA_REFUSED");
    EXPECT_EQ(outcome.data.value("reason", ""), reason);
    EXPECT_EQ(outcome.data.value("message", ""), outcome.message);
    const mowbite_api::EventEntry* group = mowbite_api::FindReasonGroup("CAMERA_REFUSED");
    ASSERT_NE(group, nullptr);
    bool known = false;
    for (const auto& entry : group->reasons) known = known || entry.code == reason;
    EXPECT_TRUE(known) << reason;
    const mowbite_api::Rating rating = mowbite_api::RateReason("CAMERA_REFUSED", reason);
    EXPECT_EQ(outcome.data.value("severity", ""), rating.severity);
    EXPECT_EQ(outcome.data.value("resume", ""), rating.resume);
  }

  // The events left to send, each checked against the catalog.
  std::vector<Json> Events() {
    std::vector<Json> events = service_.TakeEffects().events;
    for (const Json& details : events) {
      const Json event = mowbite_bridge::BuildEvent("CAMERA_CHANGED", details, wall_);
      for (const auto& problem : mowbite_api::CheckEvent("CAMERA_CHANGED", event)) ADD_FAILURE() << problem;
    }
    return events;
  }

  double steady_ = 50.0;
  double wall_ = 1700000000.75;
  std::string route_ = "198.51.100.20";
  CameraService service_;
};

TEST_F(CameraServiceTest, OffersItsTwoMethods) {
  EXPECT_EQ(CameraService::Methods(), (std::vector<std::string>{"camera.list", "camera.stream"}));
  EXPECT_FALSE(service_.Call("sound.list", Json::object()).has_value());
  EXPECT_FALSE(service_.HasCameras());
  EXPECT_EQ(Call("camera.list").result, Json::array());
}

TEST_F(CameraServiceTest, ListsTheCamerasAndTellsEachNewOne) {
  EXPECT_EQ(service_.OnCamera(Obstacle()), "");
  EXPECT_EQ(service_.OnCamera(Stereo()), "");
  EXPECT_TRUE(service_.HasCameras());
  const RpcOutcome list = Call("camera.list");
  ASSERT_TRUE(list.ok);
  ASSERT_EQ(list.result.size(), 2u);
  EXPECT_EQ(list.result[0]["id"], "oa_left");
  EXPECT_EQ(list.result[0]["position"], "front");
  EXPECT_TRUE(list.result[1]["position"].is_null());
  EXPECT_EQ(list.result[1]["protocols"], Json::parse(R"(["webrtc"])"));
  const std::vector<Json> events = Events();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0], Json::parse(R"({"camera_id":"oa_left","available":true,"severity":"info","resume":"auto"})"));
  EXPECT_EQ(events[1]["camera_id"], "stereo");
  EXPECT_EQ(service_.Processes(), (std::vector<std::string>{"vision", "stereo"}));

  CameraEntry bad = Obstacle();
  bad.id = "no id";
  EXPECT_NE(service_.OnCamera(bad), "");
  EXPECT_EQ(Call("camera.list").result.size(), 2u);
}

TEST_F(CameraServiceTest, GivesAnAddressWithATokenForMjpeg) {
  service_.OnCamera(Obstacle());
  const RpcOutcome outcome = Stream("oa_left");
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(outcome.result["url"], std::string("http://198.51.100.20:8890/camera/oa_left.mjpg?t=") + kFirstToken);
  EXPECT_EQ(outcome.result["protocol"], "mjpeg");
  // 30 s from now, rounded down.
  EXPECT_EQ(outcome.result["expires"], 1700000030);
  EXPECT_TRUE(outcome.result["expires"].is_number_integer());
}

TEST_F(CameraServiceTest, GivesAWhepAddressForWebrtc) {
  service_.OnCamera(Stereo());
  const RpcOutcome outcome = Call("camera.stream", {{"id", "stereo"}, {"protocol", "webrtc"}});
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(outcome.result["url"], std::string("http://198.51.100.20:8889/stereo/whep?t=") + kFirstToken);
  EXPECT_EQ(outcome.result["protocol"], "webrtc");
}

TEST_F(CameraServiceTest, TakesTheHostOfTheParameterBeforeTheRoute) {
  ServiceConfig config;
  config.public_host = "mower.lan";
  config.mjpeg_port = 18890;
  CameraService service(config, Clocks{[] { return 0.0; }, [] { return 0.0; }}, Counting(), [] { return ""; });
  service.OnCamera(Obstacle());
  const std::optional<RpcOutcome> outcome = service.Call("camera.stream", {{"id", "oa_left"}});
  ASSERT_TRUE(outcome && outcome->ok);
  EXPECT_EQ(outcome->result["url"], std::string("http://mower.lan:18890/camera/oa_left.mjpg?t=") + kFirstToken);
}

TEST_F(CameraServiceTest, RefusesWithoutAnAddressAndKeepsThePlace) {
  service_.OnCamera(Obstacle());
  route_ = "";
  ExpectRefused(Stream("oa_left"), "NOT_SET_UP");
  route_ = "10.0.0.2";
  EXPECT_TRUE(Stream("oa_left").ok);
  EXPECT_TRUE(Stream("oa_left").ok);
}

TEST_F(CameraServiceTest, ParamsItCannotUseAreInvalid) {
  service_.OnCamera(Obstacle());
  for (const Json& params : {Json::array(), Json::object(), Json{{"id", 5}}, Json{{"id", "nope"}},
                             Json{{"id", "oa_left"}, {"protocol", "webrtc"}},
                             Json{{"id", "oa_left"}, {"protocol", nullptr}}}) {
    const RpcOutcome outcome = Call("camera.stream", params);
    EXPECT_FALSE(outcome.ok) << params.dump();
    EXPECT_EQ(outcome.code, mowbite_bridge::kInvalidParams) << params.dump();
    EXPECT_TRUE(outcome.data.is_null());
  }
  // None of them took a place.
  EXPECT_TRUE(Stream("oa_left").ok);
  EXPECT_TRUE(Stream("oa_left").ok);
}

TEST_F(CameraServiceTest, AThirdViewerIsRefusedUntilAPlaceIsFree) {
  service_.OnCamera(Obstacle());
  service_.OnCamera(Stereo());
  const RpcOutcome first = Stream("oa_left");
  ASSERT_TRUE(first.ok);
  ASSERT_TRUE(Stream("stereo").ok);
  ExpectRefused(Stream("oa_left"), "CAMERA_BUSY");

  // The first one connects, the token of the second runs out.
  EXPECT_EQ(View("oa_left", TokenOf(first), 7).status, 200);
  steady_ += kTokenSeconds + 0.1;
  service_.Tick();
  EXPECT_TRUE(Stream("stereo").ok);
  ExpectRefused(Stream("stereo"), "CAMERA_BUSY");
  service_.OnViewerGone(7);
  EXPECT_TRUE(Stream("oa_left").ok);
}

TEST_F(CameraServiceTest, AViewerConnectsOnceWithItsToken) {
  service_.OnCamera(Obstacle());
  CameraEntry rear = Obstacle();
  rear.id = "rear";
  service_.OnCamera(rear);
  const std::string token = TokenOf(Stream("oa_left"));
  EXPECT_EQ(View("oa_left", "", 1).status, 401);
  EXPECT_EQ(View("oa_left", std::string(32, '0'), 1).status, 401);
  EXPECT_EQ(View("rear", token, 1).status, 401);
  EXPECT_EQ(View("nope", token, 1).status, 401);
  EXPECT_TRUE(service_.MjpegCameras().empty());

  const HttpAnswer answer = View("oa_left", token, 1);
  EXPECT_EQ(answer.status, 200);
  EXPECT_EQ(answer.stream, "oa_left");
  EXPECT_EQ(service_.MjpegCameras(), std::vector<std::string>{"oa_left"});
  EXPECT_EQ(View("oa_left", token, 2).status, 401);

  service_.OnViewerGone(1);
  EXPECT_TRUE(service_.MjpegCameras().empty());
  EXPECT_EQ(View("oa_left", token, 3).status, 401);
}

TEST_F(CameraServiceTest, OnlyAGetOfACameraPathIsAViewer) {
  service_.OnCamera(Obstacle());
  const std::string token = TokenOf(Stream("oa_left"));
  EXPECT_EQ(service_.OnViewer({0, 1, "POST", "/camera/oa_left.mjpg", "t=" + token, ""}).status, 405);
  for (const char* path : {"/", "/camera/", "/camera/.mjpg", "/camera/oa_left", "/camera/oa_left.jpg",
                           "/oa_left.mjpg", "/camera/oa_left.mjpg/"}) {
    EXPECT_EQ(service_.OnViewer({0, 1, "GET", path, "t=" + token, ""}).status, 404) << path;
  }
  EXPECT_EQ(View("oa_left", token, 1).status, 200);
}

TEST_F(CameraServiceTest, ATokenThatRanOutDoesNotConnect) {
  service_.OnCamera(Obstacle());
  const std::string token = TokenOf(Stream("oa_left"));
  steady_ += kTokenSeconds + 0.1;
  EXPECT_EQ(View("oa_left", token, 1).status, 401);
}

TEST_F(CameraServiceTest, RefusesWhileTheCameraCannotStream) {
  CameraEntry entry = Obstacle();
  entry.available = false;
  for (const char* reason : {"LOCKED_AFTER_CRASH", "NOT_SET_UP", "BLADE_RUNNING", "TOO_HOT", "NO_SIGNAL",
                             "DRIVER_MISSING", "SELF_TEST"}) {
    entry.reason = reason;
    entry.detail = "why";
    service_.OnCamera(entry);
    ExpectRefused(Stream("oa_left"), reason);
  }
  const std::vector<Json> events = Events();
  ASSERT_EQ(events.size(), 7u);
  EXPECT_EQ(events[3], Json::parse(R"({"camera_id":"oa_left","available":false,"reason":"TOO_HOT","message":"why",
      "severity":"info","resume":"auto"})"));
}

TEST_F(CameraServiceTest, ACameraThatCanNoLongerStreamLosesItsViewers) {
  service_.OnCamera(Obstacle());
  service_.OnCamera(Stereo());
  const std::string watching = TokenOf(Stream("oa_left"));
  const std::string waiting = TokenOf(Stream("oa_left"));
  ASSERT_EQ(View("oa_left", watching, 7).status, 200);
  Events();

  CameraEntry entry = Obstacle();
  entry.available = false;
  entry.reason = "BLADE_RUNNING";
  service_.OnCamera(entry);
  const Effects effects = service_.TakeEffects();
  ASSERT_EQ(effects.events.size(), 1u);
  EXPECT_EQ(effects.events[0]["reason"], "BLADE_RUNNING");
  EXPECT_FALSE(effects.events[0].contains("message"));
  EXPECT_EQ(effects.close, std::vector<uint64_t>{7});
  EXPECT_TRUE(effects.kick.empty());
  EXPECT_TRUE(service_.MjpegCameras().empty());

  // Both places are free, and the token that waited is gone with its camera.
  service_.OnCamera(Obstacle());
  EXPECT_EQ(View("oa_left", waiting, 8).status, 401);
  EXPECT_TRUE(Stream("stereo").ok);
  EXPECT_TRUE(Stream("stereo").ok);
}

TEST_F(CameraServiceTest, TheExitNoteOfItsProcessTakesACameraOffUntilItReportsAgain) {
  service_.OnCamera(Obstacle());
  Events();
  service_.OnProcessEnded("vision", steady_ - 1, false);
  EXPECT_TRUE(Events().empty());
  service_.OnProcessEnded("vision", steady_ + 1, false);
  std::vector<Json> events = Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["reason"], "NO_SIGNAL");
  EXPECT_EQ(events[0]["message"], "process vision ended");
  ExpectRefused(Stream("oa_left"), "NO_SIGNAL");

  steady_ += 3;
  service_.OnCamera(Obstacle());
  events = Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["available"], true);
  EXPECT_TRUE(Stream("oa_left").ok);
}

TEST_F(CameraServiceTest, WebrtcWaitsForTheStreamServer) {
  service_.OnCamera(Stereo());
  CameraEntry both = Obstacle();
  both.protocols = {"webrtc", "mjpeg"};
  both.source = "rtsp://127.0.0.1:8556/oa_left";
  service_.OnCamera(both);
  EXPECT_EQ(service_.WebrtcSources().size(), 2u);
  Events();

  service_.SetWebrtc("NO_SIGNAL", "the stream server starts");
  std::vector<Json> events = Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["camera_id"], "stereo");
  ExpectRefused(Stream("stereo"), "NO_SIGNAL");
  // A camera with both protocols stays, only its webrtc is refused.
  ExpectRefused(Stream("oa_left"), "NO_SIGNAL");
  EXPECT_TRUE(Call("camera.stream", {{"id", "oa_left"}, {"protocol", "mjpeg"}}).ok);

  service_.SetWebrtc("", "");
  events = Events();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["available"], true);
  EXPECT_EQ(Stream("oa_left").result["protocol"], "webrtc");
}

TEST_F(CameraServiceTest, TheStreamServerLetsOneSessionInPerToken) {
  service_.OnCamera(Stereo());
  const std::string token = TokenOf(Stream("stereo"));
  EXPECT_EQ(Reader("stereo", "", "").status, 401);
  EXPECT_EQ(Reader("stereo", std::string(32, 'f'), "s1").status, 401);
  EXPECT_EQ(Reader("other", token, "s1").status, 401);
  // The request before the session takes nothing.
  EXPECT_EQ(Reader("stereo", token, "").status, 200);
  EXPECT_EQ(Reader("stereo", token, "").status, 200);
  EXPECT_TRUE(service_.WebrtcSessions().empty());

  EXPECT_EQ(Reader("stereo", token, "s1").status, 200);
  EXPECT_EQ(service_.WebrtcSessions(), std::vector<std::string>{"s1"});
  EXPECT_EQ(Reader("stereo", token, "s1").status, 200);
  EXPECT_EQ(Reader("stereo", token, "s2").status, 401);
  EXPECT_EQ(Reader("stereo", token, "").status, 401);
  // mjpeg has no part in it.
  EXPECT_EQ(View("stereo", token, 1).status, 401);
}

TEST_F(CameraServiceTest, AWebrtcTokenRunsOutToo) {
  service_.OnCamera(Stereo());
  const std::string token = TokenOf(Stream("stereo"));
  steady_ += kTokenSeconds + 0.1;
  EXPECT_EQ(Reader("stereo", token, "").status, 401);
  EXPECT_EQ(Reader("stereo", token, "s1").status, 401);
}

TEST_F(CameraServiceTest, ASessionTheStreamServerLacksTwiceIsOver) {
  service_.OnCamera(Stereo());
  const std::string a = TokenOf(Stream("stereo"));
  const std::string b = TokenOf(Stream("stereo"));
  ASSERT_EQ(Reader("stereo", a, "s1").status, 200);
  ASSERT_EQ(Reader("stereo", b, "s2").status, 200);
  service_.OnSessions({"s1", "s2", "foreign"});
  service_.OnSessions({"s2"});
  EXPECT_EQ(service_.WebrtcSessions().size(), 2u);
  // Back in between: counted from the start again.
  service_.OnSessions({"s1", "s2"});
  service_.OnSessions({"s2"});
  EXPECT_EQ(service_.WebrtcSessions().size(), 2u);
  service_.OnSessions({"s2"});
  EXPECT_EQ(service_.WebrtcSessions(), std::vector<std::string>{"s2"});
  EXPECT_TRUE(Stream("stereo").ok);
  ExpectRefused(Stream("stereo"), "CAMERA_BUSY");
}

TEST_F(CameraServiceTest, TheEndOfTheStreamServerFreesItsPlaces) {
  service_.OnCamera(Stereo());
  service_.OnCamera(Obstacle());
  ASSERT_EQ(Reader("stereo", TokenOf(Stream("stereo")), "s1").status, 200);
  ASSERT_EQ(View("oa_left", TokenOf(Stream("oa_left")), 4).status, 200);
  service_.OnStreamServerGone();
  EXPECT_TRUE(service_.WebrtcSessions().empty());
  EXPECT_EQ(service_.MjpegCameras(), std::vector<std::string>{"oa_left"});
  EXPECT_TRUE(Stream("stereo").ok);
}

TEST_F(CameraServiceTest, AWebrtcCameraThatCanNoLongerStreamHasItsSessionsKicked) {
  service_.OnCamera(Stereo());
  ASSERT_EQ(Reader("stereo", TokenOf(Stream("stereo")), "s1").status, 200);
  service_.TakeEffects();
  CameraEntry entry = Stereo();
  entry.available = false;
  entry.reason = "TOO_HOT";
  service_.OnCamera(entry);
  const Effects effects = service_.TakeEffects();
  EXPECT_EQ(effects.kick, std::vector<std::string>{"s1"});
  EXPECT_TRUE(effects.close.empty());
  EXPECT_TRUE(service_.WebrtcSessions().empty());
}

TEST_F(CameraServiceTest, OnlyTheNodeMayUseTheApiOfTheStreamServer) {
  EXPECT_EQ(service_.api_user(), "mowbite");
  EXPECT_EQ(service_.api_password(), kApiPassword);
  const auto api = [&](const std::string& user, const std::string& password) {
    return Auth({{"ip", "127.0.0.1"}, {"user", user}, {"password", password}, {"action", "api"}, {"path", ""},
                 {"protocol", ""}, {"id", nullptr}, {"query", ""}}).status;
  };
  EXPECT_EQ(api("mowbite", kApiPassword), 200);
  EXPECT_EQ(api("mowbite", ""), 401);
  EXPECT_EQ(api("mowbite", "000102030405060708090a0b0c0d0e0e"), 401);
  EXPECT_EQ(api("admin", kApiPassword), 401);
  EXPECT_EQ(api("", ""), 401);
}

TEST_F(CameraServiceTest, WithoutRandomNumbersNobodyUsesTheApi) {
  CameraService service(ServiceConfig(), Clocks{[] { return 0.0; }, [] { return 0.0; }},
                        [](uint8_t*, size_t) { return false; }, [] { return "10.0.0.2"; });
  EXPECT_EQ(service.api_password(), "");
  const Json asked = {{"user", "mowbite"}, {"password", ""}, {"action", "api"}};
  EXPECT_EQ(service.OnAuth({1, 1, "POST", "/auth", "", asked.dump()}).status, 401);
  service.OnCamera(Obstacle());
  const std::optional<RpcOutcome> outcome = service.Call("camera.stream", {{"id", "oa_left"}});
  ASSERT_TRUE(outcome.has_value());
  EXPECT_EQ(outcome->code, mowbite_bridge::kInternalError);
}

TEST_F(CameraServiceTest, EverythingElseTheStreamServerAsksIsKeptOut) {
  service_.OnCamera(Stereo());
  const std::string token = TokenOf(Stream("stereo"));
  for (const char* action : {"publish", "playback", "metrics", "pprof", ""}) {
    EXPECT_EQ(Auth({{"action", action}, {"path", "stereo"}, {"protocol", "webrtc"}, {"id", "s1"},
                    {"query", "t=" + token}}).status, 401) << action;
  }
  for (const char* protocol : {"rtsp", "hls", "rtmp", "srt", ""}) {
    EXPECT_EQ(Auth({{"action", "read"}, {"path", "stereo"}, {"protocol", protocol}, {"id", "s1"},
                    {"query", "t=" + token}}).status, 401) << protocol;
  }
  EXPECT_EQ(Auth({{"action", 1}, {"path", 2}, {"protocol", 3}, {"id", 4}, {"query", 5}}).status, 401);
  EXPECT_EQ(service_.OnAuth({1, 1, "POST", "/auth", "", "[]"}).status, 400);
  EXPECT_EQ(service_.OnAuth({1, 1, "POST", "/auth", "", "{"}).status, 400);
  EXPECT_EQ(service_.OnAuth({1, 1, "GET", "/auth", "", "{}"}).status, 404);
  EXPECT_EQ(service_.OnAuth({1, 1, "POST", "/other", "", "{}"}).status, 404);
  // The token is still good for its session.
  EXPECT_EQ(Reader("stereo", token, "s1").status, 200);
}

}  // namespace
}  // namespace mowbite_camera
