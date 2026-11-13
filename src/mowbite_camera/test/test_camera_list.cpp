#include <gtest/gtest.h>

#include "mowbite_camera/camera_list.h"
#include "test_util.h"

namespace mowbite_camera {
namespace {

TEST(CameraList, ListsWhatTheCameraSays) {
  CameraList list;
  std::string problem;
  ASSERT_TRUE(list.Update(Obstacle(), 1, &problem));
  EXPECT_EQ(problem, "");
  const Json expected = Json::parse(R"([{"id":"oa_left","name":"Obstacle camera","kind":"obstacle",
      "position":"front","width":640,"height":480,"detections":true,"protocols":["mjpeg"],"available":true}])");
  EXPECT_EQ(list.List(), expected);
}

TEST(CameraList, KeepsTheOrderOfTheFirstReports) {
  CameraList list;
  std::string problem;
  ASSERT_TRUE(list.Update(Stereo(), 1, &problem));
  ASSERT_TRUE(list.Update(Obstacle(), 2, &problem));
  ASSERT_TRUE(list.Update(Stereo(), 3, &problem));
  const Json listed = list.List();
  ASSERT_EQ(listed.size(), 2u);
  EXPECT_EQ(listed[0]["id"], "stereo");
  EXPECT_EQ(listed[1]["id"], "oa_left");
  EXPECT_EQ(list.Processes(), (std::vector<std::string>{"stereo", "vision"}));
}

TEST(CameraList, AnUnknownPositionIsNull) {
  CameraList list;
  std::string problem;
  CameraEntry entry = Obstacle();
  entry.position = "up";
  ASSERT_TRUE(list.Update(entry, 1, &problem));
  EXPECT_TRUE(list.List()[0]["position"].is_null());
  entry.position = "";
  ASSERT_TRUE(list.Update(entry, 2, &problem));
  EXPECT_TRUE(list.List()[0]["position"].is_null());
}

TEST(CameraList, RefusesAnIdThatIsNoName) {
  CameraList list;
  std::string problem;
  for (const char* id : {"", "1cam", "a/b", "a b", "a.b", "abcdefghijklmnopqrstuvwxyz0123456"}) {
    CameraEntry entry = Obstacle();
    entry.id = id;
    EXPECT_FALSE(list.Update(entry, 1, &problem)) << id;
    EXPECT_NE(problem, "");
  }
  EXPECT_TRUE(list.Empty());
  EXPECT_TRUE(IsCameraId("abcdefghijklmnopqrstuvwxyz012345"));
}

TEST(CameraList, PassesOnlyTheProtocolsTheOsServes) {
  CameraList list;
  std::string problem;
  CameraEntry entry = Stereo();
  entry.protocols = {"hls", "webrtc", "mjpeg", "webrtc"};
  ASSERT_TRUE(list.Update(entry, 1, &problem));
  EXPECT_EQ(list.List()[0]["protocols"], Json::parse(R"(["webrtc","mjpeg"])"));
  EXPECT_EQ(list.WebrtcSources(), (std::map<std::string, std::string>{{"stereo", "rtsp://127.0.0.1:8555/stereo"}}));
}

TEST(CameraList, WebrtcOnlyFromItsOwnAddressOnThisHost) {
  for (const char* source : {"", "rtsp://127.0.0.1:8555/other", "rtsp://127.0.0.1:8555/stereo/x",
                             "rtsp://127.0.0.1:0/stereo", "rtsp://127.0.0.1:65536/stereo",
                             "rtsp://127.0.0.1:/stereo", "rtsp://127.0.0.1:85a5/stereo",
                             "rtsp://198.51.100.5:8555/stereo", "rtsp://127.0.0.1:8555/stereo\nrunOnInit: x",
                             "http://127.0.0.1:8555/stereo"}) {
    CameraList list;
    std::string problem;
    CameraEntry entry = Stereo();
    entry.source = source;
    ASSERT_TRUE(list.Update(entry, 1, &problem)) << source;
    EXPECT_NE(problem, "") << source;
    EXPECT_TRUE(list.WebrtcSources().empty()) << source;
    // Without a protocol it cannot stream.
    EXPECT_EQ(list.State("stereo"), (CameraState{false, "NOT_SET_UP", "no protocol the OS serves"})) << source;
  }
}

TEST(CameraList, TellsWhyACameraCannotStream) {
  CameraList list;
  std::string problem;
  CameraEntry entry = Obstacle();
  entry.available = false;
  entry.reason = "TOO_HOT";
  entry.detail = "SoC 81 C";
  ASSERT_TRUE(list.Update(entry, 1, &problem));
  const Json listed = list.List();
  EXPECT_EQ(listed[0]["available"], false);
  EXPECT_EQ(listed[0]["reason"], "TOO_HOT");
  EXPECT_EQ(listed[0]["detail"], "SoC 81 C");
  // What an available camera says about a reason does not count.
  entry.available = true;
  ASSERT_TRUE(list.Update(entry, 2, &problem));
  EXPECT_FALSE(list.List()[0].contains("reason"));
  EXPECT_FALSE(list.List()[0].contains("detail"));
}

TEST(CameraList, AReasonOutsideTheCatalogBecomesNoSignal) {
  CameraList list;
  std::string problem;
  CameraEntry entry = Obstacle();
  entry.available = false;
  entry.reason = "LENS_CAP";
  entry.detail = "on";
  ASSERT_TRUE(list.Update(entry, 1, &problem));
  EXPECT_EQ(list.State("oa_left"), (CameraState{false, "NO_SIGNAL", "reason LENS_CAP: on"}));
}

TEST(CameraList, EveryNewStateIsAChangeOnce) {
  CameraList list;
  std::string problem;
  ASSERT_TRUE(list.Update(Obstacle(), 1, &problem));
  std::vector<CameraChange> changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].camera_id, "oa_left");
  EXPECT_TRUE(changes[0].news.state.available);
  EXPECT_TRUE(list.TakeChanges().empty());
  ASSERT_TRUE(list.Update(Obstacle(), 2, &problem));
  EXPECT_TRUE(list.TakeChanges().empty());

  CameraEntry entry = Obstacle();
  entry.available = false;
  entry.reason = "SELF_TEST";
  ASSERT_TRUE(list.Update(entry, 3, &problem));
  changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].news.state, (CameraState{false, "SELF_TEST", ""}));
}

TEST(CameraList, OtherProtocolsOrAnotherSizeAreAChange) {
  CameraList list;
  std::string problem;
  CameraEntry entry = Obstacle();
  entry.protocols = {"webrtc", "mjpeg"};
  entry.source = "rtsp://127.0.0.1:8556/oa_left";
  ASSERT_TRUE(list.Update(entry, 1, &problem));
  std::vector<CameraChange> changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].news, (CameraNews{{true, "", ""}, {"webrtc", "mjpeg"}, 640, 480}));
  EXPECT_TRUE(changes[0].dropped.empty());

  // The source alone is no news.
  entry.source = "rtsp://127.0.0.1:8557/oa_left";
  ASSERT_TRUE(list.Update(entry, 2, &problem));
  EXPECT_TRUE(list.TakeChanges().empty());

  entry.protocols = {"mjpeg"};
  entry.width = 320;
  entry.height = 240;
  ASSERT_TRUE(list.Update(entry, 3, &problem));
  changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].news, (CameraNews{{true, "", ""}, {"mjpeg"}, 320, 240}));
  EXPECT_EQ(changes[0].dropped, (std::vector<std::string>{"webrtc"}));

  entry.width = 640;
  entry.height = 480;
  ASSERT_TRUE(list.Update(entry, 4, &problem));
  changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].news.width, 640u);
  EXPECT_TRUE(changes[0].dropped.empty());

  // Another order of the same protocols is news, but drops none.
  entry.protocols = {"webrtc", "mjpeg"};
  ASSERT_TRUE(list.Update(entry, 5, &problem));
  list.TakeChanges();
  entry.protocols = {"mjpeg", "webrtc"};
  ASSERT_TRUE(list.Update(entry, 6, &problem));
  changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_TRUE(changes[0].dropped.empty());
}

TEST(CameraList, AProcessThatEndedGivesNoPictureUntilItReportsAgain) {
  CameraList list;
  std::string problem;
  ASSERT_TRUE(list.Update(Obstacle(), 10, &problem));
  ASSERT_TRUE(list.Update(Stereo(), 10, &problem));
  list.TakeChanges();

  // A note from before its last report is an old one.
  list.ProcessEnded("vision", 9, false);
  EXPECT_TRUE(list.TakeChanges().empty());
  list.ProcessEnded("vision", 11, false);
  std::vector<CameraChange> changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].camera_id, "oa_left");
  EXPECT_EQ(changes[0].news.state, (CameraState{false, "NO_SIGNAL", "process vision ended"}));
  EXPECT_TRUE(list.State("stereo").available);
  list.ProcessEnded("vision", 11, false);
  EXPECT_TRUE(list.TakeChanges().empty());

  ASSERT_TRUE(list.Update(Obstacle(), 13, &problem));
  changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_TRUE(changes[0].news.state.available);

  list.ProcessEnded("vision", 20, true);
  EXPECT_EQ(list.State("oa_left"), (CameraState{false, "NO_SIGNAL", "process vision ended and stays down"}));
}

TEST(CameraList, AProcessNameThatIsNoneHasNoExitNote) {
  CameraList list;
  std::string problem;
  CameraEntry entry = Obstacle();
  entry.process = "../vision";
  ASSERT_TRUE(list.Update(entry, 1, &problem));
  EXPECT_NE(problem, "");
  EXPECT_TRUE(list.Processes().empty());
  list.ProcessEnded("../vision", 2, true);
  EXPECT_TRUE(list.State("oa_left").available);
}

TEST(CameraList, WithoutTheStreamServerACameraWithOnlyWebrtcCannotStream) {
  CameraList list;
  std::string problem;
  ASSERT_TRUE(list.Update(Stereo(), 1, &problem));
  CameraEntry both = Obstacle();
  both.protocols = {"webrtc", "mjpeg"};
  both.source = "rtsp://127.0.0.1:8556/oa_left";
  ASSERT_TRUE(list.Update(both, 1, &problem));
  list.TakeChanges();

  list.SetWebrtc("NOT_SET_UP", "stream server missing");
  std::vector<CameraChange> changes = list.TakeChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].camera_id, "stereo");
  EXPECT_EQ(changes[0].news.state, (CameraState{false, "NOT_SET_UP", "stream server missing"}));
  EXPECT_TRUE(list.State("oa_left").available);

  // What the camera says itself comes first.
  CameraEntry hot = Stereo();
  hot.available = false;
  hot.reason = "TOO_HOT";
  ASSERT_TRUE(list.Update(hot, 2, &problem));
  EXPECT_EQ(list.State("stereo").reason, "TOO_HOT");

  ASSERT_TRUE(list.Update(Stereo(), 3, &problem));
  list.SetWebrtc("", "ignored");
  EXPECT_EQ(list.State("stereo"), (CameraState{true, "", ""}));
}

TEST(CameraList, ReadsTheExitNoteOfTheLaunch) {
  double mono = 0;
  bool down = false;
  EXPECT_TRUE(ParseExitNote(R"({"exit_code":3,"restart":1,"max_restarts":3,"t":1700000000.5,"mono":812.25,
      "stalled":null,"down":false,"stable_s":600.0})", &mono, &down));
  EXPECT_DOUBLE_EQ(mono, 812.25);
  EXPECT_FALSE(down);
  EXPECT_TRUE(ParseExitNote(R"({"mono":5,"down":true})", &mono, &down));
  EXPECT_TRUE(down);
  EXPECT_FALSE(ParseExitNote(R"({"t":1700000000.5,"down":true})", &mono, &down));
  EXPECT_FALSE(ParseExitNote(R"({"mono":"5"})", &mono, &down));
  EXPECT_FALSE(ParseExitNote("[1]", &mono, &down));
  EXPECT_FALSE(ParseExitNote("{", &mono, &down));
  EXPECT_FALSE(ParseExitNote("", &mono, &down));
}

}  // namespace
}  // namespace mowbite_camera
