#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>

#include "mowbite_camera/seats.h"
#include "test_util.h"

namespace mowbite_camera {
namespace {

TEST(Seats, ATokenIsHexOfTheRandomBytes) {
  Seats seats(Counting());
  std::string token;
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &token), Seats::Granted::kOk);
  EXPECT_EQ(token, "000102030405060708090a0b0c0d0e0f");
}

TEST(Seats, TheSystemGivesDifferentTokens) {
  Seats seats;
  std::string a;
  std::string b;
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &a), Seats::Granted::kOk);
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &b), Seats::Granted::kOk);
  EXPECT_EQ(a.size(), 2 * kTokenBytes);
  EXPECT_TRUE(std::all_of(a.begin(), a.end(), [](unsigned char c) { return std::isxdigit(c); }));
  EXPECT_NE(a, b);
}

TEST(Seats, NoTokenWithoutRandomBytes) {
  Seats seats([](uint8_t*, size_t) { return false; });
  std::string token;
  EXPECT_EQ(seats.Grant("cam", "mjpeg", 0, &token), Seats::Granted::kNoRandom);
  EXPECT_TRUE(seats.seats().empty());
}

TEST(Seats, TwoPlacesOverAllCamerasAndProtocols) {
  Seats seats(Counting());
  std::string token;
  EXPECT_EQ(seats.Grant("front", "mjpeg", 0, &token), Seats::Granted::kOk);
  EXPECT_EQ(seats.Grant("stereo", "webrtc", 0, &token), Seats::Granted::kOk);
  EXPECT_EQ(seats.Grant("front", "mjpeg", 0, &token), Seats::Granted::kBusy);
  EXPECT_EQ(seats.Grant("rear", "webrtc", 0, &token), Seats::Granted::kBusy);
}

TEST(Seats, AnUnusedTokenGivesItsPlaceBackWhenItExpires) {
  Seats seats(Counting());
  std::string first;
  std::string token;
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 100, &first), Seats::Granted::kOk);
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 110, &token), Seats::Granted::kOk);
  EXPECT_EQ(seats.Grant("cam", "mjpeg", 100 + kTokenSeconds, &token), Seats::Granted::kBusy);
  EXPECT_EQ(seats.Grant("cam", "mjpeg", 100.1 + kTokenSeconds, &token), Seats::Granted::kOk);
  EXPECT_FALSE(seats.Connect(first, "cam", "mjpeg", "1", 100.1 + kTokenSeconds));
}

TEST(Seats, ATokenIsForOneConnectionOfItsCameraAndProtocol) {
  Seats seats(Counting());
  std::string token;
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &token), Seats::Granted::kOk);
  EXPECT_TRUE(seats.Waiting(token, "cam", "mjpeg", 1));
  EXPECT_FALSE(seats.Waiting(token, "other", "mjpeg", 1));
  EXPECT_FALSE(seats.Connect(token, "other", "mjpeg", "1", 1));
  EXPECT_FALSE(seats.Connect(token, "cam", "webrtc", "1", 1));
  EXPECT_FALSE(seats.Connect(token, "cam", "mjpeg", "", 1));
  EXPECT_FALSE(seats.Connect(token + "0", "cam", "mjpeg", "1", 1));
  EXPECT_FALSE(seats.Connect("", "cam", "mjpeg", "1", 1));
  EXPECT_TRUE(seats.Connect(token, "cam", "mjpeg", "1", 1));
  EXPECT_FALSE(seats.Waiting(token, "cam", "mjpeg", 1));
  // The same connection may ask again, another one not.
  EXPECT_TRUE(seats.Connect(token, "cam", "mjpeg", "1", 2));
  EXPECT_FALSE(seats.Connect(token, "cam", "mjpeg", "2", 2));
}

TEST(Seats, ATokenExpiresUnlessConnected) {
  Seats seats(Counting());
  std::string late;
  std::string used;
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &late), Seats::Granted::kOk);
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &used), Seats::Granted::kOk);
  ASSERT_TRUE(seats.Connect(used, "cam", "mjpeg", "7", kTokenSeconds));
  EXPECT_FALSE(seats.Waiting(late, "cam", "mjpeg", kTokenSeconds + 0.1));
  EXPECT_FALSE(seats.Connect(late, "cam", "mjpeg", "8", kTokenSeconds + 0.1));
  // A connection keeps its place past the time of its token.
  seats.Expire(1000);
  ASSERT_EQ(seats.seats().size(), 1u);
  EXPECT_EQ(seats.seats()[0].connection, "7");
}

TEST(Seats, TheEndOfAConnectionFreesItsPlace) {
  Seats seats(Counting());
  std::string a;
  std::string b;
  std::string c;
  ASSERT_EQ(seats.Grant("cam", "mjpeg", 0, &a), Seats::Granted::kOk);
  ASSERT_EQ(seats.Grant("cam", "webrtc", 0, &b), Seats::Granted::kOk);
  ASSERT_TRUE(seats.Connect(a, "cam", "mjpeg", "1", 0));
  ASSERT_TRUE(seats.Connect(b, "cam", "webrtc", "1", 0));
  EXPECT_FALSE(seats.Release("mjpeg", "2"));
  EXPECT_FALSE(seats.Release("mjpeg", ""));
  EXPECT_TRUE(seats.Release("mjpeg", "1"));
  ASSERT_EQ(seats.seats().size(), 1u);
  EXPECT_EQ(seats.seats()[0].protocol, "webrtc");
  EXPECT_EQ(seats.Grant("cam", "mjpeg", 0, &c), Seats::Granted::kOk);
  // The token of the ended connection is used up.
  EXPECT_FALSE(seats.Connect(a, "cam", "mjpeg", "3", 0));
}

TEST(Seats, DroppingACameraOrAProtocolReturnsTheConnections) {
  Seats seats(Counting());
  std::string a;
  std::string b;
  ASSERT_EQ(seats.Grant("front", "mjpeg", 0, &a), Seats::Granted::kOk);
  ASSERT_EQ(seats.Grant("stereo", "webrtc", 0, &b), Seats::Granted::kOk);
  ASSERT_TRUE(seats.Connect(b, "stereo", "webrtc", "session", 0));
  EXPECT_TRUE(seats.DropCamera("front").empty());
  EXPECT_FALSE(seats.Waiting(a, "front", "mjpeg", 0));
  const std::vector<Seat> dropped = seats.DropProtocol("webrtc");
  ASSERT_EQ(dropped.size(), 1u);
  EXPECT_EQ(dropped[0].connection, "session");
  EXPECT_TRUE(seats.seats().empty());
}

TEST(Seats, DroppingOneProtocolOfACameraKeepsItsOthers) {
  Seats seats(Counting());
  std::string a;
  std::string b;
  ASSERT_EQ(seats.Grant("front", "mjpeg", 0, &a), Seats::Granted::kOk);
  ASSERT_EQ(seats.Grant("front", "webrtc", 0, &b), Seats::Granted::kOk);
  ASSERT_TRUE(seats.Connect(b, "front", "webrtc", "session", 0));
  const std::vector<Seat> dropped = seats.DropCamera("front", "webrtc");
  ASSERT_EQ(dropped.size(), 1u);
  EXPECT_EQ(dropped[0].connection, "session");
  EXPECT_TRUE(seats.Waiting(a, "front", "mjpeg", 0));
}

TEST(Seats, SameTextComparesWholeTexts) {
  EXPECT_TRUE(SameText("", ""));
  EXPECT_TRUE(SameText("abc", "abc"));
  EXPECT_FALSE(SameText("abc", "abd"));
  EXPECT_FALSE(SameText("abc", "ab"));
}

}  // namespace
}  // namespace mowbite_camera
