#include <gtest/gtest.h>

#include "tron_gnss/board_link.h"

namespace tron {
namespace {

const std::string kOn = kBoardRtkOn;
const std::string kReset = kBoardReset;

class BoardTest : public ::testing::Test {
 protected:
  BoardLink board{BoardConfig{}};
};

TEST_F(BoardTest, SilentWithoutRtcm) {
  board.OnPortOpened();
  EXPECT_EQ(board.Poll(0), "");
  EXPECT_EQ(board.Poll(100), "");
  EXPECT_FALSE(board.host_correction_age(100));
}

TEST_F(BoardTest, FirstRtcmGivesOnWithoutResetAndIsHeldUntilOn) {
  board.OnPortOpened();
  board.OnHeard();
  EXPECT_FALSE(board.OnRtcm(100, 1.0));
  EXPECT_EQ(board.dropped_bytes(), 100u);
  EXPECT_EQ(board.Poll(1.0), kOn);
  EXPECT_EQ(board.Poll(1.1), "");
  board.OnBoardReport(true, 2.4);
  EXPECT_EQ(board.state(), BoardState::kOn);
  EXPECT_TRUE(board.OnRtcm(200, 2.5));
  EXPECT_EQ(board.forwarded_bytes(), 200u);
  // The age starts anew only once the RTCM is written.
  EXPECT_FALSE(board.host_correction_age(3.0));
  board.OnWritten(2.5);
  EXPECT_NEAR(*board.host_correction_age(3.0), 0.5, 1e-9);
  EXPECT_EQ(board.resets(), 0u);
}

TEST_F(BoardTest, KeepAliveEveryFourSecondsWhileRtcmComes) {
  board.OnBoardReport(true, 0);
  int ons = 0;
  for (int i = 0; i <= 200; i++) {
    const double now = i * 0.1;
    if (i % 10 == 0) board.OnRtcm(50, now);
    if (board.Poll(now) == kOn) ons++;
  }
  // At 0, 4, 8, 12, 16 and 20 s.
  EXPECT_EQ(ons, 6);
}

TEST_F(BoardTest, KeepAliveStopsAfterTheWindowSoTheBoardReturnsToLora) {
  board.OnBoardReport(true, 0);
  board.OnRtcm(50, 0);
  EXPECT_EQ(board.Poll(0), kOn);
  EXPECT_EQ(board.Poll(8.0), kOn);
  EXPECT_TRUE(board.wanted(10.0));
  EXPECT_EQ(board.Poll(10.1), "");
  EXPECT_EQ(board.Poll(30.0), "");
  // RTCM again: ON at once.
  board.OnRtcm(50, 40.0);
  EXPECT_EQ(board.Poll(40.0), kOn);
}

TEST_F(BoardTest, FallBackFromOnGivesResetAndOn) {
  board.OnBoardReport(true, 0);
  board.OnRtcm(50, 0);
  EXPECT_EQ(board.Poll(0), kOn);
  board.OnBoardReport(false, 1.0);
  EXPECT_EQ(board.Poll(1.0), kReset + kOn);
  EXPECT_EQ(board.resets(), 1u);
  EXPECT_FALSE(board.OnRtcm(50, 1.2));
  EXPECT_EQ(board.Poll(1.2), "");
}

// A board that keeps falling back gets one reset a minute; ON goes on meanwhile.
TEST_F(BoardTest, ResetsAtMostOnceAMinute) {
  board.OnBoardReport(true, 0);
  board.OnRtcm(50, 0);
  EXPECT_EQ(board.Poll(0), kOn);
  board.OnBoardReport(false, 1.0);
  EXPECT_EQ(board.Poll(1.0), kReset + kOn);
  board.OnBoardReport(true, 3.0);
  board.OnRtcm(50, 4.0);
  board.OnBoardReport(false, 5.0);
  EXPECT_EQ(board.Poll(5.0), kOn);
  board.OnRtcm(50, 9.0);
  EXPECT_EQ(board.Poll(9.0), kOn);
  EXPECT_EQ(board.resets(), 1u);
  for (double t = 10.0; t < 61.0; t += 1.0) {
    board.OnRtcm(50, t);
    EXPECT_EQ(board.Poll(t).find(kReset), std::string::npos) << t;
  }
  board.OnRtcm(50, 61.0);
  EXPECT_EQ(board.Poll(61.0), kReset + kOn);
  EXPECT_EQ(board.resets(), 2u);
}

TEST_F(BoardTest, OnAgainBeforeTheGapDropsTheWaitingReset) {
  board.OnBoardReport(true, 0);
  board.OnRtcm(50, 0);
  board.Poll(0);
  board.OnBoardReport(false, 1.0);
  EXPECT_EQ(board.Poll(1.0), kReset + kOn);
  board.OnBoardReport(true, 3.0);
  board.OnBoardReport(false, 5.0);
  board.OnBoardReport(true, 6.0);
  for (double t = 5.0; t <= 70.0; t += 1.0) {
    board.OnRtcm(50, t);
    EXPECT_EQ(board.Poll(t).find(kReset), std::string::npos) << t;
  }
  EXPECT_EQ(board.resets(), 1u);
}

TEST_F(BoardTest, FirstOffOnlyWaitsForOn) {
  board.OnPortOpened();
  board.OnRtcm(50, 0);
  board.OnBoardReport(false, 0.1);
  EXPECT_EQ(board.Poll(0.1), kOn);
  EXPECT_EQ(board.resets(), 0u);
}

TEST_F(BoardTest, NoResetWhenNothingIsWanted) {
  board.OnBoardReport(true, 0);
  board.OnBoardReport(false, 50);
  EXPECT_EQ(board.Poll(50), "");
  EXPECT_EQ(board.resets(), 0u);
}

TEST_F(BoardTest, NoResetWhenSwitchedOff) {
  BoardConfig config;
  config.reset_on_off = false;
  BoardLink link(config);
  link.OnBoardReport(true, 0);
  link.OnRtcm(50, 0);
  link.Poll(0);
  link.OnBoardReport(false, 1);
  EXPECT_EQ(link.Poll(4.0), kOn);
  EXPECT_EQ(link.resets(), 0u);
}

TEST_F(BoardTest, ReopenedPortMakesTheStateUnknownAndOnGoesOutAgain) {
  board.OnBoardReport(true, 0);
  board.OnRtcm(50, 0);
  EXPECT_EQ(board.Poll(0), kOn);
  board.OnPortOpened();
  EXPECT_EQ(board.state(), BoardState::kUnknown);
  EXPECT_FALSE(board.OnRtcm(50, 1));
  EXPECT_EQ(board.Poll(1), "");
  board.OnHeard();
  EXPECT_EQ(board.Poll(1), kOn);
}

TEST_F(BoardTest, WithoutGateEverythingIsForwarded) {
  BoardConfig config;
  config.gate = false;
  BoardLink link(config);
  link.OnHeard();
  EXPECT_TRUE(link.OnRtcm(50, 0));
  EXPECT_EQ(link.Poll(0), kOn);
}

TEST_F(BoardTest, NothingGoesToABoardNotHeard) {
  BoardConfig config;
  config.gate = false;
  BoardLink link(config);
  link.OnPortOpened();
  EXPECT_FALSE(link.OnRtcm(50, 0));
  EXPECT_EQ(link.dropped_bytes(), 50u);
  EXPECT_EQ(link.Poll(0), "");
  EXPECT_EQ(link.Poll(5), "");
  link.OnHeard();
  EXPECT_TRUE(link.OnRtcm(50, 6));
  EXPECT_EQ(link.Poll(6), kOn);
  // A reopened port is unheard again; a board report counts as heard.
  link.OnPortOpened();
  EXPECT_FALSE(link.OnRtcm(50, 7));
  EXPECT_EQ(link.Poll(7), "");
  link.OnBoardReport(false, 7.5);
  EXPECT_EQ(link.Poll(7.5), kOn);
}

}  // namespace
}  // namespace tron
