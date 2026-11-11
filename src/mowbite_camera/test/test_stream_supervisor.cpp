#include <gtest/gtest.h>

#include "mowbite_camera/camera_list.h"
#include "mowbite_camera/stream_supervisor.h"

namespace mowbite_camera {
namespace {

class FakeServer : public StreamServer {
 public:
  bool Installed() override {
    return installed;
  }
  bool Start(std::string* error) override {
    ++starts;
    *error = start_error;
    running = start_error.empty();
    return running;
  }
  bool Running(std::string* how) override {
    if (!running) *how = "exit code 1";
    return running;
  }
  void Stop() override {
    ++stops;
    running = false;
  }

  bool installed = true;
  bool running = false;
  int starts = 0;
  int stops = 0;
  std::string start_error;
};

class StreamSupervisorTest : public ::testing::Test {
 protected:
  // A tick every second for this long; how often the server ended meanwhile.
  int Run(double seconds, bool wanted = true) {
    int ended = 0;
    for (double t = 0; t < seconds; t += 1.0) {
      now_ += 1.0;
      ended += supervisor_.Tick(wanted) ? 1 : 0;
    }
    return ended;
  }

  // The server ends, and the supervisor has started the next one.
  void EndAndStartAgain() {
    server_.running = false;
    EXPECT_EQ(Run(kStreamRestartDelay + 1.0), 1);
    EXPECT_TRUE(supervisor_.Starting());
  }

  double now_ = 100.0;
  FakeServer server_;
  StreamSupervisor supervisor_{&server_, [this] { return now_; }};
};

TEST_F(StreamSupervisorTest, NothingStartsBeforeACameraNeedsIt) {
  EXPECT_EQ(Run(5.0, false), 0);
  EXPECT_EQ(server_.starts, 0);
  EXPECT_EQ(supervisor_.reason(), kNoSignal);
  EXPECT_FALSE(supervisor_.Starting());
  EXPECT_FALSE(supervisor_.Up());
}

TEST_F(StreamSupervisorTest, StartsWithTheFirstCameraAndIsUpOnceItsApiAnswers) {
  EXPECT_FALSE(supervisor_.Tick(true));
  EXPECT_EQ(server_.starts, 1);
  EXPECT_TRUE(supervisor_.Starting());
  EXPECT_EQ(supervisor_.reason(), kNoSignal);
  EXPECT_EQ(supervisor_.detail(), "the stream server is starting");
  supervisor_.Answered();
  EXPECT_TRUE(supervisor_.Up());
  EXPECT_EQ(supervisor_.reason(), "");
  EXPECT_EQ(supervisor_.detail(), "");
  EXPECT_EQ(Run(kStreamStartSeconds + 5.0), 0);
  EXPECT_EQ(server_.starts, 1);
  EXPECT_EQ(server_.stops, 0);
}

TEST_F(StreamSupervisorTest, WithoutItsProgramWebrtcIsNotSetUpUntilItIsThere) {
  server_.installed = false;
  EXPECT_EQ(Run(3.0), 0);
  EXPECT_EQ(server_.starts, 0);
  EXPECT_EQ(supervisor_.reason(), kNotSetUp);
  EXPECT_EQ(supervisor_.detail(), "the stream server is not installed");
  server_.installed = true;
  Run(1.0);
  EXPECT_EQ(server_.starts, 1);
  EXPECT_EQ(supervisor_.reason(), kNoSignal);
}

TEST_F(StreamSupervisorTest, StartsAgainAfterAnEndButNotAtOnce) {
  supervisor_.Tick(true);
  supervisor_.Answered();
  server_.running = false;
  EXPECT_TRUE(supervisor_.Tick(true));
  EXPECT_EQ(supervisor_.reason(), kNoSignal);
  EXPECT_EQ(supervisor_.detail(), "the stream server ended with exit code 1 and starts again");
  EXPECT_FALSE(supervisor_.Up());
  EXPECT_EQ(server_.starts, 1);
  EXPECT_EQ(Run(kStreamRestartDelay - 1.0), 0);
  EXPECT_EQ(server_.starts, 1);
  EXPECT_EQ(Run(1.0), 0);
  EXPECT_EQ(server_.starts, 2);
  EXPECT_TRUE(supervisor_.Starting());
}

TEST_F(StreamSupervisorTest, GivesUpAfterThreeRestartsInARow) {
  supervisor_.Tick(true);
  for (int restart = 0; restart < kStreamRestarts; ++restart) EndAndStartAgain();
  EXPECT_EQ(server_.starts, 1 + kStreamRestarts);
  supervisor_.Answered();
  server_.running = false;
  EXPECT_EQ(Run(1.0), 1);
  EXPECT_EQ(supervisor_.reason(), kNotSetUp);
  EXPECT_EQ(supervisor_.detail(), "the stream server ended 4 times in a row, last with exit code 1");
  EXPECT_EQ(Run(60.0), 0);
  EXPECT_EQ(server_.starts, 1 + kStreamRestarts);
  supervisor_.Answered();
  EXPECT_FALSE(supervisor_.Up());
}

TEST_F(StreamSupervisorTest, AStableRunCountsTheEndsFromZero) {
  supervisor_.Tick(true);
  for (int restart = 0; restart < kStreamRestarts; ++restart) EndAndStartAgain();
  supervisor_.Answered();
  EXPECT_EQ(Run(kStreamStableAfter), 0);
  for (int restart = 0; restart < kStreamRestarts; ++restart) EndAndStartAgain();
  EXPECT_EQ(server_.starts, 1 + 2 * kStreamRestarts);
}

TEST_F(StreamSupervisorTest, AStartWhoseApiStaysSilentIsEnded) {
  supervisor_.Tick(true);
  EXPECT_EQ(Run(kStreamStartSeconds - 1.0), 0);
  EXPECT_EQ(server_.stops, 0);
  EXPECT_EQ(Run(1.0), 1);
  EXPECT_EQ(server_.stops, 1);
  EXPECT_EQ(supervisor_.detail(), "the stream server ended with no answer of its API and starts again");
  EXPECT_EQ(Run(kStreamRestartDelay), 0);
  EXPECT_EQ(server_.starts, 2);
}

TEST_F(StreamSupervisorTest, AStartThatFailsCountsAsAnEnd) {
  server_.start_error = "fork: no memory";
  EXPECT_FALSE(supervisor_.Tick(true));
  EXPECT_EQ(supervisor_.detail(), "the stream server ended with fork: no memory and starts again");
  EXPECT_EQ(Run(60.0), 0);
  EXPECT_EQ(server_.starts, 1 + kStreamRestarts);
  EXPECT_EQ(supervisor_.reason(), kNotSetUp);
}

}  // namespace
}  // namespace mowbite_camera
