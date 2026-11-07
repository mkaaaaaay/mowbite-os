#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "tron_mcu/serial_port.h"

namespace {

// A pty whose far end stands for the MCU; its near end starts with echo on, like a fresh tty.
class PtyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char name[128];
    ASSERT_EQ(openpty(&mcu_, &host_, name, nullptr, nullptr), 0);
    path_ = name;
    termios tio{};
    ASSERT_EQ(tcgetattr(host_, &tio), 0);
    tio.c_lflag |= ECHO | ICANON;
    tio.c_iflag |= IXOFF;
    ASSERT_EQ(tcsetattr(host_, TCSANOW, &tio), 0);
  }

  void TearDown() override {
    close(mcu_);
    close(host_);
  }

  // What the MCU side receives within timeout_ms.
  std::string Received(int timeout_ms) {
    std::string out;
    pollfd pfd{mcu_, POLLIN, 0};
    while (poll(&pfd, 1, timeout_ms) > 0) {
      char buffer[64];
      ssize_t n = read(mcu_, buffer, sizeof(buffer));
      if (n <= 0) break;
      out.append(buffer, static_cast<size_t>(n));
    }
    return out;
  }

  int mcu_ = -1;
  int host_ = -1;
  std::string path_;
};

TEST_F(PtyTest, ReadOnlyReadsAndSendsNothing) {
  tron::SerialPort port;
  std::string error;
  ASSERT_TRUE(port.Open(path_, 115200, error, true)) << error;
  const std::string frame = "\xA5\x08\x0A\x5A\n";
  ASSERT_EQ(write(mcu_, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));
  uint8_t buffer[64];
  ssize_t n = port.Read(buffer, sizeof(buffer), 200);
  ASSERT_EQ(n, static_cast<ssize_t>(frame.size()));
  EXPECT_EQ(std::string(reinterpret_cast<char*>(buffer), static_cast<size_t>(n)), frame);
  EXPECT_FALSE(port.Write({0xA5, 0x00, 0x5A}));
  // No echo of the received bytes and nothing written.
  EXPECT_EQ(Received(200), "");
  termios tio{};
  ASSERT_EQ(tcgetattr(host_, &tio), 0);
  EXPECT_EQ(tio.c_lflag & (ECHO | ICANON), 0u);
  EXPECT_EQ(tio.c_iflag & (IXON | IXOFF), 0u);
}

TEST_F(PtyTest, ReadWriteWrites) {
  tron::SerialPort port;
  std::string error;
  ASSERT_TRUE(port.Open(path_, 115200, error)) << error;
  EXPECT_TRUE(port.Write({0xA5, 0x00, 0x5A}));
  EXPECT_EQ(Received(200), std::string("\xA5\x00\x5A", 3));
}

}  // namespace
