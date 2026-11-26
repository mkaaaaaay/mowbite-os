#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "mowbite_serial/ports.h"

namespace mowbite_serial {
namespace {

namespace fs = std::filesystem;

class TempDir {
 public:
  TempDir() {
    char name[] = "/tmp/mowbite_ports_XXXXXX";
    path_ = mkdtemp(name);
  }
  ~TempDir() {
    std::error_code error;
    fs::remove_all(path_, error);
  }
  const fs::path& path() const {
    return path_;
  }

 private:
  fs::path path_;
};

void Write(const fs::path& path, const std::string& text) {
  fs::create_directories(path.parent_path());
  std::ofstream(path) << text << "\n";
}

// A sysfs and /dev of a machine with one USB adapter, one real and one empty 8250 port and a console.
struct Machine {
  Machine() {
    const fs::path sys = dir.path() / "sys";
    const fs::path usb = sys / "devices/usb1/1-1";
    Write(usb / "idVendor", "0403");
    Write(usb / "idProduct", "6001");
    Write(usb / "manufacturer", "FTDI");
    Write(usb / "product", "FT232R USB UART");
    fs::create_directories(usb / "1-1:1.0/ttyUSB0");
    fs::create_directories(sys / "bus/usb-serial/drivers/ftdi_sio");
    fs::create_directory_symlink(sys / "bus/usb-serial/drivers/ftdi_sio", usb / "1-1:1.0/ttyUSB0/driver");
    const fs::path tty = sys / "class/tty";
    fs::create_directories(tty / "ttyUSB0");
    fs::create_directory_symlink(usb / "1-1:1.0/ttyUSB0", tty / "ttyUSB0/device");
    fs::create_directories(sys / "devices/platform/uart4");
    fs::create_directories(tty / "ttyS4");
    fs::create_directory_symlink(sys / "devices/platform/uart4", tty / "ttyS4/device");
    Write(tty / "ttyS4/type", "4");
    fs::create_directories(sys / "devices/platform/serial8250");
    fs::create_directories(tty / "ttyS0");
    fs::create_directory_symlink(sys / "devices/platform/serial8250", tty / "ttyS0/device");
    Write(tty / "ttyS0/type", "0");
    fs::create_directories(tty / "tty0");
    const fs::path dev = dir.path() / "dev";
    Write(dev / "ttyUSB0", "");
    fs::create_directories(dev / "serial/by-id");
    fs::create_symlink("../../ttyUSB0", dev / "serial/by-id/usb-FTDI_FT232R_USB_UART_A1-if00-port0");
  }
  std::string sys() const {
    return (dir.path() / "sys").string();
  }
  std::string dev() const {
    return (dir.path() / "dev").string();
  }
  TempDir dir;
};

TEST(Ports, ListsDevicesWithUsbAndLinks) {
  const Machine machine;
  const auto ports = ListPorts(machine.sys(), machine.dev());
  ASSERT_EQ(ports.size(), 2u);
  EXPECT_EQ(ports[0].path, machine.dev() + "/ttyS4");
  EXPECT_FALSE(ports[0].usb);
  EXPECT_TRUE(ports[0].by_id.empty());
  EXPECT_EQ(ports[1].path, machine.dev() + "/ttyUSB0");
  EXPECT_EQ(ports[1].driver, "ftdi_sio");
  ASSERT_TRUE(ports[1].usb);
  EXPECT_EQ(ports[1].usb->vendor_id, "0403");
  EXPECT_EQ(ports[1].usb->product_id, "6001");
  EXPECT_EQ(ports[1].usb->manufacturer, "FTDI");
  EXPECT_EQ(ports[1].usb->product, "FT232R USB UART");
  ASSERT_EQ(ports[1].by_id.size(), 1u);
  EXPECT_NE(ports[1].by_id[0].find("usb-FTDI_FT232R"), std::string::npos);
}

TEST(Ports, NoSysfsNoPorts) {
  EXPECT_TRUE(ListPorts("/nonexistent/sys", "/nonexistent/dev").empty());
}

// A port as the pty of a device: the test writes to its master side.
class Pty {
 public:
  Pty() {
    char name[256];
    if (openpty(&master_, &slave_, name, nullptr, nullptr) == 0) path_ = name;
  }
  ~Pty() {
    if (master_ >= 0) close(master_);
    if (slave_ >= 0) close(slave_);
  }
  const std::string& path() const {
    return path_;
  }
  void Send(const std::string& text) const {
    ASSERT_EQ(write(master_, text.data(), text.size()), static_cast<ssize_t>(text.size()));
  }

 private:
  int master_ = -1;
  int slave_ = -1;
  std::string path_;
};

std::string Gga() {
  const std::string body = "GNGGA,120000.00,5200.0,N,00800.0,E,1,12,0.8,60.0,M,47.0,M,,";
  unsigned sum = 0;
  for (char c : body) sum ^= static_cast<uint8_t>(c);
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
  return "$" + body + tail;
}

TEST(Ports, ListenRecognizesNmeaAndStopsThere) {
  Pty pty;
  ASSERT_FALSE(pty.path().empty());
  std::atomic<bool> stop{false};
  std::atomic<bool> sending{true};
  std::thread sender([&] {
    while (sending) {
      pty.Send(Gga());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  const auto start = std::chrono::steady_clock::now();
  const Listened listened = Listen(pty.path(), {115200, 9600}, 4, stop);
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  sending = false;
  sender.join();
  EXPECT_TRUE(listened.error.empty()) << listened.error;
  EXPECT_EQ(listened.baud, 115200);
  EXPECT_EQ(listened.heard.kind, Kind::kNmea);
  EXPECT_EQ(listened.heard.fix, 1);
  EXPECT_EQ(listened.tried.size(), 1u);
  EXPECT_LT(took, 3.0);
}

TEST(Ports, ListenOnAQuietPortTriesEachRate) {
  Pty pty;
  ASSERT_FALSE(pty.path().empty());
  std::atomic<bool> stop{false};
  const Listened listened = Listen(pty.path(), {115200, 9600}, 2, stop);
  EXPECT_TRUE(listened.error.empty());
  EXPECT_EQ(listened.baud, 0);
  EXPECT_EQ(listened.heard.kind, Kind::kSilent);
  ASSERT_EQ(listened.tried.size(), 2u);
  EXPECT_EQ(listened.tried[1].first, 9600);
  EXPECT_FALSE(listened.stopped);
}

TEST(Ports, ListenEndsOnStop) {
  Pty pty;
  std::atomic<bool> stop{false};
  std::thread stopper([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    stop = true;
  });
  const auto start = std::chrono::steady_clock::now();
  const Listened listened = Listen(pty.path(), ListenBauds(), 20, stop);
  stopper.join();
  EXPECT_TRUE(listened.stopped);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), 2.0);
}

TEST(Ports, ListenOnAMissingPortTellsWhy) {
  std::atomic<bool> stop{false};
  const Listened listened = Listen("/dev/mowbite-no-such-port", {115200}, 1, stop);
  EXPECT_FALSE(listened.error.empty());
  EXPECT_TRUE(listened.tried.empty());
}

TEST(Ports, NobodyHoldsAFreshPty) {
  Pty pty;
  EXPECT_TRUE(HoldersOf(pty.path()).empty());
}

}  // namespace
}  // namespace mowbite_serial
