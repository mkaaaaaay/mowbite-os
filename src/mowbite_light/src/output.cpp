#include "mowbite_light/output.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "mowbite_hold/file_io.h"

namespace mowbite_light {

namespace {

constexpr uint32_t kSpiHz = 8000000;
constexpr uint8_t kSpiBits = 8;

std::string Errno(const std::string& what) {
  char text[128];
  // The GNU strerror_r gives the text, in text or a static string.
  return what + ": " + strerror_r(errno, text, sizeof(text));
}

class SpidevOutput : public Output {
 public:
  explicit SpidevOutput(int fd) : fd_(fd) {}
  ~SpidevOutput() override { ::close(fd_); }

  bool Write(const std::vector<uint8_t>& frame, std::string* error) override {
    spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<uintptr_t>(frame.data());
    transfer.len = static_cast<uint32_t>(frame.size());
    transfer.speed_hz = kSpiHz;
    transfer.bits_per_word = kSpiBits;
    if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      *error = Errno("SPI_IOC_MESSAGE");
      return false;
    }
    return true;
  }

 private:
  int fd_;
};

class FileOutput : public Output {
 public:
  explicit FileOutput(std::string path) : path_(std::move(path)) {}

  bool Write(const std::vector<uint8_t>& frame, std::string* error) override {
    return mowbite_hold::WriteFileAtomic(path_, std::string(frame.begin(), frame.end()), error);
  }

 private:
  std::string path_;
};

}  // namespace

std::unique_ptr<Output> OpenSpidev(const std::string& device, std::string* error) {
  const int fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    *error = Errno("cannot open " + device);
    return nullptr;
  }
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = kSpiBits;
  uint32_t hz = kSpiHz;
  if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
    *error = Errno("cannot set up " + device);
    ::close(fd);
    return nullptr;
  }
  return std::make_unique<SpidevOutput>(fd);
}

std::unique_ptr<Output> OpenFile(const std::string& path) {
  return std::make_unique<FileOutput>(path);
}

}  // namespace mowbite_light
