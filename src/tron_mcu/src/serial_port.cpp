#include "tron_mcu/serial_port.h"

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <climits>
#include <cstring>

#include "mowbite_hold/file_io.h"

namespace tron {

namespace {

bool BaudConstant(int baud, speed_t& out) {
  switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 921600: out = B921600; return true;
    default: return false;
  }
}

std::string RealPath(const std::string& path) {
  char resolved[PATH_MAX];
  if (realpath(path.c_str(), resolved) == nullptr) return path;
  return resolved;
}

bool IsNumber(const char* s) {
  if (*s == '\0') return false;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
  }
  return true;
}

}  // namespace

SerialPort::~SerialPort() {
  Close();
}

bool SerialPort::Open(const std::string& path, int baud, std::string& error, bool read_only) {
  Close();
  speed_t speed;
  if (!BaudConstant(baud, speed)) {
    error = "unsupported baud rate " + std::to_string(baud);
    return false;
  }
  int fd = open(path.c_str(), (read_only ? O_RDONLY : O_RDWR) | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    error = "open " + path + ": " + mowbite_hold::ErrorText(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    error = "tcgetattr: " + mowbite_hold::ErrorText(errno);
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    error = "tcsetattr: " + mowbite_hold::ErrorText(errno);
    ::close(fd);
    return false;
  }
  // tcsetattr succeeds when any of the changes took, so read back what the port does now.
  termios now{};
  if (tcgetattr(fd, &now) != 0 || (now.c_lflag & (ECHO | ECHONL | ICANON | ISIG | IEXTEN)) != 0 ||
      (now.c_iflag & (IXON | IXOFF | IXANY)) != 0) {
    error = "the port did not become raw (echo or flow control left on)";
    ::close(fd);
    return false;
  }
  if (ioctl(fd, TIOCEXCL) != 0) {
    error = "TIOCEXCL: " + mowbite_hold::ErrorText(errno);
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  read_only_ = read_only;
  return true;
}

void SerialPort::Close() {
  if (fd_ < 0) return;
  ioctl(fd_, TIOCNXCL);
  ::close(fd_);
  fd_ = -1;
}

ssize_t SerialPort::Read(uint8_t* buffer, size_t size, int timeout_ms) {
  if (fd_ < 0) return -1;
  pollfd pfd{fd_, POLLIN, 0};
  int ready = poll(&pfd, 1, timeout_ms);
  if (ready < 0) return errno == EINTR ? 0 : -1;
  if (ready == 0) return 0;
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
  ssize_t n = read(fd_, buffer, size);
  if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
  return n;
}

bool SerialPort::Write(const std::vector<uint8_t>& data) {
  if (fd_ < 0 || read_only_) return false;
  size_t done = 0;
  while (done < data.size()) {
    ssize_t n = write(fd_, data.data() + done, data.size() - done);
    if (n > 0) {
      done += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) return false;
    pollfd pfd{fd_, POLLOUT, 0};
    if (poll(&pfd, 1, 100) <= 0) return false;
  }
  return true;
}

std::vector<int> ProcessesUsing(const std::string& path) {
  std::vector<int> pids;
  const std::string target = RealPath(path);
  const int self = getpid();
  DIR* proc = opendir("/proc");
  if (proc == nullptr) return pids;
  // readdir is safe for a stream no other thread uses; both streams here are local.
  while (dirent* entry = readdir(proc)) {  // NOLINT(concurrency-mt-unsafe)
    if (!IsNumber(entry->d_name)) continue;
    int pid = 0;
    const char* name = entry->d_name;
    if (std::from_chars(name, name + std::strlen(name), pid).ec != std::errc()) continue;
    if (pid == self) continue;
    std::string fd_dir = std::string("/proc/") + entry->d_name + "/fd";
    DIR* fds = opendir(fd_dir.c_str());
    if (fds == nullptr) continue;
    while (dirent* fd_entry = readdir(fds)) {  // NOLINT(concurrency-mt-unsafe)
      if (!IsNumber(fd_entry->d_name)) continue;
      char link[PATH_MAX];
      std::string fd_path = fd_dir + "/" + fd_entry->d_name;
      ssize_t len = readlink(fd_path.c_str(), link, sizeof(link) - 1);
      if (len <= 0) continue;
      link[len] = '\0';
      if (target == link) {
        pids.push_back(pid);
        break;
      }
    }
    closedir(fds);
  }
  closedir(proc);
  return pids;
}

}  // namespace tron
