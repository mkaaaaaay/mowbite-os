#ifndef TRON_MCU_SERIAL_PORT_H
#define TRON_MCU_SERIAL_PORT_H

#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace tron {

// Raw 8N1 port without flow control.
class SerialPort {
 public:
  SerialPort() = default;
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  ~SerialPort();

  // Also sets TIOCEXCL, so further opens fail for processes without CAP_SYS_ADMIN. Fails unless the port ends up raw
  // (no echo, no software flow control): the line discipline would send bytes of its own otherwise. read_only opens it
  // O_RDONLY and Write always fails.
  bool Open(const std::string& path, int baud, std::string& error, bool read_only = false);
  void Close();
  bool IsOpen() const {
    return fd_ >= 0;
  }

  // Waits up to timeout_ms for data. Returns the number of bytes read, 0 on timeout, -1 on error.
  ssize_t Read(uint8_t* buffer, size_t size, int timeout_ms);
  bool Write(const std::vector<uint8_t>& data);

 private:
  int fd_ = -1;
  bool read_only_ = false;
};

// Process ids (other than our own) that have the device open. Only processes in our pid namespace are visible.
std::vector<int> ProcessesUsing(const std::string& path);

}  // namespace tron

#endif  // TRON_MCU_SERIAL_PORT_H
