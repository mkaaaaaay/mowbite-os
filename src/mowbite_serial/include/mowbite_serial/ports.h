#ifndef MOWBITE_SERIAL_PORTS_H
#define MOWBITE_SERIAL_PORTS_H

#include <atomic>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_serial/recognize.h"

namespace mowbite_serial {

struct UsbInfo {
  // Four hex digits each, as lsusb shows them.
  std::string vendor_id;
  std::string product_id;
  std::string manufacturer;
  std::string product;
};

struct PortInfo {
  // As <dev>/<name>, e.g. /dev/ttyUSB0.
  std::string path;
  // The links of <dev>/serial/by-id that lead to it.
  std::vector<std::string> by_id;
  // The kernel driver of the device behind it, e.g. ftdi_sio or dw-apb-uart.
  std::string driver;
  std::optional<UsbInfo> usb;
};

// The serial devices of the machine, read from <sys>/class/tty without opening any: every tty with a device behind
// it, a 8250 one (ttyS*) only where its UART type is known; sorted by path.
std::vector<PortInfo> ListPorts(const std::string& sys = "/sys", const std::string& dev = "/dev");

struct Holder {
  int pid = 0;
  std::string name;
};

// The processes besides this one that have the device open, with the name of each.
std::vector<Holder> HoldersOf(const std::string& path);

// The rates Listen tries, the usual ones first.
const std::vector<int>& ListenBauds();

struct Listened {
  // Why the port could not be opened, empty when it was.
  std::string error;
  // The rate it was recognized at, 0 for none.
  int baud = 0;
  // At that rate, else at the rate with the most bytes.
  Heard heard;
  // Each rate tried, in order.
  std::vector<std::pair<int, Heard>> tried;
  // stop was set before the end.
  bool stopped = false;
};

// Listens on the port read-only, at each rate of bauds in turn for an equal share of seconds (at least 1 s), and stops
// at the first rate a kind is recognized at; stop ends it early. Never writes.
Listened Listen(const std::string& path, const std::vector<int>& bauds, double seconds,
                const std::atomic<bool>& stop);

}  // namespace mowbite_serial

#endif  // MOWBITE_SERIAL_PORTS_H
