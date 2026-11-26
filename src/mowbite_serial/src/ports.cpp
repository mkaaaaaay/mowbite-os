#include "mowbite_serial/ports.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>

#include "mowbite_serial/serial_port.h"

namespace mowbite_serial {

namespace {

namespace fs = std::filesystem;

std::string ReadLine(const fs::path& path) {
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  return line;
}

// The USB device a tty belongs to: the nearest folder above its device with idVendor, at most four levels up.
std::optional<UsbInfo> UsbOf(const fs::path& device) {
  std::error_code error;
  fs::path at = fs::canonical(device, error);
  if (error) return std::nullopt;
  for (int level = 0; level < 5 && !at.empty() && at != at.root_path(); level++, at = at.parent_path()) {
    if (!fs::exists(at / "idVendor", error)) continue;
    return UsbInfo{ReadLine(at / "idVendor"), ReadLine(at / "idProduct"), ReadLine(at / "manufacturer"),
                   ReadLine(at / "product")};
  }
  return std::nullopt;
}

}  // namespace

std::vector<PortInfo> ListPorts(const std::string& sys, const std::string& dev) {
  std::vector<PortInfo> ports;
  std::error_code error;
  // The links of serial/by-id by the device they lead to.
  std::map<std::string, std::vector<std::string>> links;
  for (const auto& entry : fs::directory_iterator(fs::path(dev) / "serial" / "by-id", error)) {
    const fs::path target = fs::weakly_canonical(entry.path(), error);
    if (!error) links[target.filename().string()].push_back(entry.path().string());
  }
  for (const auto& entry : fs::directory_iterator(fs::path(sys) / "class" / "tty", error)) {
    const std::string name = entry.path().filename().string();
    const fs::path device = entry.path() / "device";
    if (!fs::exists(device, error)) continue;
    // A 8250 port without hardware behind it reports type 0.
    const fs::path type = entry.path() / "type";
    if (name.rfind("ttyS", 0) == 0 && fs::exists(type, error) && ReadLine(type) == "0") continue;
    PortInfo port;
    port.path = (fs::path(dev) / name).string();
    port.by_id = links[name];
    std::sort(port.by_id.begin(), port.by_id.end());
    const fs::path driver = device / "driver";
    if (fs::is_symlink(driver, error)) port.driver = fs::read_symlink(driver, error).filename().string();
    port.usb = UsbOf(device);
    ports.push_back(std::move(port));
  }
  std::sort(ports.begin(), ports.end(), [](const PortInfo& a, const PortInfo& b) { return a.path < b.path; });
  return ports;
}

std::vector<Holder> HoldersOf(const std::string& path) {
  std::vector<Holder> holders;
  for (int pid : ProcessesUsing(path)) holders.push_back({pid, ReadLine("/proc/" + std::to_string(pid) + "/comm")});
  return holders;
}

const std::vector<int>& ListenBauds() {
  static const std::vector<int> bauds = {115200, 9600, 38400, 57600, 230400, 460800, 921600, 19200};
  return bauds;
}

Listened Listen(const std::string& path, const std::vector<int>& bauds, double seconds,
                const std::atomic<bool>& stop) {
  using Clock = std::chrono::steady_clock;
  Listened listened;
  if (bauds.empty()) return listened;
  const double slot = std::max(1.0, seconds / static_cast<double>(bauds.size()));
  const auto span = [](double s) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
  };
  const Clock::time_point end = Clock::now() + span(seconds);
  uint8_t buffer[4096];
  for (int baud : bauds) {
    if (stop || Clock::now() >= end) break;
    SerialPort port;
    if (!port.Open(path, baud, listened.error, true)) return listened;
    Recognizer recognizer;
    const auto start = Clock::now();
    const Clock::time_point until = std::min(end, start + span(slot));
    while (!stop && Clock::now() < until) {
      const ssize_t n = port.Read(buffer, sizeof(buffer), 100);
      if (n < 0) {
        listened.error = "reading failed";
        return listened;
      }
      if (n > 0) recognizer.Feed(buffer, static_cast<size_t>(n));
    }
    port.Close();
    const Heard heard = recognizer.Result(std::chrono::duration<double>(Clock::now() - start).count());
    listened.tried.emplace_back(baud, heard);
    if (heard.kind != Kind::kSilent && heard.kind != Kind::kUnknown) {
      listened.baud = baud;
      listened.heard = heard;
      return listened;
    }
  }
  listened.stopped = stop;
  for (const auto& [baud, heard] : listened.tried) {
    if (heard.bytes > listened.heard.bytes || listened.heard.seconds == 0) listened.heard = heard;
  }
  return listened;
}

}  // namespace mowbite_serial
