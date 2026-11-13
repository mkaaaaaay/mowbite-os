#include "mowbite_bridge/system_stats.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/statvfs.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <vector>

namespace mowbite_bridge {

namespace {

std::vector<std::string> Lines(const std::string& path) {
  std::ifstream in(path);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

std::vector<std::string> Words(const std::string& line) {
  std::istringstream in(line);
  std::vector<std::string> words;
  for (std::string word; in >> word;) words.push_back(word);
  return words;
}

bool Number(const std::string& text, double* value) {
  try {
    std::size_t used = 0;
    *value = std::stod(text, &used);
    return used > 0;
  } catch (const std::exception&) {
    return false;
  }
}

void Memory(const std::string& proc, Json* out) {
  for (const auto& line : Lines(proc + "/meminfo")) {
    const auto words = Words(line);
    double kib = 0;
    if (words.size() < 2 || !Number(words[1], &kib)) continue;
    if (words[0] == "MemTotal:") (*out)["mem_total"] = static_cast<uint64_t>(kib) * 1024;
    if (words[0] == "MemAvailable:") (*out)["mem_available"] = static_cast<uint64_t>(kib) * 1024;
  }
}

// Size and space left for an unprivileged user, as df shows them.
void Disk(const std::string& path, const std::string& prefix, Json* out) {
  struct statvfs fs {};
  if (path.empty() || statvfs(path.c_str(), &fs) != 0) return;
  (*out)[prefix + "_total"] = static_cast<uint64_t>(fs.f_blocks) * fs.f_frsize;
  (*out)[prefix + "_free"] = static_cast<uint64_t>(fs.f_bavail) * fs.f_frsize;
}

void LoadUptimeCpus(const std::string& proc, Json* out) {
  double value = 0;
  const auto load = Lines(proc + "/loadavg");
  if (!load.empty() && Number(load[0], &value)) (*out)["load"] = value;
  const auto uptime = Lines(proc + "/uptime");
  if (!uptime.empty() && Number(uptime[0], &value)) (*out)["uptime"] = static_cast<int64_t>(value);
  int cpus = 0;
  for (const auto& line : Lines(proc + "/cpuinfo")) cpus += line.rfind("processor", 0) == 0;
  if (cpus > 0) (*out)["cpus"] = cpus;
}

// The zone of the CPU or SoC, else the first one with a temperature.
void CpuTemperature(const std::string& thermal, Json* out) {
  std::error_code error;
  std::vector<std::filesystem::path> zones;
  for (const auto& entry : std::filesystem::directory_iterator(thermal, error)) {
    if (entry.path().filename().string().rfind("thermal_zone", 0) == 0) zones.push_back(entry.path());
  }
  std::sort(zones.begin(), zones.end());
  std::optional<double> first;
  for (const auto& zone : zones) {
    const auto temp = Lines((zone / "temp").string());
    double milli = 0;
    if (temp.empty() || !Number(temp[0], &milli)) continue;
    if (!first) first = milli / 1000;
    const auto type = Lines((zone / "type").string());
    const std::string name = type.empty() ? "" : type[0];
    if (name.find("cpu") != std::string::npos || name.find("soc") != std::string::npos ||
        name.find("CPU") != std::string::npos) {
      first = milli / 1000;
      break;
    }
  }
  if (first) (*out)["cpu_temp"] = *first;  // NOLINT(bugprone-unchecked-optional-access): checked in the if
}

// The first wifi interface and its signal in dBm; some drivers count it from 0 up to 255.
void Wifi(const std::string& proc, Json* out) {
  const auto lines = Lines(proc + "/net/wireless");
  for (std::size_t i = 2; i < lines.size(); ++i) {
    const auto words = Words(lines[i]);
    double level = 0;
    if (words.size() < 4 || words[0].back() != ':' || !Number(words[3], &level)) continue;
    if (level > 0) level -= 256;
    (*out)["wifi_iface"] = words[0].substr(0, words[0].size() - 1);
    (*out)["wifi_signal"] = static_cast<int>(level);
    return;
  }
}

// The interface of the default route with the lowest metric and its IPv4 address.
void Network(const std::string& proc, Json* out) {
  const auto lines = Lines(proc + "/net/route");
  std::string iface;
  long best = 0;
  for (std::size_t i = 1; i < lines.size(); ++i) {
    const auto words = Words(lines[i]);
    if (words.size() < 7 || words[1] != "00000000") continue;
    const long metric = std::strtol(words[6].c_str(), nullptr, 10);
    if (iface.empty() || metric < best) {
      iface = words[0];
      best = metric;
    }
  }
  if (iface.empty()) return;
  (*out)["net_iface"] = iface;
  ifaddrs* addresses = nullptr;
  if (getifaddrs(&addresses) != 0) return;
  for (const ifaddrs* a = addresses; a; a = a->ifa_next) {
    if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET || iface != a->ifa_name) continue;
    char text[INET_ADDRSTRLEN] = {};
    const auto* in = reinterpret_cast<const sockaddr_in*>(a->ifa_addr);
    if (inet_ntop(AF_INET, &in->sin_addr, text, sizeof(text))) (*out)["net_ip"] = std::string(text);
    break;
  }
  freeifaddrs(addresses);
}

}  // namespace

Json SystemStats(const SystemSources& sources) {
  Json out = Json::object();
  Memory(sources.proc, &out);
  Disk(sources.root, "disk", &out);
  Disk(sources.data, "data", &out);
  LoadUptimeCpus(sources.proc, &out);
  CpuTemperature(sources.thermal, &out);
  Wifi(sources.proc, &out);
  Network(sources.proc, &out);
  return out;
}

}  // namespace mowbite_bridge
