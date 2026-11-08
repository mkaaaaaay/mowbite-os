#ifndef MOWBITE_BRIDGE_SYSTEM_STATS_H
#define MOWBITE_BRIDGE_SYSTEM_STATS_H

#include <string>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// Where system.stats reads from; tests point these at files of their own.
struct SystemSources {
  std::string proc = "/proc";
  std::string thermal = "/sys/class/thermal";
  std::string root = "/";
  // data_dir; empty leaves data_total and data_free out.
  std::string data;
};

// system.stats: memory, disks, load, temperature, uptime and network of the machine with the keys and units of the
// app's system.cgi (bytes, °C, s, dBm); a value that cannot be read is left out.
Json SystemStats(const SystemSources& sources);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_SYSTEM_STATS_H
