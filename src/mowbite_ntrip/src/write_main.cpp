// mowbite_ntrip_write <path>: the access as a JSON object on stdin (host, port, mountpoint, user, password; port and
// version optional) written to path as mowbite_ntrip reads it, mode 600. Exit 1 with why, never with a value.
#include <iostream>
#include <iterator>
#include <string>

#include <nlohmann/json.hpp>

#include "mowbite_ntrip/ntrip_config.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: mowbite_ntrip_write <path> < access.json\n";
    return 2;
  }
  const std::string text((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
  const auto given = nlohmann::json::parse(text, nullptr, false);
  if (!given.is_object()) {
    std::cout << "no JSON object\n";
    return 1;
  }
  mowbite_ntrip::NtripConfig config;
  for (const auto& [key, field] : {std::pair<const char*, std::string*>{"host", &config.host},
                                   {"mountpoint", &config.mountpoint}, {"user", &config.user},
                                   {"password", &config.password}}) {
    const auto value = given.find(key);
    if (value == given.end()) continue;
    if (!value->is_string()) {
      std::cout << key << ": not a text\n";
      return 1;
    }
    *field = value->get<std::string>();
  }
  config.mountpoint = mowbite_ntrip::NormalMountpoint(config.mountpoint);
  for (const auto& [key, field] : {std::pair<const char*, int*>{"port", &config.port}, {"version", &config.version}}) {
    const auto value = given.find(key);
    if (value == given.end()) continue;
    if (!value->is_number_integer()) {
      std::cout << key << ": not a whole number\n";
      return 1;
    }
    *field = value->get<int>();
  }
  std::string error;
  if (!mowbite_ntrip::WriteAccessFile(argv[1], config, error)) {
    std::cout << error << "\n";
    return 1;
  }
  return 0;
}
