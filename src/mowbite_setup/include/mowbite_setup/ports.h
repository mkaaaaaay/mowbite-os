#ifndef MOWBITE_SETUP_PORTS_H
#define MOWBITE_SETUP_PORTS_H

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "mowbite_serial/ports.h"
#include "mowbite_setup/program.h"

namespace mowbite_setup {

// setup.ports and setup.ports.listen without ROS: the node lists the ports and listens in a thread of its own.
struct PortsView {
  std::vector<mowbite_serial::PortInfo> ports;
  // The ports the nodes of the platform use, by their canonical path.
  std::map<std::string, std::string> driver_ports;
  std::function<std::vector<mowbite_serial::Holder>(const std::string&)> holders;
  bool resting = false;
  // The port of the listen that runs, empty for none.
  std::string listening;
};

// The path with its links resolved, the path itself where it cannot be: how driver_ports names a port.
std::string CanonicalPort(const std::string& path);

// The node of the platform that uses the port, under its path, a link of by-id or its canonical path; empty for none.
std::string NodeOf(const mowbite_serial::PortInfo& port, const std::map<std::string, std::string>& driver_ports);

// The ports setup.ports lists: with scan those of the machine, without it only those of the drivers (a simulation
// shares its machine with devices that are not its own); the extra paths in both cases.
std::vector<mowbite_serial::PortInfo> ListedPorts(bool scan, const std::map<std::string, std::string>& driver_ports,
                                                  const std::vector<std::string>& extra);

// The answer of setup.ports.
json PortsAnswer(const PortsView& view);

struct ListenRequest {
  std::string port;
  double seconds = 0;
  std::vector<int> bauds;
};

// Whether setup.ports.listen may listen as params ask; fills the request when it may.
std::optional<Refusal> CheckListen(const json& params, const PortsView& view, ListenRequest* request);

// The answer of setup.ports.listen, or the refusal when the port did not open.
std::optional<Refusal> ListenAnswer(const std::string& port, const mowbite_serial::Listened& listened, json* answer);

}  // namespace mowbite_setup

#endif  // MOWBITE_SETUP_PORTS_H
