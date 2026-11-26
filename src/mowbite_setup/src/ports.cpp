// setup.ports and setup.ports.listen: which serial ports the mower has and what a free one sends.
#include "mowbite_setup/ports.h"

#include <algorithm>
#include <filesystem>

#include "mowbite_api/setup.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

namespace {

std::string Canonical(const std::string& path) {
  std::error_code error;
  const auto canonical = std::filesystem::weakly_canonical(path, error);
  return error ? path : canonical.string();
}

const mowbite_serial::PortInfo* FindPort(const PortsView& view, const std::string& path) {
  for (const auto& port : view.ports) {
    if (port.path == path) return &port;
  }
  return nullptr;
}

json Usb(const std::optional<mowbite_serial::UsbInfo>& usb) {
  if (!usb) return nullptr;
  return {{"vendor_id", usb->vendor_id},
          {"product_id", usb->product_id},
          {"manufacturer", usb->manufacturer},
          {"product", usb->product}};
}

}  // namespace

std::string NodeOf(const mowbite_serial::PortInfo& port, const std::map<std::string, std::string>& driver_ports) {
  std::vector<std::string> names = port.by_id;
  names.push_back(port.path);
  for (const auto& name : names) {
    const auto found = driver_ports.find(Canonical(name));
    if (found != driver_ports.end()) return found->second;
  }
  return "";
}

json PortsAnswer(const PortsView& view) {
  json ports = json::array();
  for (const auto& port : view.ports) {
    json held = json::array();
    for (const auto& holder : view.holders(port.path)) held.push_back({{"pid", holder.pid}, {"name", holder.name}});
    const std::string node = NodeOf(port, view.driver_ports);
    const bool free = node.empty() && held.empty();
    ports.push_back({{"path", port.path},
                     {"by_id", port.by_id},
                     {"driver", port.driver},
                     {"usb", Usb(port.usb)},
                     {"held_by", std::move(held)},
                     {"node", node},
                     {"free", free}});
  }
  return {{"ports", std::move(ports)}, {"resting", view.resting}, {"listening", !view.listening.empty()}};
}

std::optional<Refusal> CheckListen(const json& params, const PortsView& view, ListenRequest* request) {
  if (!params.is_object()) return BadParam("port", "the port is missing");
  for (const auto& item : params.items()) {
    if (item.key() != "port" && item.key() != "seconds" && item.key() != "baud") {
      return BadParam(item.key(), item.key() + " is no param of setup.ports.listen");
    }
  }
  if (!params.contains("port") || !params["port"].is_string()) return BadParam("port", "the port is missing");
  ListenRequest out;
  out.port = params["port"].get<std::string>();
  out.seconds = mowbite_api::kSetupListenDefaultS;
  if (params.contains("seconds")) {
    const json& seconds = params["seconds"];
    if (!seconds.is_number_integer() || seconds.get<int>() < 1 || seconds.get<int>() > mowbite_api::kSetupListenMaxS) {
      return BadParam("seconds", "a whole number from 1 to " + std::to_string(mowbite_api::kSetupListenMaxS));
    }
    out.seconds = seconds.get<int>();
  }
  out.bauds = mowbite_serial::ListenBauds();
  if (params.contains("baud")) {
    const json& baud = params["baud"];
    if (!baud.is_number_integer() ||
        std::find(out.bauds.begin(), out.bauds.end(), baud.get<int>()) == out.bauds.end()) {
      return BadParam("baud", "not one of the rates listen tries");
    }
    out.bauds = {baud.get<int>()};
  }
  if (!view.listening.empty()) {
    return Refuse("LISTENING", "Already listening on " + view.listening, {{"port", view.listening}});
  }
  if (!view.resting) return Refuse("BUSY", "The mower does not rest");
  const mowbite_serial::PortInfo* port = FindPort(view, out.port);
  if (!port) return Refuse("PORT_UNKNOWN", "No port " + out.port, {{"port", out.port}});
  const std::string node = NodeOf(*port, view.driver_ports);
  if (!node.empty()) return Refuse("PORT_IN_USE", node + " uses " + out.port, {{"port", out.port}, {"node", node}});
  const auto holders = view.holders(out.port);
  if (!holders.empty()) {
    json by = json::array();
    for (const auto& holder : holders) by.push_back(holder.name);
    return Refuse("PORT_IN_USE", out.port + " is held by " + by.dump(), {{"port", out.port}, {"by", by}});
  }
  *request = std::move(out);
  return std::nullopt;
}

std::optional<Refusal> ListenAnswer(const std::string& port, const mowbite_serial::Listened& listened, json* answer) {
  if (!listened.error.empty()) {
    return Refuse("PORT_NOT_OPENED", port + ": " + listened.error, {{"port", port}, {"error", listened.error}});
  }
  const mowbite_serial::Heard& heard = listened.heard;
  json tried = json::array();
  for (const auto& [baud, at] : listened.tried) {
    tried.push_back({{"baud", baud}, {"kind", mowbite_serial::KindName(at.kind)}, {"bytes", at.bytes}});
  }
  *answer = {{"port", port},
             {"baud", listened.baud > 0 ? json(listened.baud) : json(nullptr)},
             {"kind", mowbite_serial::KindName(heard.kind)},
             {"bytes_per_s", heard.seconds > 0 ? static_cast<double>(heard.bytes) / heard.seconds : 0.0},
             {"stopped", listened.stopped},
             {"nmea", nullptr},
             {"wit", nullptr},
             {"rtcm", nullptr},
             {"tried", std::move(tried)}};
  if (heard.nmea_lines > 0) {
    (*answer)["nmea"] = {{"lines", heard.nmea_lines},
                         {"talkers", heard.talkers},
                         {"sentences", heard.sentences},
                         {"gga_hz", heard.gga_hz},
                         {"fix", heard.fix >= 0 ? json(heard.fix) : json(nullptr)}};
  }
  if (heard.wit_frames > 0) {
    (*answer)["wit"] = {{"frames", heard.wit_frames}, {"types", heard.wit_types}};
  }
  if (heard.rtcm_frames > 0) {
    (*answer)["rtcm"] = {{"frames", heard.rtcm_frames}, {"types", heard.rtcm_types}};
  }
  return std::nullopt;
}

}  // namespace mowbite_setup
