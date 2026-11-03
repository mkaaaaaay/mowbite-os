#include "mowbite_calibration/result.h"

#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <filesystem>

namespace mowbite_calibration {

namespace {

bool Number(const json& value) {
  return value.is_number() && std::isfinite(value.get<double>());
}

}  // namespace

std::string GeometryKey(const std::string& name) {
  if (name == "ticks_per_m") return "ticks_per_m";
  if (name == "wheel_distance_m") return "track";
  if (name == "antenna_offset_x") return "antenna_x";
  if (name == "antenna_offset_y") return "antenna_y";
  if (name == "imu_roll") return "imu_roll";
  if (name == "imu_pitch") return "imu_pitch";
  return "";
}

json Compose(const Ending& ending, const json& evaluation, double station_limit) {
  json result = json::object();
  result["id"] = ending.id;
  result["time"] = ending.time;
  const bool evaluated = evaluation.is_object() && !evaluation.contains("error");
  std::string status;
  if (ending.reason != "DONE") {
    status = "failed";
  } else if (evaluation.is_null()) {
    status = "not_evaluated";
  } else if (!evaluated || !evaluation.value("ready", false)) {
    status = "failed";
  } else {
    status = "done";
  }
  result["status"] = status;
  result["reason"] = ending.reason;
  result["message"] = ending.message;
  result["recording"] = ending.recording;
  bool station_again = false;
  if (evaluated) {
    for (const auto& item : evaluation.items()) {
      if (item.key() == "values" || result.contains(item.key())) continue;
      result[item.key()] = item.value();
    }
    json values = json::array();
    if (evaluation.contains("values") && evaluation["values"].is_array()) {
      for (json value : evaluation["values"]) {
        if (!value.is_object()) continue;
        const std::string name = value.value("name", "");
        const std::string key = GeometryKey(name);
        value["key"] = key.empty() ? json() : json(key);
        if ((key == "antenna_x" || key == "antenna_y") && value.value("ok", false) && Number(value["value"]) &&
            Number(value["old"]) &&
            std::abs(value["value"].get<double>() - value["old"].get<double>()) > station_limit) {
          station_again = true;
        }
        values.push_back(std::move(value));
      }
    }
    result["values"] = std::move(values);
  } else {
    result["values"] = json::array();
    if (evaluation.is_object() && evaluation.contains("error")) result["error"] = evaluation["error"];
  }
  result["station_again"] = station_again;
  result["summary"] = Summary(result);
  return result;
}

std::string Summary(const json& result) {
  const std::string status = result.value("status", "");
  if (status == "not_evaluated") return "Not evaluated: the recording is kept for later";
  if (status == "failed" && result.value("reason", "") != "DONE") {
    return "The drive stopped: " + result.value("reason", std::string("unknown"));
  }
  if (status == "failed") return "The evaluation found no usable values";
  size_t ok = 0;
  size_t all = 0;
  if (result.contains("values") && result["values"].is_array()) {
    for (const auto& value : result["values"]) {
      all++;
      if (value.is_object() && value.value("ok", false)) ok++;
    }
  }
  return std::to_string(ok) + " of " + std::to_string(all) + " values ok";
}

bool WriteWhole(const std::string& path, const std::string& text, std::string* why) {
  const auto fail = [why](const std::string& message) {
    if (why) *why = message;
    return false;
  };
  std::error_code error;
  const std::filesystem::path target(path);
  std::filesystem::create_directories(target.parent_path(), error);
  if (error) return fail("cannot create " + target.parent_path().string() + ": " + error.message());
  const std::string temp = path + ".tmp";
  const int fd = ::open(temp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return fail("cannot write " + temp);
  size_t done = 0;
  while (done < text.size()) {
    const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
    if (n <= 0) break;
    done += static_cast<size_t>(n);
  }
  const bool synced = done == text.size() && ::fsync(fd) == 0;
  ::close(fd);
  if (!synced || std::rename(temp.c_str(), path.c_str()) != 0) {
    std::remove(temp.c_str());
    return fail("cannot write " + path);
  }
  const int folder = ::open(target.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
  if (folder >= 0) {
    ::fsync(folder);
    ::close(folder);
  }
  return true;
}

}  // namespace mowbite_calibration
