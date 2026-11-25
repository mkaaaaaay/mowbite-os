// The calibration drive at the node: its status, calibration.check, .start, .abort and .apply.
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "logic_node.h"
#include "mowbite_api/catalog.h"

namespace mowbite_logic {

namespace {

// The value of key in result that is ok, null without one.
json OkValueOf(const json& result, const std::string& key) {
  if (!result.contains("values") || !result["values"].is_array()) return json();
  for (const auto& value : result["values"]) {
    if (!value.is_object() || value.value("key", json()) != key || value.value("ok", false) != true) continue;
    const json number = value.value("value", json());
    if (number.is_number() && std::isfinite(number.get<double>())) return number;
  }
  return json();
}

}  // namespace

void LogicNode::OnCalibrationStatus(const mowbite_msgs::msg::CalibrationStatus& msg) {
  CalibrationStatus status;
  for (const auto& problem : msg.problems) {
    Reason why(problem.reason, problem.message);
    const json fields = json::parse(problem.fields.empty() ? "{}" : problem.fields, nullptr, false);
    if (fields.is_object()) why.fields = fields;
    status.problems.push_back(why);
  }
  status.heading = msg.heading;
  status.lane_length = msg.lane_length;
  status.strip_length = msg.strip_length;
  status.strip_width = msg.strip_width;
  status.rounds = msg.rounds;
  status.expected_s = msg.expected_s;
  status.running = msg.running;
  std::lock_guard<std::mutex> lock(mutex_);
  core_.Call<&LogicCore::OnCalibrationStatus>(status, MonotonicSeconds());
}

// What the core finds against a calibration drive, then what only the node knows. Called with the mutex held.
std::vector<Reason> LogicNode::CalibrationProblemsLocked(double t) const {
  std::vector<Reason> problems = core_.View().CalibrationProblems(t);
  const auto has = [&problems](const std::string& code) {
    for (const auto& why : problems) {
      if (why.code == code) return true;
    }
    return false;
  };
  if (calibration_.active && !core_.View().Calibrating() && !has("STILL_STOPPING")) {
    problems.push_back(Reason("STILL_STOPPING", "The last calibration drive is still stopping, try again in a moment"));
  }
  if (!calibration_client_->action_server_is_ready() && !has("CALIBRATION_MISSING")) {
    problems.push_back(Reason("CALIBRATION_MISSING", "The calibration program is not running"));
  }
  return problems;
}

json LogicNode::ReasonJson(const Reason& why) {
  json out = {{"reason", why.code}, {"message", why.message}};
  for (const auto& [key, value] : why.fields.items()) out[key] = value;
  return out;
}

// {ok, problems: [{reason, message, ...}], space: {length, width, lane_length, heading} or null}, without a drive.
void LogicNode::OnCalibrationCheck(const mowbite_msgs::msg::RpcRequest& request) {
  std::vector<Reason> problems;
  std::optional<CalibrationStatus> status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    problems = CalibrationProblemsLocked(t);
    status = core_.View().CalibrationStatusNow(t);
  }
  json result = {{"ok", problems.empty()}, {"problems", json::array()}, {"space", nullptr}};
  for (const auto& why : problems) result["problems"].push_back(ReasonJson(why));
  if (status && status->lane_length > 0) {
    result["space"] = {{"length", status->strip_length},
                       {"width", status->strip_width},
                       {"lane_length", status->lane_length},
                       {"heading", status->heading}};
  }
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = result.dump(-1, ' ', false, json::error_handler_t::replace);
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
}

// {"state": "CALIBRATING"} once the drive began, else -32600 with the data of a refused command and
// CALIBRATION_REFUSED, or -32602 for params that do not fit.
void LogicNode::OnCalibrationStart(const mowbite_msgs::msg::RpcRequest& request) {
  json params;
  if (!ReadMethodParams(request, &params)) return;
  const std::string client = params.value("client", "");
  CommandOutcome outcome;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    outcome = core_.Call<&LogicCore::CalibrationStart>(CalibrationProblemsLocked(t), client, t);
  }
  PublishOutcome(request.id, request.method, client, outcome);
  Step();
}

// The app and HA (pause, dock): {"ok": true} when a drive was running; the mower stops where it is.
void LogicNode::OnCalibrationAbort(const mowbite_msgs::msg::RpcRequest& request) {
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ok = core_.Call<&LogicCore::AbortCalibration>(Reason("STOPPED", "Stopped by calibration.abort"));
  }
  if (ok) RCLCPP_INFO(get_logger(), "calibration.abort: stopped");
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = json{{"ok", ok}}.dump();
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
  Step();
}

// calibration.apply: the OS takes the values of its own last result (G32), as robot.geometry.set would, with the
// note calibration_applied; the refusals are those of the catalog.
void LogicNode::OnCalibrationApply(const mowbite_msgs::msg::RpcRequest& request) {
  using mowbite_msgs::msg::RpcError;
  json params;
  if (!ReadMethodParams(request, &params)) return;
  const std::string client = params.value("client", "");
  json answer;
  int changed_keys = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (RefuseGeometryNowLocked(request.id)) return;
    const json& result = calibration_result_;
    if (!result.is_object() || result.value("status", "") != "done" || !result.contains("id") ||
        !result["id"].is_string()) {
      PublishRpcError(request.id, RpcError::ERROR_INVALID_REQUEST, "No evaluated calibration to take values from",
                      json{{"reason", "NO_RESULT"}}.dump());
      return;
    }
    const std::string result_id = result["id"].get<std::string>();
    // The geometry the drive ran with, or the one right after the last calibration.apply of this result.
    const json& applied = geometry_.calibration_applied();
    const bool again = mowbite_api::AppliedTo(applied, result);
    const json base = again ? applied.value("geometry_version", json()) : result.value("geometry_version", json());
    if (!base.is_number_integer() || base.get<int64_t>() != geometry_.version()) {
      const json data = {{"reason", "OUTDATED"}, {"geometry_version", base}, {"version", geometry_.version()}};
      PublishRpcError(request.id, RpcError::ERROR_INVALID_REQUEST,
                      "The dimensions changed since the calibration drive: drive again", data.dump());
      return;
    }
    json settings = json::object();
    for (const auto& item : params["keys"]) {
      const std::string key = item.get<std::string>();
      const auto& calibrated = mowbite_api::CalibrationKeys();
      const json value = OkValueOf(result, key);
      if (std::find(calibrated.begin(), calibrated.end(), key) == calibrated.end() || value.is_null() ||
          settings.contains(key)) {
        PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, key + " has no value that is ok in the result",
                        json{{"reason", "NOT_OK"}, {"key", key}}.dump());
        return;
      }
      settings[key] = value;
    }
    Geometry changed = geometry_;
    const GeometryChange change = changed.ApplyCalibration(result_id, settings, WallSeconds());
    if (!change.ok) {
      PublishGeometryRefusal(request.id, change);
      return;
    }
    std::string error;
    if (!CommitGeometryLocked(std::move(changed), change, &error)) {
      RCLCPP_ERROR(get_logger(), "calibration.apply: %s", error.c_str());
      PublishRpcError(request.id, RpcError::ERROR_INTERNAL, "Cannot save the dimensions: " + error);
      return;
    }
    changed_keys = static_cast<int>(change.changed.size());
    answer = {{"applied", geometry_.calibration_applied()}, {"geometry", geometry_.Get()}};
  }
  RCLCPP_INFO(get_logger(), "calibration.apply%s: version %d, changed %d key(s)", ClientSuffix(client).c_str(),
              answer["geometry"]["version"].get<int>(), changed_keys);
  PublishGeometry();
  Step();
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = answer.dump();
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
}
}  // namespace mowbite_logic
