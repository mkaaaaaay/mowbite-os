#include "xcore_driver/gnss.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "xcore_driver/services.h"

namespace xcore {

namespace {

// Neither is reported by the board; casters expect plausible values.
constexpr int kGgaSatellites = 10;
constexpr double kGgaHdop = 1.0;
constexpr double kMaxBoardAccuracyM = 1000.0;

}  // namespace

std::optional<GnssSolution> DecodeGnss(const std::vector<Output>& outputs) {
  GnssSolution solution;
  bool has_position = false;
  for (const auto& out : outputs) {
    if (out.service_id != kGpsService) continue;
    switch (out.channel_id) {
      case gps::kOutPosition: {
        auto p = DecodeDoubles(out.value, 3);
        if (!p) return std::nullopt;
        solution.lat_deg = (*p)[0];
        solution.lon_deg = (*p)[1];
        solution.height_m = (*p)[2];
        solution.board_time_us = out.board_time_us;
        has_position = true;
        break;
      }
      case gps::kOutHorizontalAccuracy: {
        auto a = DecodeDoubles(out.value, 1);
        if (a && std::isfinite((*a)[0]) && (*a)[0] > 0 && (*a)[0] < kMaxBoardAccuracyM) {
          solution.horizontal_accuracy_m = (*a)[0];
        }
        break;
      }
      case gps::kOutFixType: {
        const std::string text = DecodeText(out.value);
        if (text == "FIX") solution.fix = BoardFix::kFixed;
        if (text == "FLOAT") solution.fix = BoardFix::kFloat;
        break;
      }
      case gps::kOutMotionEnu: {
        auto m = DecodeDoubles(out.value, 3);
        if (m && std::isfinite((*m)[0]) && std::isfinite((*m)[1]) && std::isfinite((*m)[2])) {
          solution.motion_enu = std::array<double, 3>{(*m)[0], (*m)[1], (*m)[2]};
        }
        break;
      }
      default:
        break;
    }
  }
  if (!has_position) return std::nullopt;
  const double lat = solution.lat_deg, lon = solution.lon_deg;
  if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(solution.height_m) || std::abs(lat) > 90.0 ||
      std::abs(lon) > 180.0 || (lat == 0.0 && lon == 0.0)) {
    return std::nullopt;
  }
  return solution;
}

std::optional<GnssPose> GnssConverter::Convert(const GnssSolution& solution,
                                               const std::optional<double>& host_correction_age_s) {
  if (last_board_time_us_ && *last_board_time_us_ == solution.board_time_us) {
    repeated_++;
    return std::nullopt;
  }
  last_board_time_us_ = solution.board_time_us;

  std::optional<double> board;
  if (solution.horizontal_accuracy_m) board = std::max(*solution.horizontal_accuracy_m, config_.min_accuracy_m);
  GnssPose pose;
  switch (solution.fix) {
    case BoardFix::kFixed:
      if (host_correction_age_s && *host_correction_age_s <= config_.max_correction_age_s) {
        pose.solution = mowbite_gnss::Solution::kRtkFixed;
        pose.accuracy_m = board.value_or(config_.accuracy_rtk_fixed_m);
        break;
      }
      // Corrections the host did not pass on, or old ones: the fix may rest on data nobody checks.
      fixed_without_corrections_++;
      last_downgrade_age_ = host_correction_age_s;
      pose.solution = mowbite_gnss::Solution::kRtkFloat;
      pose.accuracy_m = std::max(board.value_or(0.0), config_.accuracy_rtk_float_m);
      break;
    case BoardFix::kFloat:
      pose.solution = mowbite_gnss::Solution::kRtkFloat;
      pose.accuracy_m = board.value_or(config_.accuracy_rtk_float_m);
      break;
    case BoardFix::kNone:
      // A position without RTK: the board does not tell single from DGPS.
      pose.solution = mowbite_gnss::Solution::kSingle;
      pose.accuracy_m = board.value_or(config_.accuracy_single_m);
      break;
  }
  pose.position = frame_.ToEnu(solution.lat_deg, solution.lon_deg, solution.height_m);
  pose.sensor_stamp_ms = static_cast<uint32_t>(solution.board_time_us / 1000);
  if (solution.motion_enu) {
    pose.motion_valid = true;
    pose.motion_east_mps = (*solution.motion_enu)[0];
    pose.motion_north_mps = (*solution.motion_enu)[1];
    pose.motion_heading = std::atan2(pose.motion_north_mps, pose.motion_east_mps);
  }
  return pose;
}

mowbite_gnss::GgaFields GgaOf(const GnssSolution& solution, double utc_s) {
  mowbite_gnss::GgaFields f;
  f.utc_s = utc_s;
  f.lat_deg = solution.lat_deg;
  f.lon_deg = solution.lon_deg;
  f.altitude_m = solution.height_m;
  switch (solution.fix) {
    case BoardFix::kFixed: f.quality = mowbite_gnss::GgaQuality::kRtkFixed; break;
    case BoardFix::kFloat: f.quality = mowbite_gnss::GgaQuality::kRtkFloat; break;
    case BoardFix::kNone: f.quality = mowbite_gnss::GgaQuality::kSingle; break;
  }
  f.satellites = kGgaSatellites;
  f.hdop = kGgaHdop;
  return f;
}

bool RtcmQueue::Push(const uint8_t* data, size_t size) {
  if (bytes_.size() + size > max_bytes_) {
    dropped_ += size;
    return false;
  }
  bytes_.insert(bytes_.end(), data, data + size);
  return true;
}

std::vector<Bytes> RtcmQueue::Take() {
  std::vector<Bytes> pieces;
  while (!bytes_.empty()) {
    const size_t n = std::min(bytes_.size(), max_piece_);
    pieces.emplace_back(bytes_.begin(), bytes_.begin() + static_cast<std::ptrdiff_t>(n));
    bytes_.erase(bytes_.begin(), bytes_.begin() + static_cast<std::ptrdiff_t>(n));
  }
  return pieces;
}

void RtcmQueue::Clear() {
  dropped_ += bytes_.size();
  bytes_.clear();
}

}  // namespace xcore
