#include "tron_gnss/board_link.h"

namespace tron {

void BoardLink::OnPortOpened() {
  state_ = BoardState::kUnknown;
  heard_ = false;
  active_ = false;
  pending_reset_ = false;
}

void BoardLink::OnBoardReport(bool on, double now) {
  // Only a fall back from ON is worth a reset; a first OFF (unknown -> OFF) just waits for ON, and ON again drops it.
  if (state_ == BoardState::kOn && !on && config_.reset_on_off && wanted(now)) pending_reset_ = true;
  if (on) pending_reset_ = false;
  state_ = on ? BoardState::kOn : BoardState::kOff;
  heard_ = true;
}

bool BoardLink::wanted(double now) const {
  return last_rtcm_ && now - *last_rtcm_ <= config_.active_window_s;
}

bool BoardLink::OnRtcm(size_t bytes, double now) {
  last_rtcm_ = now;
  if (!heard_ || (config_.gate && state_ != BoardState::kOn)) {
    dropped_bytes_ += bytes;
    return false;
  }
  forwarded_bytes_ += bytes;
  return true;
}

std::string BoardLink::Poll(double now) {
  if (!heard_ || !wanted(now)) {
    active_ = false;
    pending_reset_ = false;
    return {};
  }
  std::string out;
  if (!active_) {
    active_ = true;
    next_on_ = now;
  }
  if (pending_reset_ && (!last_reset_ || now - *last_reset_ >= config_.min_reset_gap_s)) {
    pending_reset_ = false;
    last_reset_ = now;
    resets_++;
    out += kBoardReset;
    next_on_ = now;
  }
  if (now >= next_on_) {
    out += kBoardRtkOn;
    next_on_ = now + config_.keepalive_s;
  }
  return out;
}

std::optional<double> BoardLink::host_correction_age(double now) const {
  if (!last_forward_) return std::nullopt;
  return now - *last_forward_;
}

}  // namespace tron
