#include "mowbite_ntrip/ntrip_client.h"

#include <algorithm>

namespace mowbite_ntrip {

namespace {

// $ccGGA,...*hh with a talker of two letters.
bool IsGga(const std::string& s) {
  return s.size() > 10 && s.size() < 200 && s[0] == '$' && s.compare(3, 4, "GGA,") == 0 &&
         s[s.size() - 3] == '*' && s.find_first_of("\r\n") == std::string::npos;
}

}  // namespace

const char* StateName(ClientState state) {
  switch (state) {
    case ClientState::kOff: return "off";
    case ClientState::kWaiting: return "waiting";
    case ClientState::kConnecting: return "connecting";
    case ClientState::kConnected: return "connected";
    case ClientState::kRejected: return "rejected";
  }
  return "off";
}

void NtripClient::SetConfig(const std::optional<NtripConfig>& config, double now) {
  if (config == config_) return;
  config_ = config;
  retry_delay_ = timing_.first_retry_s;
  next_attempt_ = now;
  // A new access may be refused again: that is worth an event again.
  rejected_reported_ = false;
  if (!config_) {
    // Switched off on purpose: no LOST for it.
    last_frame_.reset();
    lost_reported_ = false;
  }
  if (socket_open_) {
    close_reason_ = config_ ? "access changed" : "access removed";
    restart_now_ = true;
  } else {
    state_ = config_ ? ClientState::kWaiting : ClientState::kOff;
  }
}

bool NtripClient::ShouldConnect(double now) const {
  return config_ && !socket_open_ && (state_ == ClientState::kWaiting || state_ == ClientState::kRejected) &&
         now >= next_attempt_;
}

std::string NtripClient::BeginAttempt(double now) {
  socket_open_ = true;
  state_ = ClientState::kConnecting;
  close_reason_.reset();
  reader_ = ResponseReader();
  framer_.Reset();
  last_data_ = now;
  last_gga_sent_.reset();
  attempts_++;
  return config_ ? BuildRequest(*config_) : std::string();
}

void NtripClient::OnBytes(const uint8_t* data, size_t size, double now, std::vector<std::vector<uint8_t>>& frames) {
  if (!socket_open_ || close_reason_) return;
  last_data_ = now;
  bytes_ += size;
  std::vector<uint8_t> body;
  const auto result = reader_.Feed(data, size, body);
  switch (result) {
    case ResponseReader::Result::kPending: return;
    case ResponseReader::Result::kStreaming: break;
    case ResponseReader::Result::kUnauthorized:
      state_ = ClientState::kRejected;
      last_error_ = "login refused (" + std::to_string(reader_.status_code()) + ")";
      close_reason_ = last_error_;
      if (!rejected_reported_) {
        rejected_reported_ = true;
        // The login stays refused until the access changes: no LOST on top of it.
        lost_reported_ = last_frame_.has_value();
        events_.push_back({"REJECTED", "The NTRIP caster refused the login", "error", "user", reader_.status_code(), 0});
      }
      return;
    case ResponseReader::Result::kNoMountpoint:
      last_error_ = "mountpoint not offered: " + reader_.status_line();
      close_reason_ = last_error_;
      return;
    case ResponseReader::Result::kError:
      last_error_ = "unexpected answer: " + reader_.status_line();
      close_reason_ = last_error_;
      return;
  }
  state_ = ClientState::kConnected;
  const size_t before = frames.size();
  const uint64_t crc_before = framer_.crc_errors();
  if (!body.empty()) framer_.Feed(body.data(), body.size(), frames);
  crc_errors_ += framer_.crc_errors() - crc_before;
  if (frames.size() > before) {
    frames_ += frames.size() - before;
    last_frame_ = now;
    retry_delay_ = timing_.first_retry_s;
    last_error_.clear();
    rejected_reported_ = false;
    if (lost_reported_) {
      lost_reported_ = false;
      events_.push_back({"RESTORED", "Corrections from the NTRIP caster again", "info", "auto", 0, 0});
    }
  }
  if (reader_.ended()) close_reason_ = "the caster ended the stream";
}

std::optional<std::string> NtripClient::CloseReason(double now) const {
  if (!socket_open_) return std::nullopt;
  if (close_reason_) return close_reason_;
  if (now - last_data_ > timing_.data_timeout_s) {
    return std::string(state_ == ClientState::kConnected ? "no data" : "no answer") + " for " +
           std::to_string(static_cast<int>(timing_.data_timeout_s)) + " s";
  }
  return std::nullopt;
}

void NtripClient::OnClosed(double now, const std::string& error) {
  if (!socket_open_) return;
  socket_open_ = false;
  if (!close_reason_ && !error.empty()) last_error_ = error;
  if (close_reason_ && last_error_.empty()) last_error_ = *close_reason_;
  close_reason_.reset();
  if (!config_) {
    restart_now_ = false;
    state_ = ClientState::kOff;
    return;
  }
  if (restart_now_) {
    restart_now_ = false;
    state_ = ClientState::kWaiting;
    next_attempt_ = now;
    return;
  }
  if (state_ == ClientState::kRejected) {
    next_attempt_ = now + timing_.rejected_retry_s;
    return;
  }
  state_ = ClientState::kWaiting;
  if (retry_delay_ <= 0) retry_delay_ = timing_.first_retry_s;
  next_attempt_ = now + retry_delay_;
  retry_delay_ = std::min(retry_delay_ * 2.0, timing_.max_retry_s);
}

void NtripClient::SetGga(const std::string& sentence) {
  if (IsGga(sentence)) gga_ = sentence;
}

std::optional<std::string> NtripClient::TakeGga(double now) {
  if (!socket_open_ || state_ != ClientState::kConnected || close_reason_ || gga_.empty()) return std::nullopt;
  if (last_gga_sent_ && now - *last_gga_sent_ < timing_.gga_interval_s) return std::nullopt;
  last_gga_sent_ = now;
  return gga_ + "\r\n";
}

void NtripClient::Tick(double now) {
  if (last_frame_ && !lost_reported_ && now - *last_frame_ > timing_.lost_after_s) {
    lost_reported_ = true;
    events_.push_back({"LOST", "No corrections from the NTRIP caster", "warning", "auto", 0, timing_.lost_after_s});
  }
}

std::vector<ClientEvent> NtripClient::TakeEvents() {
  std::vector<ClientEvent> events;
  events.swap(events_);
  return events;
}

std::optional<double> NtripClient::last_frame_age(double now) const {
  if (!last_frame_) return std::nullopt;
  return now - *last_frame_;
}

double NtripClient::retry_in(double now) const {
  if (socket_open_ || (state_ != ClientState::kWaiting && state_ != ClientState::kRejected)) return 0.0;
  return std::max(0.0, next_attempt_ - now);
}

}  // namespace mowbite_ntrip
