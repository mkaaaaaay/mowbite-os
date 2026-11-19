#include "mowbite_ntrip/ntrip_client.h"

#include <algorithm>

namespace mowbite_ntrip {

namespace {

// $ccGGA,...*hh with a talker of two letters.
bool IsGga(const std::string& s) {
  return s.size() > 10 && s.size() < 200 && s[0] == '$' && s.compare(3, 4, "GGA,") == 0 &&
         s[s.size() - 3] == '*' && s.find_first_of("\r\n") == std::string::npos;
}

// "may": the new source can just as well have the same reference.
const char kSourceChanged[] =
    "Corrections from another caster or mountpoint: positions may be offset from the map if its reference differs; a "
    "change between sources with the same reference moves nothing";

// host, port and mountpoint of SOURCE_CHANGED, with prefix "" or "previous_".
void PutSource(nlohmann::ordered_json& details, const std::string& prefix, const CorrectionSource& source) {
  details[prefix + "host"] = source.host;
  details[prefix + "port"] = source.port;
  details[prefix + "mountpoint"] = source.mountpoint;
}

}  // namespace

nlohmann::ordered_json CorrectionsDetails(const ClientEvent& event) {
  nlohmann::ordered_json details = {{"reason", event.reason}, {"message", event.message}, {"severity", event.severity}};
  if (!event.resume.empty()) details["resume"] = event.resume;
  if (event.code) details["code"] = event.code;
  if (event.limit > 0) details["limit"] = event.limit;
  if (event.source && event.previous) {
    PutSource(details, "", *event.source);
    PutSource(details, "previous_", *event.previous);
  }
  return details;
}

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
  source_checked_ = false;
  retry_delay_ = timing_.first_retry_s;
  next_attempt_ = now;
  gga_wait_until_ = now + timing_.gga_wait_s;
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
         now >= next_attempt_ && (!gga_.empty() || now >= gga_wait_until_);
}

std::string NtripClient::BeginAttempt(double now) {
  socket_open_ = true;
  state_ = ClientState::kConnecting;
  close_reason_.reset();
  reader_ = ResponseReader();
  framer_.Reset();
  last_data_ = now;
  last_gga_sent_.reset();
  streaming_since_.reset();
  attempts_++;
  return config_ ? BuildRequest(*config_) : std::string();
}

void NtripClient::OnRequestSent(double now) {
  if (socket_open_ && state_ == ClientState::kConnecting) last_data_ = now;
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
  if (state_ != ClientState::kConnected) {
    answered_without_gga_ = gga_.empty();
    streaming_since_ = now;
  }
  state_ = ClientState::kConnected;
  const size_t before = frames.size();
  const uint64_t crc_before = framer_.crc_errors();
  if (!body.empty()) framer_.Feed(body.data(), body.size(), frames);
  crc_errors_ += framer_.crc_errors() - crc_before;
  if (frames.size() > before) {
    frames_ += frames.size() - before;
    last_frame_ = now;
    // A caster that ends every stream after its first frames keeps the delay growing.
    if (streaming_since_ && now - *streaming_since_ >= timing_.stable_s) retry_delay_ = timing_.first_retry_s;
    last_error_.clear();
    rejected_reported_ = false;
    if (!source_checked_ && config_) UseSource();
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
  if (state_ == ClientState::kConnected && streaming_since_) {
    const double since = std::max(*streaming_since_, last_frame_.value_or(*streaming_since_));
    if (now - since > timing_.lost_after_s) {
      return "no valid RTCM frame for " + std::to_string(static_cast<int>(timing_.lost_after_s)) + " s";
    }
  }
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
  // The caster answered and waited for a GGA that came only after the attempt: no failure, once more soon.
  const bool gga_came_late = state_ == ClientState::kConnected && answered_without_gga_ && !last_gga_sent_ &&
                             !gga_.empty();
  state_ = ClientState::kWaiting;
  if (gga_came_late) {
    next_attempt_ = now + timing_.first_retry_s;
    return;
  }
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

void NtripClient::SetLastSource(const std::optional<CorrectionSource>& source) {
  last_source_ = source;
}

std::optional<CorrectionSource> NtripClient::TakeSourceToSave() {
  std::optional<CorrectionSource> source;
  source.swap(source_to_save_);
  return source;
}

void NtripClient::UseSource() {
  source_checked_ = true;
  const CorrectionSource source = SourceOf(*config_);
  if (last_source_ && !SameSource(*last_source_, source)) {
    ClientEvent event{"SOURCE_CHANGED", kSourceChanged, "warning", ""};
    event.source = source;
    event.previous = last_source_;
    events_.push_back(event);
  }
  if (last_source_ != source) {
    last_source_ = source;
    source_to_save_ = source;
  }
}

std::optional<double> NtripClient::last_frame_age(double now) const {
  if (!last_frame_) return std::nullopt;
  return now - *last_frame_;
}

double NtripClient::retry_in(double now) const {
  if (socket_open_ || (state_ != ClientState::kWaiting && state_ != ClientState::kRejected)) return 0.0;
  const double next = gga_.empty() ? std::max(next_attempt_, gga_wait_until_) : next_attempt_;
  return std::max(0.0, next - now);
}

}  // namespace mowbite_ntrip
