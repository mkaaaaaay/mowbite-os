#include "mowbite_camera/stream_supervisor.h"

#include "mowbite_camera/camera_list.h"

namespace mowbite_camera {

StreamSupervisor::StreamSupervisor(StreamServer* server, std::function<double()> now)
    : server_(server), now_(std::move(now)), reason_(kNoSignal), detail_("the stream server is not started yet") {}

void StreamSupervisor::Tell(const std::string& reason, const std::string& detail) {
  reason_ = reason;
  detail_ = detail;
}

bool StreamSupervisor::Tick(bool wanted) {
  const double now = now_();
  bool ended = false;
  if (phase_ == Phase::kStarting || phase_ == Phase::kUp) {
    std::string how;
    if (!server_->Running(&how)) {
      ended = true;
      Ended(now, true, how);
    } else if (phase_ == Phase::kStarting && now - began_ >= kStreamStartSeconds) {
      server_->Stop();
      ended = true;
      Ended(now, true, "no answer of its API");
    }
  }
  if (!wanted) {
    if ((phase_ == Phase::kStarting || phase_ == Phase::kUp) && now - wanted_at_ >= kStreamIdleStop) {
      server_->Stop();
      ended = true;
      phase_ = Phase::kIdle;
      Tell(kNoSignal, "the stream server is stopped while no camera needs it");
    }
    return ended;
  }
  wanted_at_ = now;
  if (phase_ == Phase::kIdle) {
    if (!server_->Installed()) {
      Tell(kNotSetUp, "the stream server is not installed");
      return ended;
    }
    phase_ = Phase::kWaiting;
    next_start_ = now;
  }
  if (phase_ == Phase::kWaiting && now >= next_start_) {
    std::string error;
    if (server_->Start(&error)) {
      phase_ = Phase::kStarting;
      began_ = now;
      Tell(kNoSignal, "the stream server is starting");
    } else {
      Ended(now, false, error);
    }
  }
  return ended;
}

void StreamSupervisor::Ended(double now, bool ran, const std::string& how) {
  if (ran && now - began_ >= kStreamStableAfter) ends_ = 0;
  if (ends_ >= kStreamRestarts) {
    phase_ = Phase::kGaveUp;
    Tell(kNotSetUp, "the stream server ended " + std::to_string(ends_ + 1) + " times in a row, last with " + how);
    return;
  }
  ++ends_;
  phase_ = Phase::kWaiting;
  next_start_ = now + kStreamRestartDelay;
  Tell(kNoSignal, "the stream server ended with " + how + " and starts again");
}

void StreamSupervisor::Answered() {
  if (phase_ != Phase::kStarting) return;
  phase_ = Phase::kUp;
  Tell("", "");
}

}  // namespace mowbite_camera
