#include "tron_vision/video_feed.h"

#include <utility>

namespace tron_vision {

VideoFeed::VideoFeed(VideoFeedConfig config, H264Stream& stream, std::function<double()> now)
    : config_(config), stream_(stream), now_(std::move(now)) {}

std::string VideoFeed::Step(bool may_start, bool streaming) {
  std::string detail;
  if (may_start && !active_ && failures_ < config_.max_failures && now_() >= next_prepare_) {
    if (stream_.Prepare(config_.encoder, detail)) {
      active_ = true;
    } else {
      detail = Failed(detail);
    }
  }
  // A withdrawn stream answers 503 and drops its client.
  const bool offer = active_ && may_start;
  if (offer != offered_) {
    offered_ = offer;
    if (offer) {
      stream_.Offer();
    } else {
      stream_.Withdraw();
    }
  }
  stream_.Poll(streaming ? 0 : config_.idle_wait_ms);
  if (stream_.Running() && !stream_.Playing()) stream_.End();
  return detail;
}

std::string VideoFeed::Send(Camera& camera, const std::function<void(const Nv12Buffer&)>& edit) {
  std::string detail;
  if (!Begun(detail)) return detail;
  const CapturedFrame* frame = camera.Captured();
  if (frame == nullptr) {
    stream_.End();
    stream_.DropClient();
    return Failed("the camera does not export its frames");
  }
  // Without the edit the picture goes on as it is.
  std::string error;
  if (!camera.EditFrame(edit, error)) detail = error;
  const std::string failed = Encode(*frame);
  return failed.empty() ? detail : failed;
}

std::string VideoFeed::Send(const CapturedFrame& frame) {
  std::string detail;
  return Begun(detail) ? Encode(frame) : detail;
}

bool VideoFeed::Begun(std::string& detail) {
  if (!Playing()) return false;
  std::string error;
  if (!stream_.Running() && !stream_.Begin(error)) {
    stream_.DropClient();
    detail = Failed(error);
    return false;
  }
  return true;
}

std::string VideoFeed::Encode(const CapturedFrame& frame) {
  std::string error;
  if (stream_.Send(frame, error) == H264Stream::Sent::kEncoderFailed) return Failed(error);
  if (stream_.RunningFor() >= kFailuresForgetS) failures_ = 0;
  return "";
}

std::string VideoFeed::Failed(const std::string& detail) {
  failures_++;
  next_prepare_ = now_() + config_.retry_s;
  if (failures_ >= config_.max_failures) active_ = false;
  return "video: " + detail;
}

}  // namespace tron_vision
