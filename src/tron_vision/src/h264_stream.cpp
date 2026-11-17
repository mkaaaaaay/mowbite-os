#include "tron_vision/h264_stream.h"

#include <cstdio>
#include <random>
#include <utility>

namespace tron_vision {

namespace {

uint32_t Random() {
  return std::random_device{}();
}

}  // namespace

H264Stream::H264Stream(H264Encoder& encoder, RtspServer& server, double slow_encode_s, std::function<double()> now,
                       size_t path)
    : encoder_(encoder),
      server_(server),
      path_(path),
      slow_encode_s_(slow_encode_s),
      now_(std::move(now)),
      packetizer_(Random(), static_cast<uint16_t>(Random())),
      clock_(Random()) {}

H264Stream::~H264Stream() {
  End();
}

bool H264Stream::Prepare(const EncoderSettings& settings, std::string& error) {
  End();
  prepared_ = false;
  std::vector<uint8_t> header;
  bool ok = encoder_.Open(settings, error) && encoder_.Header(header, error);
  encoder_.Close();
  if (ok && !FindParameterSets(header.data(), header.size(), sets_)) {
    ok = false;
    error = "no SPS and PPS from the encoder";
  }
  if (!ok) return false;
  settings_ = settings;
  prepared_ = true;
  return true;
}

void H264Stream::Offer() {
  if (prepared_) server_.SetAvailable(sets_, path_);
}

void H264Stream::Withdraw() {
  End();
  server_.SetUnavailable(path_);
}

bool H264Stream::Begin(std::string& error) {
  End();
  if (!prepared_) {
    error = "stream not prepared";
    return false;
  }
  if (!encoder_.Open(settings_, error)) {
    encoder_.Close();
    return false;
  }
  running_ = true;
  clock_ = RtpClock(Random());
  window_start_ = now_();
  window_frames_ = 0;
  window_bytes_ = 0;
  return true;
}

H264Stream::Sent H264Stream::Send(const CapturedFrame& frame, std::string& error) {
  const uint8_t* data = nullptr;
  size_t size = 0;
  const double before = now_();
  bool ok = running_ && encoder_.Encode(frame, data, size, error);
  const double after = now_();
  if (!running_) error = "encoder not open";
  if (ok && after - before > slow_encode_s_) {
    ok = false;
    char text[48];
    std::snprintf(text, sizeof(text), "encode took %.1f s", after - before);
    error = text;
  }
  if (!ok) {
    End();
    server_.DropClient(path_);
    return Sent::kEncoderFailed;
  }
  encode_ms_ = (after - before) * 1000.0;
  packetizer_.Packetize(SplitAnnexB(data, size), clock_.Next(frame.timestamp_us), packets_);
  if (!server_.SendRtp(packets_, path_)) {
    End();
    return Sent::kClientGone;
  }
  frames_++;
  window_frames_++;
  window_bytes_ += size;
  if (after - window_start_ >= 1.0) {
    const double span = after - window_start_;
    fps_ = window_frames_ / span;
    kbps_ = static_cast<double>(window_bytes_) * 8.0 / 1000.0 / span;
    window_start_ = after;
    window_frames_ = 0;
    window_bytes_ = 0;
  }
  return Sent::kOk;
}

void H264Stream::End() {
  if (running_) encoder_.Close();
  running_ = false;
  fps_ = 0.0;
  kbps_ = 0.0;
}

}  // namespace tron_vision
