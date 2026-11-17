#ifndef TRON_VISION_FAKE_ENCODER_H
#define TRON_VISION_FAKE_ENCODER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/h264_encoder.h"

namespace tron_vision::test {

// Encoder whose work takes encode_s on the clock of the test: SPS, PPS and a large IDR picture first, then small
// pictures.
class FakeEncoder : public H264Encoder {
 public:
  explicit FakeEncoder(double& clock) : clock_(clock) {}

  bool Probe(std::string& error) override {
    if (!present) error = "cannot load librockchip_mpp.so.1";
    return present;
  }
  bool Open(const EncoderSettings& s, std::string& error) override {
    opens++;
    settings = s;
    if (fail_open) {
      error = "mpp_init -1002";
      return false;
    }
    is_open = true;
    first = true;
    return true;
  }
  void Close() override {
    is_open = false;
  }
  bool Header(std::vector<uint8_t>& header, std::string&) override {
    header = {0, 0, 0, 1, 0x67, 0x42, 0xe0, 0x1f, 0xaa, 0, 0, 0, 1, 0x68, 0xce, 0x3c, 0x80};
    return true;
  }
  bool Encode(const CapturedFrame& frame, const uint8_t*& data, size_t& size, std::string& error) override {
    clock_ += encode_s;
    if (on_encode) on_encode();
    if (fail_encode || !is_open || frame.dmabuf < 0) {
      error = "put frame -1";
      return false;
    }
    Header(output_, error);
    if (!first) output_.clear();
    output_.insert(output_.end(), {0, 0, 0, 1, static_cast<uint8_t>(first ? 0x65 : 0x41)});
    output_.insert(output_.end(), first ? 2000 : 200, 0x11);
    first = false;
    data = output_.data();
    size = output_.size();
    return true;
  }

  bool present = true;
  bool fail_open = false;
  bool fail_encode = false;
  double encode_s = 0.002;
  int opens = 0;
  bool is_open = false;
  bool first = true;
  EncoderSettings settings;
  // Called with each frame the encoder takes.
  std::function<void()> on_encode;

 private:
  double& clock_;
  std::vector<uint8_t> output_;
};

}  // namespace tron_vision::test

#endif  // TRON_VISION_FAKE_ENCODER_H
