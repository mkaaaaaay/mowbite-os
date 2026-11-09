#ifndef TRON_VISION_H264_ENCODER_H
#define TRON_VISION_H264_ENCODER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tron_vision/mpp_abi.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

struct EncoderSettings {
  // Size of the coded picture; it is the left part of each line of the frame.
  uint32_t width = 640;
  uint32_t height = 480;
  // Bytes per line and lines of the whole frame.
  uint32_t line_bytes = 2560;
  uint32_t lines = 480;
  // Constant bit rate, held within 0.5..4 Mbit/s.
  int bitrate_bps = 1500000;
  // Frames per second of the source; an IDR picture comes once a second.
  int fps = 25;
};

// H.264 encoder for YUYV frames that it reads from their dma-buf.
class H264Encoder {
 public:
  virtual ~H264Encoder() = default;
  // False if the encoder cannot be used at all (library missing); nothing stays open.
  virtual bool Probe(std::string& error) = 0;
  virtual bool Open(const EncoderSettings& settings, std::string& error) = 0;
  virtual void Close() = 0;
  // SPS and PPS as Annex B.
  virtual bool Header(std::vector<uint8_t>& header, std::string& error) = 0;
  // One frame in, its access unit out as Annex B; `data` stays valid until the next call. The dma-buf of a buffer
  // index must stay the same and open until Close().
  virtual bool Encode(const CapturedFrame& frame, const uint8_t*& data, size_t& size, std::string& error) = 0;
};

// The hardware encoder of the RK3588 through librockchip_mpp, loaded with dlopen: constrained baseline, level 3.1,
// constant bit rate, SPS and PPS with every IDR picture, no SEI, one frame after the other.
class MppEncoder : public H264Encoder {
 public:
  explicit MppEncoder(std::string library_path);
  MppEncoder(const MppEncoder&) = delete;
  MppEncoder& operator=(const MppEncoder&) = delete;
  ~MppEncoder() override;

  bool Probe(std::string& error) override;
  bool Open(const EncoderSettings& settings, std::string& error) override;
  void Close() override;
  bool Header(std::vector<uint8_t>& header, std::string& error) override;
  bool Encode(const CapturedFrame& frame, const uint8_t*& data, size_t& size, std::string& error) override;

 private:
  bool Load(std::string& error);
  void Unload();
  bool Configure(std::string& error);
  bool Fail(std::string message, std::string& error);

  struct Imported {
    mpp::Handle buffer = nullptr;
    int fd = -1;
  };

  std::string library_path_;
  EncoderSettings settings_;
  void* library_ = nullptr;
  mpp::Handle context_ = nullptr;
  mpp::Api* api_ = nullptr;
  std::vector<Imported> imported_;
  std::vector<uint8_t> output_;

  mpp::CreateFn create_ = nullptr;
  mpp::InitFn init_ = nullptr;
  mpp::DestroyFn destroy_ = nullptr;
  mpp::ConfigInitFn config_init_ = nullptr;
  mpp::ConfigDeinitFn config_deinit_ = nullptr;
  mpp::ConfigSetS32Fn config_set_s32_ = nullptr;
  mpp::ConfigSetU32Fn config_set_u32_ = nullptr;
  mpp::FrameInitFn frame_init_ = nullptr;
  mpp::FrameDeinitFn frame_deinit_ = nullptr;
  mpp::FrameSetU32Fn frame_set_width_ = nullptr;
  mpp::FrameSetU32Fn frame_set_height_ = nullptr;
  mpp::FrameSetU32Fn frame_set_hor_stride_ = nullptr;
  mpp::FrameSetU32Fn frame_set_ver_stride_ = nullptr;
  mpp::FrameSetFormatFn frame_set_format_ = nullptr;
  mpp::FrameSetBufferFn frame_set_buffer_ = nullptr;
  mpp::FrameSetPtsFn frame_set_pts_ = nullptr;
  mpp::BufferImportFn buffer_import_ = nullptr;
  mpp::BufferPutFn buffer_put_ = nullptr;
  mpp::PacketInitFn packet_init_ = nullptr;
  mpp::PacketDeinitFn packet_deinit_ = nullptr;
  mpp::PacketSetLengthFn packet_set_length_ = nullptr;
  mpp::PacketGetPosFn packet_get_pos_ = nullptr;
  mpp::PacketGetLengthFn packet_get_length_ = nullptr;
};

}  // namespace tron_vision

#endif  // TRON_VISION_H264_ENCODER_H
