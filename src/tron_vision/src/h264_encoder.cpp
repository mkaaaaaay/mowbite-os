#include "tron_vision/h264_encoder.h"

#include <dlfcn.h>

#include <algorithm>
#include <utility>

#include "tron_vision/dynamic_library.h"

namespace tron_vision {

namespace {

constexpr char kTag[] = "tron_vision";
constexpr int kMinBitrate = 500000;
constexpr int kMaxBitrate = 4000000;
// Baseline profile; the flags force constraint_set0, 1 and 2 to one, which makes it 42e01f at level 3.1.
constexpr int kProfile = 66;
// Level 3.1 holds 3600 macroblocks per picture (1280 x 720), 4.0 holds 8192 (1920 x 1080).
constexpr uint32_t kLevel31Macroblocks = 3600;
constexpr int kLevel31 = 31;
constexpr int kLevel40 = 40;
constexpr uint32_t kConstraintFlags = 0x00070007;
// Room for SPS and PPS; the library copies them without asking for the size.
constexpr size_t kHeaderBytes = 1024;

}  // namespace

MppEncoder::MppEncoder(std::string library_path) : library_path_(std::move(library_path)) {}

MppEncoder::~MppEncoder() {
  MppEncoder::Close();
  Unload();
}

bool MppEncoder::Load(std::string& error) {
  if (library_ != nullptr) return true;
  library_ = OpenLibrary(library_path_, error);
  if (library_ == nullptr) return false;
  const bool ok =
      Symbol(library_, "mpp_create", create_, error) && Symbol(library_, "mpp_init", init_, error) &&
      Symbol(library_, "mpp_destroy", destroy_, error) && Symbol(library_, "mpp_enc_cfg_init", config_init_, error) &&
      Symbol(library_, "mpp_enc_cfg_deinit", config_deinit_, error) &&
      Symbol(library_, "mpp_enc_cfg_set_s32", config_set_s32_, error) &&
      Symbol(library_, "mpp_enc_cfg_set_u32", config_set_u32_, error) &&
      Symbol(library_, "mpp_frame_init", frame_init_, error) &&
      Symbol(library_, "mpp_frame_deinit", frame_deinit_, error) &&
      Symbol(library_, "mpp_frame_set_width", frame_set_width_, error) &&
      Symbol(library_, "mpp_frame_set_height", frame_set_height_, error) &&
      Symbol(library_, "mpp_frame_set_hor_stride", frame_set_hor_stride_, error) &&
      Symbol(library_, "mpp_frame_set_ver_stride", frame_set_ver_stride_, error) &&
      Symbol(library_, "mpp_frame_set_fmt", frame_set_format_, error) &&
      Symbol(library_, "mpp_frame_set_buffer", frame_set_buffer_, error) &&
      Symbol(library_, "mpp_frame_set_pts", frame_set_pts_, error) &&
      Symbol(library_, "mpp_buffer_import_with_tag", buffer_import_, error) &&
      Symbol(library_, "mpp_buffer_put_with_caller", buffer_put_, error) &&
      Symbol(library_, "mpp_packet_init", packet_init_, error) &&
      Symbol(library_, "mpp_packet_deinit", packet_deinit_, error) &&
      Symbol(library_, "mpp_packet_set_length", packet_set_length_, error) &&
      Symbol(library_, "mpp_packet_get_pos", packet_get_pos_, error) &&
      Symbol(library_, "mpp_packet_get_length", packet_get_length_, error);
  if (!ok) Unload();
  return ok;
}

void MppEncoder::Unload() {
  if (library_ != nullptr) dlclose(library_);
  library_ = nullptr;
}

bool MppEncoder::Probe(std::string& error) {
  return Load(error);
}

int MppEncoder::Format() const {
  return settings_.format == PixelFormat::kNv12 ? mpp::kFormatNv12 : mpp::kFormatYuyv;
}

size_t MppEncoder::FrameBytes() const {
  const size_t plane = static_cast<size_t>(settings_.line_bytes) * settings_.lines;
  return settings_.format == PixelFormat::kNv12 ? plane / 2 * 3 : plane;
}

bool MppEncoder::Fail(std::string message, std::string& error) {
  error = std::move(message);
  Close();
  return false;
}

bool MppEncoder::Open(const EncoderSettings& settings, std::string& error) {
  Close();
  if (!Load(error)) return false;
  if (settings.format != PixelFormat::kYuyv && settings.format != PixelFormat::kNv12) {
    error = "encoder takes YUYV or NV12 only";
    return false;
  }
  settings_ = settings;
  settings_.bitrate_bps = std::clamp(settings.bitrate_bps, kMinBitrate, kMaxBitrate);
  settings_.fps = std::clamp(settings.fps, 1, 60);
  int ret = create_(&context_, &api_);
  if (ret != 0 || context_ == nullptr || api_ == nullptr) {
    context_ = nullptr;
    return Fail("mpp_create " + std::to_string(ret), error);
  }
  // The wait has to be set before the context is initialised.
  int64_t wait = mpp::kWaitForever;
  ret = api_->control(context_, mpp::kSetOutputTimeout, &wait);
  if (ret != 0) return Fail("output timeout " + std::to_string(ret), error);
  ret = init_(context_, mpp::kContextEncoder, mpp::kCodingAvc);
  if (ret != 0) return Fail("mpp_init " + std::to_string(ret), error);
  return Configure(error);
}

bool MppEncoder::Configure(std::string& error) {
  mpp::Handle config = nullptr;
  int ret = config_init_(&config);
  if (ret != 0 || config == nullptr) return Fail("settings init " + std::to_string(ret), error);
  ret = api_->control(context_, mpp::kEncGetConfig, config);
  const uint32_t macroblocks = ((settings_.width + 15) / 16) * ((settings_.height + 15) / 16);
  const int level = macroblocks <= kLevel31Macroblocks ? kLevel31 : kLevel40;
  const std::pair<const char*, int32_t> values[] = {
      {"prep:width", static_cast<int32_t>(settings_.width)},
      {"prep:height", static_cast<int32_t>(settings_.height)},
      {"prep:hor_stride", static_cast<int32_t>(settings_.line_bytes)},
      {"prep:ver_stride", static_cast<int32_t>(settings_.lines)},
      {"prep:format", Format()},
      {"rc:mode", mpp::kRateControlCbr},
      {"rc:bps_target", settings_.bitrate_bps},
      {"rc:bps_max", settings_.bitrate_bps / 5 * 6},
      {"rc:bps_min", settings_.bitrate_bps / 3 * 2},
      {"rc:fps_in_flex", 0},
      {"rc:fps_in_num", settings_.fps},
      {"rc:fps_in_denom", 1},
      {"rc:fps_out_flex", 0},
      {"rc:fps_out_num", settings_.fps},
      {"rc:fps_out_denom", 1},
      {"rc:gop", settings_.fps},
      {"codec:type", mpp::kCodingAvc},
      {"h264:profile", kProfile},
      {"h264:level", level},
      {"h264:cabac_en", 0},
      {"h264:trans8x8", 0},
  };
  const char* failed = ret != 0 ? "read" : nullptr;
  for (const auto& value : values) {
    if (failed == nullptr && config_set_s32_(config, value.first, value.second) != 0) failed = value.first;
  }
  if (failed == nullptr && config_set_u32_(config, "h264:constraint_set", kConstraintFlags) != 0) {
    failed = "h264:constraint_set";
  }
  if (failed == nullptr && api_->control(context_, mpp::kEncSetConfig, config) != 0) failed = "apply";
  config_deinit_(config);
  if (failed != nullptr) return Fail(std::string("encoder settings: ") + failed, error);

  int sei = mpp::kSeiOff;
  ret = api_->control(context_, mpp::kEncSetSei, &sei);
  if (ret != 0) return Fail("SEI mode " + std::to_string(ret), error);
  int header_mode = mpp::kHeaderEachIdr;
  ret = api_->control(context_, mpp::kEncSetHeaderMode, &header_mode);
  if (ret != 0) return Fail("header mode " + std::to_string(ret), error);
  return true;
}

void MppEncoder::Close() {
  for (Imported& b : imported_) {
    if (b.buffer != nullptr) buffer_put_(b.buffer, kTag);
  }
  imported_.clear();
  if (context_ != nullptr) destroy_(context_);
  context_ = nullptr;
  api_ = nullptr;
}

bool MppEncoder::Header(std::vector<uint8_t>& header, std::string& error) {
  if (context_ == nullptr) {
    error = "encoder not open";
    return false;
  }
  std::vector<uint8_t> room(kHeaderBytes);
  mpp::Handle packet = nullptr;
  int ret = packet_init_(&packet, room.data(), room.size());
  if (ret != 0 || packet == nullptr) {
    error = "header packet " + std::to_string(ret);
    return false;
  }
  packet_set_length_(packet, 0);
  ret = api_->control(context_, mpp::kEncGetHeader, packet);
  const auto* pos = static_cast<const uint8_t*>(packet_get_pos_(packet));
  const size_t length = packet_get_length_(packet);
  const bool ok = ret == 0 && pos != nullptr && length > 0 && length <= room.size();
  if (ok) header.assign(pos, pos + length);
  packet_deinit_(&packet);
  if (!ok) error = "read header " + std::to_string(ret);
  return ok;
}

bool MppEncoder::Encode(const CapturedFrame& frame, const uint8_t*& data, size_t& size, std::string& error) {
  if (context_ == nullptr) {
    error = "encoder not open";
    return false;
  }
  if (frame.dmabuf < 0) {
    error = "frame without a dma-buf";
    return false;
  }
  if (frame.stride != settings_.line_bytes || frame.bytes < FrameBytes()) {
    error = "frame of stride " + std::to_string(frame.stride) + " and " + std::to_string(frame.bytes) +
            " bytes, the encoder takes " + std::to_string(settings_.line_bytes) + " and " +
            std::to_string(FrameBytes());
    return false;
  }
  if (frame.index >= imported_.size()) imported_.resize(frame.index + 1);
  Imported& in = imported_[frame.index];
  if (in.buffer == nullptr) {
    mpp::BufferInfo info{};
    info.type = mpp::kBufferExternalDma;
    info.size = FrameBytes();
    info.fd = frame.dmabuf;
    info.index = static_cast<int>(frame.index);
    const int ret = buffer_import_(nullptr, &info, &in.buffer, kTag, kTag);
    if (ret != 0 || in.buffer == nullptr) {
      in.buffer = nullptr;
      error = "import buffer " + std::to_string(ret);
      return false;
    }
    in.fd = frame.dmabuf;
  } else if (in.fd != frame.dmabuf) {
    error = "buffer " + std::to_string(frame.index) + " changed its dma-buf";
    return false;
  }

  mpp::Handle picture = nullptr;
  int ret = frame_init_(&picture);
  if (ret != 0 || picture == nullptr) {
    error = "frame init " + std::to_string(ret);
    return false;
  }
  frame_set_width_(picture, settings_.width);
  frame_set_height_(picture, settings_.height);
  frame_set_hor_stride_(picture, settings_.line_bytes);
  frame_set_ver_stride_(picture, settings_.lines);
  frame_set_format_(picture, Format());
  frame_set_buffer_(picture, in.buffer);
  frame_set_pts_(picture, frame.timestamp_us);
  ret = api_->encode_put_frame(context_, picture);
  mpp::Handle packet = nullptr;
  const int got = ret == 0 ? api_->encode_get_packet(context_, &packet) : 0;
  frame_deinit_(&picture);
  if (ret != 0 || got != 0 || packet == nullptr) {
    error = ret != 0 ? "put frame " + std::to_string(ret) : "no packet, " + std::to_string(got);
    return false;
  }
  const auto* pos = static_cast<const uint8_t*>(packet_get_pos_(packet));
  const size_t length = packet_get_length_(packet);
  if (pos != nullptr) output_.assign(pos, pos + length);
  packet_deinit_(&packet);
  if (pos == nullptr || length == 0) {
    error = "empty packet";
    return false;
  }
  data = output_.data();
  size = output_.size();
  return true;
}

}  // namespace tron_vision
