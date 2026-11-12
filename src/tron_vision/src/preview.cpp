#include "tron_vision/preview.h"

#include <turbojpeg.h>

#include <algorithm>
#include <cmath>

namespace tron_vision {

namespace {

constexpr int kQualityStepDown = 10;
constexpr int kQualityStepUp = 5;
constexpr uint32_t kLineWidth = 2;
constexpr uint8_t kColors[][3] = {{255, 64, 64}, {64, 230, 64}, {64, 160, 255},
                                  {255, 220, 0}, {255, 0, 255}, {0, 255, 255}};

void FillRect(std::vector<uint8_t>& rgb, uint32_t width, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
              const uint8_t* color) {
  for (uint32_t y = y0; y < y1; y++) {
    uint8_t* p = rgb.data() + (static_cast<size_t>(y) * width + x0) * 3;
    for (uint32_t x = x0; x < x1; x++, p += 3) std::copy(color, color + 3, p);
  }
}

uint32_t Pixel(float v, uint32_t size) {
  return static_cast<uint32_t>(std::clamp(std::lround(v), 0L, static_cast<long>(size)));
}

}  // namespace

JpegEncoder::JpegEncoder() : handle_(tjInitCompress()) {}

JpegEncoder::~JpegEncoder() {
  if (handle_ != nullptr) tjDestroy(handle_);
}

bool JpegEncoder::Encode(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height, int quality,
                         std::vector<uint8_t>& jpeg, std::string& error) {
  if (handle_ == nullptr) {
    error = "no JPEG encoder";
    return false;
  }
  const auto w = static_cast<int>(width);
  const auto h = static_cast<int>(height);
  if (rgb.size() != static_cast<size_t>(width) * height * 3) {
    error = "RGB buffer does not match the image size";
    return false;
  }
  // Large enough for any quality, so the library never reallocates the buffer.
  jpeg.resize(tjBufSize(w, h, TJSAMP_420));
  unsigned char* out = jpeg.data();
  unsigned long size = jpeg.size();
  if (tjCompress2(handle_, rgb.data(), w, 0, h, TJPF_RGB, &out, &size, TJSAMP_420, quality,
                  TJFLAG_NOREALLOC | TJFLAG_FASTDCT) != 0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    jpeg.clear();
    return false;
  }
  jpeg.resize(size);
  return true;
}

uint32_t ScaledSize(uint32_t size, JpegScale scale) {
  if (scale.num <= 0 || scale.denom <= 0) return 0;
  const auto num = static_cast<uint64_t>(scale.num);
  const auto denom = static_cast<uint64_t>(scale.denom);
  return static_cast<uint32_t>((size * num + denom - 1) / denom);
}

JpegDecoder::JpegDecoder() : handle_(tjInitDecompress()) {}

JpegDecoder::~JpegDecoder() {
  if (handle_ != nullptr) tjDestroy(handle_);
}

bool JpegDecoder::Decode(const uint8_t* jpeg, size_t bytes, uint32_t width, uint32_t height, JpegScale scale,
                         std::vector<uint8_t>& rgb, std::string& error) {
  if (handle_ == nullptr) {
    error = "no JPEG decoder";
    return false;
  }
  int count = 0;
  const tjscalingfactor* factors = tjGetScalingFactors(&count);
  bool offered = false;
  for (int i = 0; factors != nullptr && i < count; i++) {
    offered = offered || (factors[i].num * scale.denom == scale.num * factors[i].denom);
  }
  if (!offered) {
    error = "no decoder scale " + std::to_string(scale.num) + "/" + std::to_string(scale.denom);
    return false;
  }
  int w = 0;
  int h = 0;
  int subsampling = 0;
  int colorspace = 0;
  // libturbojpeg takes a non-const pointer but does not write through it.
  auto* data = const_cast<uint8_t*>(jpeg);
  if (tjDecompressHeader3(handle_, data, bytes, &w, &h, &subsampling, &colorspace) != 0) {
    error = std::string("JPEG header: ") + tjGetErrorStr2(handle_);
    return false;
  }
  if (w != static_cast<int>(width) || h != static_cast<int>(height)) {
    error = "JPEG of " + std::to_string(w) + "x" + std::to_string(h);
    return false;
  }
  const auto out_w = static_cast<int>(ScaledSize(width, scale));
  const auto out_h = static_cast<int>(ScaledSize(height, scale));
  rgb.resize(static_cast<size_t>(out_w) * out_h * 3);
  // With the scaled size asked for, the library picks exactly that scale.
  if (tjDecompress2(handle_, data, bytes, rgb.data(), out_w, 0, out_h, TJPF_RGB, TJFLAG_FASTDCT) != 0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    return false;
  }
  return true;
}

void DrawBoxes(std::vector<uint8_t>& rgb, uint32_t width, uint32_t height, const std::vector<Detection>& boxes,
               float scale) {
  for (const Detection& d : boxes) {
    const uint32_t x0 = Pixel(d.x1 * scale, width);
    const uint32_t y0 = Pixel(d.y1 * scale, height);
    const uint32_t x1 = Pixel(d.x2 * scale, width);
    const uint32_t y1 = Pixel(d.y2 * scale, height);
    if (x1 <= x0 || y1 <= y0) continue;
    const uint8_t* color = kColors[d.class_id % std::size(kColors)];
    const uint32_t t = kLineWidth;
    FillRect(rgb, width, x0, y0, x1, std::min(y0 + t, y1), color);
    FillRect(rgb, width, x0, y1 > t ? std::max(y1 - t, y0) : y0, x1, y1, color);
    FillRect(rgb, width, x0, y0, std::min(x0 + t, x1), y1, color);
    FillRect(rgb, width, x1 > t ? std::max(x1 - t, x0) : x0, y0, x1, y1, color);
  }
}

void PreviewSize(uint32_t frame_width, uint32_t frame_height, uint32_t max_width, uint32_t& width, uint32_t& height) {
  width = std::min(max_width, frame_width) & ~1U;
  height = 0;
  if (frame_width == 0) return;
  height = static_cast<uint32_t>(std::lround(static_cast<double>(frame_height) * width / frame_width)) & ~1U;
}

BoundedJpeg::BoundedJpeg(JpegLimits limits) : limits_(limits), quality_(limits.quality) {
  limits_.max_rate = std::clamp(limits_.max_rate, 0.5, 5.0);
  limits_.min_quality = std::clamp(limits_.min_quality, 1, limits_.quality);
}

bool BoundedJpeg::Make(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height, double now,
                       std::string& error) {
  // On time: the next slot; after a pause: one period from now, so no burst to catch up.
  const double period = 1.0 / limits_.max_rate;
  next_ = next_ + period > now ? next_ + period : now + period;
  for (;;) {
    if (!encoder_.Encode(rgb, width, height, quality_, candidate_, error)) return false;
    if (candidate_.size() <= limits_.max_bytes) break;
    if (quality_ <= limits_.min_quality) return true;
    quality_ = std::max(quality_ - kQualityStepDown, limits_.min_quality);
  }
  if (candidate_.size() < limits_.max_bytes / 2) quality_ = std::min(quality_ + kQualityStepUp, limits_.quality);
  jpeg_.swap(candidate_);
  width_ = width;
  height_ = height;
  count_++;
  return true;
}

Preview::Preview(PreviewConfig config) : width_(config.width), jpeg_(config) {}

bool Preview::Make(const Nv12View& frame, const ColorGains& gains, const std::vector<Detection>& boxes, double now,
                   std::string& error) {
  uint32_t out_w = 0;
  uint32_t out_h = 0;
  PreviewSize(frame.width, frame.height, width_, out_w, out_h);
  if (out_w == 0 || out_h == 0) {
    error = "frame too small for a picture";
    return false;
  }
  ConvertFrame(frame, PlaceFrame(frame.width, frame.height, Fit::kPad, out_w, out_h), gains, false, out_w, out_h,
               rgb_);
  DrawBoxes(rgb_, out_w, out_h, boxes, static_cast<float>(out_w) / static_cast<float>(frame.width));
  return jpeg_.Make(rgb_, out_w, out_h, now, error);
}

}  // namespace tron_vision
