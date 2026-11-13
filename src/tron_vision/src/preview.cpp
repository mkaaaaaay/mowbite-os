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

// BT.601 limited range.
struct Yuv {
  uint8_t y;
  uint8_t u;
  uint8_t v;
};

Yuv ToYuv(const uint8_t* rgb) {
  const double r = rgb[0];
  const double g = rgb[1];
  const double b = rgb[2];
  return {static_cast<uint8_t>(std::lround(16.0 + (65.481 * r + 128.553 * g + 24.966 * b) / 255.0)),
          static_cast<uint8_t>(std::lround(128.0 + (-37.797 * r - 74.203 * g + 112.0 * b) / 255.0)),
          static_cast<uint8_t>(std::lround(128.0 + (112.0 * r - 93.786 * g - 18.214 * b) / 255.0))};
}

// Corners on even pixels.
void FillRect(const Nv12Buffer& f, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Yuv& c) {
  for (uint32_t y = y0; y < y1; y++) std::fill_n(f.y + static_cast<size_t>(y) * f.stride + x0, x1 - x0, c.y);
  for (uint32_t y = y0 / 2; y < y1 / 2; y++) {
    uint8_t* p = f.uv + static_cast<size_t>(y) * f.stride;
    for (uint32_t x = x0; x < x1; x += 2) {
      p[x] = c.u;
      p[x + 1] = c.v;
    }
  }
}

uint32_t Even(uint32_t v) {
  return v & ~1U;
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

bool JpegDecoder::Open(const uint8_t* jpeg, size_t bytes, uint32_t width, uint32_t height, JpegScale scale,
                       int& subsampling, std::string& error) {
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
  int colorspace = 0;
  // libturbojpeg takes a non-const pointer but does not write through it.
  if (tjDecompressHeader3(handle_, const_cast<uint8_t*>(jpeg), bytes, &w, &h, &subsampling, &colorspace) != 0) {
    error = std::string("JPEG header: ") + tjGetErrorStr2(handle_);
    return false;
  }
  if (w != static_cast<int>(width) || h != static_cast<int>(height)) {
    error = "JPEG of " + std::to_string(w) + "x" + std::to_string(h);
    return false;
  }
  return true;
}

bool JpegDecoder::Decode(const uint8_t* jpeg, size_t bytes, uint32_t width, uint32_t height, JpegScale scale,
                         std::vector<uint8_t>& rgb, std::string& error) {
  int subsampling = 0;
  if (!Open(jpeg, bytes, width, height, scale, subsampling, error)) return false;
  const auto out_w = static_cast<int>(ScaledSize(width, scale));
  const auto out_h = static_cast<int>(ScaledSize(height, scale));
  rgb.resize(static_cast<size_t>(out_w) * out_h * 3);
  // With the scaled size asked for, the library picks exactly that scale.
  if (tjDecompress2(handle_, const_cast<uint8_t*>(jpeg), bytes, rgb.data(), out_w, 0, out_h, TJPF_RGB,
                    TJFLAG_FASTDCT) != 0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    return false;
  }
  return true;
}

bool JpegDecoder::DecodeNv12(const uint8_t* jpeg, size_t bytes, uint32_t width, uint32_t height, JpegScale scale,
                             const Nv12Buffer& out, uint32_t lines, std::string& error) {
  const uint32_t out_w = ScaledSize(width, scale);
  const uint32_t out_h = ScaledSize(height, scale);
  if (out.width != out_w || out.height != out_h || out.stride < out_w || lines < out_h || out_h == 0) {
    error = "NV12 frame of " + std::to_string(out.width) + "x" + std::to_string(out.height) + " for a picture of " +
            std::to_string(out_w) + "x" + std::to_string(out_h);
    return false;
  }
  int subsampling = 0;
  if (!Open(jpeg, bytes, width, height, scale, subsampling, error)) return false;
  const bool gray = subsampling == TJSAMP_GRAY;
  const auto w = static_cast<int>(out_w);
  const auto h = static_cast<int>(out_h);
  const int chroma_plane_w = gray ? 0 : tjPlaneWidth(1, w, subsampling);
  const int chroma_plane_h = gray ? 0 : tjPlaneHeight(1, h, subsampling);
  if (chroma_plane_w < 0 || chroma_plane_h < 0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    return false;
  }
  const auto plane_w = static_cast<uint32_t>(chroma_plane_w);
  const auto plane_h = static_cast<uint32_t>(chroma_plane_h);
  u_.resize(static_cast<size_t>(plane_w) * plane_h);
  v_.resize(u_.size());
  unsigned char* planes[3] = {out.y, u_.data(), v_.data()};
  int strides[3] = {static_cast<int>(out.stride), chroma_plane_w, chroma_plane_w};
  // Without TJFLAG_FASTDCT: with it libturbojpeg 2.1 gives four times the chroma of 4:2:0 at half size.
  if (tjDecompressToYUVPlanes(handle_, const_cast<uint8_t*>(jpeg), bytes, planes, w, strides, h, 0) != 0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    return false;
  }
  // JPEG uses the full range, the encoder signals the limited one of BT.601.
  uint8_t luma[256];
  uint8_t chroma[256];
  for (int i = 0; i < 256; i++) {
    luma[i] = static_cast<uint8_t>((i * 219 + 127) / 255 + 16);
    chroma[i] = static_cast<uint8_t>(((i - 128) * 224 + (i < 128 ? -127 : 127)) / 255 + 128);
  }
  for (uint32_t y = 0; y < out_h; y++) {
    uint8_t* row = out.y + static_cast<size_t>(y) * out.stride;
    for (uint32_t x = 0; x < out_w; x++) row[x] = luma[row[x]];
  }
  // Nearest chroma sample of each 2 x 2 block.
  const uint32_t chroma_w = (out_w + 1) / 2;
  const uint32_t chroma_h = (out_h + 1) / 2;
  for (uint32_t y = 0; y < chroma_h; y++) {
    uint8_t* row = out.uv + static_cast<size_t>(y) * out.stride;
    if (gray) {
      std::fill_n(row, chroma_w * 2, 128);
      continue;
    }
    const size_t from = static_cast<size_t>(std::min(2 * y * plane_h / out_h, plane_h - 1)) * plane_w;
    for (uint32_t x = 0; x < chroma_w; x++) {
      const size_t at = from + std::min(2 * x * plane_w / out_w, plane_w - 1);
      const size_t pair = 2 * static_cast<size_t>(x);
      row[pair] = chroma[u_[at]];
      row[pair + 1] = chroma[v_[at]];
    }
  }
  // The rows below the picture repeat its last one, so that the encoder reads whole macroblocks of it.
  const uint8_t* last_luma = out.y + static_cast<size_t>(out_h - 1) * out.stride;
  for (uint32_t y = out_h; y < lines; y++) std::copy_n(last_luma, out_w, out.y + static_cast<size_t>(y) * out.stride);
  const uint8_t* last_chroma = out.uv + static_cast<size_t>(chroma_h - 1) * out.stride;
  for (uint32_t y = chroma_h; y < lines / 2; y++) {
    std::copy_n(last_chroma, chroma_w * 2, out.uv + static_cast<size_t>(y) * out.stride);
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

void DrawBoxes(const Nv12Buffer& frame, const std::vector<Detection>& boxes, uint32_t line) {
  const uint32_t w = Even(frame.width);
  const uint32_t h = Even(frame.height);
  const uint32_t t = std::max(2U, Even(line));
  for (const Detection& d : boxes) {
    const uint32_t x0 = Even(Pixel(d.x1, w));
    const uint32_t y0 = Even(Pixel(d.y1, h));
    const uint32_t x1 = std::min(Even(Pixel(d.x2, w) + 1), w);
    const uint32_t y1 = std::min(Even(Pixel(d.y2, h) + 1), h);
    if (x1 <= x0 || y1 <= y0) continue;
    const Yuv c = ToYuv(kColors[d.class_id % std::size(kColors)]);
    FillRect(frame, x0, y0, x1, std::min(y0 + t, y1), c);
    FillRect(frame, x0, y1 > t ? std::max(y1 - t, y0) : y0, x1, y1, c);
    FillRect(frame, x0, y0, std::min(x0 + t, x1), y1, c);
    FillRect(frame, x1 > t ? std::max(x1 - t, x0) : x0, y0, x1, y1, c);
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
