#include "tron_vision/jpeg_luma.h"

#include <turbojpeg.h>

namespace tron_vision {

JpegLuma::JpegLuma() : handle_(tjInitDecompress()) {}

JpegLuma::~JpegLuma() {
  if (handle_ != nullptr) tjDestroy(handle_);
}

bool JpegLuma::Decode(const uint8_t* jpeg, size_t bytes, uint32_t width, uint32_t height, GrayView& luma,
                      std::string& error) {
  if (handle_ == nullptr) {
    error = "no JPEG decoder";
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
  plane_.resize(static_cast<size_t>(width) * height);
  // Gray output takes the Y component as it is and leaves out the chroma.
  if (tjDecompress2(handle_, data, bytes, plane_.data(), w, 0, h, TJPF_GRAY, 0) != 0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    return false;
  }
  luma = {plane_.data(), width, height, width};
  return true;
}

YuyvGrayJpeg::YuyvGrayJpeg() : handle_(tjInitCompress()) {}

YuyvGrayJpeg::~YuyvGrayJpeg() {
  if (handle_ != nullptr) tjDestroy(handle_);
}

bool YuyvGrayJpeg::Encode(const uint8_t* yuyv, uint32_t width, uint32_t height, uint32_t line_bytes, int quality,
                          std::vector<uint8_t>& jpeg, std::string& error) {
  jpeg.clear();
  if (handle_ == nullptr) {
    error = "no JPEG encoder";
    return false;
  }
  if (yuyv == nullptr || width == 0 || height == 0 || line_bytes < 2 * width) {
    error = "no YUYV frame of " + std::to_string(width) + "x" + std::to_string(height);
    return false;
  }
  gray_.resize(static_cast<size_t>(width) * height);
  uint8_t* out = gray_.data();
  for (uint32_t y = 0; y < height; y++) {
    const uint8_t* line = yuyv + static_cast<size_t>(y) * line_bytes;
    for (uint32_t x = 0; x < width; x++) *out++ = line[2 * x];
  }
  const auto w = static_cast<int>(width);
  const auto h = static_cast<int>(height);
  // Large enough for any quality, so the library never reallocates the buffer.
  jpeg.resize(tjBufSize(w, h, TJSAMP_GRAY));
  unsigned char* data = jpeg.data();
  unsigned long size = jpeg.size();
  if (tjCompress2(handle_, gray_.data(), w, 0, h, TJPF_GRAY, &data, &size, TJSAMP_GRAY, quality, TJFLAG_NOREALLOC) !=
      0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    jpeg.clear();
    return false;
  }
  jpeg.resize(size);
  return true;
}

YuyvJpeg::YuyvJpeg() : handle_(tjInitCompress()) {}

YuyvJpeg::~YuyvJpeg() {
  if (handle_ != nullptr) tjDestroy(handle_);
}

bool YuyvJpeg::Encode(const uint8_t* yuyv, uint32_t width, uint32_t height, uint32_t line_bytes, int quality,
                      std::vector<uint8_t>& jpeg, std::string& error) {
  jpeg.clear();
  if (handle_ == nullptr) {
    error = "no JPEG encoder";
    return false;
  }
  if (yuyv == nullptr || width < 2 || width % 2 != 0 || height == 0 || line_bytes < 2 * width) {
    error = "no YUYV frame of " + std::to_string(width) + "x" + std::to_string(height);
    return false;
  }
  const size_t half = width / 2;
  y_.resize(static_cast<size_t>(width) * height);
  u_.resize(half * height);
  v_.resize(half * height);
  for (uint32_t y = 0; y < height; y++) {
    const uint8_t* line = yuyv + static_cast<size_t>(y) * line_bytes;
    uint8_t* luma = y_.data() + static_cast<size_t>(y) * width;
    uint8_t* u = u_.data() + y * half;
    uint8_t* v = v_.data() + y * half;
    for (size_t x = 0; x < half; x++) {
      luma[2 * x] = line[4 * x];
      u[x] = line[4 * x + 1];
      luma[2 * x + 1] = line[4 * x + 2];
      v[x] = line[4 * x + 3];
    }
  }
  const auto w = static_cast<int>(width);
  const auto h = static_cast<int>(height);
  const unsigned char* planes[3] = {y_.data(), u_.data(), v_.data()};
  const int strides[3] = {w, w / 2, w / 2};
  jpeg.resize(tjBufSize(w, h, TJSAMP_422));
  unsigned char* data = jpeg.data();
  unsigned long size = jpeg.size();
  if (tjCompressFromYUVPlanes(handle_, planes, w, strides, h, TJSAMP_422, &data, &size, quality, TJFLAG_NOREALLOC) !=
      0) {
    error = std::string("JPEG: ") + tjGetErrorStr2(handle_);
    jpeg.clear();
    return false;
  }
  jpeg.resize(size);
  return true;
}

}  // namespace tron_vision
