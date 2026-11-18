#ifndef TRON_VISION_JPEG_LUMA_H
#define TRON_VISION_JPEG_LUMA_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tron_vision/dock_marker.h"

namespace tron_vision {

// The luma plane of a JPEG through libturbojpeg, at full size; the chroma is not decoded. One handle per object.
class JpegLuma {
 public:
  JpegLuma();
  ~JpegLuma();
  JpegLuma(const JpegLuma&) = delete;
  JpegLuma& operator=(const JpegLuma&) = delete;

  // False with the error if the data is no JPEG of exactly this size; the view is valid until the next call.
  bool Decode(const uint8_t* jpeg, size_t bytes, uint32_t width, uint32_t height, GrayView& luma, std::string& error);

 private:
  void* handle_ = nullptr;
  std::vector<uint8_t> plane_;
};

// The luma of a YUYV frame as a gray JPEG through libturbojpeg; the chroma is left out. One handle per object.
class YuyvGrayJpeg {
 public:
  YuyvGrayJpeg();
  ~YuyvGrayJpeg();
  YuyvGrayJpeg(const YuyvGrayJpeg&) = delete;
  YuyvGrayJpeg& operator=(const YuyvGrayJpeg&) = delete;

  // `yuyv` holds `height` lines of `line_bytes` bytes, the first 2 * width of each are the picture. False with the
  // error, then `jpeg` is empty.
  bool Encode(const uint8_t* yuyv, uint32_t width, uint32_t height, uint32_t line_bytes, int quality,
              std::vector<uint8_t>& jpeg, std::string& error);

 private:
  void* handle_ = nullptr;
  std::vector<uint8_t> gray_;
};

// A YUYV picture as a color JPEG with 4:2:2 chroma through libturbojpeg. One handle per object.
class YuyvJpeg {
 public:
  YuyvJpeg();
  ~YuyvJpeg();
  YuyvJpeg(const YuyvJpeg&) = delete;
  YuyvJpeg& operator=(const YuyvJpeg&) = delete;

  // As YuyvGrayJpeg::Encode; the width must be even.
  bool Encode(const uint8_t* yuyv, uint32_t width, uint32_t height, uint32_t line_bytes, int quality,
              std::vector<uint8_t>& jpeg, std::string& error);

 private:
  void* handle_ = nullptr;
  std::vector<uint8_t> y_;
  std::vector<uint8_t> u_;
  std::vector<uint8_t> v_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_JPEG_LUMA_H
