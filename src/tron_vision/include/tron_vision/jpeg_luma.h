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

}  // namespace tron_vision

#endif  // TRON_VISION_JPEG_LUMA_H
