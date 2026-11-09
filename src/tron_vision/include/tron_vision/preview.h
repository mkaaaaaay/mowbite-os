#ifndef TRON_VISION_PREVIEW_H
#define TRON_VISION_PREVIEW_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tron_vision/frame_convert.h"
#include "tron_vision/yolo_decode.h"

namespace tron_vision {

// RGB to baseline JPEG with 4:2:0 chroma through libturbojpeg; one handle per object.
class JpegEncoder {
 public:
  JpegEncoder();
  ~JpegEncoder();
  JpegEncoder(const JpegEncoder&) = delete;
  JpegEncoder& operator=(const JpegEncoder&) = delete;

  bool Encode(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height, int quality, std::vector<uint8_t>& jpeg,
              std::string& error);

 private:
  void* handle_ = nullptr;
};

// Box outlines, 2 pixels wide, a color per class; boxes in source pixels, `scale` maps them onto the RGB image.
void DrawBoxes(std::vector<uint8_t>& rgb, uint32_t width, uint32_t height, const std::vector<Detection>& boxes,
               float scale);

// Size of the picture of a frame: at most `max_width` wide, same aspect, both even.
void PreviewSize(uint32_t frame_width, uint32_t frame_height, uint32_t max_width, uint32_t& width, uint32_t& height);

struct PreviewConfig {
  uint32_t width = 640;
  // Pictures per second, held within 0.5..5.
  double max_rate = 5.0;
  size_t max_bytes = 100UL * 1024;
  int quality = 80;
  int min_quality = 30;
};

// The shown image of a camera: the frame scaled to the configured width, white balanced, with the boxes, as JPEG
// within the size limit. The quality steps down when a picture is too big and slowly up again when there is room.
class Preview {
 public:
  explicit Preview(PreviewConfig config);

  bool Due(double now) const {
    return now >= next_;
  }
  // False with `error` if encoding failed; a picture that stays over the size limit at the lowest quality is dropped.
  bool Make(const Nv12View& frame, const ColorGains& gains, const std::vector<Detection>& boxes, double now,
            std::string& error);

  const std::vector<uint8_t>& Jpeg() const {
    return jpeg_;
  }
  uint32_t Width() const {
    return width_;
  }
  uint32_t Height() const {
    return height_;
  }
  // Pictures made since the start; a change means a new one.
  uint64_t Count() const {
    return count_;
  }
  int Quality() const {
    return quality_;
  }

 private:
  PreviewConfig config_;
  JpegEncoder encoder_;
  std::vector<uint8_t> rgb_;
  std::vector<uint8_t> jpeg_;
  std::vector<uint8_t> candidate_;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  int quality_;
  double next_ = 0.0;
  uint64_t count_ = 0;
};

}  // namespace tron_vision

#endif  // TRON_VISION_PREVIEW_H
