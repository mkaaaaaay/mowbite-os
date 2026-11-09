#ifndef TRON_VISION_FRAME_CONVERT_H
#define TRON_VISION_FRAME_CONVERT_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "tron_vision/yolo_decode.h"

namespace tron_vision {

// NV12 image: full-size luma plane, then interleaved U/V at half size; both with the same line stride.
struct Nv12View {
  const uint8_t* y = nullptr;
  const uint8_t* uv = nullptr;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
};

enum class Fit {
  kCrop,  // centered 3:4 part of the frame fills the model input
  kPad,   // whole frame, gray bars above and below
};

// Which part of the source frame lands where in the model input.
struct Placement {
  float src_x = 0.0F;
  float src_y = 0.0F;
  float src_w = 0.0F;
  float src_h = 0.0F;
  uint32_t dst_x = 0;
  uint32_t dst_y = 0;
  uint32_t dst_w = 0;
  uint32_t dst_h = 0;
};

Placement PlaceFrame(uint32_t width, uint32_t height, Fit fit, uint32_t out_width, uint32_t out_height);

// Box in model pixels to source pixels, clipped to the used part of the source.
Detection ToSource(const Detection& d, const Placement& p);

// Channel gains in 1/256; 256 each leaves the colors as they are.
struct ColorGains {
  uint16_t r = 256;
  uint16_t g = 256;
  uint16_t b = 256;
};

// Means over a grid of the region in `p` (every 8th pixel), luma and chroma in 0..255.
struct FrameStats {
  float y = 0.0F;
  float u = 128.0F;
  float v = 128.0F;
};

FrameStats MeasureFrame(const Nv12View& frame, const Placement& p);

// Gray world: gains that bring the mean color to gray, each within 0.5..3.
ColorGains GrayWorld(const FrameStats& stats);

// RGB, 3 bytes per pixel, out_width * out_height; outside the placed area the pad gray. Luma bilinear, chroma
// nearest, BT.601 limited range.
void ConvertFrame(const Nv12View& frame, const Placement& p, const ColorGains& gains, bool swap_rb,
                  uint32_t out_width, uint32_t out_height, std::vector<uint8_t>& rgb);

}  // namespace tron_vision

#endif  // TRON_VISION_FRAME_CONVERT_H
