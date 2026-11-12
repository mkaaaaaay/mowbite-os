#include "tron_vision/frame_convert.h"

#include <algorithm>
#include <cmath>

namespace tron_vision {

namespace {

constexpr uint8_t kPadGray = 114;
constexpr uint32_t kGrid = 8;

struct Tap {
  uint32_t i0;
  uint32_t i1;
  uint32_t weight;  // of i1, in 1/256
  uint32_t nearest;
};

// Source taps of each output position along one axis.
std::vector<Tap> Taps(float src_start, float src_len, uint32_t dst_len, uint32_t limit) {
  std::vector<Tap> taps(dst_len);
  const float step = src_len / static_cast<float>(dst_len);
  const auto last = static_cast<float>(limit - 1);
  for (uint32_t i = 0; i < dst_len; i++) {
    const float f = std::clamp(src_start + (static_cast<float>(i) + 0.5F) * step - 0.5F, 0.0F, last);
    const auto i0 = static_cast<uint32_t>(f);
    taps[i].i0 = i0;
    taps[i].i1 = std::min(i0 + 1, limit - 1);
    taps[i].weight = static_cast<uint32_t>((f - static_cast<float>(i0)) * 256.0F);
    taps[i].nearest = static_cast<uint32_t>(std::lround(f));
  }
  return taps;
}

uint8_t Clamp8(int v) {
  return static_cast<uint8_t>(std::clamp(v, 0, 255));
}

}  // namespace

Placement PlaceFrame(uint32_t width, uint32_t height, Fit fit, uint32_t out_width, uint32_t out_height) {
  Placement p;
  const auto w = static_cast<float>(width);
  const auto h = static_cast<float>(height);
  const auto ow = static_cast<float>(out_width);
  const auto oh = static_cast<float>(out_height);
  if (fit == Fit::kCrop) {
    p.dst_w = out_width;
    p.dst_h = out_height;
    if (w * oh > h * ow) {
      p.src_h = h;
      p.src_w = h * ow / oh;
      p.src_x = (w - p.src_w) / 2.0F;
    } else {
      p.src_w = w;
      p.src_h = w * oh / ow;
      p.src_y = (h - p.src_h) / 2.0F;
    }
    return p;
  }
  const float scale = std::min(ow / w, oh / h);
  p.src_w = w;
  p.src_h = h;
  p.dst_w = std::min(out_width, static_cast<uint32_t>(std::lround(w * scale)));
  p.dst_h = std::min(out_height, static_cast<uint32_t>(std::lround(h * scale)));
  p.dst_x = (out_width - p.dst_w) / 2;
  p.dst_y = (out_height - p.dst_h) / 2;
  return p;
}

Detection ToSource(const Detection& d, const Placement& p) {
  const float sx = p.src_w / static_cast<float>(p.dst_w);
  const float sy = p.src_h / static_cast<float>(p.dst_h);
  auto x = [&](float v) {
    return std::clamp(p.src_x + (v - static_cast<float>(p.dst_x)) * sx, p.src_x, p.src_x + p.src_w);
  };
  auto y = [&](float v) {
    return std::clamp(p.src_y + (v - static_cast<float>(p.dst_y)) * sy, p.src_y, p.src_y + p.src_h);
  };
  Detection out = d;
  out.x1 = x(d.x1);
  out.x2 = x(d.x2);
  out.y1 = y(d.y1);
  out.y2 = y(d.y2);
  return out;
}

FrameStats MeasureFrame(const Nv12View& frame, const Placement& p) {
  FrameStats stats;
  const auto x0 = static_cast<uint32_t>(p.src_x);
  const auto y0 = static_cast<uint32_t>(p.src_y);
  const uint32_t x1 = std::min(frame.width, static_cast<uint32_t>(p.src_x + p.src_w));
  const uint32_t y1 = std::min(frame.height, static_cast<uint32_t>(p.src_y + p.src_h));
  uint64_t sy = 0;
  uint64_t su = 0;
  uint64_t sv = 0;
  uint64_t n = 0;
  for (uint32_t row = y0; row < y1; row += kGrid) {
    const uint8_t* yl = frame.y + static_cast<size_t>(row) * frame.stride;
    const uint8_t* uvl = frame.uv + static_cast<size_t>(row / 2) * frame.stride;
    for (uint32_t col = x0; col < x1; col += kGrid) {
      sy += yl[col];
      su += uvl[col & ~1U];
      sv += uvl[(col & ~1U) + 1];
      n++;
    }
  }
  if (n == 0) return stats;
  const auto fn = static_cast<float>(n);
  stats.y = static_cast<float>(sy) / fn;
  stats.u = static_cast<float>(su) / fn;
  stats.v = static_cast<float>(sv) / fn;
  return stats;
}

ColorGains GrayWorld(const FrameStats& stats) {
  const float c = 1.164F * (stats.y - 16.0F);
  const float d = stats.u - 128.0F;
  const float e = stats.v - 128.0F;
  const float r = c + 1.596F * e;
  const float g = c - 0.392F * d - 0.813F * e;
  const float b = c + 2.017F * d;
  ColorGains gains;
  // Too dark for a color estimate.
  if (r < 4.0F || g < 4.0F || b < 4.0F) return gains;
  auto gain = [g](float v) { return static_cast<uint16_t>(std::lround(std::clamp(g / v, 0.5F, 3.0F) * 256.0F)); };
  gains.r = gain(r);
  gains.b = gain(b);
  return gains;
}

void ShiftChroma(uint8_t* uv, uint32_t width, uint32_t height, uint32_t stride, int du, int dv) {
  if (du == 0 && dv == 0) return;
  uint8_t u_map[256];
  uint8_t v_map[256];
  for (int i = 0; i < 256; i++) {
    u_map[i] = Clamp8(i - du);
    v_map[i] = Clamp8(i - dv);
  }
  const uint32_t pairs = width / 2;
  for (uint32_t line = 0; line < height / 2; line++) {
    uint8_t* p = uv + static_cast<size_t>(line) * stride;
    for (uint32_t i = 0; i < pairs; i++) {
      p[2 * i] = u_map[p[2 * i]];
      p[2 * i + 1] = v_map[p[2 * i + 1]];
    }
  }
}

void ConvertFrame(const Nv12View& frame, const Placement& p, const ColorGains& gains, bool swap_rb,
                  uint32_t out_width, uint32_t out_height, std::vector<uint8_t>& rgb) {
  rgb.resize(static_cast<size_t>(out_width) * out_height * 3);
  // Only bars outside the placed area keep the pad gray; the placed area is written below.
  if (p.dst_w != out_width || p.dst_h != out_height) std::fill(rgb.begin(), rgb.end(), kPadGray);
  if (p.dst_w == 0 || p.dst_h == 0) return;
  const std::vector<Tap> cols = Taps(p.src_x, p.src_w, p.dst_w, frame.width);
  const std::vector<Tap> rows = Taps(p.src_y, p.src_h, p.dst_h, frame.height);
  for (uint32_t j = 0; j < p.dst_h; j++) {
    const Tap& ty = rows[j];
    const uint8_t* r0 = frame.y + static_cast<size_t>(ty.i0) * frame.stride;
    const uint8_t* r1 = frame.y + static_cast<size_t>(ty.i1) * frame.stride;
    const uint8_t* uvl = frame.uv + static_cast<size_t>(ty.nearest / 2) * frame.stride;
    uint8_t* out = rgb.data() + ((static_cast<size_t>(p.dst_y) + j) * out_width + p.dst_x) * 3;
    for (uint32_t i = 0; i < p.dst_w; i++) {
      const Tap& tx = cols[i];
      const uint32_t top = r0[tx.i0] * (256 - tx.weight) + r0[tx.i1] * tx.weight;
      const uint32_t bottom = r1[tx.i0] * (256 - tx.weight) + r1[tx.i1] * tx.weight;
      const int luma = static_cast<int>((top * (256 - ty.weight) + bottom * ty.weight + 32768) >> 16);
      const uint32_t uv = std::min(tx.nearest, frame.width - 1) & ~1U;
      const int c = 298 * (luma - 16);
      const int d = uvl[uv] - 128;
      const int e = uvl[uv + 1] - 128;
      const int r = Clamp8((c + 409 * e + 128) / 256);
      const int g = Clamp8((c - 100 * d - 208 * e + 128) / 256);
      const int b = Clamp8((c + 516 * d + 128) / 256);
      out[swap_rb ? 2 : 0] = Clamp8((r * gains.r + 128) / 256);
      out[1] = Clamp8((g * gains.g + 128) / 256);
      out[swap_rb ? 0 : 2] = Clamp8((b * gains.b + 128) / 256);
      out += 3;
    }
  }
}

}  // namespace tron_vision
