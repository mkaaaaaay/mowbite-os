#ifndef TRON_VISION_DOCK_JPEG_H
#define TRON_VISION_DOCK_JPEG_H

#include <turbojpeg.h>

#include <cstdint>
#include <vector>

#include "dock_scene.h"

namespace tron_vision::test {

// A drawn image as the rear camera sends it: baseline JPEG with 4:2:2 chroma; empty if the encoder fails.
inline std::vector<uint8_t> EncodeJpeg(const SceneImage& image, int quality = 90) {
  std::vector<uint8_t> rgb;
  rgb.reserve(image.px.size() * 3);
  for (uint8_t v : image.px) rgb.insert(rgb.end(), {v, v, v});
  tjhandle handle = tjInitCompress();
  unsigned char* out = nullptr;
  unsigned long size = 0;
  const int failed = tjCompress2(handle, rgb.data(), static_cast<int>(image.width), 0, static_cast<int>(image.height),
                                 TJPF_RGB, &out, &size, TJSAMP_422, quality, 0);
  std::vector<uint8_t> jpeg;
  if (failed == 0) jpeg.assign(out, out + size);
  tjFree(out);
  tjDestroy(handle);
  return jpeg;
}

}  // namespace tron_vision::test

#endif  // TRON_VISION_DOCK_JPEG_H
