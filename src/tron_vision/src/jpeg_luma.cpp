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

}  // namespace tron_vision
