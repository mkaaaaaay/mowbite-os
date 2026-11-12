#ifndef TRON_VISION_ENCODER_LIBRARY_H
#define TRON_VISION_ENCODER_LIBRARY_H

#include <ament_index_cpp/get_package_prefix.hpp>
#include <exception>
#include <filesystem>
#include <string>

namespace tron_vision {

// The image puts the media library next to the packages, under <install>/mpp; empty if tron_vision is not installed.
inline std::string DefaultEncoderLibrary() {
  try {
    const std::filesystem::path prefix = ament_index_cpp::get_package_prefix("tron_vision");
    return (prefix.parent_path() / "mpp/lib/librockchip_mpp.so.1").string();
  } catch (const std::exception&) {
    return "";
  }
}

}  // namespace tron_vision

#endif  // TRON_VISION_ENCODER_LIBRARY_H
