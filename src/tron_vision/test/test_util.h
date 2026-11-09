#ifndef TRON_VISION_TEST_UTIL_H
#define TRON_VISION_TEST_UTIL_H

#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace tron_vision::test {

// Directory removed with everything in it at the end of the test.
class TempDir {
 public:
  TempDir() {
    std::string pattern = (std::filesystem::temp_directory_path() / "tron_vision_XXXXXX").string();
    path_ = mkdtemp(pattern.data()) != nullptr ? pattern : "";
  }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;

  const std::string& Path() const {
    return path_;
  }
  std::string Write(const std::string& name, const std::string& text) const {
    const std::string file = path_ + "/" + name;
    std::ofstream(file) << text;
    return file;
  }

 private:
  std::string path_;
};

// Uniform NV12 frame.
struct Nv12Frame {
  Nv12Frame(uint32_t w, uint32_t h, uint8_t y, uint8_t u, uint8_t v) : width(w), height(h) {
    data.assign(static_cast<size_t>(w) * h, y);
    for (size_t i = 0; i < static_cast<size_t>(w) * h / 2; i += 2) {
      data.push_back(u);
      data.push_back(v);
    }
  }
  uint32_t width;
  uint32_t height;
  std::vector<uint8_t> data;
};

inline std::string Lines(size_t n) {
  std::string s;
  for (size_t i = 0; i < n; i++) s += "class" + std::to_string(i) + "\n";
  return s;
}

}  // namespace tron_vision::test

#endif  // TRON_VISION_TEST_UTIL_H
