#ifndef MOWBITE_SOUND_TEST_TEMP_DIR_H
#define MOWBITE_SOUND_TEST_TEMP_DIR_H

#include <stdlib.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

// A new folder under the temp directory, removed with all in it at the end.
struct TempDir {
  TempDir() {
    std::string pattern = (std::filesystem::temp_directory_path() / "mowbite_sound_XXXXXX").string();
    path = ::mkdtemp(pattern.data());
  }
  ~TempDir() { std::filesystem::remove_all(path); }
  std::string path;
};

inline std::string ReadAll(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), {});
}

#endif  // MOWBITE_SOUND_TEST_TEMP_DIR_H
