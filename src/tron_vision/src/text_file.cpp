#include "tron_vision/text_file.h"

#include <fstream>

namespace tron_vision {

std::string FirstLine(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  return line;
}

}  // namespace tron_vision
