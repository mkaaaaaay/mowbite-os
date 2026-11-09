#ifndef TRON_VISION_TEXT_FILE_H
#define TRON_VISION_TEXT_FILE_H

#include <string>

namespace tron_vision {

// First line of a text file without its newline, empty if unreadable.
std::string FirstLine(const std::string& path);

}  // namespace tron_vision

#endif  // TRON_VISION_TEXT_FILE_H
