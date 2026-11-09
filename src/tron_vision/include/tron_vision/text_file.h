#ifndef TRON_VISION_TEXT_FILE_H
#define TRON_VISION_TEXT_FILE_H

#include <string>

namespace tron_vision {

// First line of a text file without its newline, empty if unreadable.
std::string FirstLine(const std::string& path);

// Writes `text` to `path` through a temporary file, synced before and after the rename.
bool WriteDurably(const std::string& path, const std::string& text, std::string& error);

// Removes a file and syncs its directory; true if there was one.
bool RemoveDurably(const std::string& path);

}  // namespace tron_vision

#endif  // TRON_VISION_TEXT_FILE_H
