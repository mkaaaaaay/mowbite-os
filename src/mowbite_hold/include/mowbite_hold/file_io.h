#ifndef MOWBITE_HOLD_FILE_IO_H
#define MOWBITE_HOLD_FILE_IO_H

#include <cstddef>
#include <string>

namespace mowbite_hold {

enum class FileRead { kMissing, kText, kTooLarge, kFailed };

// The whole file into text if it has at most max_bytes; kFailed with error when it cannot be opened or read.
FileRead ReadSmallFile(const std::string& path, size_t max_bytes, std::string* text, std::string* error);
// Whole or not at all: written next to it as <path>.tmp, synced, renamed over it, the folder synced.
bool WriteFileAtomic(const std::string& path, const std::string& text, std::string* error);
// As WriteFileAtomic, in a folder made when missing.
bool WriteFileSafely(const std::string& path, const std::string& text, std::string* error);
// As WriteFileSafely, readable and writable by the owner alone (0600), also when it was open to others before.
bool WriteFileOwnerOnly(const std::string& path, const std::string& text, std::string* error);
// Removes it and syncs the folder; a file that is not there counts as removed.
bool RemoveFileSynced(const std::string& path, std::string* error);
// The text of an errno value, safe from any thread.
std::string ErrorText(int error);

}  // namespace mowbite_hold

#endif  // MOWBITE_HOLD_FILE_IO_H
