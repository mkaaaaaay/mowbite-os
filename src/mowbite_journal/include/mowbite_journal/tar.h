#ifndef MOWBITE_JOURNAL_TAR_H
#define MOWBITE_JOURNAL_TAR_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_journal {

struct TarEntry {
  // Relative name in the archive; over 100 characters it is split at a '/' into the ustar prefix.
  std::string name;
  // A file read at every Read, size bytes of it; empty: the entry holds data.
  std::string path;
  std::string data;
  // Bytes of the file; for data its own size counts.
  uint64_t size = 0;
  // Seconds since 1970.
  int64_t mtime = 0;
};

// A ustar archive put together at each Read from its entries, nothing copied to the disk: mode 0644, owner 0 without
// names, the mtime of each entry, two zero blocks at the end. The same entries give the same bytes.
class VirtualTar {
 public:
  explicit VirtualTar(std::vector<TarEntry> entries);

  // Why the archive cannot be built (a name, size or mtime ustar cannot hold), empty if it can.
  const std::string& error() const;
  uint64_t size() const;
  // Up to size bytes from offset into out; fails when a file is shorter than its entry, never pads it with zeros.
  bool Read(uint64_t offset, size_t size, std::string* out, std::string* error) const;

 private:
  bool ReadFile(const TarEntry& entry, uint64_t from, uint64_t count, std::string* out, std::string* error) const;

  std::vector<TarEntry> entries_;
  std::vector<std::string> headers_;
  // Where the header of each entry starts.
  std::vector<uint64_t> offsets_;
  uint64_t size_ = 0;
  std::string error_;
};

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_TAR_H
