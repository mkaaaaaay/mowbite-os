#include "mowbite_journal/tar.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>

namespace mowbite_journal {

namespace {

constexpr uint64_t kBlock = 512;
constexpr size_t kNameSize = 100;
constexpr size_t kPrefixSize = 155;

uint64_t Padded(uint64_t size) {
  return (size + kBlock - 1) / kBlock * kBlock;
}

// width - 1 octal digits and a NUL; false if the value does not fit.
bool PutOctal(char* at, size_t width, uint64_t value) {
  const unsigned digits = static_cast<unsigned>(width - 1);
  if (digits < 22 && value >> (3 * digits) != 0) return false;
  for (size_t i = digits; i-- > 0;) {
    at[i] = static_cast<char>('0' + (value & 7));
    value >>= 3;
  }
  at[digits] = '\0';
  return true;
}

bool SafeName(const std::string& name) {
  if (name.empty() || name.front() == '/' || name.find('\0') != std::string::npos) return false;
  size_t start = 0;
  for (;;) {
    const size_t end = name.find('/', start);
    if (name.compare(start, end == std::string::npos ? std::string::npos : end - start, "..") == 0) return false;
    if (end == std::string::npos) return true;
    start = end + 1;
  }
}

// The 512-byte ustar header of an entry, empty with why when ustar cannot hold it.
std::string Header(const TarEntry& entry, std::string* why) {
  if (!SafeName(entry.name)) {
    *why = "name '" + entry.name + "' is not a relative path";
    return {};
  }
  std::string name = entry.name;
  std::string prefix;
  if (name.size() > kNameSize) {
    const size_t cut = name.find('/', name.size() - kNameSize - 1);
    if (cut == std::string::npos || cut > kPrefixSize || cut + 1 == name.size()) {
      *why = "name '" + entry.name + "' is too long for ustar";
      return {};
    }
    prefix = name.substr(0, cut);
    name = name.substr(cut + 1);
  }
  std::string header(kBlock, '\0');
  char* h = header.data();
  std::memcpy(h, name.data(), name.size());
  PutOctal(h + 100, 8, 0644);
  PutOctal(h + 108, 8, 0);
  PutOctal(h + 116, 8, 0);
  if (!PutOctal(h + 124, 12, entry.size)) {
    *why = entry.name + " is too large for ustar";
    return {};
  }
  if (entry.mtime < 0 || !PutOctal(h + 136, 12, static_cast<uint64_t>(entry.mtime))) {
    *why = entry.name + " has an mtime ustar cannot hold";
    return {};
  }
  h[156] = '0';
  std::memcpy(h + 257, "ustar", 6);
  std::memcpy(h + 263, "00", 2);
  PutOctal(h + 329, 8, 0);
  PutOctal(h + 337, 8, 0);
  std::memcpy(h + 345, prefix.data(), prefix.size());
  // The checksum counts its own field as spaces.
  std::memset(h + 148, ' ', 8);
  unsigned sum = 0;
  for (char c : header) sum += static_cast<unsigned char>(c);
  PutOctal(h + 148, 7, sum);
  h[155] = ' ';
  return header;
}

}  // namespace

VirtualTar::VirtualTar(std::vector<TarEntry> entries) : entries_(std::move(entries)) {
  for (auto& entry : entries_) {
    if (entry.path.empty()) entry.size = entry.data.size();
    std::string why;
    std::string header = Header(entry, &why);
    if (header.empty()) {
      error_ = why;
      entries_.clear();
      headers_.clear();
      offsets_.clear();
      size_ = 0;
      return;
    }
    offsets_.push_back(size_);
    headers_.push_back(std::move(header));
    size_ += kBlock + Padded(entry.size);
  }
  size_ += 2 * kBlock;
}

const std::string& VirtualTar::error() const {
  return error_;
}

uint64_t VirtualTar::size() const {
  return size_;
}

bool VirtualTar::Read(uint64_t offset, size_t size, std::string* out, std::string* error) const {
  out->clear();
  if (!error_.empty()) {
    if (error) *error = error_;
    return false;
  }
  if (offset > size_) {
    if (error) *error = "offset " + std::to_string(offset) + " beyond the archive of " + std::to_string(size_);
    return false;
  }
  const uint64_t end = offset + std::min<uint64_t>(size, size_ - offset);
  out->reserve(static_cast<size_t>(end - offset));
  uint64_t at = offset;
  size_t i = static_cast<size_t>(std::upper_bound(offsets_.begin(), offsets_.end(), offset) - offsets_.begin());
  for (i = i == 0 ? 0 : i - 1; i < entries_.size() && at < end; ++i) {
    const TarEntry& entry = entries_[i];
    const uint64_t data = offsets_[i] + kBlock;
    const uint64_t data_end = data + entry.size;
    const uint64_t next = data + Padded(entry.size);
    if (at < data) {
      const uint64_t to = std::min(end, data);
      out->append(headers_[i], static_cast<size_t>(at - offsets_[i]), static_cast<size_t>(to - at));
      at = to;
    }
    if (at < end && at < data_end) {
      const uint64_t to = std::min(end, data_end);
      if (entry.path.empty()) {
        out->append(entry.data, static_cast<size_t>(at - data), static_cast<size_t>(to - at));
      } else if (!ReadFile(entry, at - data, to - at, out, error)) {
        out->clear();
        return false;
      }
      at = to;
    }
    if (at < end && at < next) {
      const uint64_t to = std::min(end, next);
      out->append(static_cast<size_t>(to - at), '\0');
      at = to;
    }
  }
  // The two zero blocks at the end.
  out->append(static_cast<size_t>(end - at), '\0');
  return true;
}

bool VirtualTar::ReadFile(const TarEntry& entry, uint64_t from, uint64_t count, std::string* out,
                          std::string* error) const {
  const int fd = open(entry.path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    if (error) *error = "cannot open " + entry.path + ": " + std::strerror(errno);
    return false;
  }
  std::string why;
  struct stat st {};
  if (fstat(fd, &st) != 0) {
    why = "cannot stat " + entry.path + ": " + std::strerror(errno);
  } else if (static_cast<uint64_t>(st.st_size) < entry.size) {
    why = entry.path + " has " + std::to_string(st.st_size) + " bytes, the archive " + std::to_string(entry.size);
  }
  const size_t start = out->size();
  if (why.empty()) out->resize(start + static_cast<size_t>(count));
  size_t done = 0;
  while (why.empty() && done < count) {
    const ssize_t n = pread(fd, out->data() + start + done, static_cast<size_t>(count) - done,
                            static_cast<off_t>(from + done));
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) {
      why = "cannot read " + entry.path + ": " + std::strerror(errno);
    } else if (n == 0) {
      why = entry.path + " is shorter than in the archive";
    } else {
      done += static_cast<size_t>(n);
    }
  }
  close(fd);
  if (why.empty()) return true;
  if (error) *error = why;
  return false;
}

}  // namespace mowbite_journal
