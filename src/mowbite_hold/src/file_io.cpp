#include "file_io.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace mowbite_hold {

namespace {

std::string Folder(const std::string& path) {
  const size_t slash = path.rfind('/');
  if (slash == std::string::npos) return ".";
  return slash == 0 ? "/" : path.substr(0, slash);
}

void SyncFolder(const std::string& path) {
  const int fd = ::open(Folder(path).c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) return;
  ::fsync(fd);
  ::close(fd);
}

}  // namespace

FileRead ReadSmallFile(const std::string& path, size_t max_bytes, std::string* text, std::string* error) {
  text->clear();
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    if (errno == ENOENT) return FileRead::kMissing;
    *error = "cannot open " + path + ": " + std::strerror(errno);
    return FileRead::kFailed;
  }
  char buffer[512];
  while (text->size() <= max_bytes) {
    const ssize_t n = ::read(fd, buffer, sizeof(buffer));
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) {
      *error = "cannot read " + path + ": " + std::strerror(errno);
      ::close(fd);
      return FileRead::kFailed;
    }
    if (n == 0) break;
    text->append(buffer, static_cast<size_t>(n));
  }
  ::close(fd);
  return text->size() > max_bytes ? FileRead::kTooLarge : FileRead::kText;
}

bool WriteFileAtomic(const std::string& path, const std::string& text, std::string* error) {
  const std::string temp = path + ".tmp";
  const int fd = ::open(temp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    *error = "cannot write " + temp + ": " + std::strerror(errno);
    return false;
  }
  size_t done = 0;
  while (done < text.size()) {
    const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) {
      *error = "cannot write " + temp + ": " + std::strerror(errno);
      ::close(fd);
      ::unlink(temp.c_str());
      return false;
    }
    done += static_cast<size_t>(n);
  }
  const bool synced = ::fsync(fd) == 0;
  ::close(fd);
  if (!synced) {
    *error = "cannot sync " + temp + ": " + std::strerror(errno);
    ::unlink(temp.c_str());
    return false;
  }
  if (std::rename(temp.c_str(), path.c_str()) != 0) {
    *error = "cannot replace " + path + ": " + std::strerror(errno);
    ::unlink(temp.c_str());
    return false;
  }
  SyncFolder(path);
  return true;
}

bool RemoveFileSynced(const std::string& path, std::string* error) {
  if (::unlink(path.c_str()) != 0 && errno != ENOENT) {
    *error = "cannot remove " + path + ": " + std::strerror(errno);
    return false;
  }
  SyncFolder(path);
  return true;
}

}  // namespace mowbite_hold
