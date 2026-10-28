#include "mowbite_map/map_store.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace mowbite_map {

namespace {

const char* const kMapFile = "map.json";
const char* const kBackupFile = "map.json.bak";
const char* const kTempFile = "map.json.tmp";

namespace fs = std::filesystem;

std::string Errno(const std::string& what) {
  return what + ": " + std::strerror(errno);
}

// Writes the file and asks the disk to keep it before returning.
bool WriteSynced(const std::string& path, const std::string& text, std::string& error) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    error = Errno("cannot write " + path);
    return false;
  }
  size_t done = 0;
  while (done < text.size()) {
    const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
    if (n < 0) {
      if (errno == EINTR) continue;
      error = Errno("cannot write " + path);
      ::close(fd);
      return false;
    }
    done += static_cast<size_t>(n);
  }
  if (::fsync(fd) != 0) {
    error = Errno("cannot sync " + path);
    ::close(fd);
    return false;
  }
  if (::close(fd) != 0) {
    error = Errno("cannot close " + path);
    return false;
  }
  return true;
}

void SyncDirectory(const std::string& dir) {
  const int fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) return;
  ::fsync(fd);
  ::close(fd);
}

struct ReadResult {
  std::optional<MapData> map;
  // Empty if the file is missing or fine.
  std::string problem;
};

ReadResult ReadMap(const fs::path& path, size_t max_bytes) {
  std::error_code ec;
  const auto size = fs::file_size(path, ec);
  if (ec) return {};
  const std::string name = path.filename().string();
  if (size > max_bytes) {
    return {std::nullopt, name + " has " + std::to_string(size) + " bytes, the limit is " + std::to_string(max_bytes)};
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) return {std::nullopt, "cannot read " + name};
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const json parsed = json::parse(text, nullptr, false);
  if (parsed.is_discarded()) return {std::nullopt, name + " is not valid JSON"};
  ParseResult result = ParseMap(parsed);
  if (!result.map) return {std::nullopt, name + ": " + result.error};
  return {std::move(result.map), ""};
}

}  // namespace

MapStore::MapStore(std::string dir, size_t max_bytes) : dir_(std::move(dir)), max_bytes_(max_bytes) {
}

MapStore::Loaded MapStore::Load() const {
  Loaded loaded;
  ReadResult current = ReadMap(fs::path(dir_) / kMapFile, max_bytes_);
  if (current.map) {
    loaded.map = std::move(current.map);
    loaded.source = kMapFile;
    return loaded;
  }
  loaded.problem = current.problem;
  ReadResult backup = ReadMap(fs::path(dir_) / kBackupFile, max_bytes_);
  if (backup.map) {
    loaded.map = std::move(backup.map);
    loaded.source = kBackupFile;
    return loaded;
  }
  if (!backup.problem.empty()) loaded.problem += (loaded.problem.empty() ? "" : "; ") + backup.problem;
  return loaded;
}

std::string MapStore::Save(const std::string& text) {
  if (text.size() > max_bytes_) {
    return "the map has " + std::to_string(text.size()) + " bytes, the limit is " + std::to_string(max_bytes_);
  }
  std::error_code ec;
  fs::create_directories(dir_, ec);
  if (ec) return "cannot create " + dir_ + ": " + ec.message();

  const fs::path map_path = fs::path(dir_) / kMapFile;
  const fs::path backup_path = fs::path(dir_) / kBackupFile;
  const fs::path temp_path = fs::path(dir_) / kTempFile;
  std::string error;
  if (!WriteSynced(temp_path.string(), text, error)) {
    fs::remove(temp_path, ec);
    return error;
  }
  // The version being replaced becomes map.json.bak if it is a usable map, by a hard link: map.json never goes missing.
  if (ReadMap(map_path, max_bytes_).map) {
    fs::remove(backup_path, ec);
    fs::create_hard_link(map_path, backup_path, ec);
    if (ec) {
      ec.clear();
      fs::copy_file(map_path, backup_path, fs::copy_options::overwrite_existing, ec);
    }
    if (ec) {
      const std::string reason = ec.message();
      fs::remove(temp_path, ec);
      return "cannot keep the previous map: " + reason;
    }
  }
  if (::rename(temp_path.c_str(), map_path.c_str()) != 0) {
    error = Errno("cannot replace " + map_path.string());
    fs::remove(temp_path, ec);
    return error;
  }
  SyncDirectory(dir_);
  return "";
}

}  // namespace mowbite_map
