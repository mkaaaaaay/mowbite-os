#include "mowbite_map/map_store.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "mowbite_map/sha256.h"

namespace mowbite_map {

namespace {

const char* const kMapFile = "map.json";
const char* const kBackupFile = "map.json.bak";
const char* const kTempFile = "map.json.tmp";
const char* const kDatumFile = "map_datum.json";
const char* const kDatumTempFile = "map_datum.json.tmp";
// The version of map_datum.json; a newer one is not read, the map counts as without a datum then.
constexpr int kDatumSchema = 1;
constexpr uintmax_t kDatumMaxBytes = 4096;

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
  std::string sha256;
  // Empty if the file is missing or fine.
  std::string problem;
};

ReadResult ReadMap(const fs::path& path, size_t max_bytes) {
  std::error_code ec;
  const auto size = fs::file_size(path, ec);
  if (ec) return {};
  const std::string name = path.filename().string();
  if (size > max_bytes) {
    return {std::nullopt, "",
            name + " has " + std::to_string(size) + " bytes, the limit is " + std::to_string(max_bytes)};
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) return {std::nullopt, "", "cannot read " + name};
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const json parsed = json::parse(text, nullptr, false);
  if (parsed.is_discarded()) return {std::nullopt, "", name + " is not valid JSON"};
  ParseResult result = ParseMap(parsed);
  if (!result.map) return {std::nullopt, "", name + ": " + result.error};
  return {std::move(result.map), Sha256Hex(text), ""};
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
    loaded.sha256 = std::move(current.sha256);
    return loaded;
  }
  loaded.problem = current.problem;
  ReadResult backup = ReadMap(fs::path(dir_) / kBackupFile, max_bytes_);
  if (backup.map) {
    loaded.map = std::move(backup.map);
    loaded.source = kBackupFile;
    loaded.sha256 = std::move(backup.sha256);
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

MapStore::LoadedDatum MapStore::LoadDatum() const {
  const fs::path path = fs::path(dir_) / kDatumFile;
  std::error_code ec;
  const auto size = fs::file_size(path, ec);
  if (ec) return {};
  if (size > kDatumMaxBytes) return {std::nullopt, "", std::string(kDatumFile) + " is too large"};
  std::ifstream file(path, std::ios::binary);
  if (!file) return {std::nullopt, "", std::string("cannot read ") + kDatumFile};
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const json parsed = json::parse(text, nullptr, false);
  if (parsed.is_discarded() || !parsed.is_object()) {
    return {std::nullopt, "", std::string(kDatumFile) + " is not valid JSON"};
  }
  const auto schema = parsed.find("schema");
  if (schema == parsed.end() || !schema->is_number_integer()) {
    return {std::nullopt, "", std::string(kDatumFile) + " has no schema"};
  }
  if (schema->get<long long>() != kDatumSchema) {
    return {std::nullopt, "", std::string(kDatumFile) + " has the schema " + std::to_string(schema->get<long long>()) +
                              ", this version reads " + std::to_string(kDatumSchema)};
  }
  Datum datum;
  const std::pair<const char*, double*> fields[] = {
      {"datum_lat", &datum.lat}, {"datum_long", &datum.lon}, {"datum_height", &datum.height}};
  for (const auto& [key, field] : fields) {
    const auto value = parsed.find(key);
    if (value == parsed.end() || !value->is_number()) {
      return {std::nullopt, "", std::string(kDatumFile) + " holds no valid datum"};
    }
    *field = value->get<double>();
  }
  if (!IsValid(datum)) return {std::nullopt, "", std::string(kDatumFile) + " holds no valid datum"};
  const auto sha = parsed.find("map_sha256");
  if (sha != parsed.end() && (!sha->is_string() || !IsSha256Hex(sha->get<std::string>()))) {
    return {std::nullopt, "", std::string(kDatumFile) + " holds no valid map_sha256"};
  }
  return {datum, sha != parsed.end() ? sha->get<std::string>() : "", ""};
}

std::string MapStore::SaveDatum(const Datum& datum, const std::string& map_sha256) {
  if (!IsValid(datum)) return "not a valid datum";
  if (!IsSha256Hex(map_sha256)) return "not a SHA-256 of a map";
  std::error_code ec;
  fs::create_directories(dir_, ec);
  if (ec) return "cannot create " + dir_ + ": " + ec.message();
  json j = json::object();
  j["schema"] = kDatumSchema;
  j["datum_lat"] = datum.lat;
  j["datum_long"] = datum.lon;
  j["datum_height"] = datum.height;
  j["map_sha256"] = map_sha256;
  const fs::path path = fs::path(dir_) / kDatumFile;
  const fs::path temp_path = fs::path(dir_) / kDatumTempFile;
  std::string error;
  if (!WriteSynced(temp_path.string(), j.dump(2) + "\n", error)) {
    fs::remove(temp_path, ec);
    return error;
  }
  if (::rename(temp_path.c_str(), path.c_str()) != 0) {
    error = Errno("cannot replace " + path.string());
    fs::remove(temp_path, ec);
    return error;
  }
  SyncDirectory(dir_);
  return "";
}

}  // namespace mowbite_map
