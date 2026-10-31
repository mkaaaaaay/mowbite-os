#include "mowbite_ntrip/ntrip_config.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace mowbite_ntrip {

namespace {

// A larger file is no access file; it is not read.
constexpr off_t kMaxFileBytes = 64 * 1024;

std::string Trim(const std::string& s) {
  const size_t first = s.find_first_not_of(" \t\r");
  if (first == std::string::npos) return "";
  const size_t last = s.find_last_not_of(" \t\r");
  return s.substr(first, last - first + 1);
}

// The value after "key:", without quotes and comment. Empty optional for an unclosed quote or text after it.
std::optional<std::string> Value(const std::string& raw) {
  std::string s = Trim(raw);
  if (!s.empty() && (s[0] == '"' || s[0] == '\'')) {
    const char quote = s[0];
    std::string value;
    size_t i = 1;
    for (; i < s.size(); i++) {
      if (s[i] != quote) {
        value += s[i];
      } else if (quote == '\'' && i + 1 < s.size() && s[i + 1] == '\'') {
        value += '\'';
        i++;
      } else {
        break;
      }
    }
    if (i >= s.size()) return std::nullopt;
    const std::string rest = Trim(s.substr(i + 1));
    if (!rest.empty() && rest[0] != '#') return std::nullopt;
    return value;
  }
  const size_t comment = s.find(" #");
  if (comment != std::string::npos) s = s.substr(0, comment);
  if (!s.empty() && s[0] == '#') s.clear();
  return Trim(s);
}

// Host and mountpoint go into the request line and the Host header: printable, no spaces.
bool Plain(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) {
    if (c <= ' ' || c == 0x7F) return false;
  }
  return true;
}

bool Printable(const std::string& s) {
  for (char c : s) {
    if (static_cast<unsigned char>(c) < ' ' || c == 0x7F) return false;
  }
  return true;
}

std::optional<int> Integer(const std::string& s) {
  if (s.empty() || s.size() > 6) return std::nullopt;
  char* end = nullptr;
  long v = std::strtol(s.c_str(), &end, 10);
  if (*end != '\0') return std::nullopt;
  return static_cast<int>(v);
}

// In single quotes, a quote doubled.
std::string Quoted(const std::string& s) {
  std::string out = "'";
  for (char c : s) {
    out += c;
    if (c == '\'') out += '\'';
  }
  return out + "'";
}

std::string Errno(const std::string& what) {
  return what + ": " + std::strerror(errno);
}

// Asks the disk to keep a rename or removal in the folder of path.
void SyncFolder(const std::string& path) {
  std::string dir = std::filesystem::path(path).parent_path().string();
  if (dir.empty()) dir = ".";
  const int fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) return;
  ::fsync(fd);
  ::close(fd);
}

}  // namespace

std::string CheckValue(const std::string& key, const std::string& value) {
  if (value.size() > kMaxValueLength) return "longer than " + std::to_string(kMaxValueLength) + " bytes";
  if (key == "host" || key == "mountpoint") return Plain(value) ? "" : "empty or with spaces";
  return Printable(value) ? "" : "control characters";
}

std::optional<NtripConfig> ParseNtripConfig(const std::string& text, std::string& error) {
  NtripConfig config;
  bool has_host = false, has_mountpoint = false;
  std::istringstream lines(text);
  std::string line;
  int number = 0;
  while (std::getline(lines, line)) {
    number++;
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed[0] == '#') continue;
    const size_t colon = trimmed.find(':');
    if (colon == std::string::npos) {
      error = "line " + std::to_string(number) + ": expected key: value";
      return std::nullopt;
    }
    const std::string key = Trim(trimmed.substr(0, colon));
    const auto value = Value(trimmed.substr(colon + 1));
    const std::string where = "line " + std::to_string(number) + ", " + key + ": ";
    if (!value) {
      error = where + "unclosed quote or text after it";
      return std::nullopt;
    }
    if (key == "host" || key == "mountpoint" || key == "user" || key == "password") {
      std::string text_value = *value;
      if (key == "mountpoint" && !text_value.empty() && text_value[0] == '/') text_value.erase(0, 1);
      const std::string problem = CheckValue(key, text_value);
      if (!problem.empty()) {
        error = where + problem;
        return std::nullopt;
      }
      if (key == "host") {
        config.host = text_value;
        has_host = true;
      } else if (key == "mountpoint") {
        config.mountpoint = text_value;
        has_mountpoint = true;
      } else {
        (key == "user" ? config.user : config.password) = text_value;
      }
    } else if (key == "port") {
      auto port = Integer(*value);
      if (!port || *port < 1 || *port > 65535) {
        error = where + "not a port number";
        return std::nullopt;
      }
      config.port = *port;
    } else if (key == "version") {
      auto version = Integer(*value);
      if (!version || (*version != 1 && *version != 2)) {
        error = where + "1 or 2";
        return std::nullopt;
      }
      config.version = *version;
    } else {
      // The key is not named: a password typed into the wrong place would end up in the log.
      error = "line " + std::to_string(number) + ": unknown key";
      return std::nullopt;
    }
  }
  if (!has_host || !has_mountpoint) {
    error = has_host ? "mountpoint missing" : "host missing";
    return std::nullopt;
  }
  return config;
}

std::string FormatNtripConfig(const NtripConfig& config) {
  std::ostringstream out;
  out << "# NTRIP access of MowBite OS, from ntrip.set or by hand. Keep it mode 600.\n"
      << "host: " << Quoted(config.host) << "\n"
      << "port: " << config.port << "\n"
      << "mountpoint: " << Quoted(config.mountpoint) << "\n"
      << "user: " << Quoted(config.user) << "\n"
      << "password: " << Quoted(config.password) << "\n"
      << "version: " << config.version << "\n";
  return out.str();
}

AccessFile ReadAccessFile(const std::string& path) {
  AccessFile file;
  struct stat st;
  if (::stat(path.c_str(), &st) != 0) return file;
  file.exists = true;
  file.readable_by_others = (st.st_mode & 077) != 0;
  if (!S_ISREG(st.st_mode) || st.st_size > kMaxFileBytes) {
    file.error = "not a regular file of at most 64 KB";
    return file;
  }
  std::ifstream in(path);
  if (!in) {
    file.error = "cannot read";
    return file;
  }
  std::stringstream text;
  text << in.rdbuf();
  std::string error;
  file.config = ParseNtripConfig(text.str(), error);
  if (!file.config) file.error = error;
  return file;
}

bool WriteAccessFile(const std::string& path, const NtripConfig& config, std::string& error) {
  const std::string text = FormatNtripConfig(config);
  // Only what reads back as the same access goes to disk.
  std::string problem;
  if (ParseNtripConfig(text, problem) != config) {
    error = "the access would not read back the same" + (problem.empty() ? "" : " (" + problem + ")");
    return false;
  }
  const std::string temp = path + ".new";
  ::unlink(temp.c_str());
  // Mode 600 from the start, so others never get a moment to read it.
  const int fd = ::open(temp.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0) {
    error = Errno("cannot create " + temp);
    return false;
  }
  size_t done = 0;
  while (done < text.size()) {
    const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) {
      error = Errno("cannot write " + temp);
      ::close(fd);
      ::unlink(temp.c_str());
      return false;
    }
    done += static_cast<size_t>(n);
  }
  if (::fsync(fd) != 0) {
    error = Errno("cannot sync " + temp);
    ::close(fd);
    ::unlink(temp.c_str());
    return false;
  }
  if (::close(fd) != 0) {
    error = Errno("cannot close " + temp);
    ::unlink(temp.c_str());
    return false;
  }
  if (::rename(temp.c_str(), path.c_str()) != 0) {
    error = Errno("cannot replace " + path);
    ::unlink(temp.c_str());
    return false;
  }
  SyncFolder(path);
  return true;
}

bool RemoveAccessFile(const std::string& path, std::string& error) {
  if (::unlink(path.c_str()) != 0 && errno != ENOENT) {
    error = Errno("cannot remove " + path);
    return false;
  }
  SyncFolder(path);
  return true;
}

}  // namespace mowbite_ntrip
