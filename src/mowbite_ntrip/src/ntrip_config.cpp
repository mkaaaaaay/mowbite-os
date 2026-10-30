#include "mowbite_ntrip/ntrip_config.h"

#include <cstdlib>
#include <sstream>

namespace mowbite_ntrip {

namespace {

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
    const size_t end = s.find(quote, 1);
    if (end == std::string::npos) return std::nullopt;
    const std::string rest = Trim(s.substr(end + 1));
    if (!rest.empty() && rest[0] != '#') return std::nullopt;
    return s.substr(1, end - 1);
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

}  // namespace

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
    if (key == "host") {
      if (!Plain(*value)) {
        error = where + "empty or with spaces";
        return std::nullopt;
      }
      config.host = *value;
      has_host = true;
    } else if (key == "port") {
      auto port = Integer(*value);
      if (!port || *port < 1 || *port > 65535) {
        error = where + "not a port number";
        return std::nullopt;
      }
      config.port = *port;
    } else if (key == "mountpoint") {
      std::string mountpoint = *value;
      if (!mountpoint.empty() && mountpoint[0] == '/') mountpoint.erase(0, 1);
      if (!Plain(mountpoint)) {
        error = where + "empty or with spaces";
        return std::nullopt;
      }
      config.mountpoint = mountpoint;
      has_mountpoint = true;
    } else if (key == "user" || key == "password") {
      if (!Printable(*value)) {
        error = where + "control characters";
        return std::nullopt;
      }
      (key == "user" ? config.user : config.password) = *value;
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

}  // namespace mowbite_ntrip
