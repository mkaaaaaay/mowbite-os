#include "mowbite_ntrip/ntrip_source.h"

#include <sys/stat.h>

#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace mowbite_ntrip {

namespace {

using json = nlohmann::ordered_json;

// A larger file was not written by this client; it is not read.
constexpr off_t kMaxSourceBytes = 4096;

// ASCII only, whatever the locale.
char Lower(char c) {
  return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
}

bool SameIgnoringCase(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (Lower(a[i]) != Lower(b[i])) return false;
  }
  return true;
}

// Text that is not UTF-8 is replaced instead of throwing; WriteSourceFile then refuses it.
std::string FormatSource(const CorrectionSource& source) {
  const json doc = {{"host", source.host}, {"port", source.port}, {"mountpoint", source.mountpoint}};
  return doc.dump(-1, ' ', false, json::error_handler_t::replace) + "\n";
}

std::optional<CorrectionSource> ParseSource(const std::string& text, std::string& error) {
  const json doc = json::parse(text, nullptr, false);
  if (!doc.is_object()) {
    error = "not a JSON object";
    return std::nullopt;
  }
  CorrectionSource source;
  for (const std::string key : {"host", "mountpoint"}) {
    const auto it = doc.find(key);
    if (it == doc.end() || !it->is_string()) {
      error = key + ": missing or not a text";
      return std::nullopt;
    }
    std::string value = it->get<std::string>();
    if (key == "mountpoint") value = NormalMountpoint(value);
    const std::string problem = CheckValue(key, value);
    if (!problem.empty()) {
      error = key + ": " + problem;
      return std::nullopt;
    }
    (key == "host" ? source.host : source.mountpoint) = value;
  }
  const auto port = doc.find("port");
  if (port == doc.end() || !port->is_number_integer() || port->get<int64_t>() < 1 || port->get<int64_t>() > 65535) {
    error = "port: not a port number";
    return std::nullopt;
  }
  source.port = static_cast<int>(port->get<int64_t>());
  return source;
}

}  // namespace

CorrectionSource SourceOf(const NtripConfig& config) {
  return {config.host, config.port, NormalMountpoint(config.mountpoint)};
}

bool SameSource(const CorrectionSource& a, const CorrectionSource& b) {
  return SameIgnoringCase(a.host, b.host) && a.port == b.port &&
         NormalMountpoint(a.mountpoint) == NormalMountpoint(b.mountpoint);
}

SourceFile ReadSourceFile(const std::string& path) {
  SourceFile file;
  struct stat st;
  if (::stat(path.c_str(), &st) != 0) return file;
  file.exists = true;
  if (!S_ISREG(st.st_mode) || st.st_size > kMaxSourceBytes) {
    file.error = "not a regular file of at most 4 KB";
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
  file.source = ParseSource(text.str(), error);
  if (!file.source) file.error = error;
  return file;
}

bool WriteSourceFile(const std::string& path, const CorrectionSource& source, std::string& error) {
  const std::string text = FormatSource(source);
  std::string problem;
  if (ParseSource(text, problem) != source) {
    error = "the source would not read back the same" + (problem.empty() ? "" : " (" + problem + ")");
    return false;
  }
  return ReplaceFile(path, text, 0644, error);
}

}  // namespace mowbite_ntrip
