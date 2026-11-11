#include "mowbite_camera/stream_config.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include "mowbite_camera/seats.h"

namespace mowbite_camera {

namespace {

constexpr char kFolder[] = "mowbite_camera";

// Text that stands between double quotes of YAML as it is.
bool IsPlain(const std::string& text) {
  return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char c) {
    return std::isgraph(c) && c != '"' && c != '\\';
  });
}

}  // namespace

std::string StreamConfigText(const StreamConfig& config) {
  std::ostringstream paths;
  for (const auto& [id, source] : config.sources) {
    if (!IsPlain(id) || !IsPlain(source)) continue;
    paths << "  \"" << id << "\":\n"
          << "    source: \"" << source << "\"\n"
          << "    rtspTransport: tcp\n"
          << "    sourceOnDemand: true\n"
          << "    maxReaders: " << kSeats << "\n";
  }
  std::ostringstream text;
  text << "logLevel: warn\n"
       << "logDestinations: [stdout]\n"
       << "authMethod: http\n"
       << "authHTTPAddress: \"http://127.0.0.1:" << config.auth_port << "/auth\"\n"
       << "authHTTPExclude: []\n"
       << "api: true\n"
       << "apiAddress: \"127.0.0.1:" << config.api_port << "\"\n"
       << "metrics: false\n"
       << "pprof: false\n"
       << "playback: false\n"
       << "rtsp: false\n"
       << "rtmp: false\n"
       << "hls: false\n"
       << "srt: false\n"
       << "moq: false\n"
       << "webrtc: true\n"
       << "webrtcAddress: \":" << config.webrtc_port << "\"\n"
       << "webrtcLocalUDPAddress: \":" << config.ice_port << "\"\n"
       << "webrtcLocalTCPAddress: \"\"\n"
       << (paths.str().empty() ? "paths: {}\n" : "paths:\n" + paths.str());
  return text.str();
}

std::string RuntimeDir(const std::string& parameter, std::string* error) {
  std::string base = parameter;
  if (base.empty()) {
    // systemd separates several folders with ':'.
    const char* given = std::getenv("RUNTIME_DIRECTORY");
    base = given ? std::string(given).substr(0, std::string(given).find(':')) : "";
  }
  if (base.empty()) base = "/tmp";
  const std::string dir = base + "/" + kFolder;
  if (::mkdir(dir.c_str(), 0700) != 0 && errno != EEXIST) {
    *error = dir + ": " + std::strerror(errno);
    return "";
  }
  // A link is not followed: in /tmp anyone could have put one there.
  struct stat info {};
  if (::lstat(dir.c_str(), &info) != 0 || !S_ISDIR(info.st_mode) || info.st_uid != ::geteuid() ||
      ((info.st_mode & 0077) != 0 && ::chmod(dir.c_str(), 0700) != 0)) {
    *error = dir + " is not a folder of this user";
    return "";
  }
  return dir;
}

bool ReplaceFile(const std::string& dir, const std::string& name, const std::string& text, std::string* error) {
  const std::string path = dir + "/" + name;
  const std::string fresh = path + ".new";
  // What an earlier run left goes first, so the new file is made here with its mode.
  ::unlink(fresh.c_str());
  const int fd = ::open(fresh.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (fd < 0) {
    *error = fresh + ": " + std::strerror(errno);
    return false;
  }
  size_t done = 0;
  while (done < text.size()) {
    const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) break;
    done += static_cast<size_t>(n);
  }
  const bool written = done == text.size();
  if (::close(fd) != 0 || !written || ::rename(fresh.c_str(), path.c_str()) != 0) {
    *error = path + ": " + std::strerror(errno);
    ::unlink(fresh.c_str());
    return false;
  }
  return true;
}

}  // namespace mowbite_camera
