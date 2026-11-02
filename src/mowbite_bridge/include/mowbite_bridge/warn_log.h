#ifndef MOWBITE_BRIDGE_WARN_LOG_H
#define MOWBITE_BRIDGE_WARN_LOG_H

#include <deque>
#include <string>
#include <string_view>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// Levels as in /rosout.
constexpr int kLogDebug = 10;
constexpr int kLogInfo = 20;
constexpr int kLogWarn = 30;
constexpr int kLogError = 40;
constexpr int kLogFatal = 50;

// The last log lines from WARN on for logs.recent, oldest first; text as UTF-8, cut after 2000 bytes, logins hidden.
class WarnLog {
 public:
  explicit WarnLog(std::size_t capacity = 1000) : capacity_(capacity) {}

  void Add(double t, int level, std::string_view node, std::string_view message);

  // params: since, level (lowest by name, unknown counts as WARN), limit (the newest n; otherwise all).
  Json Recent(const Json& params) const;

 private:
  struct Line {
    double t;
    int level;
    std::string node;
    std::string message;
  };

  std::size_t capacity_;
  std::deque<Line> lines_;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_WARN_LOG_H
