#include "mowbite_watch/watch_core.h"

#include <cstring>
#include <stdexcept>

namespace mowbite_watch {

WatchCore::WatchCore(const std::vector<std::string>& expected, int64_t started_ns, int64_t start_limit_ns,
                     int64_t shutdown_limit_ns)
    : expected_(expected),
      started_ns_(started_ns),
      start_limit_ns_(start_limit_ns),
      shutdown_limit_ns_(shutdown_limit_ns) {
  if (expected_.size() > kMaxPlaces) throw std::length_error("more expected nodes than places of the watch");
}

int WatchCore::Enter(std::string_view name, int64_t limit_ns, int64_t now_ns) {
  if (name.size() >= kNameSize) throw std::length_error("node name too long for the watch: " + std::string(name));
  const int index = used_.load(std::memory_order_relaxed);
  if (index >= kMaxPlaces) return -1;
  Place& place = places_[index];
  std::memcpy(place.name, name.data(), name.size());
  place.name[name.size()] = '\0';
  place.limit_ns = limit_ns;
  place.last_ns.store(now_ns, std::memory_order_relaxed);
  place.state.store(kActive, std::memory_order_release);
  used_.store(index + 1, std::memory_order_release);
  for (size_t i = 0; i < expected_.size(); ++i) {
    if (expected_[i] == name) seen_[i].store(true, std::memory_order_release);
  }
  return index;
}

void WatchCore::Sign(int place, int64_t now_ns) {
  places_[place].last_ns.store(now_ns, std::memory_order_release);
}

void WatchCore::SetMark(int place, const char* where) {
  places_[place].mark.store(where, std::memory_order_release);
}

void WatchCore::Leave(int place) {
  places_[place].state.store(kLeft, std::memory_order_release);
}

void WatchCore::Shutdown(int64_t now_ns) {
  if (shut_down_.load(std::memory_order_acquire)) return;
  shutdown_ns_.store(now_ns, std::memory_order_relaxed);
  shut_down_.store(true, std::memory_order_release);
}

Verdict WatchCore::Judge(int64_t now_ns) const {
  Verdict verdict;
  if (shut_down_.load(std::memory_order_acquire)) {
    const int64_t since = now_ns - shutdown_ns_.load(std::memory_order_relaxed);
    if (since > shutdown_limit_ns_) verdict = {Finding::kShutdown, "", since, shutdown_limit_ns_, nullptr};
    return verdict;
  }
  const int used = used_.load(std::memory_order_acquire);
  int64_t oldest = 0;
  for (int i = 0; i < used; ++i) {
    const Place& place = places_[i];
    if (place.state.load(std::memory_order_acquire) != kActive) continue;
    const int64_t last = place.last_ns.load(std::memory_order_acquire);
    const int64_t silent = now_ns - last;
    if (silent <= place.limit_ns) continue;
    if (verdict.finding == Finding::kNone || last < oldest) {
      verdict = {Finding::kSilent, place.name, silent, place.limit_ns, place.mark.load(std::memory_order_acquire)};
      oldest = last;
    }
  }
  if (verdict.finding != Finding::kNone) return verdict;
  const int64_t since_start = now_ns - started_ns_;
  if (since_start <= start_limit_ns_) return verdict;
  for (size_t i = 0; i < expected_.size(); ++i) {
    if (!seen_[i].load(std::memory_order_acquire)) {
      return {Finding::kMissing, expected_[i].c_str(), since_start, start_limit_ns_, nullptr};
    }
  }
  return verdict;
}

}  // namespace mowbite_watch
