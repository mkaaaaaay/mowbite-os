#ifndef MOWBITE_OUTBOX_OUTBOX_H
#define MOWBITE_OUTBOX_OUTBOX_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mowbite_outbox {

// Between the I/O thread of a driver, which must keep its period, and its output thread, which publishes, logs and
// may block. The producer never waits for the consumer: every lock below guards a copy or a swap, nothing slower.

// Wakes the output thread when there is something to send.
class Wakeup {
 public:
  void Notify();
  // Until Notify or the timeout; true when notified. Clears the notice.
  bool WaitFor(std::chrono::nanoseconds timeout);
  // Wakes the waiter for good.
  void Stop();
  bool stopped() const {
    return stopped_;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool notified_ = false;
  std::atomic<bool> stopped_{false};
};

// What a full ring gives up.
enum class Full {
  // Streams: the newest values count, the oldest go.
  kDropOldest,
  // Log lines: the first lines of an incident name its cause, the later ones go.
  kDropNewest,
};

// Entries in their order, at most `capacity` of them; what does not fit is counted.
template <typename T>
class Ring {
 public:
  Ring(size_t capacity, Full full) : capacity_(capacity), full_(full) {
    entries_.reserve(capacity);
  }

  // False when an entry was given up for it (kDropOldest) or the entry itself (kDropNewest).
  bool Push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (entries_.size() < capacity_) {
      entries_.push_back(std::move(value));
      return true;
    }
    dropped_++;
    if (full_ == Full::kDropNewest || capacity_ == 0) return false;
    entries_[first_] = std::move(value);
    first_ = (first_ + 1) % capacity_;
    return false;
  }

  // Every entry, oldest first; the ring is empty after it.
  std::vector<T> TakeAll() {
    std::vector<T> taken;
    taken.reserve(capacity_);
    size_t first = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      taken.swap(entries_);
      first = std::exchange(first_, 0);
    }
    if (first != 0) std::rotate(taken.begin(), taken.begin() + static_cast<std::ptrdiff_t>(first), taken.end());
    return taken;
  }

  // Entries given up since the last call.
  uint64_t TakeDropped() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(dropped_, 0);
  }

 private:
  const size_t capacity_;
  const Full full_;
  std::mutex mutex_;
  std::vector<T> entries_;
  // Index of the oldest entry once the ring wrapped.
  size_t first_ = 0;
  uint64_t dropped_ = 0;
};

// Only the newest value; how many were replaced unseen is counted.
template <typename T>
class Latest {
 public:
  void Set(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (value_) replaced_++;
    value_ = std::move(value);
  }
  std::optional<T> Take() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(value_, std::nullopt);
  }
  uint64_t TakeReplaced() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(replaced_, 0);
  }

 private:
  std::mutex mutex_;
  std::optional<T> value_;
  uint64_t replaced_ = 0;
};

// A count of frequent events instead of a line each; the output thread turns it into throttled lines.
class Counter {
 public:
  void Add(uint64_t n = 1) {
    count_.fetch_add(n, std::memory_order_relaxed);
  }
  uint64_t Take() {
    return count_.exchange(0, std::memory_order_relaxed);
  }

 private:
  std::atomic<uint64_t> count_{0};
};

// The largest value since the last Take, with how often one came; for late ticks and similar.
class Peak {
 public:
  void Add(double value);
  struct Taken {
    uint64_t count = 0;
    double max = 0;
  };
  Taken Take();

 private:
  std::mutex mutex_;
  Taken taken_;
};

struct LogLine {
  enum class Level : uint8_t { kDebug, kInfo, kWarn, kError };
  Level level = Level::kInfo;
  std::string text;
};

// Log lines of the threads that must not write them themselves, 64 at most: a full ring keeps the first.
class LogQueue {
 public:
  explicit LogQueue(Wakeup* wakeup = nullptr) : wakeup_(wakeup) {}
  void Add(LogLine::Level level, std::string text);
  void Debug(std::string text) {
    Add(LogLine::Level::kDebug, std::move(text));
  }
  void Info(std::string text) {
    Add(LogLine::Level::kInfo, std::move(text));
  }
  void Warn(std::string text) {
    Add(LogLine::Level::kWarn, std::move(text));
  }
  void Error(std::string text) {
    Add(LogLine::Level::kError, std::move(text));
  }
  std::vector<LogLine> TakeAll() {
    return lines_.TakeAll();
  }
  uint64_t TakeDropped() {
    return lines_.TakeDropped();
  }

  static constexpr size_t kCapacity = 64;

 private:
  Ring<LogLine> lines_{kCapacity, Full::kDropNewest};
  Wakeup* wakeup_;
};

// printf into a std::string, for log lines built in the I/O thread.
std::string Format(const char* format, ...) __attribute__((format(printf, 1, 2)));

// At most one line per period, on the monotonic seconds passed in; one per kind of line, used by one thread.
class Throttle {
 public:
  explicit Throttle(double period_s) : period_s_(period_s) {}
  bool Allow(double now);

 private:
  double period_s_;
  std::optional<double> last_;
};

}  // namespace mowbite_outbox

#endif  // MOWBITE_OUTBOX_OUTBOX_H
