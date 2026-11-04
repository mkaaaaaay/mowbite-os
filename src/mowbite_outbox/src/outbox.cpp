#include "mowbite_outbox/outbox.h"

#include <cstdarg>
#include <cstdio>

namespace mowbite_outbox {

void Wakeup::Notify() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    notified_ = true;
  }
  cv_.notify_one();
}

bool Wakeup::WaitFor(std::chrono::nanoseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_for(lock, timeout, [this] { return notified_ || stopped_; });
  return std::exchange(notified_, false);
}

void Wakeup::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
}

void Peak::Add(double value) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (taken_.count == 0 || value > taken_.max) taken_.max = value;
  taken_.count++;
}

Peak::Taken Peak::Take() {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::exchange(taken_, Taken{});
}

void LogQueue::Add(LogLine::Level level, std::string text) {
  lines_.Push(LogLine{level, std::move(text)});
  if (wakeup_) wakeup_->Notify();
}

std::string Format(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  const int n = std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (n < 0) return format;
  return std::string(buffer, std::min(static_cast<size_t>(n), sizeof(buffer) - 1));
}

bool Throttle::Allow(double now) {
  if (last_ && now - *last_ < period_s_) return false;
  last_ = now;
  return true;
}

}  // namespace mowbite_outbox
