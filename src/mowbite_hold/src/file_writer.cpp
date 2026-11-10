#include "mowbite_hold/file_writer.h"

#include <algorithm>
#include <utility>

namespace mowbite_hold {

FileWriter::FileWriter(Write write, Report report) : write_(std::move(write)), report_(std::move(report)) {
  thread_ = std::thread([this] { Run(); });
}

FileWriter::~FileWriter() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  work_.notify_one();
  thread_.join();
}

void FileWriter::Submit(const std::string& path, std::string text) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [&](const Entry& e) { return e.path == path; }),
                 queue_.end());
    queue_.push_back({path, std::move(text)});
  }
  work_.notify_one();
}

FileWriter::Wait FileWriter::WaitFor(const std::string& path, std::chrono::steady_clock::duration limit) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!done_.wait_for(lock, limit, [&] { return !Pending(path); })) return Wait::kTimeout;
  const auto it = written_.find(path);
  return it != written_.end() && it->second ? Wait::kWritten : Wait::kFailed;
}

std::chrono::steady_clock::duration FileWriter::Busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (writing_.empty()) return {};
  return std::chrono::steady_clock::now() - started_;
}

bool FileWriter::Pending(const std::string& path) const {
  return writing_ == path ||
         std::any_of(queue_.begin(), queue_.end(), [&](const Entry& e) { return e.path == path; });
}

void FileWriter::Run() {
  std::unique_lock<std::mutex> lock(mutex_);
  for (;;) {
    work_.wait(lock, [this] { return stop_ || !queue_.empty(); });
    // A stop ends the thread only once the queue is written.
    if (queue_.empty()) return;
    Entry entry = std::move(queue_.front());
    queue_.pop_front();
    writing_ = entry.path;
    started_ = std::chrono::steady_clock::now();
    const auto started = started_;
    lock.unlock();
    Done done;
    done.path = std::move(entry.path);
    done.ok = write_(done.path, entry.text, &done.error);
    done.took = std::chrono::steady_clock::now() - started;
    report_(done);
    lock.lock();
    written_[done.path] = done.ok;
    writing_.clear();
    done_.notify_all();
  }
}

}  // namespace mowbite_hold
