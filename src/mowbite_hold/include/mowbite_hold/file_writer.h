#ifndef MOWBITE_HOLD_FILE_WRITER_H
#define MOWBITE_HOLD_FILE_WRITER_H

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace mowbite_hold {

// Writes small files in a thread of its own, so the caller does not wait for the disk. Of several texts for one file
// only the latest is written, and the files are written in the order of their latest texts.
class FileWriter {
 public:
  struct Done {
    std::string path;
    bool ok = false;
    std::string error;
    std::chrono::steady_clock::duration took{};
  };
  // Called in the writer thread after each write, before a WaitFor for its file returns.
  using Report = std::function<void(const Done&)>;
  using Write = std::function<bool(const std::string& path, const std::string& text, std::string* error)>;

  FileWriter(Write write, Report report);
  // Writes what is queued, then ends the thread.
  ~FileWriter();
  FileWriter(const FileWriter&) = delete;
  FileWriter& operator=(const FileWriter&) = delete;

  void Submit(const std::string& path, std::string text);
  // Until no text for path is queued or being written; false when that takes longer than limit.
  bool WaitFor(const std::string& path, std::chrono::steady_clock::duration limit);
  // How long the write in progress has run, zero without one.
  std::chrono::steady_clock::duration Busy() const;

 private:
  struct Entry {
    std::string path;
    std::string text;
  };

  void Run();
  bool Pending(const std::string& path) const;

  const Write write_;
  const Report report_;
  mutable std::mutex mutex_;
  std::condition_variable work_;
  std::condition_variable done_;
  std::deque<Entry> queue_;
  // The file being written, empty between writes.
  std::string writing_;
  std::chrono::steady_clock::time_point started_;
  bool stop_ = false;
  std::thread thread_;
};

}  // namespace mowbite_hold

#endif  // MOWBITE_HOLD_FILE_WRITER_H
