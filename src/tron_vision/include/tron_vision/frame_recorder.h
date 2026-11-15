#ifndef TRON_VISION_FRAME_RECORDER_H
#define TRON_VISION_FRAME_RECORDER_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace tron_vision {

struct RecordLimits {
  std::string dir;
  // Off with 0.
  size_t max_bytes = 0;
  // Images older than this by the capture clock go, also those of an earlier run.
  double keep_s = 3600.0;
  // At most one image per this many seconds of capture time.
  double interval_s = 0.5;
};

// Keeps camera images as they came, each as <ms>.jpg with a text <ms>.txt beside it, <ms> the capture time on the
// monotonic clock. Together they stay within max_bytes and keep_s, the oldest go first. Files appear complete under
// their name. A write that fails ends the recording for the life of the process.
class FrameRecorder {
 public:
  explicit FrameRecorder(RecordLimits limits);

  bool Active() const {
    return active_;
  }
  // Whether an image taken at `taken` is to be kept.
  bool Due(double taken) const;
  // Keeps the image and its text; false with the error when it could not, which ends the recording.
  bool Write(double taken, const uint8_t* data, size_t bytes, const std::string& text, std::string& error);

 private:
  struct Kept {
    int64_t ms = 0;
    size_t bytes = 0;
  };
  std::string Path(int64_t ms, const char* ending) const;
  // Drops the oldest until `incoming` more bytes fit and nothing is older than keep_s before `ms`.
  void Trim(int64_t ms, size_t incoming);
  void Remove(const Kept& kept);
  bool Fail(const std::string& what, std::string& error);

  RecordLimits limits_;
  bool active_ = false;
  std::deque<Kept> kept_;
  size_t total_ = 0;
  double last_ = -1e18;
};

}  // namespace tron_vision

#endif  // TRON_VISION_FRAME_RECORDER_H
