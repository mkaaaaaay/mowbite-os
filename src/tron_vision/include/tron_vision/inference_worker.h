#ifndef TRON_VISION_INFERENCE_WORKER_H
#define TRON_VISION_INFERENCE_WORKER_H

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tron_vision/model_check.h"
#include "tron_vision/npu_model.h"
#include "tron_vision/yolo_decode.h"

namespace tron_vision {

// The model on a thread of its own, one run at a time, so that the camera thread never waits for the NPU.
// Owned through a shared_ptr: the thread holds one too, so a run that never ends keeps the object alive.
class InferenceWorker : public std::enable_shared_from_this<InferenceWorker> {
 public:
  struct Result {
    bool ok = false;
    std::string error;
    // Duration of the run in real time.
    double run_s = 0.0;
    // In model pixels.
    std::vector<Detection> detections;
  };

  InferenceWorker() = default;
  InferenceWorker(const InferenceWorker&) = delete;
  InferenceWorker& operator=(const InferenceWorker&) = delete;

  // Opened and checked before Start(); afterwards only the thread uses it.
  NpuModel& Model() {
    return model_;
  }
  void Start(const ModelLayout& layout, const DecodeParams& decode);
  // Ends the thread, which closes the model. False if a run did not end within wait_s: the thread is then left to
  // itself.
  bool Stop(double wait_s);

  // The model input; fill it only while Idle(), then Submit().
  std::vector<uint8_t>& Input() {
    return input_;
  }
  bool Idle() const;
  // Submitted and not ended.
  bool Busy() const;
  void Submit();
  // True once for each run that ended.
  bool Take(Result& result);

 private:
  enum class Phase { kIdle, kQueued, kRunning, kDone };

  void Run();

  NpuModel model_;
  ModelLayout layout_;
  DecodeParams decode_;
  std::vector<uint8_t> input_;
  std::vector<std::vector<int8_t>> outputs_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::thread thread_;
  Phase phase_ = Phase::kIdle;
  bool quit_ = false;
  bool ended_ = false;
  Result result_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_INFERENCE_WORKER_H
