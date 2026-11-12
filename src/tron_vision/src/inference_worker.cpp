#include "tron_vision/inference_worker.h"

#include <chrono>
#include <utility>

namespace tron_vision {

void InferenceWorker::Start(const ModelLayout& layout, const DecodeParams& decode) {
  layout_ = layout;
  decode_ = decode;
  thread_ = std::thread([self = shared_from_this()] { self->Run(); });
}

bool InferenceWorker::Stop(double wait_s) {
  if (!thread_.joinable()) {
    model_.Close();
    return true;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  quit_ = true;
  wake_.notify_all();
  const bool ended = wake_.wait_for(lock, std::chrono::duration<double>(wait_s), [this] { return ended_; });
  lock.unlock();
  if (ended) {
    thread_.join();
  } else {
    thread_.detach();
  }
  return ended;
}

bool InferenceWorker::Idle() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return phase_ == Phase::kIdle;
}

bool InferenceWorker::Busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return phase_ == Phase::kQueued || phase_ == Phase::kRunning;
}

void InferenceWorker::Submit() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != Phase::kIdle) return;
  phase_ = Phase::kQueued;
  wake_.notify_all();
}

bool InferenceWorker::Take(Result& result) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != Phase::kDone) return false;
  result = std::move(result_);
  result_ = Result();
  phase_ = Phase::kIdle;
  return true;
}

void InferenceWorker::Run() {
  std::unique_lock<std::mutex> lock(mutex_);
  for (;;) {
    wake_.wait(lock, [this] { return quit_ || phase_ == Phase::kQueued; });
    if (quit_) break;
    phase_ = Phase::kRunning;
    lock.unlock();
    Result result;
    const auto start = std::chrono::steady_clock::now();
    result.ok = model_.Run(input_.data(), input_.size(), outputs_, result.error);
    result.run_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (result.ok) result.detections = DecodeDetections(layout_, model_.Outputs(), outputs_, decode_);
    lock.lock();
    result_ = std::move(result);
    phase_ = Phase::kDone;
  }
  model_.Close();
  ended_ = true;
  wake_.notify_all();
}

}  // namespace tron_vision
