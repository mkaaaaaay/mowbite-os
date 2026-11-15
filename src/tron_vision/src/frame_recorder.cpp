#include "tron_vision/frame_recorder.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

namespace tron_vision {

namespace fs = std::filesystem;

namespace {

// Writes the whole file under a temporary name and renames it, so a reader never sees a part.
bool WriteWhole(const std::string& path, const char* data, size_t bytes, std::string& error) {
  const std::string part = path + ".part";
  {
    std::ofstream out(part, std::ios::binary | std::ios::trunc);
    if (out) out.write(data, static_cast<std::streamsize>(bytes));
    if (!out) {
      error = "write " + part + " failed";
      std::error_code ignored;
      fs::remove(part, ignored);
      return false;
    }
  }
  std::error_code ec;
  fs::rename(part, path, ec);
  if (ec) {
    error = "rename " + part + ": " + ec.message();
    fs::remove(part, ec);
    return false;
  }
  return true;
}

}  // namespace

FrameRecorder::FrameRecorder(RecordLimits limits) : limits_(std::move(limits)) {
  if (limits_.max_bytes == 0 || limits_.dir.empty()) return;
  std::error_code ec;
  fs::create_directories(limits_.dir, ec);
  if (ec) return;
  // Images of an earlier run count against the limits.
  for (const auto& entry : fs::directory_iterator(limits_.dir, ec)) {
    const fs::path& p = entry.path();
    if (p.extension() != ".jpg") continue;
    const std::string stem = p.stem().string();
    const bool digits = std::all_of(stem.begin(), stem.end(), [](char c) { return c >= '0' && c <= '9'; });
    if (stem.empty() || stem.size() > 15 || !digits) continue;
    const int64_t ms = std::stoll(stem);
    std::error_code jpg_ec;
    std::error_code txt_ec;
    const auto jpg = fs::file_size(p, jpg_ec);
    const auto txt = fs::file_size(Path(ms, ".txt"), txt_ec);
    if (jpg_ec) continue;
    const size_t bytes = static_cast<size_t>(jpg) + (txt_ec ? 0 : static_cast<size_t>(txt));
    kept_.push_back({ms, bytes});
    total_ += bytes;
  }
  if (ec) return;
  std::sort(kept_.begin(), kept_.end(), [](const Kept& a, const Kept& b) { return a.ms < b.ms; });
  active_ = true;
}

bool FrameRecorder::Due(double taken) const {
  return active_ && (taken < last_ || taken - last_ >= limits_.interval_s);
}

bool FrameRecorder::Write(double taken, const uint8_t* data, size_t bytes, const std::string& text,
                          std::string& error) {
  if (!active_) return false;
  const auto ms = static_cast<int64_t>(std::llround(taken * 1000.0));
  const size_t incoming = bytes + text.size();
  if (incoming > limits_.max_bytes) {
    return Fail("image of " + std::to_string(bytes) + " bytes over the limit", error);
  }
  Trim(ms, incoming);
  if (!kept_.empty() && kept_.back().ms >= ms) return true;
  if (!WriteWhole(Path(ms, ".txt"), text.data(), text.size(), error)) return Fail(error, error);
  if (!WriteWhole(Path(ms, ".jpg"), reinterpret_cast<const char*>(data), bytes, error)) {
    std::error_code ignored;
    fs::remove(Path(ms, ".txt"), ignored);
    return Fail(error, error);
  }
  kept_.push_back({ms, incoming});
  total_ += incoming;
  last_ = taken;
  return true;
}

std::string FrameRecorder::Path(int64_t ms, const char* ending) const {
  return limits_.dir + "/" + std::to_string(ms) + ending;
}

void FrameRecorder::Trim(int64_t ms, size_t incoming) {
  const auto oldest = ms - static_cast<int64_t>(limits_.keep_s * 1000.0);
  while (!kept_.empty() && (total_ + incoming > limits_.max_bytes || kept_.front().ms < oldest)) {
    Remove(kept_.front());
    total_ -= kept_.front().bytes;
    kept_.pop_front();
  }
}

void FrameRecorder::Remove(const Kept& kept) {
  std::error_code ignored;
  fs::remove(Path(kept.ms, ".jpg"), ignored);
  fs::remove(Path(kept.ms, ".txt"), ignored);
}

bool FrameRecorder::Fail(const std::string& what, std::string& error) {
  error = what;
  active_ = false;
  return false;
}

}  // namespace tron_vision
