#include "mowbite_sound/player.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "mowbite_hold/file_io.h"
#include "mowbite_sound/wav.h"

namespace mowbite_sound {

namespace {

// Pieces per second: Stop ends a sound after at most one piece.
constexpr uint32_t kPiecesPerSecond = 20;
constexpr auto kQueuedPoll = std::chrono::milliseconds(10);

class FileOutput : public Output {
 public:
  explicit FileOutput(std::string path) : path_(std::move(path)) {}

  bool Open(const Cue& cue, uint32_t rate, std::string* error) override {
    situation_ = cue.situation;
    rate_ = rate;
    return Append(Json{{"event", "start"}, {"situation", cue.situation}, {"gain", cue.gain}, {"rate", rate}}, error);
  }

  bool Write(const int16_t*, size_t count, std::string*) override {
    std::this_thread::sleep_for(std::chrono::duration<double>(static_cast<double>(count) / rate_));
    return true;
  }

  double Queued() override { return 0; }

  void Close(bool drop) override {
    std::string error;
    Append(Json{{"event", drop ? "stopped" : "done"}, {"situation", situation_}}, &error);
  }

 private:
  bool Append(const Json& line, std::string* error) {
    const int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd < 0) {
      *error = "cannot open " + path_;
      return false;
    }
    const std::string text = line.dump() + "\n";
    const bool ok = ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size());
    ::close(fd);
    if (!ok) *error = "cannot write " + path_;
    return ok;
  }

  std::string path_;
  std::string situation_;
  uint32_t rate_ = 1;
};

}  // namespace

std::unique_ptr<Output> MakeFileOutput(const std::string& path) {
  return std::make_unique<FileOutput>(path);
}

Player::Player(std::unique_ptr<Output> output, std::function<void(const std::string&)> log)
    : output_(std::move(output)), log_(std::move(log)), thread_([this] { Run(); }) {}

Player::~Player() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    quit_ = true;
    stop_ = true;
  }
  wake_.notify_all();
  thread_.join();
}

void Player::Play(const Cue& cue) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    waiting_ = cue;
  }
  wake_.notify_all();
}

void Player::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  waiting_.reset();
  stop_ = true;
}

void Player::Run() {
  while (true) {
    Cue cue;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return quit_ || waiting_.has_value(); });
      if (quit_) return;
      cue = std::move(*waiting_);
      waiting_.reset();
      stop_ = false;
    }
    PlayOne(cue);
  }
}

void Player::PlayOne(const Cue& cue) {
  std::string error;
  std::optional<Wav> wav;
  if (!cue.path.empty()) {
    std::string bytes;
    if (mowbite_hold::ReadSmallFile(cue.path, kMaxWavBytes, &bytes, &error) != mowbite_hold::FileRead::kText) {
      log_(cue.situation + ": cannot read " + cue.path + " " + error);
    } else {
      wav = DecodeWav(bytes, &error);
      if (!wav) log_(cue.situation + ": " + cue.path + ": " + error);
    }
  }
  if (!wav && cue.alarm) wav = AlarmTone();
  if (!wav) return;
  if (!output_->Open(cue, wav->rate, &error)) {
    log_(cue.situation + ": " + error);
    return;
  }
  const size_t piece = std::max<size_t>(1, wav->rate / kPiecesPerSecond);
  std::vector<int16_t> buffer(piece);
  const auto& samples = wav->samples;
  for (size_t at = 0; at < samples.size() && !stop_; at += piece) {
    const size_t count = std::min(piece, samples.size() - at);
    for (size_t i = 0; i < count; ++i) {
      const double value = std::lround(samples[at + i] * cue.gain);
      buffer[i] = static_cast<int16_t>(std::clamp(value, -32768.0, 32767.0));
    }
    if (!output_->Write(buffer.data(), count, &error)) {
      log_(cue.situation + ": " + error);
      output_->Close(true);
      return;
    }
  }
  while (!stop_ && output_->Queued() > 0) std::this_thread::sleep_for(kQueuedPoll);
  output_->Close(stop_);
}

}  // namespace mowbite_sound
