#ifndef MOWBITE_SOUND_PLAYER_H
#define MOWBITE_SOUND_PLAYER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "mowbite_sound/sound_service.h"

namespace mowbite_sound {

// Where the samples of a cue go; the device is open only from Open to Close.
class Output {
 public:
  virtual ~Output() = default;
  // false and why when it cannot play at rate now.
  virtual bool Open(const Cue& cue, uint32_t rate, std::string* error) = 0;
  // Blocks while the device has no room, at most about one piece.
  virtual bool Write(const int16_t* samples, size_t count, std::string* error) = 0;
  // Seconds written but not heard yet.
  virtual double Queued() = 0;
  // drop: at once, else after the last sample.
  virtual void Close(bool drop) = 0;
};

// libasound on device, with room for 0.1 s.
std::unique_ptr<Output> MakeAlsaOutput(const std::string& device);
// Whether device opens for playback now; why not.
bool ProbeAlsa(const std::string& device, std::string* error);
// For the simulation: a JSON line per start and end of a cue in path, in real time.
std::unique_ptr<Output> MakeFileOutput(const std::string& path);

// Plays the cues in a thread of its own, in pieces of 50 ms so that Stop ends a sound within 0.1 s. Play and Stop
// never wait for the sound; problems go to log.
class Player : public Sink {
 public:
  Player(std::unique_ptr<Output> output, std::function<void(const std::string&)> log);
  ~Player() override;

  void Play(const Cue& cue) override;
  void Stop() override;

 private:
  void Run();
  void PlayOne(const Cue& cue);

  std::unique_ptr<Output> output_;
  std::function<void(const std::string&)> log_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<Cue> waiting_;
  bool quit_ = false;
  // Set by Stop, cleared when the next cue starts.
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace mowbite_sound

#endif  // MOWBITE_SOUND_PLAYER_H
