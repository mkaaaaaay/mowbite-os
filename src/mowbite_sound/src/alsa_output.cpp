#include <alsa/asoundlib.h>

#include "mowbite_sound/player.h"

namespace mowbite_sound {

namespace {

// Room in the device for 0.1 s: a stop drops at most that much, a piece of 50 ms always fits.
constexpr unsigned kLatencyUs = 100000;

class AlsaOutput : public Output {
 public:
  explicit AlsaOutput(std::string device) : device_(std::move(device)) {}
  ~AlsaOutput() override { Close(true); }

  bool Open(const Cue&, uint32_t rate, std::string* error) override {
    // A device another process holds is an error at once, not a wait before every sound; writing then waits.
    int result = snd_pcm_open(&pcm_, device_.c_str(), SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
    if (result < 0) {
      pcm_ = nullptr;
      *error = "cannot open " + device_ + ": " + snd_strerror(result);
      return false;
    }
    result = snd_pcm_nonblock(pcm_, 0);
    if (result < 0) {
      *error = "cannot set up " + device_ + ": " + snd_strerror(result);
      Close(true);
      return false;
    }
    result = snd_pcm_set_params(pcm_, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 1, rate, 1, kLatencyUs);
    if (result < 0) {
      *error = "cannot set up " + device_ + ": " + snd_strerror(result);
      Close(true);
      return false;
    }
    rate_ = rate;
    return true;
  }

  bool Write(const int16_t* samples, size_t count, std::string* error) override {
    while (count > 0) {
      snd_pcm_sframes_t written = snd_pcm_writei(pcm_, samples, count);
      if (written < 0) written = snd_pcm_recover(pcm_, static_cast<int>(written), 1);
      if (written < 0) {
        *error = "cannot write to " + device_ + ": " + snd_strerror(static_cast<int>(written));
        return false;
      }
      samples += written;
      count -= static_cast<size_t>(written);
    }
    return true;
  }

  double Queued() override {
    // A sound shorter than the buffer has not started yet.
    if (snd_pcm_state(pcm_) == SND_PCM_STATE_PREPARED) snd_pcm_start(pcm_);
    snd_pcm_sframes_t delay = 0;
    if (snd_pcm_delay(pcm_, &delay) < 0 || delay <= 0) return 0;
    return static_cast<double>(delay) / rate_;
  }

  void Close(bool drop) override {
    if (!pcm_) return;
    if (drop) {
      snd_pcm_drop(pcm_);
    } else {
      snd_pcm_drain(pcm_);
    }
    snd_pcm_close(pcm_);
    pcm_ = nullptr;
  }

 private:
  std::string device_;
  snd_pcm_t* pcm_ = nullptr;
  uint32_t rate_ = 1;
};

}  // namespace

std::unique_ptr<Output> MakeAlsaOutput(const std::string& device) {
  return std::make_unique<AlsaOutput>(device);
}

bool ProbeAlsa(const std::string& device, std::string* error) {
  snd_pcm_t* pcm = nullptr;
  const int result = snd_pcm_open(&pcm, device.c_str(), SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
  if (result < 0) {
    *error = "cannot open " + device + ": " + snd_strerror(result);
    return false;
  }
  snd_pcm_close(pcm);
  return true;
}

}  // namespace mowbite_sound
