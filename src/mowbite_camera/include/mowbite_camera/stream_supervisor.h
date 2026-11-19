#ifndef MOWBITE_CAMERA_STREAM_SUPERVISOR_H
#define MOWBITE_CAMERA_STREAM_SUPERVISOR_H

#include <functional>
#include <string>

namespace mowbite_camera {

// After an end the stream server starts again at most this often in a row, each time after kStreamRestartDelay s;
// a run of kStreamStableAfter s counts the ends from 0 again.
constexpr int kStreamRestarts = 3;
constexpr double kStreamRestartDelay = 2.0;
constexpr double kStreamStableAfter = 600.0;
// A start whose API has not answered after this long is ended and counts as an end, in s.
constexpr double kStreamStartSeconds = 20.0;
// A running server is stopped once no camera has wanted it for this long, in s; a stop is no end.
constexpr double kStreamIdleStop = 60.0;
// Questions in a row a running server's API leaves without an answer before it counts as ended.
constexpr int kStreamApiMisses = 5;

// The stream server as its supervisor drives it.
class StreamServer {
 public:
  virtual ~StreamServer() = default;
  // Whether its program is there to be run.
  virtual bool Installed() = 0;
  virtual bool Start(std::string* error) = 0;
  // Whether it runs; the call that finds it ended tells in *how what it ended with.
  virtual bool Running(std::string* how) = 0;
  // Asks it to end without waiting for it; Reap finishes that.
  virtual void Stop() = 0;
  // Once a tick: ends what a stop left running.
  virtual void Reap() {}
};

// Starts the stream server with the first camera that needs it and again after it ended, stops it while none needs
// it, and tells why webrtc does not work meanwhile: NOT_SET_UP without the program or after the last restart,
// NO_SIGNAL while it starts.
class StreamSupervisor {
 public:
  // now is a steady clock in s; server outlives the supervisor.
  StreamSupervisor(StreamServer* server, std::function<double()> now);

  // Once a second. wanted: a camera streams over the server and its configuration is written. True when the
  // server ended or was stopped since the last call, and its sessions with it.
  bool Tick(bool wanted);
  // The API of the server answered.
  void Answered();
  // Asked for its sessions while it is up: after kStreamApiMisses in a row without an answer it counts as ended (and
  // is stopped), true then: its sessions are gone with it.
  bool Asked(bool answered);
  // It runs and its API has not answered yet: to be asked.
  bool Starting() const {
    return phase_ == Phase::kStarting;
  }
  bool Up() const {
    return phase_ == Phase::kUp;
  }
  // A reason of the cameras with a few words; an empty reason: webrtc works.
  const std::string& reason() const {
    return reason_;
  }
  const std::string& detail() const {
    return detail_;
  }

 private:
  enum class Phase { kIdle, kWaiting, kStarting, kUp, kGaveUp };

  void Ended(double now, bool ran, const std::string& how);
  void Tell(const std::string& reason, const std::string& detail);

  StreamServer* const server_;
  const std::function<double()> now_;
  Phase phase_ = Phase::kIdle;
  // Ends in a row, when the next start is due, when the last one was and when the server was last wanted.
  int ends_ = 0;
  int api_misses_ = 0;
  double next_start_ = 0;
  double began_ = 0;
  double wanted_at_ = 0;
  std::string reason_;
  std::string detail_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_STREAM_SUPERVISOR_H
