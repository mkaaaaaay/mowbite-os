#ifndef MOWBITE_SOUND_SOUND_SERVICE_H
#define MOWBITE_SOUND_SOUND_SERVICE_H

#include <cstdint>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_sound {

using Json = nlohmann::ordered_json;

// The emergency bits of a lift: LIFT and LIFT_MULTIPLE.
constexpr uint32_t kLiftBits = (1u << 3) | (1u << 4);
// Charge (0..1) at which charging is done.
constexpr double kCharged = 0.99;
// Largest original of a vendor announcement in bytes.
constexpr int64_t kMaxVendorBytes = 1 << 20;
// All originals and WAVs together in bytes.
constexpr int64_t kMaxStoreBytes = 40 << 20;

// A situation of the catalog (mowbite_api::SoundSituations); which file each has is the platform's.
bool IsSituation(const std::string& name);

// <dir>/<situation><extension>: the original (.mp3) and the WAV (.wav) of a situation.
std::string SoundPath(const std::string& dir, const std::string& situation, const char* extension);

// The sound of a situation as sounds.json keeps it; the files are <name>.mp3 (original) and <name>.wav.
struct SoundFile {
  // vendor: an announcement of the maker of the mower; upload: one of the user.
  std::string source;
  // The vendor's code of the announcement, empty for an upload.
  std::string code;
  // Of the original.
  int64_t size = 0;
  std::string sha256;
  double seconds = 0;
};

struct Cue {
  std::string situation;
  // The WAV to play.
  std::string path;
  // Factor on every sample, more than 0 and at most 1.
  double gain = 0;
};

// What plays the cues, never waiting for the sound: one at a time, a newer cue replaces one that waits; Stop ends the one
// playing within 0.1 s and drops the one waiting.
class Sink {
 public:
  virtual ~Sink() = default;
  virtual void Play(const Cue& cue) = 0;
  virtual void Stop() = 0;
};

struct RpcOutcome {
  bool ok = true;
  Json result;
  int code = 0;
  std::string message;
  // Details of a refusal, null for none.
  Json data;
};

struct SoundConfig {
  // Where the originals, WAVs and sounds.json are kept.
  std::string dir;
  // Gain at volume 100, 0 to 1: the OS never plays louder.
  double max_volume = 1.0;
  // Volume (0 to 100) without one in sounds.json.
  int volume = 60;
  // Shortest time in s between two plays of one situation for its event.
  double min_gap = 10.0;
};

// The sounds of the mower without ROS: which situation an event or a change of state means, the emergency over all of
// them, the RPC methods sound.* and sounds.json.
class SoundService {
 public:
  SoundService(SoundConfig config, Sink* sink);

  // Files and volume of sounds.json; why it is not usable, "" when it is fine or missing.
  std::string Load();
  // The files after an import; writes sounds.json, "" or why it could not.
  std::string SetFiles(std::map<std::string, SoundFile> files);
  const std::map<std::string, SoundFile>& files() const { return files_; }
  int volume() const { return volume_; }

  static const std::vector<std::string>& Methods();
  // nullopt for a method of another node.
  std::optional<RpcOutcome> Call(const std::string& method, const Json& params);

  // An event of events/json.
  void OnEvent(const Json& event, double now);
  // The emergency of the driver (hw/status, 10 Hz) and of robot_state (1 Hz): either one counts, the bits of both.
  void OnDriverEmergency(bool emergency, uint32_t bits, double now);
  void OnRobotState(const std::string& state, bool emergency, uint32_t bits, bool charging, double battery,
                    double now);
  // The hold of an update in unix s, 0 for none.
  void SetHold(int64_t until) { hold_until_ = until; }

 private:
  struct Emergency {
    bool seen = false;
    bool on = false;
    uint32_t bits = 0;
  };

  void UpdateEmergency(Emergency* source, bool emergency, uint32_t bits, double now);
  bool InEmergency() const { return driver_.on || robot_.on; }
  uint32_t EmergencyBits() const { return driver_.bits | robot_.bits; }
  // Plays the situation's file unless the emergency, volume 0 or its last play min_gap ago keep it quiet.
  void Announce(const std::string& situation, double now);
  void OnState(const std::string& state, double now);
  double Gain() const;
  std::string StorePath() const;
  std::string Save();

  RpcOutcome List() const;
  RpcOutcome Play(const Json& params);
  RpcOutcome Stop();
  RpcOutcome Volume(const Json& params);
  RpcOutcome Refuse(const std::string& reason, const std::string& message) const;

  SoundConfig config_;
  Sink* sink_;
  std::map<std::string, SoundFile> files_;
  int volume_;
  int64_t hold_until_ = 0;
  Emergency driver_;
  Emergency robot_;
  // The state of the logic, from STATE once one came, before that from robot_state.
  std::string state_;
  bool state_from_events_ = false;
  std::optional<bool> charging_;
  double battery_ = 0;
  std::map<std::string, double> last_played_;
};

// The folders of the vendor's announcements in the order to look in: a * matches as in the shell, matches of one
// pattern newest version first.
std::vector<std::string> VendorDirs(const std::vector<std::string>& patterns);

// MP3 to WAV, false and why.
using Convert = std::function<bool(const std::string& mp3, const std::string& wav, std::string* error)>;

// The vendor's announcement by situation (codes: situation -> code, the file <code>.mp3 in the first of dirs that has
// it) next to what have already holds: a situation with the same code and its WAV is kept, an upload always; every
// other one is copied, converted and checked. notes tells what was left out and why.
std::map<std::string, SoundFile> ImportVendor(const std::string& dir, const std::map<std::string, SoundFile>& have,
                                              const std::map<std::string, std::string>& codes,
                                              const std::vector<std::string>& dirs, const Convert& convert,
                                              std::vector<std::string>* notes);

}  // namespace mowbite_sound

#endif  // MOWBITE_SOUND_SOUND_SERVICE_H
