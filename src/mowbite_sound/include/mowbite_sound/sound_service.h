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
// Largest part of an upload in bytes, before base64.
constexpr int64_t kMaxPartBytes = 512 << 10;
// An unfinished upload is dropped this long after its last part, in s.
constexpr double kUploadExpiry = 60.0;

// A situation of the catalog (mowbite_api::SoundSituations); which file each has is the platform's.
bool IsSituation(const std::string& name);

// <dir>/<situation><extension>: the original (.mp3) and the WAV (.wav) of a vendor's announcement.
std::string SoundPath(const std::string& dir, const std::string& situation, const char* extension);

// How a conversion of an MP3 to a WAV ended.
enum class Converted { kOk, kNotMp3, kTooLong, kFailed };

// MP3 to WAV, and why not.
using Convert = std::function<Converted(const std::string& mp3, const std::string& wav, std::string* error)>;

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
  // An alarm: without a file that can be read (also an empty path) AlarmTone plays.
  bool alarm = false;
};

// What plays the cues, never waiting for the sound: one at a time, a newer cue replaces one that waits; Stop ends the one
// playing within 0.1 s and drops the one waiting.
class Sink {
 public:
  virtual ~Sink() = default;
  virtual void Play(const Cue& cue) = 0;
  virtual void Stop() = 0;
};

// Free bytes for unprivileged users on the file system of a path, -1 when unknown.
using FreeBytes = std::function<int64_t(const std::string& path)>;
int64_t FreeBytesOf(const std::string& path);

// The checks of a received upload away from the executor.
struct UploadJob {
  std::string situation;
  std::string sha256;
  // The original as received.
  std::string bytes;
  // The folder of the uploads.
  std::string dir;
  // Bytes of all other originals and WAVs.
  int64_t stored = 0;
  int64_t reserve_bytes = 0;
  FreeBytes free_bytes;
};

struct RpcOutcome {
  bool ok = true;
  Json result;
  int code = 0;
  std::string message;
  // Details of a refusal, null for none.
  Json data;
  // The last part of an upload: the answer is CompleteUpload of FinishUpload of this job.
  std::optional<UploadJob> job;
};

struct SoundConfig {
  // Where the originals, WAVs and sounds.json are kept; uploads in <dir>/upload.
  std::string dir;
  // Gain at volume 100, 0 to 1: the OS never plays louder.
  double max_volume = 1.0;
  // Volume (0 to 100) without one in sounds.json.
  int volume = 60;
  // Shortest time in s between two plays of one situation for its event.
  double min_gap = 10.0;
  // Bytes an upload leaves free on the file system of dir: the reserve of the journal.
  int64_t reserve_bytes = 0;
  FreeBytes free_bytes = FreeBytesOf;
};

// What FinishUpload found: the file when it is good, else the refusal or error to answer with.
struct UploadResult {
  std::optional<SoundFile> file;
  RpcOutcome failure;
};

// Checks the received original (sha256, MP3, length, room), converts it and puts both files under their names in the
// upload folder, nothing of a refusal. Safe to run in a thread of its own.
UploadResult FinishUpload(const UploadJob& job, const Convert& convert);

// <dir>/<situation>-<first 16 of sha256><extension>: the files of an upload.
std::string UploadPath(const std::string& dir, const std::string& situation, const std::string& sha256,
                       const char* extension);

// The sounds of the mower without ROS: which situation an event or a change of state means, the emergency over all of
// them, the RPC methods sound.* and sounds.json. An upload of the user plays instead of the vendor's announcement,
// which stays for sound.delete.
class SoundService {
 public:
  SoundService(SoundConfig config, Sink* sink);

  // Files, uploads and volume of sounds.json, removing upload files it does not name; why it is not usable, "" when it
  // is fine or missing.
  std::string Load();
  // The vendor's files after an import; writes sounds.json, "" or why it could not.
  std::string SetFiles(std::map<std::string, SoundFile> files);
  const std::map<std::string, SoundFile>& files() const { return files_; }
  const std::map<std::string, SoundFile>& uploads() const { return uploads_; }
  int volume() const { return volume_; }

  static const std::vector<std::string>& Methods();
  // nullopt for a method of another node; now in monotonic s.
  std::optional<RpcOutcome> Call(const std::string& method, const Json& params, double now);
  // Takes the result of FinishUpload into sounds.json and gives the answer to the last part; until then every upload
  // is BUSY.
  RpcOutcome CompleteUpload(const UploadResult& result);

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

  // The upload that receives parts or is checked.
  struct Upload {
    std::string situation;
    std::string sha256;
    int64_t parts = 0;
    // The part it waits for; parts once all came and FinishUpload runs.
    int64_t next = 0;
    std::string bytes;
    double last = 0;
    bool Checked() const { return next == parts; }
  };

  void UpdateEmergency(Emergency* source, bool emergency, uint32_t bits, double now);
  bool InEmergency() const { return driver_.on || robot_.on; }
  uint32_t EmergencyBits() const { return driver_.bits | robot_.bits; }
  // Plays the situation's file unless the emergency, volume 0 or its last play min_gap ago keep it quiet. An alarm
  // (EMERGENCY, LIFT_TRIGGERED) plays every time: LIFT_TRIGGERED without a file of its own as EMERGENCY, without
  // either as the tone of the player.
  void Announce(const std::string& situation, double now);
  void OnState(const std::string& state, double now);
  double Gain() const;
  // The upload of the situation, else the vendor's file; null for none.
  const SoundFile* Active(const std::string& situation) const;
  std::string WavOf(const std::string& situation) const;
  std::string UploadDir() const;
  void RemoveUpload(const std::string& situation, const std::string& sha256) const;
  // Bytes of all originals and WAVs but those of the upload of situation.
  int64_t StoredBytes(const std::string& situation) const;
  std::string StorePath() const;
  std::string LoadStore();
  std::string Save();

  RpcOutcome List() const;
  RpcOutcome Play(const Json& params);
  RpcOutcome Stop();
  RpcOutcome Volume(const Json& params);
  RpcOutcome ReceivePart(const Json& params, double now);
  RpcOutcome Delete(const Json& params);
  RpcOutcome RefuseInHold(const std::string& what) const;

  SoundConfig config_;
  Sink* sink_;
  // The vendor's announcements and the uploads, by situation.
  std::map<std::string, SoundFile> files_;
  std::map<std::string, SoundFile> uploads_;
  std::optional<Upload> upload_;
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

// The vendor's announcement by situation (codes: situation -> code, the file <code>.mp3 in the first of dirs that has
// it) next to what have already holds: a situation with the same code and its WAV is kept; every other one is copied,
// converted and checked. Uploads are not its business. notes tells what was left out and why.
std::map<std::string, SoundFile> ImportVendor(const std::string& dir, const std::map<std::string, SoundFile>& have,
                                              const std::map<std::string, std::string>& codes,
                                              const std::vector<std::string>& dirs, const Convert& convert,
                                              std::vector<std::string>* notes);

}  // namespace mowbite_sound

#endif  // MOWBITE_SOUND_SOUND_SERVICE_H
