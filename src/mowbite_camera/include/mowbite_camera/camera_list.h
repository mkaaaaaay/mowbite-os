#ifndef MOWBITE_CAMERA_CAMERA_LIST_H
#define MOWBITE_CAMERA_CAMERA_LIST_H

#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_camera {

using Json = nlohmann::ordered_json;

constexpr char kMjpeg[] = "mjpeg";
constexpr char kWebrtc[] = "webrtc";
constexpr char kNoSignal[] = "NO_SIGNAL";
constexpr char kNotSetUp[] = "NOT_SET_UP";

// A camera as hw/cameras tells it (mowbite_msgs/Camera).
struct CameraEntry {
  std::string id;
  std::string name;
  std::string process;
  std::string kind;
  std::string position;
  uint32_t width = 0;
  uint32_t height = 0;
  bool detections = false;
  std::vector<std::string> protocols;
  std::string source;
  bool available = false;
  std::string reason;
  std::string detail;
};

// Whether a camera can stream now, and why not.
struct CameraState {
  bool available = false;
  std::string reason;
  std::string detail;

  bool operator==(const CameraState& other) const {
    return available == other.available && reason == other.reason && detail == other.detail;
  }
};

// What CAMERA_CHANGED tells of a camera besides its id: whether it can stream and how.
struct CameraNews {
  CameraState state;
  std::vector<std::string> protocols;
  uint32_t width = 0;
  uint32_t height = 0;

  bool operator==(const CameraNews& other) const {
    return state == other.state && protocols == other.protocols && width == other.width && height == other.height;
  }
};

struct CameraChange {
  std::string camera_id;
  CameraNews news;
};

// An id the OS takes: a letter, then letters, digits and underscores, 32 at most, not ending in kPlainSuffix. It is
// part of a topic, of urls and of the configuration of the stream server.
bool IsCameraId(const std::string& id);

// A camera with detections has a view without the boxes as well (mowbite_msgs/Camera): <id>_plain in the urls and
// the stream server, camera/<id>/jpeg_plain and <source>_plain at the camera.
constexpr char kPlainSuffix[] = "_plain";
// What a viewer watches: the id, or the view without the boxes.
std::string ViewOf(const std::string& id, bool plain);
// The picture topic of a view.
std::string PictureTopic(const std::string& view);

// When a process ended (mono, s on the steady clock) and whether it stays down, from the text of its exit note;
// false when the text is not such a note.
bool ParseExitNote(const std::string& text, double* mono, bool* down);

// The cameras of the platform by id, in the order they first reported, kept until the node ends. What a camera
// says itself counts unless its process ended after it (NO_SIGNAL) or the stream server it needs does not run.
class CameraList {
 public:
  // An entry of hw/cameras that came at t (s on the steady clock). False with *problem when it cannot be listed; a
  // problem with true names what was left out of it.
  bool Update(const CameraEntry& entry, double t, std::string* problem);
  // The process ended at t (the same clock, mono of its exit note); down: it is not started again.
  void ProcessEnded(const std::string& process, double t, bool down);
  // Why webrtc does not work now, a reason of the cameras with a few words; an empty reason: it works.
  void SetWebrtc(const std::string& reason, const std::string& detail);
  // The cameras whose news differ from those of the last call, new ones among them.
  std::vector<CameraChange> TakeChanges();

  bool Empty() const {
    return cameras_.empty();
  }
  // nullptr for an id that is not listed.
  const CameraEntry* Find(const std::string& id) const;
  CameraState State(const std::string& id) const;
  // The result of camera.list.
  Json List() const;
  // The processes of the listed cameras, each once.
  std::vector<std::string> Processes() const;
  // View -> its RTSP address, for the cameras with webrtc; a camera with detections has its view without boxes too.
  std::map<std::string, std::string> WebrtcSources() const;
  // What SetWebrtc had last; the detail is empty with an empty reason.
  const std::string& webrtc_reason() const {
    return webrtc_reason_;
  }
  const std::string& webrtc_detail() const {
    return webrtc_detail_;
  }

 private:
  struct Camera {
    CameraEntry entry;
    // When its last entry came.
    double reported = 0;
    // The news TakeChanges gave last, none before the first one.
    std::optional<CameraNews> told;
  };
  struct Ended {
    double t = 0;
    bool down = false;
  };

  CameraState StateOf(const Camera& camera) const;

  std::vector<Camera> cameras_;
  std::map<std::string, Ended> ended_;
  std::string webrtc_reason_;
  std::string webrtc_detail_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_CAMERA_LIST_H
