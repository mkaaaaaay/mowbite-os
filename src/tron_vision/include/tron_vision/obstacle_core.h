#ifndef TRON_VISION_OBSTACLE_CORE_H
#define TRON_VISION_OBSTACLE_CORE_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mowbite_obstacles/ground.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/inference_worker.h"
#include "tron_vision/jpeg_luma.h"
#include "tron_vision/obstacle_sight.h"
#include "tron_vision/stereo_depth.h"
#include "tron_vision/stereo_lens.h"
#include "tron_vision/stereo_loop.h"
#include "tron_vision/yolo_decode.h"

namespace tron_vision {

enum class ObstacleKind { kPerson, kAnimal, kObject, kIgnore };

struct ObstacleConfig {
  // Model of the camera AI; it runs on its own NPU core in this process.
  std::string library_path;
  std::string model_path;
  std::string labels_path;
  int npu_core = 2;
  bool swap_rb = false;
  DecodeParams decode;
  // Class names by kind; any other class of the model is an object. Each name has to be a class of the model and
  // person must not be empty, else NOT_SET_UP.
  std::vector<std::string> person;
  std::vector<std::string> animal;
  std::vector<std::string> ignore;
  // Heights of things of an object class (m) for the size probe; a person or an animal is never moved farther away.
  std::map<std::string, double> known_height;
  // Analyses per second; without one for late_s the camera counts as without sight.
  double rate = 5.0;
  double late_s = 0.4;
  // A result of a frame older than this (s) is dropped: no sight until a fresh one.
  double max_result_age_s = 0.5;
  // A run of the model over slow_run_s max_slow_runs times, or one not ended after hang_s, ends the process.
  double slow_run_s = 2.0;
  int max_slow_runs = 3;
  double hang_s = 6.0;
  // Calibration file of both eyes (ReadStereoCalibration) and the mount of the left eye in base_link.
  std::string calibration_path;
  bool have_mount = false;
  mowbite_obstacles::Mount mount;
  // Why the platform left the camera without a part of its setup (mount, classes); empty when all is there.
  std::string not_set_up;
  // Size of each eye in the frame: both side by side, YUYV.
  uint32_t eye_width = 640;
  uint32_t eye_height = 480;
  double wheel_radius = 0.1125;
  // Farthest ground the camera reports (m).
  double range_max = 3.0;
  MatchParams match;
  BoxDepthParams box_depth;
  // Spread of the disparity (px) and its offset of the calibration; both count into the spread of the depth.
  double disparity_noise_px = 0.25;
  double disparity_bias_px = 0.1;
  // How far a box edge may be off (px), and how well the mount is known (rad, m).
  double box_edge_px = 2.0;
  double pitch_sigma = 0.0087;
  double height_sigma = 0.01;
  // Squared Mahalanobis distance up to which ground and stereo agree.
  double fuse_gate = 9.21;
  // Size probe: a depth below size_ratio of the one from the known height takes that one, with at least
  // size_sigma_share of it as spread and never a spread that leaves out the depth of the stereo.
  double size_ratio = 0.5;
  double size_sigma_share = 0.3;
  SightLimits sight;
  // Photos: at most one per photo_interval_s, at most photo_max_bytes, quality stepped down from photo_quality.
  double photo_interval_s = 1.0;
  size_t photo_max_bytes = 100 * 1024;
  int photo_quality = 85;
};

enum class PositionFrom { kGround, kStereo, kBoth };

struct FoundObstacle {
  ObstacleKind kind = ObstacleKind::kObject;
  std::string class_name;
  double score = 0.0;
  mowbite_obstacles::Spot spot;
  double radius = 0.0;
  // NaN when unknown.
  double height = 0.0;
  bool cut_bottom = false;
  PositionFrom from = PositionFrom::kGround;
  bool agrees = false;
  // In pixels of the left eye.
  PixelBox box;
};

struct ObstacleReport {
  // When the frame was taken on the monotonic clock (s); without sight the time of the report.
  double taken = 0.0;
  SightReason reason = SightReason::kNone;
  std::string detail;
  // The sector of the ground the camera sees; zero without a mount.
  mowbite_obstacles::View view;
  std::vector<FoundObstacle> obstacles;
  // The left eye as JPEG, when one was wanted and something was found.
  std::vector<uint8_t> jpeg;
  uint32_t width = 0;
  uint32_t height = 0;
};

struct ObstacleStatus {
  double infer_ms = 0.0;
  double rate = 0.0;
  uint64_t analyses = 0;
  // A few words on the last analysis ("person 0.81 at 1.4 m" or "none").
  std::string last;
  int exit_code = kExitNone;
};

// Obstacles on the ground ahead from the stereo camera, without ROS: takes the frames of StereoLoop, runs the model on
// the left eye at a fixed rate on its own thread, finds each box on the ground (ground plane and stereo depth) and
// reports at that rate, also without sight.
class ObstacleCore : public FrameSink {
 public:
  ObstacleCore(ObstacleConfig config, std::function<double()> now);
  ~ObstacleCore() override;
  ObstacleCore(const ObstacleCore&) = delete;
  ObstacleCore& operator=(const ObstacleCore&) = delete;

  // Reads the calibration and the class list, opens and checks the model; false with the reason as NOT_SET_UP.
  bool Load();

  void OnFrame(const CapturedFrame& frame, double now, const FrameContext& context) override;
  void OnStop() override;

  // From any thread.
  void SetPhotosWanted(bool wanted) {
    photos_wanted_ = wanted;
  }
  // The next report when one is due: after each analysis, and without sight once per 1 / rate.
  bool Poll(double now, const StereoStatus& camera, ObstacleReport& report);

  const ObstacleStatus& Status() const {
    return status_;
  }
  ObstacleKind KindOf(const std::string& class_name) const;

 private:
  // The analysis in progress: a copy of its frame and when it was taken.
  struct Job {
    std::vector<uint8_t> frame;
    uint32_t line_bytes = 0;
    double taken = 0.0;
    double submitted = 0.0;
    uint64_t epoch = 0;
  };

  bool CheckWorker(double now);
  void Analyse(const std::vector<Detection>& detections, ObstacleReport& report);
  bool Locate(const Detection& d, const StereoPair& pair, FoundObstacle& found) const;
  void MakePhoto(ObstacleReport& report);
  void NoSight(double now, SightReason reason, const std::string& detail, ObstacleReport& report);
  void StopWorker();

  ObstacleConfig config_;
  std::function<double()> now_;
  std::string not_set_up_;
  StereoCalibration calibration_;
  mowbite_obstacles::Plane ground_;
  mowbite_obstacles::View view_;
  std::vector<std::string> names_;
  ModelLayout layout_;
  std::shared_ptr<InferenceWorker> worker_;
  Placement place_;
  SightCheck sight_;
  Job job_;
  bool busy_ = false;
  // Counts the starts of the camera; a result of an earlier one is dropped.
  uint64_t epoch_ = 0;
  bool capturing_ = false;
  double next_submit_ = 0.0;
  double last_report_ = -1.0;
  double last_result_ = -1.0;
  double last_photo_ = -1.0;
  double last_submit_ = -1.0;
  int slow_runs_ = 0;
  std::atomic<bool> photos_wanted_{false};
  YuyvJpeg jpeg_;
  ObstacleStatus status_;
};

// Why the camera does not see while it does not run, from the state of StereoLoop.
SightReason CameraReason(StereoState state);

}  // namespace tron_vision

#endif  // TRON_VISION_OBSTACLE_CORE_H
