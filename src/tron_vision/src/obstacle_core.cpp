#include "tron_vision/obstacle_core.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace tron_vision {

namespace {

constexpr size_t kLastShown = 3;
constexpr int kLowestQuality = 30;
constexpr int kQualityStep = 15;

bool Exists(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0;
}

std::string Fixed(double value, const char* format) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

bool Contains(const std::vector<std::string>& list, const std::string& name) {
  return std::find(list.begin(), list.end(), name) != list.end();
}

}  // namespace

SightReason CameraReason(StereoState state) {
  switch (state) {
    case StereoState::kPausedHot: return SightReason::kTooHot;
    case StereoState::kLocked: return SightReason::kLockedAfterCrash;
    case StereoState::kWaitingSetup:
    case StereoState::kNotConfigured: return SightReason::kNotSetUp;
    case StereoState::kWaitingIdle:
    case StereoState::kSelftest:
    case StereoState::kPausedDriver: return SightReason::kSelfTest;
    case StereoState::kOff:
    case StereoState::kReady:
    case StereoState::kStreaming:
    case StereoState::kPausedBlade:
    case StereoState::kNoFrames:
    case StereoState::kStalled:
    case StereoState::kEncoderMissing:
    case StereoState::kEncoderError:
    case StereoState::kFailed: break;
  }
  return SightReason::kNoSignal;
}

ObstacleCore::ObstacleCore(ObstacleConfig config, std::function<double()> now)
    : config_(std::move(config)),
      now_(std::move(now)),
      ground_(mowbite_obstacles::FlatGround(config_.wheel_radius)),
      sight_(config_.sight) {
  config_.rate = std::clamp(config_.rate, 1.0, 10.0);
}

ObstacleCore::~ObstacleCore() {
  StopWorker();
}

ObstacleKind ObstacleCore::KindOf(const std::string& class_name) const {
  if (Contains(config_.ignore, class_name)) return ObstacleKind::kIgnore;
  if (Contains(config_.person, class_name)) return ObstacleKind::kPerson;
  if (Contains(config_.animal, class_name)) return ObstacleKind::kAnimal;
  return ObstacleKind::kObject;
}

bool ObstacleCore::Load() {
  StopWorker();
  warnings_.clear();
  not_set_up_ = config_.not_set_up;
  if (!not_set_up_.empty()) return false;
  std::string error;
  if (!ReadStereoCalibration(config_.calibration_path, calibration_, error)) {
    not_set_up_ = "calibration: " + error;
    return false;
  }
  if (calibration_.width != config_.eye_width || calibration_.height != config_.eye_height) {
    not_set_up_ = "calibration is for " + std::to_string(calibration_.width) + "x" +
                  std::to_string(calibration_.height) + ", the eyes are " + std::to_string(config_.eye_width) + "x" +
                  std::to_string(config_.eye_height);
    return false;
  }
  if (!config_.have_mount) {
    not_set_up_ = "no mount";
    return false;
  }
  const FisheyeLens& lens = calibration_.left;
  Point2 left;
  Point2 right;
  Point2 bottom;
  if (!ToRay(lens, {0.0, lens.cy}, left) || !ToRay(lens, {config_.eye_width - 1.0, lens.cy}, right) ||
      !ToRay(lens, {lens.cx, config_.eye_height - 1.0}, bottom) ||
      !mowbite_obstacles::ViewOf(config_.mount, ground_, {left.x, left.y}, {right.x, right.y}, {bottom.x, bottom.y},
                                 config_.range_max, &view_)) {
    not_set_up_ = "with this mount the camera does not see the ground ahead";
    return false;
  }
  for (const std::string* path : {&config_.library_path, &config_.model_path, &config_.labels_path}) {
    if (!Exists(*path)) {
      not_set_up_ = path->empty() ? "model path not set" : "model: " + *path + " missing";
      return false;
    }
  }
  if (!ReadClassList(config_.labels_path, names_, error)) {
    not_set_up_ = "model: " + error;
    return false;
  }
  auto worker = std::make_shared<InferenceWorker>();
  NpuModel& model = worker->Model();
  if (!model.Open(config_.library_path, config_.model_path, config_.npu_core, error)) {
    not_set_up_ = "model: " + error;
    return false;
  }
  if (!CheckModel(model.Inputs(), model.Outputs(), names_.size(), layout_, error)) {
    model.Close();
    not_set_up_ = "model: " + error;
    return false;
  }
  for (const auto* list : {&config_.person, &config_.animal, &config_.ignore}) {
    for (const std::string& name : *list) {
      if (!Contains(names_, name)) warnings_.push_back("class " + name + " is not in " + config_.labels_path);
    }
  }
  for (const auto& [name, height] : config_.known_height) {
    if (!Contains(names_, name)) warnings_.push_back("class " + name + " is not in " + config_.labels_path);
  }
  place_ = PlaceFrame(config_.eye_width, config_.eye_height, Fit::kPad, kModelWidth, kModelHeight);
  worker->Start(layout_, config_.decode);
  worker_ = std::move(worker);
  return true;
}

void ObstacleCore::StopWorker() {
  if (!worker_) return;
  // A run that hangs keeps its thread; the process ends anyway.
  worker_->Stop(config_.slow_run_s);
  worker_.reset();
  busy_ = false;
}

void ObstacleCore::OnFrame(const CapturedFrame& frame, double now, const FrameContext& context) {
  sight_.Frame(now);
  if (!capturing_) {
    capturing_ = true;
    epoch_++;
  }
  if (!worker_ || busy_ || now < next_submit_) return;
  const size_t lines = config_.eye_height;
  if (frame.data == nullptr || frame.stride < 4 * config_.eye_width || frame.bytes < frame.stride * lines) return;
  job_.frame.assign(frame.data, frame.data + frame.stride * lines);
  job_.line_bytes = frame.stride;
  job_.taken = static_cast<double>(frame.timestamp_us) / 1e6;
  job_.submitted = now;
  job_.epoch = epoch_;
  const StereoPair pair = PairOf(job_.frame.data(), config_.eye_width, config_.eye_height, job_.line_bytes);
  SightSample sample = MeasureSight(pair, config_.sight.bright_value);
  sample.taken = now;
  sample.at_max_gain = context.at_max_gain;
  sample.matched = context.moving ? MatchedShare(calibration_, pair, config_.match) : -1.0;
  sight_.Add(sample);
  ConvertYuyv(job_.frame.data(), config_.eye_width, config_.eye_height, job_.line_bytes, place_, config_.swap_rb,
              kModelWidth, kModelHeight, worker_->Input());
  worker_->Submit();
  busy_ = true;
  next_submit_ = std::max(next_submit_ + 1.0 / config_.rate, now);
  if (last_submit_ >= 0.0 && now > last_submit_) {
    const double rate = 1.0 / (now - last_submit_);
    status_.rate = status_.rate == 0.0 ? rate : status_.rate + 0.3 * (rate - status_.rate);
  }
  last_submit_ = now;
}

void ObstacleCore::OnStop() {
  capturing_ = false;
  sight_.Reset();
  last_result_ = -1.0;
  last_submit_ = -1.0;
  status_.rate = 0.0;
}

bool ObstacleCore::CheckWorker(double now) {
  if (busy_ && now - job_.submitted > config_.hang_s) {
    status_.exit_code = kExitSlowNpu;
    status_.last = "inference has not ended after " + Fixed(config_.hang_s, "%.1f") + " s";
    return false;
  }
  return true;
}

bool ObstacleCore::Poll(double now, const StereoStatus& camera, ObstacleReport& report) {
  if (status_.exit_code != kExitNone || !CheckWorker(now)) return false;
  InferenceWorker::Result result;
  if (busy_ && worker_->Take(result)) {
    busy_ = false;
    if (!result.ok) {
      status_.exit_code = kExitSlowNpu;
      status_.last = result.error;
      return false;
    }
    status_.infer_ms = result.run_s * 1000.0;
    if (result.run_s > config_.slow_run_s && ++slow_runs_ >= config_.max_slow_runs) {
      status_.exit_code = kExitSlowNpu;
      status_.last = "inference over " + Fixed(config_.slow_run_s, "%.1f") + " s " + std::to_string(slow_runs_) +
                     " times";
      return false;
    }
    // A result of a frame from before the camera last stopped is dropped.
    if (capturing_ && camera.capturing && job_.epoch == epoch_) {
      last_result_ = now;
      std::string detail;
      if (sight_.Reason(now, detail) == SightReason::kNone) {
        report = ObstacleReport{};
        report.taken = job_.taken;
        report.view = view_;
        Analyse(result.detections, report);
        last_report_ = now;
        return true;
      }
    }
  }
  SightReason reason = SightReason::kNone;
  std::string detail;
  if (!not_set_up_.empty()) {
    reason = SightReason::kNotSetUp;
    detail = not_set_up_;
  } else if (!camera.capturing) {
    reason = CameraReason(camera.state);
    detail = camera.detail;
  } else {
    reason = sight_.Reason(now, detail);
    if (reason == SightReason::kNone && (last_result_ < 0.0 || now - last_result_ > config_.late_s)) {
      reason = SightReason::kNoSignal;
      detail = last_result_ < 0.0 ? "no analysis yet" : "no analysis for " + Fixed(now - last_result_, "%.1f") + " s";
    }
  }
  // With sight the next analysis brings the report.
  if (reason == SightReason::kNone) return false;
  if (last_report_ >= 0.0 && now - last_report_ < 1.0 / config_.rate) return false;
  NoSight(now, reason, detail, report);
  last_report_ = now;
  return true;
}

void ObstacleCore::NoSight(double now, SightReason reason, const std::string& detail, ObstacleReport& report) {
  report = ObstacleReport{};
  report.taken = now;
  report.reason = reason;
  report.detail = detail;
  report.view = view_;
}

void ObstacleCore::Analyse(const std::vector<Detection>& detections, ObstacleReport& report) {
  const StereoPair pair = PairOf(job_.frame.data(), config_.eye_width, config_.eye_height, job_.line_bytes);
  std::string text;
  for (const Detection& d : detections) {
    const Detection box = ToSource(d, place_);
    const std::string name = box.class_id < names_.size() ? names_[box.class_id] : std::to_string(box.class_id);
    FoundObstacle found;
    found.kind = KindOf(name);
    if (found.kind == ObstacleKind::kIgnore) continue;
    found.class_name = name;
    found.score = box.score;
    if (!Locate(box, pair, found)) continue;
    if (report.obstacles.size() < kLastShown) {
      text += (text.empty() ? "" : ", ") + name + " " + Fixed(box.score, "%.2f") + " at " +
              Fixed(std::hypot(found.spot.x, found.spot.y), "%.1f") + " m";
    }
    report.obstacles.push_back(std::move(found));
  }
  if (report.obstacles.size() > kLastShown) {
    text += " (+" + std::to_string(report.obstacles.size() - kLastShown) + ")";
  }
  status_.last = text.empty() ? "none" : text;
  status_.analyses++;
  const double now = now_();
  if (photos_wanted_ && !report.obstacles.empty() &&
      (last_photo_ < 0.0 || now - last_photo_ >= config_.photo_interval_s)) {
    MakePhoto(report);
    if (!report.jpeg.empty()) last_photo_ = now;
  }
}

bool ObstacleCore::Locate(const Detection& d, const StereoPair& pair, FoundObstacle& found) const {
  using mowbite_obstacles::Footprint;
  using mowbite_obstacles::GroundResult;
  const FisheyeLens& lens = calibration_.left;
  found.box = {d.x1, d.y1, d.x2, d.y2};
  found.cut_bottom = d.y2 >= config_.eye_height - 1.0 - config_.box_edge_px;
  const bool cut_top = d.y1 <= config_.box_edge_px;
  const double middle = 0.5 * (d.x1 + d.x2);
  mowbite_obstacles::BoxRays rays;
  Point2 bottom;
  Point2 bottom_left;
  Point2 bottom_right;
  Point2 top;
  const bool have_rays = ToRay(lens, {middle, d.y2}, bottom) && ToRay(lens, {d.x1, d.y2}, bottom_left) &&
                         ToRay(lens, {d.x2, d.y2}, bottom_right) && ToRay(lens, {middle, d.y1}, top);
  rays.bottom = {bottom.x, bottom.y};
  rays.bottom_left = {bottom_left.x, bottom_left.y};
  rays.bottom_right = {bottom_right.x, bottom_right.y};
  rays.top = {top.x, top.y};
  rays.cut_bottom = found.cut_bottom;
  const mowbite_obstacles::Errors errors{config_.box_edge_px / lens.fx, config_.pitch_sigma, config_.height_sigma};
  Footprint ground;
  if (have_rays) ground = FootprintOf(config_.mount, ground_, rays, errors, config_.range_max);
  const bool on_ground = have_rays && ground.result == GroundResult::kOk;

  bool stereo = false;
  double depth = 0.0;
  mowbite_obstacles::Spot stereo_spot;
  const BoxDepth box_depth = DepthOfBox(calibration_, pair, found.box, config_.match, config_.box_depth);
  Point2 ray;
  if (box_depth.ok && ToRay(lens, box_depth.pixel, ray)) {
    depth = box_depth.depth;
    double sigma = depth * depth / DisparityAtOneMeter(calibration_) *
                   (config_.disparity_noise_px + std::abs(config_.disparity_bias_px));
    // Size probe: stereo on something in front of the thing (grass) gives a depth far too small.
    const auto known = config_.known_height.find(found.class_name);
    if (known != config_.known_height.end() && !found.cut_bottom && !cut_top && d.y2 > d.y1) {
      const double by_size = lens.fy * known->second / (d.y2 - d.y1);
      if (depth < config_.size_ratio * by_size) {
        sigma = std::max(config_.size_sigma_share * by_size, by_size - depth);
        depth = by_size;
      }
    }
    stereo_spot = mowbite_obstacles::StereoSpot(config_.mount, {ray.x, ray.y}, depth, sigma, errors.ray);
    stereo = true;
  }
  if (on_ground && stereo) {
    const mowbite_obstacles::Fused fused = mowbite_obstacles::Fuse(ground.spot, stereo_spot, config_.fuse_gate);
    found.spot = fused.spot;
    found.agrees = fused.agrees;
    found.from = PositionFrom::kBoth;
  } else if (on_ground) {
    found.spot = ground.spot;
    found.from = PositionFrom::kGround;
  } else if (stereo) {
    found.spot = stereo_spot;
    found.from = PositionFrom::kStereo;
  } else {
    return false;
  }
  if (on_ground) {
    found.radius = ground.radius;
    found.height = ground.height;
  } else {
    found.radius = 0.5 * (d.x2 - d.x1) * depth / lens.fx;
    found.height = found.cut_bottom || cut_top ? std::numeric_limits<double>::quiet_NaN()
                                               : (d.y2 - d.y1) * depth / lens.fy;
  }
  return true;
}

void ObstacleCore::MakePhoto(ObstacleReport& report) {
  std::string error;
  for (int quality = config_.photo_quality; quality >= kLowestQuality; quality -= kQualityStep) {
    if (!jpeg_.Encode(job_.frame.data(), config_.eye_width, config_.eye_height, job_.line_bytes, quality, report.jpeg,
                      error)) {
      break;
    }
    if (report.jpeg.size() <= config_.photo_max_bytes) {
      report.width = config_.eye_width;
      report.height = config_.eye_height;
      return;
    }
  }
  report.jpeg.clear();
}

}  // namespace tron_vision
