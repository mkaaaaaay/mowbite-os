#ifndef MOWBITE_BRIDGE_TRACK_H
#define MOWBITE_BRIDGE_TRACK_H

#include <cstdint>
#include <string>
#include <vector>

#include "mowbite_bridge/app_json.h"

namespace mowbite_bridge {

// The mowing track of position.history: the points of each job while a session runs, as runs with the same job,
// session and blade state. Kept on disk as <dir>/<epoch>_<job_id>.jsonl, one line per new run ("new_segment") or per
// save of points to the last one ("append_points").
class MowTrack {
 public:
  // Files older than max_days and the oldest over max_bytes go, never the one of the current job.
  MowTrack(std::string dir, double max_days, std::uintmax_t max_bytes);

  // At start: the newest file is the current job, without a session until the next STATE event.
  void Open(double now);
  // STATE (job_id, session_id) and BLADES (enabled); other events change nothing.
  void OnEvent(const Json& event, double now);
  // One position; kept only while a session runs.
  void Add(double x, double y);
  // Appends what is new to the file of the job (every 30 s).
  void Save();
  // Takes the open points into the run and saves (at shutdown).
  void Close();

  PoseAttributes Attributes() const;

  // position.history: {"job_id"} or none for the current job; {segments, buffer}, {"error": "not found"} if unknown.
  Json History(const Json& params) const;
  // position.history.list: [{job_id, timestamp}], newest first.
  Json List() const;
  // position.history.delete: {"job_id"} or none for all; never the current job. {"deleted": n}.
  Json Erase(const Json& params);

 private:
  struct Point {
    double x;
    double y;
  };
  struct Run {
    PoseAttributes attributes;
    double started_at = 0.0;
    std::vector<Point> points;
    // Points already in the file; a run not in the file yet gets its "new_segment" line at the next save.
    std::size_t saved = 0;
    bool in_file = false;
  };
  struct JobFile {
    std::string path;
    std::string job_id;
    long long epoch;
    std::uintmax_t bytes;
  };

  Run NewRun(double now) const;
  bool Recording() const;
  void EndRun();
  void Keep(const Point& point);
  void Trim(double now);
  std::vector<JobFile> Scan() const;
  static std::vector<Run> Load(const std::string& path);
  static Json Render(const std::vector<Run>& runs, const std::vector<Point>& open);

  std::string dir_;
  double max_days_;
  std::uintmax_t max_bytes_;
  std::string job_id_;
  std::string session_id_;
  bool blades_ = false;
  std::string file_;
  std::vector<Run> runs_;
  // Points after the last kept one, not simplified yet.
  std::vector<Point> open_;
  std::size_t kept_ = 0;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_TRACK_H
