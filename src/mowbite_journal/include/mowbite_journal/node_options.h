#ifndef MOWBITE_JOURNAL_NODE_OPTIONS_H
#define MOWBITE_JOURNAL_NODE_OPTIONS_H

#include <sys/stat.h>

#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "mowbite_journal/journal.h"

namespace mowbite_journal {

// For nodes; header only, so that the library needs no ROS.

// The journal of the process from the parameters of a node (journal_dir, else MOWBITE_JOURNAL_DIR, and the limits of
// the platform config under /**), opened by the first node that asks; a problem goes to the node's log. None while
// journal_dir holds kDisabledFile.
template <typename Node>
Journal* OpenFromParameters(Node& node) {
  Options options;
  const char* env = std::getenv("MOWBITE_JOURNAL_DIR");
  options.dir = node.declare_parameter("journal_dir", std::string(env ? env : ""));
  options.platform = node.declare_parameter("journal_platform", std::string());
  options.max_mb = node.declare_parameter("journal_max_mb", options.max_mb);
  options.max_days = node.declare_parameter("journal_max_days", options.max_days);
  options.reserve_mb = node.declare_parameter("journal_reserve_mb", options.reserve_mb);
  options.segment_s = node.declare_parameter("journal_segment_s", options.segment_s);
  options.buffer_kb = static_cast<size_t>(
      node.declare_parameter("journal_buffer_kb", static_cast<int64_t>(options.buffer_kb)));
  if (options.dir.empty()) return nullptr;
  struct stat st {};
  if (stat((options.dir + "/" + kDisabledFile).c_str(), &st) == 0) {
    RCLCPP_INFO(node.get_logger(), "No journal: journal_enabled is 0");
    return nullptr;
  }
  std::string error;
  Journal* journal = Journal::Open(options, &error);
  if (!journal) {
    RCLCPP_ERROR(node.get_logger(), "No journal: %s", error.c_str());
  } else if (journal->options().dir != options.dir) {
    RCLCPP_WARN(node.get_logger(), "The journal of this process is in %s, not %s", journal->options().dir.c_str(),
                options.dir.c_str());
  }
  return journal;
}

// Logs when the writer stops or starts again and when records were dropped; Check from a timer of the node.
class JournalWatch {
 public:
  explicit JournalWatch(Journal* journal) : journal_(journal) {}
  void Check(const rclcpp::Logger& logger) {
    if (!journal_) return;
    const Stats stats = journal_->GetStats();
    if (stats.problem != problem_) {
      if (stats.problem.empty()) {
        RCLCPP_INFO(logger, "Journal writes again to %s", stats.run_dir.c_str());
      } else {
        RCLCPP_ERROR(logger, "Journal stopped: %s", stats.problem.c_str());
      }
      problem_ = stats.problem;
    }
    if (stats.dropped > dropped_) {
      RCLCPP_WARN(logger, "Journal dropped %llu records (%llu in all): the replay stops at the gap",
                  static_cast<unsigned long long>(stats.dropped - dropped_),
                  static_cast<unsigned long long>(stats.dropped));
      dropped_ = stats.dropped;
    }
  }

 private:
  Journal* journal_;
  std::string problem_;
  uint64_t dropped_ = 0;
};

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_NODE_OPTIONS_H
