#ifndef MOWBITE_NET_NOTIFIER_H
#define MOWBITE_NET_NOTIFIER_H

#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_net/texts.h"

namespace mowbite_net {

using json = nlohmann::ordered_json;

// Ten minutes without getting 0.5 m further, not charging.
constexpr double kStandstillAfter = 600.0;
// A pause for an obstacle is told sooner: after this for an object, after kObstacleLivingAfter for a person or animal.
constexpr double kObstacleObjectAfter = 60.0;
constexpr double kObstacleLivingAfter = 300.0;
constexpr double kStandstillMove = 0.5;
// A failed spin-up of the blade this long before an emergency goes into its text.
constexpr double kSpinupBefore = 15.0;

// One push, before it has a target.
struct Push {
  std::string kind;   // an event key of NotifyEvents, maintenance_<part>, schedule_changed, notify_changed or test
  std::string phase;  // new, again or clear for a problem; empty for one that comes once
  bool problem = false;
  int prio = 3;       // ntfy 1 to 5
  std::string tag;    // ntfy tag
  std::string title;
  std::string message;
  std::string path = "/activity/";  // where a tap opens MowBite
};

// What the notifier reads of the settings.
struct NotifyRules {
  std::string lang = "en";
  std::string name;
  std::set<std::string> events;
  double remind_s = 0;      // 0: no reminders
  double emergency_wait_s = 10;
  double snooze_until = 0;  // wall, unix s
};

// robot_state once a second.
struct StateView {
  double wall = 0;
  double mono = 0;
  std::string state;
  bool emergency = false;
  bool charging = false;
  bool pose = false;
  double x = 0;
  double y = 0;
  // pause_reason and pause_kind of robot_state.
  std::string pause_reason;
  std::string pause_kind;
};

struct PartView {
  std::string key;
  double used_h = 0;
  double due_h = 0;
  bool due = false;
};

// Turns events/json, robot_state, stats/parts and schedule/notice into pushes as the container of MowBite sends them:
// one problem open at a time with new, again and clear, the rest once. No clock and no network: Take collects.
class Notifier {
 public:
  void SetRules(const NotifyRules& rules);
  // An event of events/json at mono.
  void OnEvent(const json& event, double mono);
  void OnState(const StateView& state);
  void OnParts(const std::vector<PartView>& parts);
  // schedule/notice: changed, or a logged word with the time of the plan.
  void OnScheduleChanged();
  void OnScheduleLogged(const std::string& what, const std::string& at);
  // The push notify.set sends to the targets of before.
  void OnNotifyChanged();
  std::vector<Push> Take();

  // The key of the open problem, empty for none.
  std::string open_problem() const { return open_ ? open_->key : ""; }
  // The parts told as due, at their due hours; kept over a restart so it is not told again.
  const std::map<std::string, double>& maintenance_told() const { return maintenance_told_; }
  void RestoreMaintenance(std::map<std::string, double> told) { maintenance_told_ = std::move(told); }

 private:
  struct Open {
    std::string key;      // emergency, dock_failed, undock_failed, standstill
    std::string variant;  // which text: not_charging, charging_lost, cant_go_home, dock_failed, no_gps, undock_failed
    double since = 0;     // mono
    double told = 0;      // mono of the last new or again
    bool reminds = false;
  };

  std::string Name() const;
  // {n}: the name of the mower.
  Vars N() const { return {{"n", Name()}}; }
  bool Wanted(const std::string& key) const { return rules_.events.count(key) > 0; }
  void Emit(const std::string& kind, const std::string& phase, int prio, const std::string& tag,
            const std::string& title, const std::string& message, const std::string& path = "/activity/");
  void Once(const std::string& key, const std::string& kind, const std::string& tag, const std::string& title,
            const std::string& message);
  // Opens a problem when none is open, or replaces standstill, or any when key is emergency; since (mono) is when
  // it began, now when it is told.
  bool OpenProblem(const std::string& key, const std::string& variant, double since, double now, bool reminds,
                   const std::string& title, const std::string& message, int prio, const std::string& tag);
  void Clear(const std::string& title, const std::string& message);
  void Close() { open_.reset(); }
  void OnStateEvent(const json& event);
  void Remind(const StateView& state);
  void Standstill(const StateView& state);
  // How long the mower stands before it is told: sooner in a pause for an obstacle.
  static double StandstillAfter(const StateView& state);

  NotifyRules rules_;
  std::optional<Open> open_;
  std::vector<Push> out_;
  std::optional<StateView> last_;
  std::optional<double> emergency_since_;
  bool emergency_told_ = false;
  std::optional<double> spinup_at_;
  std::string paused_reason_;  // of the last STATE PAUSED
  // Standstill: where and since when the mower stands.
  std::optional<double> still_since_;
  bool still_told_ = false;
  bool still_pose_ = false;
  double still_x_ = 0;
  double still_y_ = 0;
  // Maintenance told per part, at these due hours.
  std::map<std::string, double> maintenance_told_;
};

}  // namespace mowbite_net

#endif  // MOWBITE_NET_NOTIFIER_H
