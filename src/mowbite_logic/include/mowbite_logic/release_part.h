#ifndef MOWBITE_LOGIC_RELEASE_PART_H
#define MOWBITE_LOGIC_RELEASE_PART_H

#include <optional>
#include <string>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// The release of the emergency by emergency.reset or STOP: the request, the answer of the safety node and the outcome.
// emergency.reset or STOP asked the safety node: the end of the emergency carries the source (rpc with its client,
// button), until a refusal, a new emergency or kReleaseSourceS after the answer; the latest request names it.
class ReleasePart {
 public:
  std::string RefuseReset(CoreAccess core, const Reason& why);
  void OnReleaseAnswered(double now_s);
  bool OnStopRelease(CoreFacts core);
  // emergency.reset of the client of the command.
  void OnResetRequested(const std::string& client);
  // The emergency started or ended (UpdateEmergency): its end names who released it in details.
  void OnEmergencyChanged(bool emergency, json* details);
  const std::optional<double>& release_answered_s() const {
    return release_answered_s_;
  }
  const ReleaseOutcome& release_outcome() const {
    return release_outcome_;
  }

 private:
  bool release_pending_ = false;
  std::string release_source_;
  std::string release_client_;
  std::optional<double> release_answered_s_;
  ReleaseOutcome release_outcome_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_RELEASE_PART_H
