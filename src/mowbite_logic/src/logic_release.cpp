// The release of the emergency by emergency.reset or STOP: its request, the answer of the safety node and its outcome.
#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

std::string LogicCore::RefuseReset(const Reason& why) {
  json details = json::object();
  PutReason(&details, "EMERGENCY_RESET_REFUSED", why);
  if (release_pending_) details["source"] = release_source_;
  // The refusal carries the client of the request it answers.
  const std::string client = client_;
  if (release_pending_) client_ = release_client_;
  release_pending_ = false;
  release_answered_s_.reset();
  const std::string id = AddEvent("EMERGENCY_RESET_REFUSED", std::move(details));
  client_ = client;
  release_outcome_ = {release_outcome_.seq + 1, false, why, id};
  return id;
}

void LogicCore::OnReleaseAnswered(double now_s) {
  if (release_pending_) release_answered_s_ = now_s;
}

bool LogicCore::OnStopRelease() {
  if (!emergency_) return false;
  release_pending_ = true;
  release_source_ = "button";
  release_client_.clear();
  release_answered_s_.reset();
  return true;
}

}  // namespace mowbite_logic
