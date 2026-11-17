// The release of the emergency by emergency.reset or STOP: its request, the answer of the safety node and its outcome.
#include <string>
#include <utility>

#include "mowbite_logic/release_part.h"

namespace mowbite_logic {

std::string ReleasePart::RefuseReset(CoreAccess core, const Reason& why) {
  json details = json::object();
  PutReason(&details, "EMERGENCY_RESET_REFUSED", why);
  if (release_pending_) details["source"] = release_source_;
  // The refusal carries the client of the request it answers.
  const std::string client = core.client();
  if (release_pending_) core.client() = release_client_;
  release_pending_ = false;
  release_answered_s_.reset();
  const std::string id = core.AddEvent("EMERGENCY_RESET_REFUSED", std::move(details));
  core.client() = client;
  release_outcome_ = {release_outcome_.seq + 1, false, why, id};
  return id;
}

void ReleasePart::OnReleaseAnswered(double now_s) {
  if (release_pending_) release_answered_s_ = now_s;
}

bool ReleasePart::OnStopRelease(CoreFacts core) {
  if (!core.emergency()) return false;
  release_pending_ = true;
  release_source_ = "button";
  release_client_.clear();
  release_answered_s_.reset();
  return true;
}

void ReleasePart::OnResetRequested(const std::string& client) {
  release_pending_ = true;
  release_source_ = "rpc";
  release_client_ = client;
  release_answered_s_.reset();
}

void ReleasePart::OnEmergencyChanged(bool emergency, json* details) {
  // The end after emergency.reset or STOP tells who released it (masterplan 3.1a); a new emergency drops the request.
  if (!emergency && release_pending_) {
    (*details)["source"] = release_source_;
    if (!release_client_.empty()) (*details)["client"] = release_client_;
    release_outcome_ = {release_outcome_.seq + 1, true, Reason(), ""};
  }
  release_pending_ = false;
  release_answered_s_.reset();
}

}  // namespace mowbite_logic
