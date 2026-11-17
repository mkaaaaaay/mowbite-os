#ifndef MOWBITE_NET_OUTBOX_H
#define MOWBITE_NET_OUTBOX_H

#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_net/http.h"
#include "mowbite_net/notifier.h"
#include "mowbite_net/notify_config.h"

namespace mowbite_net {

constexpr size_t kOutboxSize = 20;
constexpr int kTries = 3;
// Wait before the next try of a push that reached no target (s), times the tries so far.
constexpr double kRetryAfter = 30.0;

// The ntfy request of a push: JSON to the server with the token as Bearer; none without a topic.
std::optional<HttpRequest> NtfyRequest(const Push& push, const NotifyConfig& config);
// The request to one phone: the JSON MowBite's app reads; t in unix s.
HttpRequest PhoneRequest(const Push& push, const NotifyConfig& config, const std::string& address, double t);

struct Delivery {
  bool sent = false;                // at least one target took it
  bool targets = false;             // there was a target at all
  bool again = false;               // worth another try: no target took it, one did not answer or was busy
  bool stopped = false;             // the stop came between two targets: not sent, not tried again
  std::vector<std::string> gone;    // phone addresses that answered 404 or 410
};

// To the topic and every phone of config, or to only (one phone, as notify.test may ask); stop is asked before each
// target after the first.
Delivery Deliver(Http& http, const Push& push, const NotifyConfig& config, double t,
                 const std::optional<std::string>& only = std::nullopt, const std::function<bool()>& stop = {});

// Pushes waiting to go out, each with the settings of when it came; the oldest goes when it is full.
class Outbox {
 public:
  struct Item {
    Push push;
    NotifyConfig config;
    double t = 0;     // wall when it came
    int tries = 0;
    double next = 0;  // mono of the next try
  };
  void Add(const Push& push, const NotifyConfig& config, double t, double mono);
  // Whether an item is due at mono.
  bool Due(double mono) const;
  // The mono of the next try, none while empty.
  std::optional<double> NextDue() const;
  // The first item due at mono, taken out.
  std::optional<Item> Take(double mono);
  // After a try that wants another: back in, unless it had kTries.
  void Again(Item item, double mono);
  bool Empty() const { return items_.empty(); }
  // Pushes given up: no room or kTries without success.
  uint64_t dropped() const { return dropped_; }

 private:
  std::deque<Item> items_;
  uint64_t dropped_ = 0;
};

}  // namespace mowbite_net

#endif  // MOWBITE_NET_OUTBOX_H
