#include "mowbite_net/outbox.h"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

#include "mowbite_net/texts.h"

namespace mowbite_net {

namespace {

constexpr char kJson[] = "Content-Type: application/json";

bool Ok(long status) {
  return status >= 200 && status < 300;
}

// No answer, a server error or too many requests: it may go through later.
bool Transient(long status) {
  return status == 0 || status == 429 || status >= 500;
}

}  // namespace

std::optional<HttpRequest> NtfyRequest(const Push& push, const NotifyConfig& config) {
  if (config.topic.empty()) return std::nullopt;
  nlohmann::ordered_json body = {{"topic", config.topic},
                                 {"title", push.title},
                                 {"message", push.message},
                                 {"priority", push.prio},
                                 {"tags", {push.tag}}};
  if (!config.url.empty()) {
    body["click"] = config.url + push.path;
    body["actions"] = {{{"action", "view"}, {"label", Say(config.lang, "open")}, {"url", config.url + push.path}}};
  }
  HttpRequest request{config.server + "/", {kJson}, body.dump()};
  if (!config.token.empty()) request.headers.push_back("Authorization: Bearer " + config.token);
  return request;
}

HttpRequest PhoneRequest(const Push& push, const NotifyConfig& config, const std::string& address, double t) {
  const nlohmann::ordered_json body = {{"title", push.title},
                                       {"message", push.message},
                                       {"kind", push.kind},
                                       {"phase", push.phase},
                                       {"problem", push.problem},
                                       {"prio", push.prio},
                                       {"t", static_cast<int64_t>(std::floor(t))},
                                       {"lang", config.lang},
                                       {"path", push.path},
                                       {"api", config.url}};
  return {address, {kJson}, body.dump()};
}

Delivery Deliver(Http& http, const Push& push, const NotifyConfig& config, double t,
                 const std::optional<std::string>& only, const std::function<bool()>& stop) {
  Delivery delivery;
  bool transient = false;
  if (!only) {
    if (const std::optional<HttpRequest> request = NtfyRequest(push, config)) {
      delivery.targets = true;
      const HttpResult result = http.Send(*request);
      delivery.sent = delivery.sent || Ok(result.status);
      transient = transient || Transient(result.status);
    }
  }
  for (const std::string& address : config.pushes) {
    if (only && address != *only) continue;
    if (delivery.targets && stop && stop()) {
      delivery.sent = false;
      delivery.stopped = true;
      return delivery;
    }
    delivery.targets = true;
    const HttpResult result = http.Send(PhoneRequest(push, config, address, t));
    delivery.sent = delivery.sent || Ok(result.status);
    transient = transient || Transient(result.status);
    if (result.status == 404 || result.status == 410) delivery.gone.push_back(address);
  }
  delivery.again = !delivery.sent && transient;
  return delivery;
}

void Outbox::Add(const Push& push, const NotifyConfig& config, double t, double mono) {
  if (items_.size() >= kOutboxSize) {
    items_.pop_front();
    ++dropped_;
  }
  items_.push_back({push, config, t, 0, mono});
}

bool Outbox::Due(double mono) const {
  return std::any_of(items_.begin(), items_.end(), [mono](const Item& item) { return item.next <= mono; });
}

std::optional<double> Outbox::NextDue() const {
  if (items_.empty()) return std::nullopt;
  return std::min_element(items_.begin(), items_.end(), [](const Item& a, const Item& b) { return a.next < b.next; })
      ->next;
}

std::optional<Outbox::Item> Outbox::Take(double mono) {
  for (auto it = items_.begin(); it != items_.end(); ++it) {
    if (it->next > mono) continue;
    Item item = *it;
    items_.erase(it);
    return item;
  }
  return std::nullopt;
}

void Outbox::Again(Item item, double mono) {
  ++item.tries;
  if (item.tries >= kTries) {
    ++dropped_;
    return;
  }
  item.next = mono + kRetryAfter * item.tries;
  if (items_.size() >= kOutboxSize) {
    ++dropped_;
    return;
  }
  items_.push_back(std::move(item));
}

}  // namespace mowbite_net
