#include "mowbite_net/notify_config.h"

#include <algorithm>
#include <cctype>
#include <regex>
#include <set>

#include "mowbite_api/catalog.h"

namespace mowbite_net {

namespace {

constexpr size_t kMaxToken = 200;
constexpr size_t kMaxAddress = 1000;

const std::regex& WebAddress() {
  static const std::regex re("^https?://[A-Za-z0-9.:/_-]{1,200}$");
  return re;
}

Rejection Bad(std::string field, std::string reason, std::string message) {
  return {std::move(field), std::move(reason), std::move(message)};
}

// Characters, counted as UTF-8 code points.
size_t Length(const std::string& text) {
  return static_cast<size_t>(std::count_if(text.begin(), text.end(), [](char c) { return (c & 0xC0) != 0x80; }));
}

bool Printable(const std::string& text) {
  return std::none_of(text.begin(), text.end(),
                      [](char c) { return static_cast<unsigned char>(c) < 0x20 || c == 0x7f; });
}

std::string StripSlashes(std::string text) {
  while (!text.empty() && text.back() == '/') text.pop_back();
  return text;
}

// Scheme, host and port of a server that passed WebAddress, the port filled in.
std::string Origin(const std::string& server) {
  const size_t start = server.find("://") + 3;
  const std::string scheme = server.substr(0, start - 3);
  std::string authority = server.substr(start, server.find('/', start) - start);
  std::transform(authority.begin(), authority.end(), authority.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const size_t colon = authority.find(':');
  if (colon == std::string::npos || colon + 1 == authority.size()) {
    authority = authority.substr(0, colon) + (scheme == "https" ? ":443" : ":80");
  }
  return scheme + "://" + authority;
}

std::optional<Rejection> CheckKeys(const json& params, const std::set<std::string>& keys, bool none_ok) {
  if (params.is_null() && none_ok) return std::nullopt;
  if (!params.is_object()) return Bad("params", "type", "an object");
  for (const auto& item : params.items()) {
    if (keys.count(item.key()) == 0) return Bad(item.key(), "unknown", "not a key of this method");
  }
  return std::nullopt;
}

std::optional<Rejection> CheckWhole(const json& params, const char* key, int64_t min, int64_t max, int64_t* out) {
  const json& value = params[key];
  if (!value.is_number_integer()) return Bad(key, "type", "a whole number");
  const int64_t n = value.get<int64_t>();
  if (n < min || n > max) {
    return Bad(key, "range", "from " + std::to_string(min) + " to " + std::to_string(max));
  }
  *out = n;
  return std::nullopt;
}

std::optional<Rejection> Address(const json& params, std::string* address) {
  if (!params.contains("address")) return Bad("address", "missing", "a UnifiedPush address");
  if (!params["address"].is_string()) return Bad("address", "type", "a UnifiedPush address");
  const std::string text = params["address"].get<std::string>();
  static const std::regex re("^https?://[^\\s\"\\\\]+$");
  if (text.size() > kMaxAddress || !std::regex_match(text, re)) return Bad("address", "format", "an http(s) URL");
  *address = text;
  return std::nullopt;
}

std::vector<std::string> Strings(const json& value) {
  std::vector<std::string> out;
  if (!value.is_array()) return out;
  for (const json& item : value) {
    if (item.is_string()) out.push_back(item.get<std::string>());
  }
  return out;
}

}  // namespace

bool NotifyConfig::operator==(const NotifyConfig& o) const {
  return enabled == o.enabled && server == o.server && topic == o.topic && token == o.token && lang == o.lang &&
         name == o.name && url == o.url && events == o.events && remind == o.remind &&
         emergency_wait == o.emergency_wait && pushes == o.pushes && stored == o.stored;
}

std::vector<std::string> DefaultEvents() {
  std::vector<std::string> events;
  for (const auto& event : mowbite_api::NotifyEvents()) {
    if (event.picked) events.push_back(event.key);
  }
  return events;
}

NotifyConfig DefaultConfig() {
  NotifyConfig config;
  config.events = DefaultEvents();
  return config;
}

NotifyConfig ConfigFromJson(const json& file, std::string* error) {
  NotifyConfig config = DefaultConfig();
  if (file.is_null()) return config;
  if (!file.is_object() || !file.contains("schema") || file["schema"] != 1) {
    *error = "not a notify file of schema 1";
    return config;
  }
  // Read back through the checks of notify.set, so a hand-edited file cannot hold what notify.set refuses.
  json params = file;
  params.erase("schema");
  params.erase("stored");
  const json pushes = params.contains("pushes") ? params["pushes"] : json::array();
  params.erase("pushes");
  if (const std::optional<Rejection> bad = ApplySet(params, &config)) {
    *error = bad->field + ": " + bad->message;
    return DefaultConfig();
  }
  for (const std::string& address : Strings(pushes)) {
    if (AddPush({{"address", address}}, &config)) *error = "a push address is not an http(s) URL";
  }
  config.stored = file.contains("stored") && file["stored"].is_boolean() && file["stored"].get<bool>();
  return config;
}

json ConfigToJson(const NotifyConfig& c) {
  return {{"schema", 1},       {"enabled", c.enabled}, {"server", c.server}, {"topic", c.topic},
          {"token", c.token},  {"lang", c.lang},       {"name", c.name},     {"url", c.url},
          {"events", c.events}, {"remind", c.remind},  {"emergency_wait", c.emergency_wait},
          {"pushes", c.pushes}, {"stored", c.stored}};
}

json ConfigAnswer(const NotifyConfig& c, int64_t snooze) {
  return {{"schema", 1},
          {"enabled", c.enabled},
          {"server", c.server},
          {"topic", c.topic},
          {"lang", c.lang},
          {"name", c.name},
          {"url", c.url},
          {"events", c.events},
          {"remind", c.remind},
          {"emergency_wait", c.emergency_wait},
          {"has_token", !c.token.empty()},
          {"pushes", c.pushes},
          {"snooze", snooze},
          {"stored", c.stored}};
}

std::optional<Rejection> ApplySet(const json& params, NotifyConfig* config) {
  if (auto bad = CheckKeys(params, {"enabled", "server", "topic", "token", "lang", "name", "url", "events", "remind",
                                    "emergency_wait"},
                           false)) {
    return bad;
  }
  NotifyConfig next = *config;
  for (const auto& item : params.items()) {
    const std::string& key = item.key();
    const json& value = item.value();
    if (key == "enabled") {
      if (!value.is_boolean()) return Bad(key, "type", "true or false");
      next.enabled = value.get<bool>();
    } else if (key == "remind" || key == "emergency_wait") {
      int64_t n = 0;
      const bool remind = key == "remind";
      const int64_t max = remind ? mowbite_api::kNotifyMaxRemind : mowbite_api::kNotifyMaxEmergencyWait;
      if (auto bad = CheckWhole(params, key.c_str(), 0, max, &n)) return bad;
      (remind ? next.remind : next.emergency_wait) = static_cast<int>(n);
    } else if (key == "events") {
      if (!value.is_array()) return Bad(key, "type", "a list of event keys");
      const auto& known = mowbite_api::NotifyEvents();
      std::vector<std::string> events;
      for (const json& event : value) {
        if (!event.is_string()) return Bad(key, "type", "a list of event keys");
        const std::string name = event.get<std::string>();
        if (std::none_of(known.begin(), known.end(), [&](const auto& e) { return e.key == name; })) {
          return Bad(key, "unknown", "no event " + name);
        }
        if (std::find(events.begin(), events.end(), name) != events.end()) {
          return Bad(key, "duplicate", name + " twice");
        }
        events.push_back(name);
      }
      next.events = events;
    } else {
      if (!value.is_string()) return Bad(key, "type", "a text");
      const std::string text = value.get<std::string>();
      if (!Printable(text)) return Bad(key, "format", "no control characters");
      if (key == "server") {
        const std::string server = StripSlashes(text);
        if (!std::regex_match(server, WebAddress())) return Bad(key, "format", "http(s)://host[:port][/path]");
        next.server = server;
      } else if (key == "url") {
        const std::string url = StripSlashes(text);
        if (!url.empty() && !std::regex_match(url, WebAddress())) return Bad(key, "format", "an http(s) address");
        next.url = url;
      } else if (key == "topic") {
        static const std::regex re(mowbite_api::kNotifyTopicPattern);
        if (!std::regex_match(text, re)) return Bad(key, "format", "letters, digits, - and _, at most 64");
        next.topic = text;
      } else if (key == "token") {
        if (text.size() > kMaxToken || text.find(' ') != std::string::npos) {
          return Bad(key, "format", "at most 200 characters, no space");
        }
        next.token = text;
      } else if (key == "lang") {
        if (text != "de" && text != "en") return Bad(key, "range", "de or en");
        next.lang = text;
      } else {
        if (Length(text) > mowbite_api::kNotifyMaxName) {
          return Bad(key, "range", "at most " + std::to_string(mowbite_api::kNotifyMaxName) + " characters");
        }
        next.name = text;
      }
    }
  }
  // A token only goes to the server it was given for.
  if (!params.contains("token") && Origin(next.server) != Origin(config->server)) next.token.clear();
  next.stored = true;
  *config = next;
  return std::nullopt;
}

std::optional<Rejection> AddPush(const json& params, NotifyConfig* config) {
  if (auto bad = CheckKeys(params, {"address"}, false)) return bad;
  std::string address;
  if (auto bad = Address(params, &address)) return bad;
  auto& pushes = config->pushes;
  if (std::find(pushes.begin(), pushes.end(), address) != pushes.end()) return std::nullopt;
  if (pushes.size() >= static_cast<size_t>(mowbite_api::kNotifyMaxPushes)) pushes.erase(pushes.begin());
  pushes.push_back(address);
  return std::nullopt;
}

std::optional<Rejection> RemovePush(const json& params, NotifyConfig* config) {
  if (auto bad = CheckKeys(params, {"address"}, false)) return bad;
  if (!params.contains("address")) return Bad("address", "missing", "an address of pushes");
  if (!params["address"].is_string()) return Bad("address", "type", "an address of pushes");
  auto& pushes = config->pushes;
  pushes.erase(std::remove(pushes.begin(), pushes.end(), params["address"].get<std::string>()), pushes.end());
  return std::nullopt;
}

std::optional<Rejection> CheckSnooze(const json& params, int64_t* seconds) {
  if (auto bad = CheckKeys(params, {"seconds"}, false)) return bad;
  if (!params.contains("seconds")) return Bad("seconds", "missing", "seconds from 0 to 86400");
  return CheckWhole(params, "seconds", 0, mowbite_api::kNotifyMaxSnooze, seconds);
}

std::optional<Rejection> CheckTest(const json& params, const NotifyConfig& config,
                                   std::optional<std::string>* address) {
  if (auto bad = CheckKeys(params, {"address"}, true)) return bad;
  if (params.is_null() || !params.contains("address")) return std::nullopt;
  if (!params["address"].is_string()) return Bad("address", "type", "an address of pushes");
  const std::string text = params["address"].get<std::string>();
  if (std::find(config.pushes.begin(), config.pushes.end(), text) == config.pushes.end()) {
    return Bad("address", "unknown", "not an address of pushes");
  }
  *address = text;
  return std::nullopt;
}

std::optional<Rejection> CheckLog(const json& params, size_t* limit) {
  *limit = mowbite_api::kNotifyMaxLog;
  if (auto bad = CheckKeys(params, {"limit"}, true)) return bad;
  if (params.is_null() || !params.contains("limit")) return std::nullopt;
  int64_t n = 0;
  if (auto bad = CheckWhole(params, "limit", 1, mowbite_api::kNotifyMaxLog, &n)) return bad;
  *limit = static_cast<size_t>(n);
  return std::nullopt;
}

}  // namespace mowbite_net
