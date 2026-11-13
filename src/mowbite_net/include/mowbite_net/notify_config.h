#ifndef MOWBITE_NET_NOTIFY_CONFIG_H
#define MOWBITE_NET_NOTIFY_CONFIG_H

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_net {

using json = nlohmann::ordered_json;

constexpr char kDefaultServer[] = "https://ntfy.sh";
constexpr size_t kMaxPushes = 10;

// notify.json: where the pushes go and which. The token never leaves the file but towards the ntfy server.
struct NotifyConfig {
  bool enabled = true;
  std::string server = kDefaultServer;
  std::string topic;
  std::string token;
  std::string lang = "en";
  std::string name;
  std::string url;
  std::vector<std::string> events;
  int remind = 30;           // min, 0 none
  int emergency_wait = 10;   // s
  std::vector<std::string> pushes;
  bool stored = false;       // saved by notify.set once
  bool operator==(const NotifyConfig& other) const;
  bool operator!=(const NotifyConfig& other) const { return !(*this == other); }
};

// Problems and maintenance until notify.set picks others.
std::vector<std::string> DefaultEvents();
NotifyConfig DefaultConfig();

// From notify.json; the default for a missing or broken file (error says why it is broken).
NotifyConfig ConfigFromJson(const json& file, std::string* error);
// The text of notify.json, token included.
json ConfigToJson(const NotifyConfig& config);

struct Rejection {
  std::string field;
  std::string reason;  // type, range, format, unknown, missing, duplicate
  std::string message;
};

// The answer of notify.get: everything but the token, has_token instead; snooze in unix s.
json ConfigAnswer(const NotifyConfig& config, int64_t snooze);
// notify.set: the keys of params on top of config, all or nothing. token: a text is the new one, empty drops it.
std::optional<Rejection> ApplySet(const json& params, NotifyConfig* config);
// notify.push_add and notify.push_remove; the address of params checked.
std::optional<Rejection> AddPush(const json& params, NotifyConfig* config);
std::optional<Rejection> RemovePush(const json& params, NotifyConfig* config);
// notify.snooze: seconds from 0 to 86400.
std::optional<Rejection> CheckSnooze(const json& params, int64_t* seconds);
// notify.test: an address of pushes, or none for all.
std::optional<Rejection> CheckTest(const json& params, const NotifyConfig& config, std::optional<std::string>* address);
// notify.log: limit 1 to 50, 50 without.
std::optional<Rejection> CheckLog(const json& params, size_t* limit);

}  // namespace mowbite_net

#endif  // MOWBITE_NET_NOTIFY_CONFIG_H
