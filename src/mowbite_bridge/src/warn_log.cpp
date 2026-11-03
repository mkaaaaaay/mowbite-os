#include "mowbite_bridge/warn_log.h"

#include <algorithm>
#include <cctype>

#include "mowbite_bridge/text.h"

namespace mowbite_bridge {

namespace {

constexpr std::size_t kMaxMessageBytes = 2000;

const char* LevelName(int level) {
  if (level >= kLogFatal) return "FATAL";
  if (level >= kLogError) return "ERROR";
  return "WARN";
}

int LevelByName(const Json& name) {
  if (!name.is_string()) return kLogWarn;
  std::string upper = name.get<std::string>();
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return std::toupper(c); });
  if (upper == "DEBUG") return kLogDebug;
  if (upper == "INFO") return kLogInfo;
  if (upper == "ERROR") return kLogError;
  if (upper == "FATAL") return kLogFatal;
  return kLogWarn;
}

}  // namespace

void WarnLog::Add(double t, int level, std::string_view node, std::string_view message) {
  if (level < kLogWarn || capacity_ == 0) return;
  const bool cut = message.size() > kMaxMessageBytes;
  std::string text = HideSecrets(CleanUtf8(message.substr(0, kMaxMessageBytes)));
  if (cut) text += "...";
  if (lines_.size() == capacity_) lines_.pop_front();
  lines_.push_back({t, level, CleanUtf8(node), std::move(text)});
}

Json WarnLog::Recent(const Json& params) const {
  const Json p = params.is_object() ? params : Json::object();
  const auto since = p.find("since");
  const bool has_since = since != p.end() && since->is_number();
  const double after = has_since ? since->get<double>() : 0.0;
  const int lowest = LevelByName(p.value("level", Json()));
  std::size_t limit = lines_.size();
  const auto wanted = p.find("limit");
  if (wanted != p.end() && wanted->is_number() && wanted->get<double>() >= 0) {
    limit = static_cast<std::size_t>(std::min(wanted->get<double>(), static_cast<double>(lines_.size())));
  }
  Json out = Json::array();
  for (const auto& line : lines_) {
    if (line.level < lowest || (has_since && line.t <= after)) continue;
    out.push_back(Json{{"t", line.t}, {"level", LevelName(line.level)}, {"node", line.node}, {"msg", line.message}});
  }
  if (out.size() > limit) out.erase(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(out.size() - limit));
  return out;
}

}  // namespace mowbite_bridge
