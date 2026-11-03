#ifndef MOWBITE_BRIDGE_EVENT_LOG_H
#define MOWBITE_BRIDGE_EVENT_LOG_H

#include <cstdint>
#include <string>
#include <vector>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// The events of events/json on disk, one JSONL file per local day: <dir>/YYYYMMDD.jsonl.
class EventLog {
 public:
  // Without a directory nothing is kept. At most max_days files (the oldest go first) and max_bytes over all of them.
  EventLog(std::string dir, std::size_t max_days, std::uintmax_t max_bytes);

  // Appends one event to the file of today; over the byte limit older days go first.
  void Write(const Json& event, double now);

  // events.history: params {"date": "YYYYMMDD"} or ["YYYYMMDD"], none for today; an unknown or invalid day gives [].
  Json Read(const Json& params, double now) const;
  // events.history.list: the days, newest first.
  Json List() const;
  // events.history.delete: params as for Read, none deletes every day; {"deleted": n}.
  Json Erase(const Json& params);

  // YYYYMMDD of t in the local time of the process.
  static std::string DayOf(double t);

 private:
  struct DayFile {
    std::string day;
    std::uintmax_t bytes;
  };
  // The day files, oldest first.
  std::vector<DayFile> Scan() const;
  std::string PathOf(const std::string& day) const;
  void Trim(const std::string& today, std::uintmax_t incoming);

  std::string dir_;
  std::size_t max_days_;
  std::uintmax_t max_bytes_;
  std::string trimmed_for_;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_EVENT_LOG_H
