#ifndef MOWBITE_STATS_INCIDENT_STORE_H
#define MOWBITE_STATS_INCIDENT_STORE_H

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_journal/run_index.h"
#include "mowbite_stats/incident_keeper.h"

namespace mowbite_stats {

// What the node knows at a trigger beyond the keeper: wall clock, boot, state and log lines for the manifest.
struct IncidentContext {
  // Unix s of the first trigger.
  double created_s = 0;
  std::string boot_id;
  json robot;
  json log = json::array();
};

// A piece of incidents.download.
struct IncidentPiece {
  uint64_t size = 0;
  std::string data;
  std::string archive_sha256;
};

// The incidents on the disk, journal_dir/incidents/<id>/: manifest.json and per run of the journal a folder with hard
// links of its segments. Linking costs no space while the journal keeps a segment; once it removed it, the incident
// holds it alone. Files only, no ROS.
class IncidentStore {
 public:
  IncidentStore(std::string journal_dir, IncidentLimits limits);

  // The incidents on the disk; one an earlier process left open is sealed as it is (cut).
  std::vector<StoredIncident> Load();
  // Links the segments of each run of this boot that overlap the window of the incident, and of a run that ended
  // without its end (a CRASH trigger) its last two; a run that takes less than a quarter of max_mb goes in from its
  // start, so that it can be replayed. Writes the manifest with state.
  bool Link(const Incident& incident, const IncidentContext& context, const std::vector<mowbite_journal::RunInfo>& runs,
            const std::string& state, std::string* error);
  // Every linked segment is closed or its run is over.
  bool ReadyToSeal(const Incident& incident, const std::vector<mowbite_journal::RunInfo>& runs) const;
  // sha256 and size of each file into the manifest, state done.
  bool Seal(const Incident& incident, const IncidentContext& context, std::string* error);

  std::vector<StoredIncident> Stored() const;
  std::optional<json> Manifest(const std::string& id) const;
  // The folders of the runs in an incident, for the timeline.
  std::vector<std::string> RunDirs(const std::string& id) const;
  bool Remove(const std::string& id, std::string* error);
  // A piece of the tar archive of a sealed incident; without location from a copy of its runs without place and with
  // the date shifted (made once, then kept in the incident).
  bool Piece(const std::string& id, uint64_t offset, size_t size, bool location, IncidentPiece* piece,
             std::string* error);

  std::string dir() const { return dir_; }

 private:
  std::string Folder(const std::string& id) const;
  bool WriteManifest(const Incident& incident, const IncidentContext& context, const std::string& state,
                     const json& runs, std::string* error);

  std::string journal_dir_;
  std::string dir_;
  IncidentLimits limits_;
  // Runs linked per incident with their facts for the manifest.
  std::map<std::string, json> runs_;
  // sha256 of whole archives by id and location.
  std::map<std::pair<std::string, bool>, std::string> archive_sha_;
};

// Days the copy without location is shifted by.
constexpr int kIncidentShiftDays = -1000;

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_INCIDENT_STORE_H
