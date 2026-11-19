// map.json on the mower: written atomically, the previous version kept, the size limited.
#pragma once

#include <optional>
#include <string>

#include "mowbite_map/datum.h"
#include "mowbite_map/map_model.h"

namespace mowbite_map {

class MapStore {
 public:
  // dir is the data_dir. A map.json larger than max_bytes is neither written nor read, so with the previous version
  // and the temporary file the directory holds at most 3 * max_bytes of maps.
  MapStore(std::string dir, size_t max_bytes);

  struct Loaded {
    std::optional<MapData> map;
    // The file the map came from, empty without a map.
    std::string source;
    // SHA-256 of that file as it lies on the disk.
    std::string sha256;
    // Why a file was not used, empty if all was well.
    std::string problem;
  };

  // map.json, or map.json.bak if map.json is missing or unusable. No file at all is no problem, just no map.
  Loaded Load() const;

  // Writes the text of a map. Returns an empty string, or why it failed; the files stay as they were then.
  std::string Save(const std::string& text);

  struct LoadedDatum {
    std::optional<Datum> datum;
    // SHA-256 of the map file the datum belongs to; empty in a file of an older version.
    std::string map_sha256;
    // Why the file was not used, empty if it is missing or fine.
    std::string problem;
  };

  // map_datum.json next to the map: the datum the map was recorded in, tied to the map file by its SHA-256.
  LoadedDatum LoadDatum() const;
  std::string SaveDatum(const Datum& datum, const std::string& map_sha256);

  const std::string& dir() const {
    return dir_;
  }
  size_t max_bytes() const {
    return max_bytes_;
  }

 private:
  std::string dir_;
  size_t max_bytes_;
};

}  // namespace mowbite_map
