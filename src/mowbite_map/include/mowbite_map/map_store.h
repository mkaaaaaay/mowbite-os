// map.json on the mower: written atomically, the previous version kept, the size limited.
#pragma once

#include <optional>
#include <string>

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
    // Why a file was not used, empty if all was well.
    std::string problem;
  };

  // map.json, or map.json.bak if map.json is missing or unusable. No file at all is no problem, just no map.
  Loaded Load() const;

  // Writes the text of a map. Returns an empty string, or why it failed; the files stay as they were then.
  std::string Save(const std::string& text);

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
