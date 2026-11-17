// mowbite_map_check: a map as JSON on stdin, held to the rules of map.json. Prints "OK <areas> <docking stations>"
// with exit 0, else why with exit 1.
#include <iostream>
#include <iterator>
#include <string>

#include "mowbite_map/map_model.h"

int main() {
  const std::string text((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
  const auto parsed = mowbite_map::json::parse(text, nullptr, false);
  if (parsed.is_discarded()) {
    std::cout << "not valid JSON\n";
    return 1;
  }
  const mowbite_map::ParseResult result = mowbite_map::ParseMap(parsed);
  if (!result.map) {
    std::cout << result.error << "\n";
    return 1;
  }
  std::cout << "OK " << result.map->areas.size() << " " << result.map->docking_stations.size() << "\n";
  return 0;
}
