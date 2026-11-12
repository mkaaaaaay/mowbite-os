#include <cstdio>
#include <cstdlib>
#include <optional>

#include "mowbite_time/sun.h"

namespace {

void Print(const char* name, const std::optional<double>& value) {
  if (value) std::printf("\"%s\": %.3f, ", name, *value);
  else std::printf("\"%s\": null, ", name);
}

}  // namespace

// For tests: sunrise, sunset and the height of the sun at <latitude> <longitude> <unix s> as one line of JSON.
int main(int argc, char** argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s <latitude> <longitude> <unix s>\n", argv[0]);
    return 2;
  }
  const double latitude = std::atof(argv[1]);
  const double longitude = std::atof(argv[2]);
  const double unix_s = std::atof(argv[3]);
  const mowbite_time::SunDay day = mowbite_time::SunTimes(latitude, longitude, unix_s);
  std::printf("{");
  Print("rise", day.rise);
  Print("set", day.set);
  std::printf("\"elevation\": %.4f}\n", mowbite_time::SunElevation(latitude, longitude, unix_s));
  return 0;
}
