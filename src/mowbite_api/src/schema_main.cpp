// mowbite_api_schema: prints the JSON Schema of the app interface. With --write <dir> it writes <dir>/api.schema.json;
// a file there of an older api (1 or more) moves to <dir>/schema/api-<N>.schema.json first, where test_schema.cpp
// checks that the new schema only adds to it within the same break. A file of the same api with other content is
// refused: raise kApi first.
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "mowbite_api/catalog.h"

namespace {

std::string Text(const mowbite_api::Json& schema) {
  return schema.dump(2) + "\n";
}

int Write(const std::filesystem::path& dir) {
  namespace fs = std::filesystem;
  const fs::path file = dir / "api.schema.json";
  const std::string text = Text(mowbite_api::Schema());
  if (fs::exists(file)) {
    std::ifstream in(file);
    const mowbite_api::Json old = mowbite_api::Json::parse(in, nullptr, false);
    if (old.is_discarded()) {
      std::cerr << file << ": not JSON\n";
      return 1;
    }
    if (old == mowbite_api::Schema()) {
      std::cout << file << " is up to date (api " << mowbite_api::kApi << ")\n";
      return 0;
    }
    const int old_api = old.value("x-api", 0);
    if (old_api >= mowbite_api::kApi) {
      std::cerr << file << " has api " << old_api << " with other content: raise kApi in catalog.h first\n";
      return 1;
    }
    // Within a break the new schema only adds; a raised kApiBreak may take away.
    if (mowbite_api::ApiBreakOf(old) == mowbite_api::kApiBreak) {
      const auto problems = mowbite_api::CheckAdditive(old, mowbite_api::Schema());
      for (const auto& problem : problems) std::cerr << "not only added: " << problem << "\n";
      if (!problems.empty()) {
        std::cerr << file << " stays: raise kApiBreak in catalog.h for a change that takes away\n";
        return 1;
      }
    }
    // Without a version (the placeholder before the first schema) there is nothing to keep.
    if (old_api >= 1) {
      fs::create_directories(dir / "schema");
      fs::rename(file, dir / "schema" / ("api-" + std::to_string(old_api) + ".schema.json"));
    }
  }
  std::ofstream out(file);
  out << text;
  std::cout << "wrote " << file << " (api " << mowbite_api::kApi << ")\n";
  return out ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string option = argc > 1 ? argv[1] : "";
    if (option == "--write" && argc == 3) return Write(argv[2]);
    if (argc == 1) {
      std::cout << Text(mowbite_api::Schema());
      return 0;
    }
    std::cerr << "usage: mowbite_api_schema [--write <dir of api.schema.json>]\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "mowbite_api_schema: " << error.what() << "\n";
    return 1;
  }
}
