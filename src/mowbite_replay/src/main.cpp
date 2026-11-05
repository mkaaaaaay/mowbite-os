#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "mowbite_replay/cores.h"

namespace {

int Usage() {
  std::cerr << "mowbite_replay check [--if-any] <journal_dir or run>...\n"
               "  replays every core from the start of its process; exit 0 bit-identical, 1 differs,\n"
               "  2 not replayable (gap, missing segments, unknown core), 3 unreadable or no journal\n"
               "  (--if-any: a folder without a journal passes)\n"
               "mowbite_replay export --jsonl|--csv [--stream <name>] [--method <name>] <journal_dir or run>\n"
               "  the records as JSON lines, or one method of one stream as a table (--csv needs both)\n"
               "mowbite_replay cat <journal_dir or run>\n"
               "  the unpacked bytes of every segment, to search them\n";
  return 3;
}

int Check(std::vector<std::string> dirs) {
  const bool if_any = !dirs.empty() && dirs.front() == "--if-any";
  if (if_any) dirs.erase(dirs.begin());
  if (dirs.empty()) return Usage();
  const auto registry = mowbite_replay::Cores();
  mowbite_journal::CheckOutcome worst = mowbite_journal::CheckOutcome::kSame;
  for (const auto& dir : dirs) {
    if (if_any && mowbite_journal::ListRuns(dir).empty()) {
      std::cout << dir << ": no journal\n";
      continue;
    }
    const auto report = mowbite_journal::CheckRuns(dir, registry);
    for (const auto& line : report.lines) std::cout << line << '\n';
    worst = mowbite_journal::Worse(worst, report.outcome);
  }
  return static_cast<int>(worst);
}

int Export(const std::vector<std::string>& args) {
  bool csv = false;
  mowbite_journal::ExportFilter filter;
  std::string dir;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--csv") {
      csv = true;
    } else if (args[i] == "--jsonl") {
      csv = false;
    } else if (args[i] == "--stream" && i + 1 < args.size()) {
      filter.stream = args[++i];
    } else if (args[i] == "--method" && i + 1 < args.size()) {
      filter.method = args[++i];
    } else if (dir.empty()) {
      dir = args[i];
    } else {
      return Usage();
    }
  }
  if (dir.empty()) return Usage();
  std::string error;
  const auto registry = mowbite_replay::Cores();
  const bool ok = csv ? mowbite_journal::ExportCsv(dir, registry, filter, std::cout, &error)
                      : mowbite_journal::ExportJsonl(dir, registry, filter, std::cout, &error);
  if (!ok) {
    std::cerr << error << '\n';
    return 3;
  }
  return 0;
}

int Cat(const std::string& dir) {
  for (const auto& run : mowbite_journal::ListRuns(dir)) {
    for (const auto& path : mowbite_journal::ListRun(run).segments) {
      mowbite_journal::Segment segment;
      std::string error;
      if (!mowbite_journal::ReadSegment(path, &segment, &error)) {
        std::cerr << error << '\n';
        return 3;
      }
      std::fwrite(segment.raw.data(), 1, segment.raw.size(), stdout);
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) return Usage();
  const std::string command = argv[1];
  const std::vector<std::string> args(argv + 2, argv + argc);
  if (command == "check") return Check(args);
  if (command == "export") return Export(args);
  if (command == "cat" && args.size() == 1) return Cat(args[0]);
  return Usage();
}
