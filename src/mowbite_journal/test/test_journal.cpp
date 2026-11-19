#include <gtest/gtest.h>
#include <sys/file.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>
#include <variant>

#include "mowbite_journal/anonymize.h"
#include "mowbite_journal/journaled.h"
#include "mowbite_journal/replay.h"
#include "mowbite_journal/timeline.h"

namespace fs = std::filesystem;

namespace toy {

struct ToyConfig {
  double gain = 1.0;
  std::string label;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ToyConfig, gain, label)

struct Sample {
  double x;
  int n;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Sample, x, n)

// An input the journal does not see, to make a replay differ.
int hidden = 0;

class Toy {
 public:
  explicit Toy(const ToyConfig& config) : config_(config) {}
  void Add(double value) {
    total_ += config_.gain * value + hidden;
  }
  double Sum(const std::vector<double>& extra) {
    double sum = total_;
    for (double v : extra) sum += v;
    return sum;
  }
  bool Take(int limit, int* taken) {
    const int n = std::min(limit, static_cast<int>(total_));
    if (taken) *taken = n;
    total_ -= n;
    return n > 0;
  }
  void OnSample(const Sample& sample) {
    total_ += sample.x * sample.n;
  }
  std::optional<std::string> Label(const std::optional<double>& nan_or_not) {
    if (nan_or_not && std::isnan(*nan_or_not)) return std::nullopt;
    return config_.label;
  }
  double total() const {
    return total_;
  }

 private:
  ToyConfig config_;
  double total_ = 0;
};

}  // namespace toy

MOWBITE_JOURNAL_POD(toy::Sample);

template <>
struct mowbite_journal::CoreTraits<toy::Toy> {
  static constexpr const char* kName = "toy";
  using Config = toy::ToyConfig;
  MOWBITE_JOURNAL_METHODS(&toy::Toy::Add, &toy::Toy::Sum, &toy::Toy::Take, &toy::Toy::OnSample, &toy::Toy::Label);
  static json View(const toy::Toy& core) {
    return {{"big", core.total() > 10}};
  }
};

namespace mowbite_journal {
namespace {

void Wait(double seconds) {
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

// Polls the condition until it holds or the time is up.
template <typename Condition>
bool WaitFor(Condition condition, double seconds) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (!condition()) {
    if (std::chrono::steady_clock::now() > end) return false;
    Wait(0.02);
  }
  return true;
}

class JournalTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("journal_test_" + std::to_string(getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    options_.dir = dir_.string();
    options_.reserve_mb = 0;
    toy::hidden = 0;
    Register<toy::Toy>(registry_);
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }

  // A run of calls; the journal is closed at the end.
  void Record(int calls) {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    ASSERT_TRUE(journal) << error;
    Journaled<toy::Toy> core(journal.get(), toy::ToyConfig{2.0, "lawn"});
    for (int i = 0; i < calls; ++i) {
      core.Call<&toy::Toy::Add>(0.5 * i);
      core.Call<&toy::Toy::OnSample>(toy::Sample{0.25, i});
      core.Call<&toy::Toy::Sum>(std::vector<double>{1.0, std::numeric_limits<double>::infinity()});
      int taken = 0;
      core.Call<&toy::Toy::Take>(3, &taken);
      core.Call<&toy::Toy::Take>(1, nullptr);
      core.Call<&toy::Toy::Label>(std::optional<double>(std::nan("")));
    }
  }

  fs::path dir_;
  Options options_;
  ReplayRegistry registry_;
};

TEST(Codec, KeepsWhatItWrites) {
  Out out;
  Write(out, 3.5);
  Write(out, std::string("abc"));
  Write(out, std::vector<int>{1, 2, 3});
  Write(out, std::optional<float>());
  Write(out, std::make_pair(true, int8_t{-4}));
  Write(out, json{{"a", std::nan("")}, {"b", -std::numeric_limits<double>::infinity()}});
  Write(out, toy::ToyConfig{0.1, "x"});
  Write(out, toy::Sample{1.5, 7});
  In in(out.data(), out.size());
  EXPECT_EQ(Read<double>(in), 3.5);
  EXPECT_EQ(Read<std::string>(in), "abc");
  EXPECT_EQ(Read<std::vector<int>>(in), (std::vector<int>{1, 2, 3}));
  EXPECT_FALSE(Read<std::optional<float>>(in).has_value());
  EXPECT_EQ((Read<std::pair<bool, int8_t>>(in)), std::make_pair(true, int8_t{-4}));
  const json j = Read<json>(in);
  EXPECT_TRUE(std::isnan(j.at("a").get<double>()));
  EXPECT_EQ(j.at("b").get<double>(), -std::numeric_limits<double>::infinity());
  const auto config = Read<toy::ToyConfig>(in);
  EXPECT_EQ(config.gain, 0.1);
  EXPECT_EQ(config.label, "x");
  const auto sample = Read<toy::Sample>(in);
  EXPECT_EQ(sample.x, 1.5);
  EXPECT_EQ(sample.n, 7);
  EXPECT_EQ(in.left(), 0u);
  EXPECT_THROW(Read<double>(in), std::runtime_error);
}

TEST(Codec, KeepsTheTypeOfAVariant) {
  struct Mark {};
  using Choice = std::variant<Mark, std::string, toy::ToyConfig>;
  Out out;
  Write(out, Choice(Mark{}));
  Write(out, Choice(toy::ToyConfig{2.0, "y"}));
  Write(out, Choice(std::string("z")));
  EXPECT_EQ(out.data()[0], 0u);
  EXPECT_EQ(out.data()[1], 2u);
  In in(out.data(), out.size());
  EXPECT_EQ(Read<Choice>(in).index(), 0u);
  const auto config = std::get<toy::ToyConfig>(Read<Choice>(in));
  EXPECT_EQ(config.label, "y");
  EXPECT_EQ(std::get<std::string>(Read<Choice>(in)), "z");
  EXPECT_EQ(in.left(), 0u);
  const uint8_t beyond = 3;
  In bad(&beyond, 1);
  EXPECT_THROW(Read<Choice>(bad), std::runtime_error);
}

TEST(Table, NamesComeFromTheMacro) {
  EXPECT_EQ(detail::MethodNames("&a::B::C, &a::B::D,\n &E"), (std::vector<std::string>{"C", "D", "E"}));
  const json definition = CoreDefinition<toy::Toy>(toy::ToyConfig{}, json::object());
  ASSERT_EQ(definition.at("methods").size(), 5u);
  EXPECT_EQ(definition.at("methods")[2].at("name"), "Take");
  EXPECT_NE(definition.at("methods")[2].at("signature").get<std::string>().find("int*"), std::string::npos);
}

// The definition of a build of toy whose table has the first n methods, or the methods in the order given.
json DefinitionWith(const std::vector<size_t>& order) {
  json definition = CoreDefinition<toy::Toy>(toy::ToyConfig{}, json::object());
  const auto names = detail::MethodNames(CoreTraits<toy::Toy>::kMethodText);
  const auto signatures = detail::Table<CoreTraits<toy::Toy>::Methods>::Signatures();
  std::vector<std::string> some_names, some_signatures;
  json methods = json::array();
  for (size_t i : order) {
    some_names.push_back(names[i]);
    some_signatures.push_back(signatures[i]);
    methods.push_back(definition.at("methods")[i]);
  }
  definition["methods"] = methods;
  definition["table"] = detail::TableChecksum(some_names, some_signatures);
  return definition;
}

TEST(Table, AJournalOfAnEarlierTablePlaysOneOfAnotherDoesNot) {
  EXPECT_NO_THROW(Replayer<toy::Toy>{DefinitionWith({0, 1, 2, 3, 4})});
  EXPECT_NO_THROW(Replayer<toy::Toy>{DefinitionWith({0, 1, 2})});
  EXPECT_THROW(Replayer<toy::Toy>{DefinitionWith({1, 0, 2})}, std::runtime_error);
  EXPECT_THROW(Replayer<toy::Toy>{DefinitionWith({0, 2})}, std::runtime_error);
  json longer = DefinitionWith({0, 1, 2, 3, 4});
  longer["methods"].push_back(longer["methods"][0]);
  EXPECT_THROW(Replayer<toy::Toy>{longer}, std::runtime_error);
}

TEST_F(JournalTest, AReplayOfTheSameCoreIsBitIdentical) {
  Record(200);
  const CheckReport report = CheckRuns(dir_.string(), registry_);
  EXPECT_EQ(report.outcome, CheckOutcome::kSame);
  ASSERT_FALSE(report.lines.empty());
  EXPECT_NE(report.lines.back().find("toy: bit-identical, 1200 calls"), std::string::npos) << report.lines.back();
}

TEST_F(JournalTest, AnInputBesideTheJournalShowsAsFirstDifference) {
  toy::hidden = 1;
  Record(5);
  toy::hidden = 0;
  const CheckReport report = CheckRuns(dir_.string(), registry_);
  EXPECT_EQ(report.outcome, CheckOutcome::kDiffers);
  ASSERT_FALSE(report.lines.empty());
  // Add returns nothing and Sum adds infinity; the first call that shows it is Take of the first round.
  EXPECT_NE(report.lines.back().find("Take (result): expected {\"out\":[1],\"result\":true}"), std::string::npos)
      << report.lines.back();
  EXPECT_NE(report.lines.back().find("after 4 calls"), std::string::npos) << report.lines.back();
}

TEST_F(JournalTest, SegmentsFollowEachOther) {
  options_.segment_s = 0.2;
  options_.flush_s = 0.05;
  {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    ASSERT_TRUE(journal) << error;
    Journaled<toy::Toy> core(journal.get(), toy::ToyConfig{});
    for (int i = 0; i < 40; ++i) {
      core.Call<&toy::Toy::Add>(1.0);
      Wait(0.02);
    }
  }
  const auto runs = ListRuns(dir_.string());
  ASSERT_EQ(runs.size(), 1u);
  EXPECT_GE(ListRun(runs[0]).segments.size(), 3u);
  EXPECT_EQ(CheckRuns(dir_.string(), registry_).outcome, CheckOutcome::kSame);
}

// The head names the build IDs of the libraries the platform names, beside the program.
TEST_F(JournalTest, TheHeadNamesTheLibrariesOfThePlatform) {
  const auto ids = [this](const std::vector<std::string>& libraries) {
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    options_.libraries = libraries;
    Record(1);
    const auto runs = ListRuns(dir_.string());
    json head;
    std::string error;
    EXPECT_EQ(runs.size(), 1u);
    EXPECT_TRUE(!runs.empty() && ReadSegmentHead(ListRun(runs[0]).segments.at(0), &head, &error)) << error;
    return head.value("build_ids", json::object());
  };
  const auto named = [](const json& ids, const std::string& part) {
    for (const auto& id : ids.items()) {
      if (id.key().find(part) != std::string::npos) return true;
    }
    return false;
  };
  const json plain = ids({});
  EXPECT_TRUE(plain.contains("program"));
  EXPECT_FALSE(named(plain, "libc.so"));
  EXPECT_TRUE(named(ids({"libc.so"}), "libc.so"));
}

TEST_F(JournalTest, AFullBufferLeavesAGapThatStopsTheReplay) {
  options_.buffer_kb = 1;
  options_.flush_s = 10;
  Record(100);
  const CheckReport report = CheckRuns(dir_.string(), registry_);
  EXPECT_EQ(report.outcome, CheckOutcome::kNotReplayable);
  EXPECT_NE(report.lines.back().find("gap in the journal (buffer full)"), std::string::npos) << report.lines.back();
}

TEST_F(JournalTest, ExportNamesMethodsAndArguments) {
  Record(2);
  std::ostringstream lines;
  std::string error;
  ASSERT_TRUE(ExportJsonl(dir_.string(), registry_, {"toy", "Take"}, lines, &error)) << error;
  std::istringstream in(lines.str());
  std::string line;
  int count = 0;
  while (std::getline(in, line)) {
    const json j = json::parse(line);
    EXPECT_EQ(j.at("method"), "Take");
    ++count;
  }
  EXPECT_EQ(count, 4);
  std::ostringstream csv;
  ASSERT_TRUE(ExportCsv(dir_.string(), registry_, {"toy", "OnSample"}, csv, &error)) << error;
  EXPECT_EQ(csv.str().substr(0, csv.str().find('\n')), "t,args.0.n,args.0.x");
}

TEST_F(JournalTest, OldRunsGoFirstAndRunningOnesStay) {
  // Two finished runs of 300 kB each and one that still runs in another process (its lock is held).
  for (const char* name : {"20260101T000000Z_old_1", "20260102T000000Z_old_2", "20260103T000000Z_busy_3"}) {
    fs::create_directories(dir_ / name);
    std::ofstream(dir_ / name / "000000.mbj") << std::string(300000, 'x');
  }
  const int held = open((dir_ / "20260103T000000Z_busy_3" / "lock").c_str(), O_WRONLY | O_CREAT, 0644);
  ASSERT_EQ(flock(held, LOCK_EX), 0);
  options_.max_mb = 0.7;
  {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    ASSERT_TRUE(journal) << error;
    Wait(0.2);
  }
  close(held);
  EXPECT_FALSE(fs::exists(dir_ / "20260101T000000Z_old_1"));
  EXPECT_TRUE(fs::exists(dir_ / "20260102T000000Z_old_2"));
  EXPECT_TRUE(fs::exists(dir_ / "20260103T000000Z_busy_3"));
}

// Sets the modification time of path to t (unix s).
void Touch(const fs::path& path, std::time_t t) {
  const timeval times[2] = {{t, 0}, {t, 0}};
  ASSERT_EQ(utimes(path.c_str(), times), 0) << path;
}

// Age counts only for times of a set clock; a run without a segment goes by the age of its folder.
TEST_F(JournalTest, AgeOnlyWithASetClockAndEmptyRunsGo) {
  const std::time_t now = std::time(nullptr);
  const std::time_t days_ago = now - 3 * 86400;
  for (const char* name : {"20260101T000000Z_unset_1", "20260102T000000Z_old_2"}) {
    fs::create_directories(dir_ / name);
    std::ofstream(dir_ / name / "000000.mbj") << "x";
  }
  Touch(dir_ / "20260101T000000Z_unset_1" / "000000.mbj", 1000);
  Touch(dir_ / "20260102T000000Z_old_2" / "000000.mbj", days_ago);
  fs::create_directories(dir_ / "20260103T000000Z_empty_3");
  std::ofstream(dir_ / "20260103T000000Z_empty_3" / "lock");
  Touch(dir_ / "20260103T000000Z_empty_3", days_ago);
  fs::create_directories(dir_ / "20260104T000000Z_fresh_4");
  options_.max_days = 1;
  {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    ASSERT_TRUE(journal) << error;
    Wait(0.2);
  }
  EXPECT_TRUE(fs::exists(dir_ / "20260101T000000Z_unset_1"));
  EXPECT_FALSE(fs::exists(dir_ / "20260102T000000Z_old_2"));
  EXPECT_FALSE(fs::exists(dir_ / "20260103T000000Z_empty_3"));
  EXPECT_TRUE(fs::exists(dir_ / "20260104T000000Z_fresh_4"));
}

// A block that cannot be written (here EFBIG beyond 64 bytes a file) closes the segment; the next one is tried only
// check_s later, not at each flush.
TEST_F(JournalTest, AWriteErrorIsTriedAgainAfterTheCheckTime) {
  options_.flush_s = 0.05;
  options_.check_s = 30;
  std::string error;
  auto journal = Journal::Create(options_, &error);
  ASSERT_TRUE(journal) << error;
  Journaled<toy::Toy> core(journal.get(), toy::ToyConfig{});
  ASSERT_TRUE(WaitFor(
      [&] {
        core.Call<&toy::Toy::Add>(1.0);
        return journal->GetStats().packed_bytes > 64;
      },
      2.0));
  const uint64_t segments = journal->GetStats().segments;
  rlimit before{};
  ASSERT_EQ(getrlimit(RLIMIT_FSIZE, &before), 0);
  rlimit small = before;
  small.rlim_cur = 64;
  const auto handler = std::signal(SIGXFSZ, SIG_IGN);
  setrlimit(RLIMIT_FSIZE, &small);
  for (int i = 0; i < 40; ++i) {
    core.Call<&toy::Toy::Add>(1.0);
    Wait(0.025);
  }
  setrlimit(RLIMIT_FSIZE, &before);
  std::signal(SIGXFSZ, handler);
  const Stats stats = journal->GetStats();
  EXPECT_EQ(stats.problem.rfind("write: ", 0), 0u) << stats.problem;
  EXPECT_EQ(stats.segments, segments);
}

TEST_F(JournalTest, ARunWithoutItsFirstSegmentIsNotReplayable) {
  options_.segment_s = 0.1;
  options_.flush_s = 0.05;
  {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    ASSERT_TRUE(journal) << error;
    Journaled<toy::Toy> core(journal.get(), toy::ToyConfig{});
    for (int i = 0; i < 20; ++i) {
      core.Call<&toy::Toy::Add>(1.0);
      Wait(0.02);
    }
  }
  const auto runs = ListRuns(dir_.string());
  ASSERT_EQ(runs.size(), 1u);
  fs::remove(fs::path(runs[0]) / "000000.mbj");
  const CheckReport report = CheckRuns(dir_.string(), registry_);
  EXPECT_EQ(report.outcome, CheckOutcome::kNotReplayable);
  EXPECT_NE(report.lines[0].find("segment 0 is missing"), std::string::npos) << report.lines[0];
}

// The run directory goes aside and a file takes its place, so that no segment can be created there, also as root.
struct MovedRun {
  fs::path run;
  fs::path aside;
  void Move() const {
    fs::rename(run, aside);
    std::ofstream(run) << "x";
  }
  void Restore() const {
    fs::remove(run);
    fs::rename(aside, run);
  }
};

TEST_F(JournalTest, ASegmentThatCannotBeCreatedIsTriedAgainLater) {
  options_.segment_s = 0.2;
  options_.flush_s = 0.05;
  options_.check_s = 0.5;
  {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    ASSERT_TRUE(journal) << error;
    Journaled<toy::Toy> core(journal.get(), toy::ToyConfig{});
    const MovedRun moved{journal->GetStats().run_dir, dir_ / "aside"};
    ASSERT_TRUE(WaitFor([&] { return journal->GetStats().segments >= 1; }, 2.0));
    moved.Move();
    ASSERT_TRUE(WaitFor(
        [&] {
          core.Call<&toy::Toy::Add>(1.0);
          return !journal->GetStats().problem.empty();
        },
        2.0));
    const Stats failed = journal->GetStats();
    EXPECT_NE(failed.problem.find("cannot create"), std::string::npos) << failed.problem;
    core.Call<&toy::Toy::Add>(1.0);
    moved.Restore();
    ASSERT_TRUE(WaitFor([&] { return journal->GetStats().problem.empty(); }, 3.0));
    EXPECT_GT(journal->GetStats().segments, failed.segments);
    core.Call<&toy::Toy::Add>(1.0);
  }
  const auto runs = ListRuns(dir_.string());
  ASSERT_EQ(runs.size(), 1u);
  EXPECT_TRUE(ListRun(runs[0]).missing.empty());
  const CheckReport report = CheckRuns(dir_.string(), registry_);
  EXPECT_EQ(report.outcome, CheckOutcome::kNotReplayable);
  ASSERT_FALSE(report.lines.empty());
  EXPECT_NE(report.lines.back().find("gap in the journal (cannot create"), std::string::npos) << report.lines.back();
}

TEST_F(JournalTest, ASegmentThatCannotBeCreatedIsNotTriedAgainBeforeTheCheckTime) {
  options_.segment_s = 0.2;
  options_.flush_s = 0.05;
  options_.check_s = 2.0;
  std::string error;
  auto journal = Journal::Create(options_, &error);
  ASSERT_TRUE(journal) << error;
  const MovedRun moved{journal->GetStats().run_dir, dir_ / "aside"};
  ASSERT_TRUE(WaitFor([&] { return journal->GetStats().segments >= 1; }, 2.0));
  moved.Move();
  ASSERT_TRUE(WaitFor([&] { return !journal->GetStats().problem.empty(); }, 2.0));
  const Stats failed = journal->GetStats();
  moved.Restore();
  // Many flushes later the directory is fine again, but the next try waits for check_s after the failure.
  Wait(1.0);
  EXPECT_EQ(journal->GetStats().problem, failed.problem);
  EXPECT_EQ(journal->GetStats().segments, failed.segments);
  ASSERT_TRUE(WaitFor([&] { return journal->GetStats().problem.empty(); }, 3.0));
  EXPECT_GT(journal->GetStats().segments, failed.segments);
}

std::vector<json> Lines(const std::string& text) {
  std::vector<json> lines;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) lines.push_back(json::parse(line));
  return lines;
}

// Two runs at once in their own journal_dirs: a calls Add, b calls OnSample, in turns; a also has a raw stream.
class TimelineTest : public JournalTest {
 protected:
  void SetUp() override {
    JournalTest::SetUp();
    std::string error;
    for (const char* name : {"a", "b"}) {
      Options options = options_;
      options.dir = (dir_ / name).string();
      journals_.push_back(Journal::Create(options, &error));
      ASSERT_TRUE(journals_.back()) << error;
    }
    Journaled<toy::Toy> a(journals_[0].get(), toy::ToyConfig{1.0, "a"});
    Journaled<toy::Toy> b(journals_[1].get(), toy::ToyConfig{1.0, "b"});
    const uint16_t port = journals_[0]->Define({{"name", "port.rx"}, {"kind", "raw"}});
    const uint8_t bytes[3] = {1, 2, 3};
    for (int i = 0; i < 20; ++i) {
      a.Call<&toy::Toy::Add>(1.0);
      b.Call<&toy::Toy::OnSample>(toy::Sample{1.0, 1});
      journals_[0]->Append(port, kRawBytes, bytes, sizeof(bytes));
    }
    journals_.clear();
  }

  std::string Run(int64_t from_ns, int64_t to_ns, size_t max_bytes) {
    std::string error;
    const std::string text =
        Timeline({(dir_ / "a").string(), (dir_ / "b").string()}, from_ns, to_ns, max_bytes, &error);
    EXPECT_TRUE(error.empty()) << error;
    return text;
  }

  std::vector<std::unique_ptr<Journal>> journals_;
};

TEST_F(TimelineTest, TakesTheRunsInTheOrderOfTime) {
  const auto lines = Lines(Run(INT64_MIN, INT64_MAX, 1 << 20));
  std::vector<std::string> methods;
  int64_t last = INT64_MIN;
  int views = 0, ends = 0;
  uint64_t raw_records = 0, raw_bytes = 0;
  for (const auto& line : lines) {
    ASSERT_FALSE(line.contains("truncated")) << line;
    EXPECT_GE(line.at("t_ns").get<int64_t>(), last) << line;
    last = line.at("t_ns");
    if (line.contains("method")) methods.push_back(line.at("method"));
    if (line.contains("view")) {
      ++views;
      EXPECT_TRUE(line.at("view").contains("big")) << line;
    }
    if (line.contains("end")) ++ends;
    if (line.contains("records")) {
      EXPECT_EQ(line.at("stream"), "port.rx");
      raw_records += line.at("records").get<uint64_t>();
      raw_bytes += line.at("bytes").get<uint64_t>();
    }
  }
  ASSERT_EQ(methods.size(), 40u);
  for (size_t i = 0; i < methods.size(); ++i) EXPECT_EQ(methods[i], i % 2 == 0 ? "Add" : "OnSample") << i;
  // Both views come once each from false to true: after 11 of a, after 11 of b; the first view of a core is a change.
  EXPECT_EQ(views, 4);
  EXPECT_EQ(ends, 2);
  EXPECT_EQ(raw_records, 20u);
  EXPECT_EQ(raw_bytes, 60u);
}

TEST_F(TimelineTest, KeepsToTheWindow) {
  const auto all = Lines(Run(INT64_MIN, INT64_MAX, 1 << 20));
  std::vector<int64_t> calls;
  for (const auto& line : all) {
    if (line.contains("method")) calls.push_back(line.at("t_ns"));
  }
  ASSERT_EQ(calls.size(), 40u);
  const auto lines = Lines(Run(calls[10], calls[19], 1 << 20));
  int methods = 0;
  for (const auto& line : lines) {
    EXPECT_GE(line.at("t_ns").get<int64_t>(), calls[10]) << line;
    EXPECT_LE(line.at("t_ns").get<int64_t>(), calls[19]) << line;
    if (line.contains("method")) ++methods;
    EXPECT_FALSE(line.contains("end")) << line;
  }
  EXPECT_EQ(methods, 10);
}

TEST_F(TimelineTest, TheLineOfTheCutCountsToo) {
  const std::string full = Run(INT64_MIN, INT64_MAX, 1 << 20);
  size_t fixed = 0;
  for (const auto& line : Lines(full)) {
    if (!line.contains("method")) fixed += line.dump().size() + 1;
  }
  for (size_t max_bytes = fixed + 20; max_bytes <= full.size(); ++max_bytes) {
    EXPECT_LE(Run(INT64_MIN, INT64_MAX, max_bytes).size(), max_bytes);
  }
}

TEST_F(TimelineTest, CutsTheEarliestMethodsFirstAndKeepsTheViews) {
  const std::string full = Run(INT64_MIN, INT64_MAX, 1 << 20);
  size_t fixed = 0;
  std::vector<int64_t> calls;
  for (const auto& line : Lines(full)) {
    if (line.contains("method")) {
      calls.push_back(line.at("t_ns"));
    } else {
      fixed += line.dump().size() + 1;
    }
  }
  const auto lines = Lines(Run(INT64_MIN, INT64_MAX, fixed + 600));
  ASSERT_FALSE(lines.empty());
  ASSERT_TRUE(lines.back().contains("truncated"));
  const auto kept = std::count_if(lines.begin(), lines.end(), [](const json& j) { return j.contains("method"); });
  EXPECT_GT(kept, 0);
  EXPECT_EQ(lines.back().at("truncated").get<long>() + kept, 40);
  int views = 0;
  int64_t first_method = INT64_MAX;
  for (const auto& line : lines) {
    if (line.contains("view")) ++views;
    if (line.contains("method")) first_method = std::min(first_method, line.at("t_ns").get<int64_t>());
  }
  EXPECT_EQ(views, 4);
  EXPECT_EQ(first_method, calls[calls.size() - static_cast<size_t>(kept)]);
  std::string error;
  EXPECT_TRUE(Timeline({(dir_ / "none").string()}, 0, 1, 100, &error).empty());
  EXPECT_FALSE(error.empty());
}

TEST(Anonymize, MovesDatesByWholeDays) {
  AnonymizeOptions options;
  options.shift_days = -1001;
  EXPECT_EQ(options.ShiftDates("run 2026-11-15T10:00, until 2024-02-29; 12026-11-15 and 2026-02-30 stay"),
            "run 2024-02-18T10:00, until 2021-06-03; 12026-11-15 and 2026-02-30 stay");
  int year = 2026, month = 1, day = 1;
  options.ShiftDate(&year, &month, &day);
  EXPECT_EQ(std::vector<int>({year, month, day}), std::vector<int>({2023, 4, 6}));
  json value = {{"2026-06-15", {"10:00"}}, {"until", 1781510400}, {"t", 5}, {"at", "2026-06-15"}};
  options.ShiftWallIn(&value, {"until"});
  EXPECT_EQ(value, json({{"2023-09-18", {"10:00"}}, {"until", 1781510400 - 1001 * 86400}, {"t", 5},
                         {"at", "2023-09-18"}}));
}

}  // namespace
}  // namespace mowbite_journal
