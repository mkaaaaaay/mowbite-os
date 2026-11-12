#include <gtest/gtest.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>
#include <variant>

#include "mowbite_journal/journaled.h"
#include "mowbite_journal/replay.h"

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

}  // namespace
}  // namespace mowbite_journal
