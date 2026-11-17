#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include "mowbite_bridge/track.h"

using namespace mowbite_bridge;
namespace fs = std::filesystem;

namespace {

Json Deleted(int n) {
  return Json{{"deleted", n}};
}

class TrackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("mowbite_track_" + std::to_string(::getpid()) + "_" +
                                         ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }

  MowTrack Make(double days = 60, std::uintmax_t bytes = 50000000,
                std::size_t answer_bytes = mowbite_api::kHistoryMaxAnswerBytes) {
    MowTrack track(dir_.string(), days, bytes, answer_bytes);
    track.Open(now_);
    return track;
  }
  void Session(MowTrack& track, const std::string& job, const std::string& session) {
    track.OnEvent(Json{{"id", "e"}, {"type", "STATE"}, {"state", "MOWING"}, {"job_id", job}, {"session_id", session}},
                  now_);
  }
  void Blades(MowTrack& track, bool on) {
    track.OnEvent(Json{{"type", "BLADES"}, {"enabled", on}}, now_);
  }
  // count points 3 cm apart along x from x0.
  static void Line(MowTrack& track, int count, double x0 = 0.0, double y = 0.0) {
    for (int i = 0; i < count; ++i) track.Add(x0 + 0.03 * i, y);
  }
  std::vector<std::string> Files() const {
    std::vector<std::string> names;
    if (!fs::exists(dir_)) return names;
    for (const auto& e : fs::directory_iterator(dir_)) names.push_back(e.path().filename().string());
    std::sort(names.begin(), names.end());
    return names;
  }
  std::vector<Json> Lines(const std::string& name) const {
    std::vector<Json> lines;
    std::ifstream in(dir_ / name);
    std::string line;
    while (std::getline(in, line)) lines.push_back(Json::parse(line));
    return lines;
  }
  static std::size_t PointCount(const Json& history) {
    std::size_t n = 0;
    for (const auto& s : history["segments"]) n += s["points"].size();
    return n;
  }

  fs::path dir_;
  double now_ = 1793600000.0;
};

}  // namespace

// B20
TEST_F(TrackTest, NothingWithoutASession) {
  MowTrack track = Make();
  Line(track, 50);
  Session(track, "job1", "");
  Line(track, 50);
  EXPECT_EQ(track.History(nullptr).dump(), R"({"segments":[],"buffer":[]})");
  track.Close();
  EXPECT_TRUE(Files().empty());
}

TEST_F(TrackTest, AttributesFollowTheEvents) {
  MowTrack track = Make();
  EXPECT_EQ(Json(track.Attributes().job_id), "");
  Session(track, "job1", "s1");
  Blades(track, true);
  const PoseAttributes a = track.Attributes();
  EXPECT_EQ(a.job_id, "job1");
  EXPECT_EQ(a.session_id, "s1");
  EXPECT_TRUE(a.blades);
  // A STATE without session_id ends the session, the job stays.
  track.OnEvent(Json{{"type", "STATE"}, {"state", "IDLE"}, {"job_id", "job1"}}, now_);
  EXPECT_EQ(track.Attributes().session_id, "");
  EXPECT_EQ(track.Attributes().job_id, "job1");
}

TEST_F(TrackTest, AStraightLineShrinksAndTheRestIsTheBuffer) {
  MowTrack track = Make();
  Session(track, "job1", "s1");
  Line(track, 200);
  const Json h = track.History(nullptr);
  ASSERT_EQ(h["segments"].size(), 1u);
  EXPECT_LE(h["segments"][0]["points"].size(), 4u);
  EXPECT_GE(h["buffer"].size(), 1u);
  EXPECT_EQ(h["segments"][0]["points"].size() + h["buffer"].size() <= 200, true);
  // The buffer goes on from the last kept point to the newest position.
  EXPECT_NEAR(h["buffer"].back()[0].get<double>(), 0.03 * 199, 1e-9);
  EXPECT_EQ(h["segments"][0]["attributes"].dump(), R"({"job_id":"job1","session_id":"s1","blades":false})");
}

TEST_F(TrackTest, TooBigAnAnswerKeepsEveryNthPointWithTheEnds) {
  // A zigzag keeps every point.
  const auto zigzag = [this](MowTrack& track) {
    Session(track, "job1", "s1");
    for (int i = 0; i < 400; ++i) track.Add(0.1 * i, (i % 2) * 0.5);
    track.Close();
  };
  MowTrack whole = Make();
  zigzag(whole);
  const Json full = whole.History(nullptr);
  EXPECT_FALSE(full.contains("thinned"));
  fs::remove_all(dir_);
  MowTrack small = Make(60, 50000000, 3000);
  zigzag(small);
  const Json thin = small.History(nullptr);
  EXPECT_LT(thin.dump().size(), 3000u);
  EXPECT_EQ(thin.value("thinned", false), true);
  const Json& all = full["segments"][0]["points"];
  const Json& kept = thin["segments"][0]["points"];
  ASSERT_GE(kept.size(), 2u);
  EXPECT_LT(kept.size(), all.size());
  EXPECT_EQ(kept.front(), all.front());
  EXPECT_EQ(kept.back(), all.back());
}

TEST_F(TrackTest, CornersStay) {
  MowTrack track = Make();
  Session(track, "job1", "s1");
  for (int i = 0; i < 30; ++i) track.Add(0.05 * i, 0);
  for (int i = 1; i <= 40; ++i) track.Add(1.45, 0.05 * i);
  track.Close();
  const Json points = track.History(nullptr)["segments"][0]["points"];
  EXPECT_LT(points.size(), 8u);
  bool corner = false;
  for (const auto& p : points) corner |= std::hypot(p[0].get<double>() - 1.45, p[1].get<double>()) < 0.08;
  EXPECT_TRUE(corner);
  EXPECT_DOUBLE_EQ(points.back()[1].get<double>(), 2.0);
  // No kept point is farther than the tolerance from the driven line.
  for (const auto& p : points) {
    const double x = p[0].get<double>(), y = p[1].get<double>();
    EXPECT_LT(std::min(std::fabs(y), std::fabs(x - 1.45)), 0.051) << p.dump();
  }
}

TEST_F(TrackTest, NewBladeStateStartsARunFromTheLastPoint) {
  MowTrack track = Make();
  Session(track, "job1", "s1");
  Line(track, 20);
  Blades(track, true);
  Blades(track, true);
  Line(track, 20, 1.0, 1.0);
  track.Close();
  const Json segments = track.History(nullptr)["segments"];
  ASSERT_EQ(segments.size(), 2u);
  EXPECT_EQ(segments[0]["attributes"]["blades"], false);
  EXPECT_EQ(segments[1]["attributes"]["blades"], true);
  EXPECT_EQ(segments[1]["points"][0], segments[0]["points"].back());
}

TEST_F(TrackTest, FileLinesAsTheAppAndOlderFilesHaveThem) {
  MowTrack track = Make();
  Session(track, "job1", "s1");
  track.Save();
  Line(track, 100);
  track.Save();
  Blades(track, true);
  Line(track, 100, 3.0);
  track.Close();
  const auto files = Files();
  ASSERT_EQ(files, (std::vector<std::string>{"1793600000_job1.jsonl"}));
  const auto lines = Lines(files[0]);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[0].dump(), R"({"type":"new_segment","attributes":{"job_id":"job1","session_id":"s1","blades":false},)"
                             R"("started_at":1793600000.0,"points":[]})");
  EXPECT_EQ(lines[1]["type"], "append_points");
  // The rest of the first run before the second one begins.
  EXPECT_EQ(lines[2]["type"], "append_points");
  EXPECT_EQ(lines[3]["type"], "new_segment");
  EXPECT_EQ(lines[3]["attributes"]["blades"], true);
  EXPECT_EQ(lines[3]["points"][0], lines[2]["points"].back());
  // Nothing twice: the file holds exactly the kept points.
  std::size_t in_file = 0;
  for (const auto& l : lines) in_file += l["points"].size();
  EXPECT_EQ(in_file, PointCount(track.History(nullptr)));
}

TEST_F(TrackTest, AfterAStartTheNewestJobGoesOnWithoutSession) {
  {
    MowTrack track = Make();
    Session(track, "job1", "s1");
    Line(track, 100);
    track.Close();
  }
  now_ += 60;
  MowTrack again = Make();
  EXPECT_EQ(again.Attributes().job_id, "job1");
  EXPECT_EQ(again.Attributes().session_id, "");
  Json h = again.History(nullptr);
  ASSERT_EQ(h["segments"].size(), 2u);
  EXPECT_FALSE(h["segments"][0]["points"].empty());
  EXPECT_TRUE(h["segments"][1]["points"].empty());
  again.Add(5, 5);
  EXPECT_TRUE(again.History(nullptr)["buffer"].empty());
  // The next session of the same job records into the same file, as a run of its own.
  Session(again, "job1", "s2");
  Line(again, 30, 0.0, 2.0);
  again.Close();
  EXPECT_EQ(Files().size(), 1u);
  h = again.History(nullptr);
  ASSERT_EQ(h["segments"].size(), 2u);
  EXPECT_EQ(h["segments"][1]["attributes"]["session_id"], "s2");
  MowTrack third = Make();
  EXPECT_EQ(PointCount(third.History(nullptr)), PointCount(h));
}

TEST_F(TrackTest, ANewJobOpensANewFile) {
  MowTrack track = Make();
  Session(track, "job1", "s1");
  Line(track, 20);
  now_ += 100;
  Session(track, "job2", "s2");
  Line(track, 20);
  track.Close();
  EXPECT_EQ(Files(), (std::vector<std::string>{"1793600000_job1.jsonl", "1793600100_job2.jsonl"}));
  EXPECT_EQ(track.List().dump(),
            R"([{"job_id":"job2","timestamp":1793600100},{"job_id":"job1","timestamp":1793600000}])");
  EXPECT_FALSE(track.History(Json{{"job_id", "job1"}})["segments"].empty());
  EXPECT_TRUE(track.History(Json{{"job_id", "job1"}})["buffer"].empty());
  EXPECT_EQ(track.History(Json{{"job_id", "nope"}}).dump(), R"({"error":"not found"})");
  EXPECT_EQ(track.History(Json{{"job_id", "../x"}}).dump(), R"({"error":"not found"})");
}

TEST_F(TrackTest, DeleteNeverTakesTheCurrentJob) {
  MowTrack track = Make();
  Session(track, "job1", "s1");
  Line(track, 20);
  now_ += 100;
  Session(track, "job2", "s2");
  Line(track, 20);
  track.Close();
  EXPECT_EQ(track.Erase(Json{{"job_id", "job2"}}), Deleted(0));
  EXPECT_EQ(track.Erase(Json{{"job_id", "nope"}}), Deleted(0));
  EXPECT_EQ(track.Erase(nullptr), Deleted(1));
  EXPECT_EQ(Files(), (std::vector<std::string>{"1793600100_job2.jsonl"}));
}

TEST_F(TrackTest, OldFilesGoButTheNewestStays) {
  for (const char* job : {"old", "newer"}) {
    MowTrack track = Make();
    Session(track, job, "s");
    Line(track, 20);
    track.Close();
    now_ += 2 * 86400;
  }
  now_ += 86400;
  MowTrack track = Make(2);
  EXPECT_EQ(Files(), (std::vector<std::string>{"1793772800_newer.jsonl"}));
  EXPECT_EQ(track.Attributes().job_id, "newer");
}

TEST_F(TrackTest, OldestFilesGoOverTheByteLimit) {
  for (int job = 0; job < 4; ++job) {
    MowTrack track = Make();
    Session(track, "job" + std::to_string(job), "s");
    Line(track, 40);
    track.Close();
    now_ += 10;
  }
  std::uintmax_t biggest = 0;
  for (const auto& e : fs::directory_iterator(dir_)) biggest = std::max(biggest, e.file_size());
  MowTrack track = Make(60, biggest * 2 + biggest / 2);
  const Json list = track.List();
  ASSERT_EQ(list.size(), 2u);
  EXPECT_EQ(list[0]["job_id"], "job3");
  EXPECT_EQ(list[1]["job_id"], "job2");
}

TEST_F(TrackTest, BrokenEventsAndFilesAreSkipped) {
  fs::create_directories(dir_);
  std::ofstream(dir_ / "garbage.jsonl") << "x\n";
  std::ofstream(dir_ / "x_bad.jsonl") << "x\n";
  std::ofstream(dir_ / "12_job.jsonl") << "{broken\n"
                                        << R"({"type":"append_points","points":[[1,2]]})" << "\n"
                                        << R"({"type":"new_segment","attributes":5})" << "\n"
                                        << R"({"type":"append_points","points":[[1,2],[3],"x",[4,"5"],[6,7]]})" << "\n";
  MowTrack track = Make();
  EXPECT_EQ(track.Attributes().job_id, "job");
  const Json h = track.History(nullptr);
  ASSERT_EQ(h["segments"].size(), 2u);
  EXPECT_EQ(h["segments"][0]["points"].dump(), "[[1.0,2.0],[6.0,7.0]]");
  for (const Json& e : {Json{{"type", "BLADES"}}, Json{{"type", "BLADES"}, {"enabled", "yes"}}, Json::array(),
                        Json{{"type", "STATE"}, {"job_id", 5}, {"session_id", "s"}},
                        Json{{"type", "STATE"}, {"job_id", "../../etc"}, {"session_id", "s"}}}) {
    EXPECT_NO_THROW(track.OnEvent(e, now_)) << e.dump();
  }
  EXPECT_EQ(track.Attributes().job_id, "job");
  track.Add(std::nan(""), 1.0);
  EXPECT_NO_THROW(track.Close());
}

TEST_F(TrackTest, WithoutADirectoryOnlyInMemory) {
  MowTrack track("", 60, 50000000);
  track.Open(now_);
  Session(track, "job1", "s1");
  Line(track, 20);
  track.Close();
  EXPECT_EQ(PointCount(track.History(nullptr)), 2u);
  EXPECT_EQ(track.List(), Json::array());
}
