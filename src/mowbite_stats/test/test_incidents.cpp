#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "mowbite_journal/format.h"
#include "mowbite_journal/journal.h"
#include "mowbite_journal/reader.h"
#include "mowbite_stats/incident_store.h"
#include "mowbite_stats/incidents.h"

namespace fs = std::filesystem;

namespace mowbite_stats {
namespace {

constexpr int64_t kS = 1000000000;
// In raw bytes of the journal, as a password of the NTRIP caster would be: only the archive with location has it.
constexpr char kDecoy[] = "decoy-ntrip-password-5f2a";

void Wait(double seconds) {
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

std::string FromBase64(const std::string& text) {
  static const std::string kDigits = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  uint32_t bits = 0;
  int count = 0;
  for (const char c : text) {
    if (c == '=') break;
    bits = bits << 6 | static_cast<uint32_t>(kDigits.find(c));
    count += 6;
    if (count >= 8) {
      count -= 8;
      out += static_cast<char>((bits >> count) & 0xFF);
    }
  }
  return out;
}

// The bytes of a ustar archive as they would be read: the json files as they are, the segments unpacked.
std::string Unpacked(const std::string& archive, const fs::path& scratch) {
  std::string out;
  fs::create_directories(scratch);
  for (size_t at = 0; at + 512 <= archive.size();) {
    const std::string name = archive.substr(at, 100).c_str();
    if (name.empty()) break;
    const uint64_t size = std::stoull(archive.substr(at + 124, 11), nullptr, 8);
    const std::string data = archive.substr(at + 512, size);
    out += name + "\n";
    if (name.size() > 5 && name.find(".json") != std::string::npos) {
      out += data;
    } else {
      const fs::path file = scratch / "segment";
      std::ofstream(file, std::ios::binary).write(data.data(), static_cast<std::streamsize>(data.size()));
      mowbite_journal::Segment segment;
      std::string error;
      EXPECT_TRUE(mowbite_journal::ReadSegment(file.string(), &segment, &error)) << name << ": " << error;
      out.append(segment.raw.begin(), segment.raw.end());
    }
    at += 512 + (size + 511) / 512 * 512;
  }
  return out;
}

std::string Today(const char* format) {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char text[32];
  std::strftime(text, sizeof(text), format, &tm);
  return text;
}

class IncidentsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("incidents_test_" + std::to_string(getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    options_.dir = dir_.string();
    options_.platform = "sim";
    options_.reserve_mb = 0;
    options_.flush_s = 0.05;
    config_.journal_dir = dir_.string();
    config_.reserve_mb = 0;
    config_.boot_id = BootId();
  }
  void TearDown() override {
    incidents_.reset();
    fs::remove_all(dir_);
  }

  std::unique_ptr<Incidents>& Make() {
    incidents_ = std::make_unique<Incidents>(
        config_, [this](const std::string&) { return free_; }, [this](const std::string& why) { warnings_.push_back(why); });
    return incidents_;
  }

  // A journal that writes the decoy as raw bytes for the given time.
  static void Feed(mowbite_journal::Journal& journal, double seconds) {
    const uint16_t stream = journal.Define({{"name", "port.rx"}, {"kind", "raw"}});
    const std::string bytes = kDecoy;
    for (int i = 0; i < static_cast<int>(seconds / 0.02); ++i) {
      journal.Append(stream, mowbite_journal::kRawBytes, reinterpret_cast<const uint8_t*>(bytes.data()),
                     bytes.size());
      Wait(0.02);
    }
  }

  std::unique_ptr<mowbite_journal::Journal> Open() {
    std::string error;
    auto journal = mowbite_journal::Journal::Create(options_, &error);
    EXPECT_TRUE(journal) << error;
    return journal;
  }

  // Ticks with the clock moved on far enough for the pass over the runs, then waits for the worker.
  void Pass() {
    now_ += 11 * kS;
    incidents_->Tick(now_, Wall());
    incidents_->Drain();
    incidents_->Tick(now_, Wall());
  }

  // An incident sealed from a journal that ended: its id.
  std::string Sealed() {
    auto journal = Open();
    Feed(*journal, 0.3);
    now_ = mowbite_journal::MonotonicNs();
    Make();
    const auto notices = incidents_->OnEmergency(5, now_, Wall());
    EXPECT_EQ(notices.size(), 1u);
    if (notices.empty()) return {};
    const std::string id = notices[0]["incident"];
    Feed(*journal, 0.2);
    journal.reset();
    Pass();
    Pass();
    return id;
  }

  json Call(const std::string& method, const json& params, std::optional<RpcError>* error = nullptr) {
    Incidents::Notices notices;
    RpcOutcome out = incidents_->Handle(method, params, now_, Wall(), &notices);
    if (error) *error = out.error;
    EXPECT_TRUE(error || !out.error) << method << ": " << out.error->message;
    return out.result;
  }

  // The whole archive in pieces of piece bytes, after waiting for it to be made.
  std::string Download(const std::string& id, bool location, size_t piece, std::string* sha) {
    const json first = Call("incidents.download", {{"id", id}, {"location", location}, {"size", piece}});
    if (!first["ready"].get<bool>()) incidents_->Drain();
    std::string archive;
    for (;;) {
      const json answer =
          Call("incidents.download", {{"id", id}, {"location", location}, {"offset", archive.size()}, {"size", piece}});
      EXPECT_TRUE(answer["ready"].get<bool>());
      const std::string data = FromBase64(answer["data"]);
      EXPECT_EQ(answer["sha256"], Sha256Hex(data));
      *sha = answer["archive_sha256"];
      if (data.empty()) {
        EXPECT_EQ(archive.size(), answer["size"].get<uint64_t>());
        return archive;
      }
      archive += data;
    }
  }

  static double Wall() {
    return static_cast<double>(std::time(nullptr));
  }

  fs::path dir_;
  mowbite_journal::Options options_;
  IncidentsConfig config_;
  int64_t free_ = -1;
  int64_t now_ = 0;
  std::vector<std::string> warnings_;
  std::unique_ptr<Incidents> incidents_;
};

TEST_F(IncidentsTest, AnEmergencyKeepsTheSegmentsOfItsWindowAndSealsThemWhenTheyAreClosed) {
  auto journal = Open();
  Feed(*journal, 0.3);
  now_ = mowbite_journal::MonotonicNs();
  Make();
  const auto notices = incidents_->OnEmergency(5, now_, Wall());
  ASSERT_EQ(notices.size(), 1u);
  EXPECT_EQ(notices[0]["reason"], "EMERGENCY");
  EXPECT_EQ(notices[0]["detail"], "LATCH, STOP");
  EXPECT_TRUE(notices[0]["safety"].get<bool>());
  const std::string id = notices[0]["incident"];
  // A further bit grows the same incident without a new event; the same bits again are nothing.
  EXPECT_TRUE(incidents_->OnEmergency(7, now_ + kS, Wall()).empty());
  EXPECT_TRUE(incidents_->OnEmergency(7, now_ + kS, Wall()).empty());

  json list = Call("incidents.list", nullptr);
  ASSERT_EQ(list.size(), 1u);
  EXPECT_EQ(list[0]["state"], "collecting");
  // The window is over, but the journal still writes the linked segment.
  Pass();
  Pass();
  EXPECT_EQ(Call("incidents.list", nullptr)[0]["state"], "sealing");
  journal.reset();
  Pass();
  Pass();
  list = Call("incidents.list", nullptr);
  ASSERT_EQ(list.size(), 1u);
  EXPECT_EQ(list[0]["state"], "done");
  EXPECT_EQ(list[0]["triggers"], json::array({"EMERGENCY"}));
  EXPECT_EQ(list[0]["replay"], "from_start");
  EXPECT_GT(list[0]["bytes"].get<uint64_t>(), 0u);

  const json got = Call("incidents.get", {{"id", id}});
  const json& manifest = got["manifest"];
  EXPECT_EQ(manifest["state"], "done");
  ASSERT_EQ(manifest["triggers"].size(), 2u);
  ASSERT_EQ(manifest["runs"].size(), 1u);
  const json& run = manifest["runs"][0];
  EXPECT_TRUE(run["ended"].get<bool>());
  ASSERT_FALSE(run["files"].empty());
  EXPECT_EQ(run["files"][0]["sha256"].get<std::string>().size(), 64u);
  EXPECT_TRUE(got["timeline"].is_array());
  EXPECT_FALSE(got["timeline"].empty());
  EXPECT_TRUE(warnings_.empty()) << warnings_.front();
}

TEST_F(IncidentsTest, TheArchiveWithoutLocationHasNoRawBytesNoBootIdAndNoRealDate) {
  const std::string id = Sealed();
  ASSERT_FALSE(id.empty());
  // The first call starts making the archive.
  EXPECT_FALSE(Call("incidents.download", {{"id", id}})["ready"].get<bool>());
  std::string sha_place, sha_none;
  const std::string with = Download(id, true, 4096, &sha_place);
  const std::string without = Download(id, false, 100000, &sha_none);
  EXPECT_EQ(Sha256Hex(with), sha_place);
  EXPECT_EQ(Sha256Hex(without), sha_none);
  const std::string place = Unpacked(with, dir_ / "with");
  const std::string none = Unpacked(without, dir_ / "without");
  EXPECT_NE(place.find(kDecoy), std::string::npos);
  EXPECT_NE(place.find(config_.boot_id), std::string::npos);
  EXPECT_NE(place.find(Today("%Y%m%d")), std::string::npos);
  EXPECT_NE(none.find("manifest.json"), std::string::npos);
  EXPECT_EQ(none.find(kDecoy), std::string::npos);
  EXPECT_EQ(none.find(config_.boot_id), std::string::npos);
  EXPECT_EQ(none.find(Today("%Y%m%d")), std::string::npos);
  EXPECT_EQ(none.find(Today("%Y-%m-%d")), std::string::npos);
  // The same archive again gives the same bytes.
  std::string again;
  EXPECT_EQ(Download(id, false, 65536, &again).size(), without.size());
  EXPECT_EQ(again, sha_none);
}

TEST_F(IncidentsTest, ARunWithoutItsEndIsACrashOnce) {
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    std::string error;
    auto journal = mowbite_journal::Journal::Create(options_, &error);
    if (journal) Feed(*journal, 0.3);
    _exit(0);
  }
  int status = 0;
  waitpid(child, &status, 0);
  now_ = mowbite_journal::MonotonicNs();
  Make();
  Incidents::Notices notices = incidents_->Tick(now_, Wall());
  ASSERT_EQ(notices.size(), 1u);
  EXPECT_EQ(notices[0]["reason"], "CRASH");
  EXPECT_TRUE(notices[0]["safety"].get<bool>());
  const std::string id = notices[0]["incident"];
  Pass();
  Pass();
  const json manifest = Call("incidents.get", {{"id", id}})["manifest"];
  EXPECT_EQ(manifest["state"], "done");
  ASSERT_EQ(manifest["runs"].size(), 1u);
  EXPECT_FALSE(manifest["runs"][0]["ended"].get<bool>());
  EXPECT_FALSE(manifest["runs"][0]["files"].empty());
  // A new process of the stats knows the run already.
  Make();
  EXPECT_TRUE(incidents_->Tick(now_ + 100 * kS, Wall()).empty());
}

TEST_F(IncidentsTest, AnEndOfTheDriverInTheNoteOfTheLaunchIsACrashOnce) {
  config_.driver_note = (dir_ / "driver_exit.json").string();
  const auto write = [this](double t, int restart) {
    std::ofstream(config_.driver_note) << json{{"name", "xcore_driver"}, {"exit_code", -9}, {"restart", restart},
                                               {"down", false}, {"t", t}}.dump();
  };
  now_ = mowbite_journal::MonotonicNs();
  // An old note at the start of the stats tells nothing.
  write(Wall() - 2 * kIncidentCrashBefore, 1);
  Make();
  EXPECT_TRUE(incidents_->Tick(now_, Wall()).empty());
  write(Wall(), 2);
  now_ += 20 * kS;
  Incidents::Notices notices = incidents_->Tick(now_, Wall());
  ASSERT_EQ(notices.size(), 1u);
  EXPECT_EQ(notices[0]["reason"], "CRASH");
  EXPECT_EQ(notices[0]["detail"], "xcore_driver");
  EXPECT_TRUE(notices[0]["safety"].get<bool>());
  const std::string id = notices[0]["incident"];
  const json trigger = Call("incidents.get", {{"id", id}})["manifest"]["triggers"][0];
  EXPECT_EQ(trigger["detail"]["exit_code"], -9);
  EXPECT_EQ(trigger["detail"]["restart"], 2);
  // The same note again, also for a new process of the stats, opens nothing; nor does one that cannot be read.
  now_ += 20 * kS;
  EXPECT_TRUE(incidents_->Tick(now_, Wall()).empty());
  Make();
  EXPECT_TRUE(incidents_->Tick(now_ + 20 * kS, Wall()).empty());
  std::ofstream(config_.driver_note) << "{";
  EXPECT_TRUE(incidents_->Tick(now_ + 40 * kS, Wall()).empty());
}

TEST_F(IncidentsTest, MarksAreThrottledAndChecked) {
  auto journal = Open();
  Feed(*journal, 0.1);
  now_ = mowbite_journal::MonotonicNs();
  Make();
  const json mark = Call("incidents.mark", {{"note", "it stopped in the middle"}});
  const std::string id = mark["id"];
  EXPECT_FALSE(id.empty());
  // A second mark in the open window goes into the same incident.
  EXPECT_EQ(Call("incidents.mark", {{"note", "again"}})["id"], id);
  std::optional<RpcError> error;
  Call("incidents.mark", {{"note", std::string(501, 'x')}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["reason"], "range");
  // Umlauts count as one character each.
  std::string umlauts;
  for (int i = 0; i < 500; ++i) umlauts += "\xc3\xa4";
  Call("incidents.mark", {{"note", umlauts}}, &error);
  EXPECT_FALSE(error);
  // After the window a further mark within kind_gap_s is throttled.
  now_ += 20 * kS;
  incidents_->Tick(now_, Wall());
  Call("incidents.mark", {{"note", "third"}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["reason"], "throttled");
  const json status = Call("journal.status", nullptr);
  EXPECT_EQ(status["incidents"]["throttled"][0]["kind"], "MARK");
}

TEST_F(IncidentsTest, DeleteOnlyWhatIsDone) {
  const std::string id = Sealed();
  ASSERT_FALSE(id.empty());
  const auto notices = incidents_->OnEvent(json{{"type", "DOCKING_FAILED"}, {"reason", "dock_failed"}}, now_, Wall());
  ASSERT_EQ(notices.size(), 1u);
  const std::string open = notices[0]["incident"];
  std::optional<RpcError> error;
  Call("incidents.delete", {{"id", open}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["reason"], "state");
  Call("incidents.delete", {{"id", "20200101T000000Z"}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["reason"], "unknown");
  Call("incidents.delete", {{"id", "../x"}}, &error);
  ASSERT_TRUE(error);
  // Without an id all that are done; the open one stays.
  EXPECT_EQ(Call("incidents.delete", nullptr)["deleted"], 1);
  const json list = Call("incidents.list", nullptr);
  ASSERT_EQ(list.size(), 1u);
  EXPECT_EQ(list[0]["id"], open);
}

TEST_F(IncidentsTest, NoIncidentBelowTheReserveOrWithTheJournalOff) {
  auto journal = Open();
  Feed(*journal, 0.1);
  now_ = mowbite_journal::MonotonicNs();
  config_.reserve_mb = 10;
  free_ = 5000000;
  Make();
  EXPECT_TRUE(incidents_->OnEmergency(4, now_, Wall()).empty());
  json status = Call("journal.status", nullptr);
  EXPECT_TRUE(status["enabled"].get<bool>());
  EXPECT_NE(status["problem"].get<std::string>().find("journal_reserve_mb"), std::string::npos);
  free_ = 50000000;
  incidents_->SetJournalEnabled(false);
  EXPECT_TRUE(fs::exists(dir_ / mowbite_journal::kDisabledFile));
  incidents_->OnEmergency(0, now_, Wall());
  EXPECT_TRUE(incidents_->OnEmergency(4, now_, Wall()).empty());
  status = Call("journal.status", nullptr);
  EXPECT_FALSE(status["enabled"].get<bool>());
  incidents_->SetJournalEnabled(true);
  EXPECT_FALSE(fs::exists(dir_ / mowbite_journal::kDisabledFile));
  incidents_->OnEmergency(0, now_, Wall());
  EXPECT_EQ(incidents_->OnEmergency(4, now_, Wall()).size(), 1u);
}

TEST_F(IncidentsTest, ParamsAreChecked) {
  Make();
  std::optional<RpcError> error;
  Call("incidents.get", {{"id", 3}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["reason"], "type");
  Call("incidents.download", {{"id", "x"}, {"size", 262145}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["reason"], "range");
  Call("incidents.download", {{"id", "x"}, {"offset", -1}}, &error);
  ASSERT_TRUE(error);
  Call("journal.status", {{"x", 1}}, &error);
  ASSERT_TRUE(error);
  Call("incidents.get", {{"id", "x"}, {"client", "Bad Client"}}, &error);
  ASSERT_TRUE(error);
  EXPECT_EQ(error->data["field"], "client");
}

TEST(AnonymizeOptions, ShiftNameMovesTheLeadingDate) {
  mowbite_journal::AnonymizeOptions options;
  options.shift_days = -7;
  EXPECT_EQ(options.ShiftName("20261115T121314Z"), "20261108T121314Z");
  EXPECT_EQ(options.ShiftName("20260301T000000Z_logic_12"), "20260222T000000Z_logic_12");
  EXPECT_EQ(options.ShiftName("seen.json"), "seen.json");
  EXPECT_EQ(options.ShiftName("20261399T"), "20261399T");
}

}  // namespace
}  // namespace mowbite_stats
