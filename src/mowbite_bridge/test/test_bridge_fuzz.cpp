#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>

#include "mowbite_bridge/bridge_core.h"
#include "mowbite_bridge/event_log.h"
#include "mowbite_bridge/text.h"
#include "mowbite_bridge/track.h"

using namespace mowbite_bridge;
namespace fs = std::filesystem;

namespace {

// Valid payloads to start the mangling from. Ids, topics and nodes come from small sets, as they do from the nodes.
std::vector<std::string> Seeds() {
  const auto bson = Json::to_bson(Json{{"vx", 0.2}, {"vz", -0.1}});
  return {R"({"jsonrpc":"2.0","method":"rpc.ping","id":"1"})",
          R"({"jsonrpc":"2.0","method":"sensors.list","params":{"ids":["a","b"]},"id":"2"})",
          R"({"jsonrpc":"2.0","method":"logs.recent","params":{"since":1,"level":"ERROR","limit":3},"id":"3"})",
          R"({"jsonrpc":"2.0","method":"params.set","params":[{"a":1}],"id":"4"})",
          R"({"node":"/n","params":{"a":1,"password":"x"}})",
          R"({"id":"a","t":1.0,"type":"STATE","job_id":"j","session_id":"s"})",
          R"({"exit_code":-9,"restart":1,"max_restarts":3})",
          std::string(bson.begin(), bson.end()),
          "ntrip://user:secret@caster:2101/MOUNT password=x token: y"};
}

class Mangler {
 public:
  explicit Mangler(unsigned seed) : rng_(seed), seeds_(Seeds()) {}

  std::string Next() {
    std::string s;
    switch (Pick(6)) {
      case 0:  // random bytes
        s.resize(Pick(300));
        for (char& c : s) c = static_cast<char>(Pick(256));
        return s;
      case 1:  // deep nesting
        return std::string(Pick(5000), "[{"[Pick(2)]) + "1";
      case 2:  // a seed cut off
        s = seeds_[Pick(seeds_.size())];
        return s.substr(0, Pick(s.size() + 1));
      default:  // a seed with a few bytes changed, added or dropped
        s = seeds_[Pick(seeds_.size())];
        for (std::size_t n = 1 + Pick(4); n > 0 && !s.empty(); --n) {
          const std::size_t at = Pick(s.size());
          switch (Pick(3)) {
            case 0:
              s[at] = static_cast<char>(Pick(256));
              break;
            case 1:
              s.insert(at, 1, "{}[]\":,\\\xff\xe4\x00 1e999"[Pick(15)]);
              break;
            default:
              s.erase(at, 1);
          }
        }
        return s;
    }
  }

  std::size_t Pick(std::size_t n) {
    return n == 0 ? 0 : std::uniform_int_distribution<std::size_t>(0, n - 1)(rng_);
  }

 private:
  std::mt19937 rng_;
  std::vector<std::string> seeds_;
};

int Rounds(int normal) {
  const char* wanted = std::getenv("MOWBITE_FUZZ_ROUNDS");
  return wanted ? std::atoi(wanted) : normal;
}

// Parses a mangled text when it is JSON, otherwise keeps it as a string. Like every input that reaches the stores it
// is valid UTF-8: events and params come parsed.
Json Loose(const std::string& text) {
  const Json j = Json::parse(text, nullptr, false);
  return j.is_discarded() ? Json(CleanUtf8(text)) : j;
}

}  // namespace

// B23: no input ends the process or gives text that is not JSON or not UTF-8.
TEST(BridgeFuzz, NoInputThrowsAndEveryAnswerIsJson) {
  BridgeCore core({"v", "{}"});
  core.AddLocalMethod("x.throws", [](const Json& p) -> Json {
    if (p.is_null()) throw std::runtime_error("x");
    return p;
  });
  core.SetEventSink([](const Json& e) { (void)e.dump(-1, ' ', false, Json::error_handler_t::replace); });
  core.Start(1.0);
  core.Connected();
  core.OnRpcMethods("node", {"params.set"});
  Mangler m(20261102);
  std::size_t answers = 0;
  // 20000 rounds in the normal run; MOWBITE_FUZZ_ROUNDS for the long one with sanitizers.
  const int rounds = Rounds(20000);
  for (int i = 0; i < rounds; ++i) {
    const std::string a = m.Next();
    const std::string b = m.Next();
    ASSERT_NO_THROW({
      switch (m.Pick(12)) {
        case 0:
          core.OnMqtt("rpc/request", a, false);
          break;
        case 1:
          core.OnMqtt("teleop", a, false);
          break;
        case 2:
          core.OnMqtt("action", a, m.Pick(2) == 0);
          break;
        case 3:
          core.OnParams(a);
          break;
        case 4:
          core.OnPublish(m.Pick(2) ? "events/json" : "t" + std::to_string(m.Pick(8)), a, m.Pick(2) == 0);
          break;
        case 5:
          core.OnRpcResponse(b, a);
          break;
        case 6:
          core.OnRpcError(b, static_cast<int>(m.Pick(70000)) - 35000, b, a);
          break;
        case 7:
          core.OnLog(i, static_cast<int>(m.Pick(60)), b, a);
          break;
        case 8: {
          SensorDescription s;
          s.id = "s" + std::to_string(m.Pick(8));
          s.name = b;
          s.unit = a;
          core.OnSensor(s);
          core.OnSensorValue(s.id, m.Pick(2) ? Json(a) : Json(static_cast<double>(i)), i);
          break;
        }
        case 9:
          core.OnActions("p" + std::to_string(m.Pick(4)), {{a, b, m.Pick(2) == 0}});
          break;
        case 10:
          core.OnMap(a);
          core.OnRpcMethods("n" + std::to_string(m.Pick(4)), {a});
          break;
        default:
          (void)RestartWarning(a);
          // logs.recent with a limit: a full ring of 1000 long lines would make each answer about 300 kB.
          core.OnMqtt("rpc/request",
                      Json{{"jsonrpc", "2.0"},
                           {"method", m.Pick(2) ? "x.throws" : "logs.recent"},
                           {"params", m.Pick(2) ? Json(a) : Json{{"limit", m.Pick(5)}, {"level", a}, {"since", b}}},
                           {"id", "f"}}
                          .dump(-1, ' ', false, Json::error_handler_t::replace),
                      false);
      }
    });
    for (const auto& msg : core.Take().mqtt) {
      if (msg.topic != "rpc/response") continue;
      ++answers;
      ASSERT_EQ(CleanUtf8(msg.payload), msg.payload);
      const Json answer = Json::parse(msg.payload, nullptr, false);
      ASSERT_TRUE(answer.is_object()) << msg.payload;
      ASSERT_EQ(answer["jsonrpc"], "2.0");
      ASSERT_TRUE(answer.contains("result") != answer.contains("error")) << msg.payload;
    }
  }
  EXPECT_GT(answers, static_cast<std::size_t>(rounds / 10));
  // Still answers.
  core.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"rpc.ping","id":"end"})", false);
  const Outbox out = core.Take();
  ASSERT_FALSE(out.mqtt.empty());
  EXPECT_EQ(out.mqtt.back().payload, R"({"jsonrpc":"2.0","result":"pong","id":"end"})");
}

// B18, B20, B23: event history and track take any event, position and request, and broken files, without throwing;
// every answer is JSON.
TEST(BridgeFuzz, HistoryAndTrackTakeAnyInput) {
  const fs::path dir = fs::temp_directory_path() / ("mowbite_fuzz_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  const std::string events_dir = (dir / "event_history").string();
  const std::string track_dir = (dir / "position_history").string();
  // Small limits, so trimming runs often.
  EventLog events(events_dir, 3, 20000);
  auto track = std::make_unique<MowTrack>(track_dir, 2.0, 30000);
  track->Open(0.0);
  Mangler m(20261103);
  const std::vector<std::string> ids = {"j1", "j2", "../x", "", "a b", std::string(300, 'k'), "j\xc3\xa4"};
  // Up to a few days apart, so day files and old jobs come and go.
  double now = 1.7e9;
  const int rounds = Rounds(4000);
  for (int i = 0; i < rounds; ++i) {
    now += static_cast<double>(m.Pick(4000));
    const std::string a = m.Next();
    // In a lambda: the preprocessor splits macro arguments at the commas of braced lists.
    const auto step = [&] {
      switch (m.Pick(10)) {
        case 0:
        case 1: {
          Json event = Json{{"id", "e"}, {"t", now}, {"type", m.Pick(3) ? "STATE" : "BLADES"}};
          if (m.Pick(4)) event["job_id"] = ids[m.Pick(ids.size())];
          if (m.Pick(4)) event["session_id"] = m.Pick(3) ? Json(ids[m.Pick(2)]) : Loose(a);
          if (m.Pick(2)) event["enabled"] = m.Pick(3) ? Json(m.Pick(2) == 0) : Loose(a);
          events.Write(event, now);
          track->OnEvent(event, now);
          break;
        }
        case 2: {
          const Json event = Loose(a);
          events.Write(event, now);
          track->OnEvent(event, now);
          break;
        }
        case 3:
          for (std::size_t n = m.Pick(200); n > 0; --n) {
            track->Add(static_cast<double>(m.Pick(2000)) * 0.01, static_cast<double>(m.Pick(2000)) * 0.01);
          }
          break;
        case 4:
          track->Save();
          break;
        case 5:
          (void)events.Read(Loose(a), now).dump();
          (void)events.List().dump();
          (void)track->History(m.Pick(2) ? Json{{"job_id", ids[m.Pick(ids.size())]}} : Loose(a)).dump();
          (void)track->List().dump();
          break;
        case 6:
          if (m.Pick(4) == 0) (void)events.Erase(Loose(a));
          if (m.Pick(4) == 0) (void)track->Erase(m.Pick(2) ? Json{{"job_id", ids[m.Pick(ids.size())]}} : Loose(a));
          break;
        case 7: {
          // A broken line in some file of either store.
          for (const std::string& d : {events_dir, track_dir}) {
            if (!fs::is_directory(d)) continue;
            for (const auto& entry : fs::directory_iterator(d)) {
              if (m.Pick(3) == 0) std::ofstream(entry.path(), std::ios::app) << a << "\n";
            }
          }
          break;
        }
        case 8:
          if (m.Pick(20) == 0) {
            // A restart: the newest file is read again.
            track->Close();
            track = std::make_unique<MowTrack>(track_dir, 2.0, 30000);
            track->Open(now);
          }
          break;
        default:
          (void)track->Attributes();
      }
    };
    ASSERT_NO_THROW(step());
  }
  ASSERT_NO_THROW(track->Close());
  fs::remove_all(dir);
}
