#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "mowbite_bridge/event_history.h"
#include "mowbite_bridge/log_buffer.h"
#include "mowbite_bridge/monitor.h"

using namespace mowbite_bridge;

TEST(Rpc, ValidRequest) {
  auto r = ParseRpcRequest(R"({"jsonrpc":"2.0","method":"map.replace","params":[{"areas":[]}],"id":"7"})");
  ASSERT_EQ(r.kind, RpcParseResult::Kind::kCall);
  EXPECT_EQ(r.call.method, "map.replace");
  EXPECT_EQ(r.call.id, "7");
  EXPECT_TRUE(r.call.params.is_array());
}

TEST(Rpc, NotificationWithoutId) {
  auto r = ParseRpcRequest(R"({"jsonrpc":"2.0","method":"rpc.ping"})");
  ASSERT_EQ(r.kind, RpcParseResult::Kind::kCall);
  EXPECT_EQ(r.call.id, "");
  EXPECT_TRUE(r.call.params.is_null());
}

TEST(Rpc, Errors) {
  auto code = [](const std::string& payload) {
    auto r = ParseRpcRequest(payload);
    EXPECT_EQ(r.kind, RpcParseResult::Kind::kError) << payload;
    return json::parse(r.error)["error"]["code"].get<int>();
  };
  EXPECT_EQ(code("{nope"), kRpcInvalidJson);
  EXPECT_EQ(code("[1,2]"), kRpcInvalidRequest);
  EXPECT_EQ(code(R"({"jsonrpc":"2.0","method":"x","id":5})"), kRpcInvalidRequest);
  EXPECT_EQ(code(R"({"jsonrpc":"1.0","method":"x","id":"1"})"), kRpcInvalidRequest);
  EXPECT_EQ(code(R"({"jsonrpc":"2.0","id":"1"})"), kRpcInvalidRequest);
  EXPECT_EQ(
      code(R"({"jsonrpc":"2.0","method":"x","id":"1","params":)" + std::string(200, '[') + std::string(200, ']') + "}"),
      kRpcInvalidRequest);
  // A number too large for a double must not end the node.
  EXPECT_EQ(code(R"({"jsonrpc":"2.0","method":"x","id":"1","params":[1e400]})"), kRpcInvalidJson);
}

TEST(Rpc, ErrorKeepsStringId) {
  auto r = ParseRpcRequest(R"({"jsonrpc":"2.0","id":"abc"})");
  EXPECT_EQ(json::parse(r.error)["id"], "abc");
}

TEST(Rpc, MetaAndExtAreIgnored) {
  EXPECT_EQ(ParseRpcRequest(R"({"jsonrpc":"2.0","method":"meta.x","id":"1"})").kind, RpcParseResult::Kind::kIgnore);
  EXPECT_EQ(ParseRpcRequest(R"({"jsonrpc":"2.0","method":"ext.x","id":"1"})").kind, RpcParseResult::Kind::kIgnore);
}

TEST(Rpc, ResultJson) {
  auto ok = json::parse(RpcResultJson("9", R"({"a":1})"));
  EXPECT_EQ(ok["result"]["a"], 1);
  EXPECT_EQ(ok["id"], "9");
  auto bad = json::parse(RpcResultJson("9", "{broken"));
  EXPECT_EQ(bad["error"]["code"], kRpcInternal);
}

TEST(Rpc, NestingSkipsStrings) {
  EXPECT_EQ(JsonNesting(R"({"a":"[[[[","b":[{}]})"), 3u);
  EXPECT_EQ(JsonNesting(R"(["\"[["])"), 1u);
}

TEST(Teleop, DecodesBson) {
  json j = {{"vx", 0.2}, {"vz", -0.1}};
  auto bytes = json::to_bson(j);
  auto t = DecodeTeleop(std::string(bytes.begin(), bytes.end()));
  ASSERT_TRUE(t);
  EXPECT_DOUBLE_EQ(t->first, 0.2);
  EXPECT_DOUBLE_EQ(t->second, -0.1);
  EXPECT_FALSE(DecodeTeleop("garbage"));
  EXPECT_FALSE(DecodeTeleop(std::string(2000, 'x')));
  auto missing = json::to_bson(json{{"vx", 1.0}});
  EXPECT_FALSE(DecodeTeleop(std::string(missing.begin(), missing.end())));
}

TEST(Incoming, RetainedCommandsAreDropped) {
  EXPECT_EQ(ClassifyIncoming("teleop", false), Incoming::kTeleop);
  EXPECT_EQ(ClassifyIncoming("action", false), Incoming::kAction);
  EXPECT_EQ(ClassifyIncoming("rpc/request", false), Incoming::kRpcRequest);
  for (const char* topic : {"teleop", "action", "rpc/request"}) {
    EXPECT_EQ(ClassifyIncoming(topic, true), Incoming::kRetainedCommand) << topic;
  }
  EXPECT_EQ(ClassifyIncoming("map/json", true), Incoming::kOther);
  EXPECT_EQ(ClassifyIncoming("teleop/x", false), Incoming::kOther);
}

TEST(Events, Payload) {
  json e = EventPayload("STATE", {{"state", "IDLE"}}, 12.5);
  EXPECT_EQ(e["type"], "STATE");
  EXPECT_EQ(e["state"], "IDLE");
  EXPECT_DOUBLE_EQ(e["t"].get<double>(), 12.5);
  EXPECT_EQ(e["id"].get<std::string>().size(), 32u);
  EXPECT_NE(EventPayload("A", nullptr, 0)["id"], EventPayload("A", nullptr, 0)["id"]);
}

TEST(Events, HistoryRoundTrip) {
  auto dir = std::filesystem::temp_directory_path() / ("mowbite_events_" + NanoId(8));
  EventHistory history;
  history.init(dir.string());
  history.add(R"({"type":"BOOTED"})");
  history.add("not json");
  EXPECT_EQ(history.getAll().size(), 1u);
  EXPECT_EQ(history.listHistories().size(), 1u);
  EventHistory reloaded;
  reloaded.init(dir.string());
  EXPECT_EQ(reloaded.getAll().size(), 1u);
  EXPECT_EQ(reloaded.deleteHistory(std::nullopt)["deleted"], 1);
  std::filesystem::remove_all(dir);
}

TEST(Events, HistoryKeepsLimits) {
  auto dir = std::filesystem::temp_directory_path() / ("mowbite_events_" + NanoId(8));
  std::filesystem::create_directories(dir);
  for (const char* day : {"20200101", "20200102", "20200103", "20200104"}) {
    std::ofstream(dir / (std::string(day) + ".jsonl")) << R"({"type":"OLD"})"
                                                       << "\n";
  }
  EventHistory history;
  history.init(dir.string(), 3, 1000000);
  history.add(R"({"type":"BOOTED"})");
  auto days = history.listHistories();
  // Today plus the two newest old days.
  ASSERT_EQ(days.size(), 3u);
  EXPECT_EQ(days[1], "20200104");
  EXPECT_EQ(days[2], "20200103");
  EXPECT_FALSE(std::filesystem::exists(dir / "20200101.jsonl"));

  // Today stops growing at the byte limit.
  EventHistory small;
  small.init(dir.string(), 10, 200);
  for (int i = 0; i < 50; i++) small.add(R"({"type":"FILLER"})");
  const std::string today = small.listHistories()[0];
  EXPECT_LE(std::filesystem::file_size(dir / (today + ".jsonl")), 200u);
  std::filesystem::remove_all(dir);
}

TEST(Logs, LevelsAndRedaction) {
  LogBuffer logs(3);
  logs.add(1.0, 20, "n", "info is dropped");
  logs.add(2.0, LogBuffer::WARN, "n", "ntrip://user:secret@caster:2101/MOUNT");
  logs.add(3.0, LogBuffer::ERROR, "n", "password=hunter2 token: abc");
  auto all = logs.get(0, LogBuffer::WARN, 10);
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0]["msg"], "ntrip://***@caster:2101/MOUNT");
  EXPECT_EQ(all[1]["msg"], "password=*** token: ***");
  EXPECT_EQ(all[1]["level"], "ERROR");
  EXPECT_EQ(logs.get(0, LogBuffer::ERROR, 10).size(), 1u);
  EXPECT_EQ(logs.get(2.5, LogBuffer::WARN, 10).size(), 1u);
}

TEST(Sensors, ValueDescriptionNames) {
  EXPECT_STREQ(ValueDescriptionName(4), "VOLTAGE");
  EXPECT_STREQ(ValueDescriptionName(8), "REVOLUTIONS");
  // OpenMower writes DISTANCE as UNKNOWN.
  EXPECT_STREQ(ValueDescriptionName(7), "UNKNOWN");
}

TEST(SystemInfo, FieldsForTheApp) {
  auto info = SystemInfo("1.2.3", "airseekers_tron");
  EXPECT_EQ(info["name"], "mowbite-os");
  EXPECT_EQ(info["version"], "1.2.3");
  EXPECT_EQ(info["platform"], "airseekers_tron");
  EXPECT_EQ(info["schema"], 1);
  EXPECT_EQ(info.size(), 4u);
}

TEST(SystemInfo, PlatformEmptyBeforeTheDriverReports) {
  EXPECT_EQ(SystemInfo("UNKNOWN VERSION", "")["platform"], "");
}
