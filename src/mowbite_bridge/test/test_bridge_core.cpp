#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/bridge_core.h"
#include "mowbite_bridge/event_builder.h"

using namespace mowbite_bridge;

namespace {

std::vector<std::string> Topics(const Outbox& out) {
  std::vector<std::string> topics;
  for (const auto& m : out.mqtt) topics.push_back(m.topic);
  return topics;
}

const MqttMessage* Find(const Outbox& out, const std::string& topic) {
  for (const auto& m : out.mqtt) {
    if (m.topic == topic) return &m;
  }
  return nullptr;
}

// The one answer on rpc/request.
Json Ask(BridgeCore& core, const std::string& request) {
  core.Take();
  core.OnMqtt("rpc/request", request, false);
  const Outbox out = core.Take();
  EXPECT_EQ(out.mqtt.size(), 1u) << request;
  if (out.mqtt.size() != 1) return nullptr;
  EXPECT_EQ(out.mqtt[0].topic, "rpc/response");
  EXPECT_FALSE(out.mqtt[0].retain);
  EXPECT_EQ(out.mqtt[0].qos, 0);
  return Json::parse(out.mqtt[0].payload);
}

Json Call(BridgeCore& core, const std::string& method, const Json& params = nullptr, const std::string& id = "1") {
  Json request = {{"jsonrpc", "2.0"}, {"method", method}, {"id", id}};
  if (!params.is_null()) request["params"] = params;
  return Ask(core, request.dump());
}

class Core : public ::testing::Test {
 protected:
  Core() : core_(BridgeConfig{"1.2.3", "{}"}) {
    core_.Connected();
    core_.Take();
  }
  BridgeCore core_;
};

}  // namespace

// B1
TEST(BridgeCoreConnection, LogsConnectAndLossOnce) {
  BridgeCore core({});
  core.ConnectionLost();
  EXPECT_TRUE(core.Take().logs.empty());
  core.Connected();
  core.ConnectionLost();
  core.ConnectionLost();
  const Outbox out = core.Take();
  ASSERT_EQ(out.logs.size(), 2u);
  EXPECT_EQ(out.logs[0].level, kLogInfo);
  EXPECT_EQ(out.logs[0].text, "MQTT connected");
  EXPECT_EQ(out.logs[1].level, kLogWarn);
  EXPECT_EQ(out.logs[1].text, "MQTT connection lost");
  EXPECT_FALSE(core.connected());
}

TEST(BridgeCoreConnection, NothingIsSentWithoutConnectionAndTheRestGoesOn) {
  BridgeCore core({});
  RobotStateView state;
  core.OnRobotState(state);
  core.OnSensorValue("v", 1.0, 1.0);
  core.OnPublish("events/json", R"({"type":"X"})", false);
  core.OnMqtt("action", "a/b", false);
  const Outbox out = core.Take();
  EXPECT_TRUE(out.mqtt.empty());
  EXPECT_EQ(out.actions.size(), 1u);
}

// B2, B3, B16
TEST(BridgeCoreConnection, AfterEachConnectTheRetainedTextsInTheirOrder) {
  BridgeCore core({"", R"({"/a/b":1})"});
  EXPECT_EQ(BridgeCore::CommandTopics(),
            (std::vector<std::string>{"action", "teleop", "rpc/request", "mowbite/host/status"}));
  core.Connected();
  Outbox out = core.Take();
  // Without sensors and map: none of them.
  EXPECT_EQ(Topics(out), (std::vector<std::string>{"capabilities/json", "version/json", "params/json", "actions/json",
                                                   "mowbite/bridge/status"}));
  for (const auto& m : out.mqtt) {
    EXPECT_TRUE(m.retain);
    EXPECT_EQ(m.qos, 1);
  }
  EXPECT_EQ(Find(out, "version/json")->payload, R"({"version":"UNKNOWN VERSION"})");
  EXPECT_EQ(Find(out, "params/json")->payload, R"({"/a/b":1})");
  EXPECT_EQ(Find(out, "actions/json")->payload, "[]");

  core.OnPublish("map_overlay/json", R"({"o":1})", true);
  core.OnMap(R"({"areas":[]})");
  SensorDescription sensor;
  sensor.id = "v";
  core.OnSensor(sensor);
  core.OnPublish("other/json", "x", true);
  core.ConnectionLost();
  core.Take();
  core.Connected();
  out = core.Take();
  // E3 (i): what nodes sent retained comes again too.
  EXPECT_EQ(Topics(out), (std::vector<std::string>{"capabilities/json", "version/json", "params/json",
                                                   "sensor_infos/json", "actions/json", "map/json",
                                                   "mowbite/bridge/status", "map_overlay/json", "other/json"}));
  EXPECT_EQ(Find(out, "mowbite/bridge/status")->payload, R"({"online":true})");
  EXPECT_EQ(Find(out, "map_overlay/json")->payload, R"({"o":1})");
  EXPECT_EQ(Find(out, "map/json")->payload, R"({"areas":[]})");
}

TEST(BridgeCoreConnection, AnEmptyRetainedTextClearsAndIsNotSentAgain) {
  BridgeCore core({});
  core.Connected();
  core.OnPublish("map_overlay/json", "x", true);
  core.OnPublish("map_overlay/json", "", true);
  const Outbox live = core.Take();
  ASSERT_TRUE(Find(live, "map_overlay/json"));
  EXPECT_EQ(live.mqtt.back().payload, "");
  EXPECT_TRUE(live.mqtt.back().retain);
  core.ConnectionLost();
  core.Connected();
  EXPECT_FALSE(Find(core.Take(), "map_overlay/json"));
}

// B18 (start and end), E3 (ii)
TEST(BridgeCoreConnection, BootedIntoTheSinkAtOnceAndLiveOnceAfterTheFirstConnect) {
  BridgeCore core({});
  std::vector<Json> sunk;
  core.SetEventSink([&](const Json& e) { sunk.push_back(e); });
  core.Start(100.0);
  ASSERT_EQ(sunk.size(), 1u);
  EXPECT_EQ(sunk[0]["type"], "BOOTED");
  EXPECT_DOUBLE_EQ(sunk[0]["t"].get<double>(), 100.0);
  EXPECT_TRUE(core.Take().mqtt.empty());
  core.Connected();
  Outbox out = core.Take();
  ASSERT_FALSE(out.mqtt.empty());
  EXPECT_EQ(out.mqtt.back().topic, "events/json");
  EXPECT_FALSE(out.mqtt.back().retain);
  EXPECT_EQ(Json::parse(out.mqtt.back().payload), sunk[0]);
  core.ConnectionLost();
  core.Connected();
  EXPECT_FALSE(Find(core.Take(), "events/json"));

  core.Stop(200.0);
  out = core.Take();
  ASSERT_TRUE(Find(out, "events/json"));
  const Json shutdown = Json::parse(Find(out, "events/json")->payload);
  EXPECT_EQ(shutdown["type"], "SHUTDOWN");
  ASSERT_EQ(sunk.size(), 2u);
  EXPECT_EQ(sunk[1], shutdown);
  // Both as the catalog of the interface has them.
  for (const auto& event : sunk) {
    EXPECT_TRUE(mowbite_api::CheckEvent(event["type"].get<std::string>(), event).empty()) << event.dump();
  }
}

// B4
TEST_F(Core, CommandsTheBrokerKeptAreDropped) {
  for (const char* topic : {"action", "teleop", "rpc/request"}) {
    core_.OnMqtt(topic, R"({"jsonrpc":"2.0","method":"rpc.ping","id":"1"})", true);
  }
  core_.OnMqtt("map/json", "{}", false);
  core_.OnMqtt("teleop/x", "{}", false);
  const Outbox out = core_.Take();
  EXPECT_TRUE(out.mqtt.empty());
  EXPECT_TRUE(out.actions.empty());
  EXPECT_TRUE(out.teleop.empty());
}

// B5, B6
TEST_F(Core, ActionTextAndTeleopGoToTheNodes) {
  core_.OnMqtt("action", "mower_logic/o19b_probe_unknown", false);
  core_.OnMqtt("action", "bad \xe4", false);
  const auto bson = Json::to_bson(Json{{"vx", 0.12}, {"vz", -0.05}});
  core_.OnMqtt("teleop", std::string(bson.begin(), bson.end()), false);
  core_.OnMqtt("teleop", "garbage", false);
  const Outbox out = core_.Take();
  EXPECT_EQ(out.actions, (std::vector<std::string>{"mower_logic/o19b_probe_unknown", "bad \xef\xbf\xbd"}));
  ASSERT_EQ(out.teleop.size(), 1u);
  EXPECT_DOUBLE_EQ(out.teleop[0].vx, 0.12);
  EXPECT_DOUBLE_EQ(out.teleop[0].vz, -0.05);
  EXPECT_TRUE(out.mqtt.empty());
}

TEST_F(Core, ActionWithANulByteIsDropped) {
  core_.OnMqtt("action", std::string("mower_logic/a\0b", 15), false);
  core_.OnMqtt("action", "mower_logic/a", false);
  const Outbox out = core_.Take();
  EXPECT_EQ(out.actions, (std::vector<std::string>{"mower_logic/a"}));
  ASSERT_EQ(out.logs.size(), 1u);
  EXPECT_EQ(out.logs[0].level, kLogWarn);
  EXPECT_EQ(out.logs[0].text, "Bridge: an action with a NUL byte was dropped");
}

// B7
TEST_F(Core, RequestErrorsAreAnsweredAndMetaNever) {
  EXPECT_EQ(Ask(core_, "{broken")["error"]["code"], kParseError);
  EXPECT_EQ(Ask(core_, R"({"jsonrpc":"2.0","method":"rpc.ping","id":5})")["id"], 5);
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"meta.version","id":"u2"})", false);
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"ext.anything","id":"u3"})", false);
  EXPECT_TRUE(core_.Take().mqtt.empty());
}

// B8
TEST_F(Core, LocalMethods) {
  EXPECT_EQ(Call(core_, "rpc.ping").dump(), R"({"jsonrpc":"2.0","result":"pong","id":"1"})");
  // E3 (iv): without an id the answer carries null.
  EXPECT_EQ(Ask(core_, R"({"jsonrpc":"2.0","method":"rpc.ping"})").dump(),
            R"({"jsonrpc":"2.0","result":"pong","id":null})");
  const Json info = Call(core_, "system.info", Json{{"x", 1}})["result"];
  EXPECT_EQ(info.dump(), R"({"name":"mowbite-os","version":"1.2.3","platform":"","schema":)" +
                             std::to_string(DataSchema()) + R"(,"api":)" + std::to_string(mowbite_api::kApi) +
                             R"(,"mqtt":5,"stop_release":null})");
  core_.OnPlatform("airseekers_tron", StopRelease(2.0, 3.0, 10.0));
  const Json tron = Call(core_, "system.info")["result"];
  EXPECT_EQ(tron["platform"], "airseekers_tron");
  EXPECT_EQ(tron["stop_release"].dump(), R"({"free_s":2.0,"hold_s":3.0,"max_hold_s":10.0})");
  core_.OnPlatform("airseekers_tron", StopRelease(0.0, 0.0, 0.0));
  EXPECT_TRUE(Call(core_, "system.info")["result"]["stop_release"].is_null());
  EXPECT_EQ(Call(core_, "rpc.methods", Json::array())["result"],
            Json({"logs.recent", "rpc.methods", "rpc.ping", "sensors.list", "system.api", "system.info"}));
  // The schema of the interface with its version, the same as system.info.api.
  const Json api = Call(core_, "system.api", Json::object())["result"];
  EXPECT_EQ(api["api"], mowbite_api::kApi);
  EXPECT_EQ(api["json_schema"], mowbite_api::Schema());
  EXPECT_EQ(api["json_schema"]["x-api"], mowbite_api::kApi);
}

TEST_F(Core, AddedLocalMethodsAndTheirErrors) {
  core_.AddLocalMethod("events.history.list", [](const Json&) { return Json::array({"20261102"}); });
  core_.AddLocalMethod("x.params", [](const Json&) -> Json { throw std::invalid_argument("date must be a text"); });
  core_.AddLocalMethod("x.fail", [](const Json&) -> Json { throw RpcFailure(kInvalidRequest, "busy"); });
  core_.AddLocalMethod("x.broken", [](const Json&) -> Json { throw std::runtime_error("disk full"); });
  EXPECT_EQ(Call(core_, "events.history.list")["result"], Json::array({"20261102"}));
  EXPECT_EQ(Call(core_, "x.params")["error"].dump(), R"({"code":-32602,"message":"date must be a text"})");
  EXPECT_EQ(Call(core_, "x.fail")["error"].dump(), R"({"code":-32600,"message":"busy"})");
  EXPECT_EQ(Call(core_, "x.broken")["error"]["code"], kInternalError);
  const Json methods = Call(core_, "rpc.methods")["result"];
  EXPECT_NE(std::find(methods.begin(), methods.end(), "events.history.list"), methods.end());
}

// B9
TEST_F(Core, MethodsOfTheNodesAreForwardedAsTheyCame) {
  core_.OnRpcMethods("mowbite_logic", {"params.set", "docking.start"});
  core_.OnRpcMethods("mowbite_planner", {"planner.plan"});
  Outbox out = core_.Take();
  ASSERT_EQ(out.logs.size(), 2u);
  EXPECT_EQ(out.logs[0].text, "mowbite_logic registered 2 methods");
  EXPECT_EQ(Call(core_, "rpc.methods")["result"],
            Json({"docking.start", "logs.recent", "params.set", "planner.plan", "rpc.methods", "rpc.ping",
                  "sensors.list", "system.api", "system.info"}));

  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"params.set","params":{"b":1,"a":[2]},"id":"7"})", false);
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"docking.start"})", false);
  out = core_.Take();
  EXPECT_TRUE(out.mqtt.empty());
  ASSERT_EQ(out.requests.size(), 2u);
  EXPECT_EQ(out.requests[0].method, "params.set");
  EXPECT_EQ(out.requests[0].params, R"({"b":1,"a":[2]})");
  // The node gets an id of the bridge, never one of an internal call; without an id it gets none.
  EXPECT_NE(out.requests[0].id, "7");
  EXPECT_FALSE(out.requests[0].id.empty());
  EXPECT_NE(out.requests[0].id.rfind(kInternalCall, 0), 0u);
  EXPECT_EQ(out.requests[1].params, "");
  EXPECT_EQ(out.requests[1].id, "");

  // An id longer than kMaxIdBytes as JSON is refused; the node never sees the request.
  const std::string long_id(kMaxIdBytes, 'i');
  core_.OnMqtt("rpc/request", Json{{"jsonrpc", "2.0"}, {"method", "params.set"}, {"id", long_id}}.dump(), false);
  out = core_.Take();
  EXPECT_TRUE(out.requests.empty());
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["error"]["code"], kInvalidRequest);
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["id"], long_id);

  // An empty list removes the node's methods.
  core_.OnRpcMethods("mowbite_planner", {});
  EXPECT_EQ(Call(core_, "planner.plan", nullptr, "u1").dump(),
            R"({"jsonrpc":"2.0","error":{"code":-32601,"message":"Method \"planner.plan\" not found"},"id":"u1"})");
}

// Master 10.11. (3.5 Punkt 3 (d)): one answer per request; the methods of an online host are its own.
TEST_F(Core, MethodsOfTheHostAreNeverAnsweredWhileItIsOnline) {
  const std::string online =
      R"({"schema":1,"online":true,"t":1,"methods":["update.status","host.restart_os","emergency.reset",7]})";
  core_.OnMqtt("mowbite/host/status", online, true);
  const Json methods = Call(core_, "rpc.methods")["result"];
  EXPECT_NE(std::find(methods.begin(), methods.end(), "update.status"), methods.end());
  EXPECT_NE(std::find(methods.begin(), methods.end(), "host.restart_os"), methods.end());
  // Only update.* and host.* of the host count; emergency.reset stays a method of the logic.
  EXPECT_EQ(std::find(methods.begin(), methods.end(), "emergency.reset"), methods.end());

  core_.Take();
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"update.status","id":"a"})", false);
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"host.restart_os","id":"b"})", false,
               {std::string("rpc/response/app1"), std::string("c")});
  Outbox out = core_.Take();
  EXPECT_TRUE(out.mqtt.empty());
  EXPECT_TRUE(out.requests.empty());

  // Not one of the host: -32601 as before.
  EXPECT_EQ(Call(core_, "update.unknown", nullptr, "u")["error"]["code"], kMethodNotFound);
  EXPECT_EQ(Call(core_, "emergency.reset", nullptr, "u")["error"]["code"], kMethodNotFound);

  // Offline (its last will) or broken: the bridge answers update.* again with -32601.
  for (const char* gone :
       {R"({"schema":1,"online":false})", R"({"online":"true","methods":["update.status"]})", "{", ""}) {
    core_.OnMqtt("mowbite/host/status", online, false);
    core_.OnMqtt("mowbite/host/status", gone, false);
    EXPECT_EQ(Call(core_, "update.status", nullptr, "u")["error"]["code"], kMethodNotFound) << gone;
  }
}

TEST(BridgeCoreConnection, TheStatusOfTheBridgeGoesOfflineAtTheEnd) {
  BridgeCore core({});
  core.Connected();
  core.Take();
  core.Stop(1.0);
  const Outbox out = core.Take();
  const MqttMessage* status = Find(out, "mowbite/bridge/status");
  ASSERT_TRUE(status);
  EXPECT_EQ(status->payload, R"({"online":false})");
  EXPECT_TRUE(status->retain);
  EXPECT_EQ(BridgeStatusText(false), R"({"online":false})");
}

// B10
TEST_F(Core, AnswersOfTheNodes) {
  core_.OnRpcMethods("mowbite_logic", {"x"});
  core_.Take();
  const auto forward = [this](const Json& id) {
    Json request = {{"jsonrpc", "2.0"}, {"method", "x"}};
    if (!id.is_null()) request["id"] = id;
    core_.OnMqtt("rpc/request", request.dump(), false);
    const Outbox out = core_.Take();
    EXPECT_EQ(out.requests.size(), 1u);
    return out.requests.empty() ? std::string("?") : out.requests[0].id;
  };
  const std::string ids[] = {forward("7"), forward(nullptr), forward("8"), forward("9"), forward("10"), forward("11")};
  core_.OnRpcResponse(ids[0], R"({"ok":true})");
  core_.OnRpcResponse(ids[1], R"("x")");
  core_.OnRpcResponse(ids[2], "{broken");
  core_.OnRpcError(ids[3], kInvalidRequest, "No docking station", R"({"reason":"NO_DOCK"})");
  core_.OnRpcError(ids[4], kInvalidParams, "bad", "");
  core_.OnRpcError(ids[5], kInternalError, "Gr\xe4" "fin", "not json");
  // Calls between nodes stay off MQTT.
  core_.OnRpcResponse("os/calibration/1", R"({"ready":true})");
  core_.OnRpcError("os/calibration/2", kInvalidParams, "bad", "");
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 6u);
  EXPECT_EQ(out.mqtt[0].payload, R"({"jsonrpc":"2.0","result":{"ok":true},"id":"7"})");
  EXPECT_EQ(out.mqtt[1].payload, R"({"jsonrpc":"2.0","result":"x","id":null})");
  EXPECT_EQ(Json::parse(out.mqtt[2].payload)["error"]["code"], kInternalError);
  EXPECT_EQ(out.mqtt[3].payload,
            R"({"jsonrpc":"2.0","error":{"code":-32600,"message":"No docking station","data":{"reason":"NO_DOCK"}},)"
            R"("id":"9"})");
  EXPECT_EQ(out.mqtt[4].payload, R"({"jsonrpc":"2.0","error":{"code":-32602,"message":"bad"},"id":"10"})");
  const Json last = Json::parse(out.mqtt[5].payload);
  EXPECT_EQ(last["error"]["message"], "Gr\xef\xbf\xbd" "fin");
  EXPECT_EQ(last["error"]["data"], "not json");
  for (const auto& m : out.mqtt) EXPECT_EQ(m.topic, "rpc/response");
}

// AM4: the client gets its own id back, of any type, also when two clients send the same.
TEST_F(Core, AnswersOfTheNodesUnderTheIdsOfTheClients) {
  core_.OnRpcMethods("mowbite_logic", {"params.set"});
  core_.Take();
  const std::vector<Json> ids = {Json("7"), Json(7), Json("7"), Json{{"n", 1}}, Json(std::string("a\0b", 3))};
  for (const auto& id : ids) {
    core_.OnMqtt("rpc/request", Json{{"jsonrpc", "2.0"}, {"method", "params.set"}, {"id", id}}.dump(), false);
  }
  Outbox out = core_.Take();
  ASSERT_EQ(out.requests.size(), ids.size());
  std::set<std::string> distinct;
  for (const auto& request : out.requests) {
    distinct.insert(request.id);
    EXPECT_EQ(request.id.find('\0'), std::string::npos);
  }
  EXPECT_EQ(distinct.size(), ids.size());
  // Answered in the other order.
  for (size_t i = ids.size(); i-- > 0;) core_.OnRpcResponse(out.requests[i].id, std::to_string(i));
  const std::vector<RpcForward> requests = out.requests;
  out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), ids.size());
  for (size_t i = 0; i < ids.size(); i++) {
    const Json reply = Json::parse(out.mqtt[i].payload);
    const size_t call = ids.size() - 1 - i;
    EXPECT_EQ(reply["id"], ids[call]);
    EXPECT_EQ(reply["result"], call);
  }
  // A second answer to the same call is dropped with a warning, one to an id the bridge never gave without.
  core_.OnRpcResponse(requests[0].id, "0");
  core_.OnRpcError("7", kInvalidParams, "bad", "");
  core_.OnRpcResponse("calib_test_3", "{}");
  out = core_.Take();
  EXPECT_TRUE(out.mqtt.empty());
  ASSERT_EQ(out.logs.size(), 1u);
  EXPECT_EQ(out.logs[0].level, kLogWarn);
}

TEST(BridgeCoreCalls, OpenCallsWaitAtMostAndAreBounded) {
  double now = 100;
  BridgeConfig config;
  config.clock = [&now] { return now; };
  BridgeCore core(config);
  core.Connected();
  core.OnRpcMethods("mowbite_logic", {"x"});
  core.Take();
  const auto forward = [&core](int id) {
    core.OnMqtt("rpc/request", Json{{"jsonrpc", "2.0"}, {"method", "x"}, {"id", id}}.dump(), false);
    const Outbox out = core.Take();
    EXPECT_EQ(out.requests.size(), 1u);
    return out.requests.empty() ? std::string("?") : out.requests[0].id;
  };
  const std::string first = forward(1);
  now += kCallWaitS - 1;
  const std::string second = forward(2);
  // The first is past its wait once the next call comes.
  now += 2;
  const std::string third = forward(3);
  core.OnRpcResponse(first, "1");
  core.OnRpcResponse(second, "2");
  Outbox out = core.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["id"], 2);
  // With kMaxOpenCalls open the oldest goes.
  std::string last;
  for (size_t i = 0; i < kMaxOpenCalls; i++) last = forward(1000 + static_cast<int>(i));
  core.OnRpcResponse(third, "3");
  core.OnRpcResponse(last, "4");
  out = core.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["id"], 1000 + static_cast<int>(kMaxOpenCalls) - 1);
}

// AS4: only rpc/response/<token> with a token of 1 to 64 of [A-Za-z0-9._-].
TEST(ResponseTopic, OnlyAPlainTokenUnderRpcResponse) {
  for (const std::string& topic : std::vector<std::string>{"rpc/response/a", "rpc/response/mowbite-app_1.2",
                                                           "rpc/response/" + std::string(64, 'Z')}) {
    EXPECT_TRUE(ValidResponseTopic(topic)) << topic;
  }
  for (const std::string& topic : std::vector<std::string>{
           "", "rpc/response", "rpc/response/", "rpc/response/" + std::string(65, 'a'), "rpc/response/a/b",
        "rpc/response/+", "rpc/response/#", "rpc/response/$x", "rpc/response/a b", "rpc/response/\xc3\xa4",
        "map/json", "robot_state/json", "rpc/responsex/a", "/rpc/response/a", std::string("rpc/response/a\0b", 16)}) {
    EXPECT_FALSE(ValidResponseTopic(topic)) << topic;
  }
}

namespace {

ResponseRoute To(const std::string& topic, std::optional<std::string> correlation = std::nullopt) {
  return {topic, std::move(correlation)};
}

}  // namespace

// AS4: an answer goes only to the response topic of its request, QoS 0, not retained, the correlation data unchanged.
TEST_F(Core, AnswersGoToTheResponseTopicWithTheCorrelation) {
  const std::string correlation("c\0\xff" "1", 4);
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"rpc.ping","id":"1"})", false,
               To("rpc/response/app-1", correlation));
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"rpc.ping","id":"2"})", false, To("rpc/response/ha"));
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"rpc.ping","id":"3"})", false);
  // Correlation without a response topic: as over 3.1.1.
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"rpc.ping","id":"4"})", false, {std::nullopt, "x"});
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 4u);
  EXPECT_EQ(out.mqtt[0].topic, "rpc/response/app-1");
  EXPECT_EQ(out.mqtt[0].correlation, correlation);
  EXPECT_EQ(out.mqtt[0].payload, R"({"jsonrpc":"2.0","result":"pong","id":"1"})");
  EXPECT_EQ(out.mqtt[1].topic, "rpc/response/ha");
  EXPECT_EQ(out.mqtt[1].correlation, std::nullopt);
  for (int i : {2, 3}) {
    EXPECT_EQ(out.mqtt[i].topic, "rpc/response");
    EXPECT_EQ(out.mqtt[i].correlation, std::nullopt);
  }
  for (const auto& m : out.mqtt) {
    EXPECT_EQ(m.qos, 0);
    EXPECT_FALSE(m.retain);
  }
}

// AS4: an invalid response topic is refused on rpc/response with REQUEST_REFUSED and nothing is done.
TEST_F(Core, AnInvalidResponseTopicIsRefusedAndNothingDone) {
  core_.OnRpcMethods("mowbite_logic", {"mowing.start"});
  RobotStateView state;
  state.current_state = "IDLE";
  core_.OnRobotState(state);
  core_.Take();
  for (const std::string topic : {"map/json", "robot_state/json", "rpc/response/+", "rpc/response", ""}) {
    for (const char* method : {"mowing.start", "rpc.ping", "nothing.here"}) {
      core_.OnMqtt("rpc/request", Json{{"jsonrpc", "2.0"}, {"method", method}, {"id", 5}}.dump(), false,
                   To(topic, "c"));
      const Outbox out = core_.Take();
      EXPECT_TRUE(out.requests.empty()) << topic << " " << method;
      EXPECT_TRUE(out.actions.empty());
      ASSERT_EQ(out.mqtt.size(), 1u) << topic << " " << method;
      EXPECT_EQ(out.mqtt[0].topic, "rpc/response");
      EXPECT_EQ(out.mqtt[0].correlation, std::nullopt);
      const Json answer = Json::parse(out.mqtt[0].payload);
      EXPECT_EQ(answer["id"], 5);
      EXPECT_EQ(answer["error"]["code"], kInvalidRequest);
      const Json& data = answer["error"]["data"];
      EXPECT_EQ(data["event"], "REQUEST_REFUSED");
      EXPECT_EQ(data["reason"], "RESPONSE_TOPIC");
      EXPECT_EQ(data["severity"], "error");
      EXPECT_EQ(data["resume"], "user");
      EXPECT_EQ(data["state"], "IDLE");
      EXPECT_EQ(data["sub_state"], "");
      for (const auto& field : mowbite_api::FindReasonGroup("REQUEST_REFUSED")->fields) {
        EXPECT_TRUE(!field.required || data.contains(field.name)) << field.name;
      }
    }
  }
  // A request that cannot be read is refused the same; meta.* stays unanswered.
  core_.OnMqtt("rpc/request", "{broken", false, To("map/json"));
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"meta.x","id":1})", false, To("map/json"));
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["error"]["data"]["reason"], "RESPONSE_TOPIC");
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["id"], nullptr);
}

// AS4: correlation data up to 1 KB comes back, longer is refused to the response topic.
TEST_F(Core, CorrelationDataUpToOneKilobyte) {
  core_.OnRpcMethods("mowbite_logic", {"x"});
  core_.Take();
  const std::string most(kMaxCorrelationBytes, 'k');
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"rpc.ping","id":1})", false, To("rpc/response/a", most));
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"x","id":2})", false, To("rpc/response/a", most + "k"));
  const Outbox out = core_.Take();
  EXPECT_TRUE(out.requests.empty());
  ASSERT_EQ(out.mqtt.size(), 2u);
  EXPECT_EQ(out.mqtt[0].correlation, most);
  EXPECT_EQ(out.mqtt[1].topic, "rpc/response/a");
  EXPECT_EQ(out.mqtt[1].correlation, std::nullopt);
  const Json data = Json::parse(out.mqtt[1].payload)["error"]["data"];
  EXPECT_EQ(data["reason"], "CORRELATION_DATA");
  EXPECT_EQ(data["limit"], kMaxCorrelationBytes);
  EXPECT_EQ(data["severity"], "error");
}

// AS4: the answers of the nodes find their way by the bridge's own ids, with response topic and correlation of each
// open call; a client on 3.1.1 with the same id gets its own on rpc/response.
TEST_F(Core, AnswersOfTheNodesGoWhereTheirRequestsWanted) {
  core_.OnRpcMethods("mowbite_logic", {"x"});
  core_.Take();
  const auto forward = [this](const Json& id, const ResponseRoute& route) {
    Json request = {{"jsonrpc", "2.0"}, {"method", "x"}};
    if (!id.is_null()) request["id"] = id;
    core_.OnMqtt("rpc/request", request.dump(), false, route);
    const Outbox out = core_.Take();
    EXPECT_TRUE(out.mqtt.empty());
    EXPECT_EQ(out.requests.size(), 1u);
    return out.requests.empty() ? std::string("?") : out.requests[0].id;
  };
  const std::string app = forward("1", To("rpc/response/app", "A"));
  const std::string ha = forward("1", To("rpc/response/ha"));
  const std::string old = forward("1", {});
  // Without an id the node gets one all the same, so the answer can go to the response topic.
  const std::string quiet = forward(nullptr, To("rpc/response/app", "Q"));
  EXPECT_FALSE(quiet.empty());
  EXPECT_EQ(forward(nullptr, {}), "");
  core_.OnRpcResponse(old, "3");
  core_.OnRpcError(ha, kInvalidParams, "bad", "");
  core_.OnRpcResponse(app, "1");
  core_.OnRpcResponse(quiet, "4");
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 4u);
  EXPECT_EQ(out.mqtt[0].topic, "rpc/response");
  EXPECT_EQ(out.mqtt[0].correlation, std::nullopt);
  EXPECT_EQ(out.mqtt[0].payload, R"({"jsonrpc":"2.0","result":3,"id":"1"})");
  EXPECT_EQ(out.mqtt[1].topic, "rpc/response/ha");
  EXPECT_EQ(Json::parse(out.mqtt[1].payload)["error"]["code"], kInvalidParams);
  EXPECT_EQ(out.mqtt[2].topic, "rpc/response/app");
  EXPECT_EQ(out.mqtt[2].correlation, "A");
  EXPECT_EQ(out.mqtt[2].payload, R"({"jsonrpc":"2.0","result":1,"id":"1"})");
  EXPECT_EQ(out.mqtt[3].topic, "rpc/response/app");
  EXPECT_EQ(out.mqtt[3].correlation, "Q");
  EXPECT_EQ(out.mqtt[3].payload, R"({"jsonrpc":"2.0","result":4,"id":null})");
  // Errors before a node: an id too long goes to the response topic too.
  core_.OnMqtt("rpc/request",
               Json{{"jsonrpc", "2.0"}, {"method", "x"}, {"id", std::string(kMaxIdBytes, 'i')}}.dump(), false,
               To("rpc/response/app", "L"));
  core_.OnMqtt("rpc/request", "{broken", false, To("rpc/response/app", "P"));
  const Outbox errors = core_.Take();
  ASSERT_EQ(errors.mqtt.size(), 2u);
  for (const auto& m : errors.mqtt) EXPECT_EQ(m.topic, "rpc/response/app");
  EXPECT_EQ(Json::parse(errors.mqtt[0].payload)["error"]["code"], kInvalidRequest);
  EXPECT_EQ(errors.mqtt[1].correlation, "P");
  EXPECT_EQ(Json::parse(errors.mqtt[1].payload)["error"]["code"], kParseError);
}

// AS4: system.info.mqtt is the version of the last connect.
TEST(BridgeCoreConnection, SystemInfoTellsTheMqttVersion) {
  BridgeCore core({});
  core.Connected(5);
  EXPECT_EQ(Call(core, "system.info")["result"]["mqtt"], 5);
  core.ConnectionLost();
  core.Connected(3);
  EXPECT_EQ(Call(core, "system.info")["result"]["mqtt"], 3);
}

// AS4: a refusal of the protocol switches to 3.1.1 at once, a dropped connect the second time in a row, network
// errors never; once on 3.1.1 it stays.
TEST(MqttVersionChoice, FallsBackOnlyWhenTheBrokerRefusesTheProtocol) {
  using F = MqttVersion::Failure;
  EXPECT_EQ(MqttVersion::Classify(1, true), F::kRefused);
  EXPECT_EQ(MqttVersion::Classify(0x84, false), F::kRefused);
  EXPECT_EQ(MqttVersion::Classify(-1, true), F::kDropped);
  EXPECT_EQ(MqttVersion::Classify(-1, false), F::kNetwork);
  for (int code : {2, 3, 4, 5, 0x80, 0x86, 0x87, 0x88, -3}) EXPECT_EQ(MqttVersion::Classify(code, true), F::kNetwork);

  MqttVersion refused;
  EXPECT_EQ(refused.version(), 5);
  EXPECT_TRUE(refused.OnFailure(F::kRefused));
  EXPECT_EQ(refused.version(), 3);
  EXPECT_FALSE(refused.OnFailure(F::kRefused));
  refused.OnConnected();
  EXPECT_EQ(refused.version(), 3);

  MqttVersion dropped;
  EXPECT_FALSE(dropped.OnFailure(F::kDropped));
  EXPECT_FALSE(dropped.OnFailure(F::kNetwork));
  EXPECT_FALSE(dropped.OnFailure(F::kDropped));
  dropped.OnConnected();
  EXPECT_FALSE(dropped.OnFailure(F::kDropped));
  EXPECT_EQ(dropped.version(), 5);
  EXPECT_TRUE(dropped.OnFailure(F::kDropped));
  EXPECT_EQ(dropped.version(), 3);

  MqttVersion network;
  for (int i = 0; i < 10; i++) EXPECT_FALSE(network.OnFailure(F::kNetwork));
  EXPECT_EQ(network.version(), 5);
}

// Events against the catalog: each problem once as an ERROR, the event goes out all the same.
TEST_F(Core, EventsAreCheckedAgainstTheCatalog) {
  std::vector<Json> seen;
  core_.SetEventSink([&seen](const Json& event) { seen.push_back(event); });
  const std::string odd = BuildEvent("BOOTED", Json{{"odd", 1}}, 5).dump();
  core_.OnPublish("events/json", odd, false);
  core_.OnPublish("events/json", odd, false);
  core_.OnPublish("events/json", BuildEvent("BOOTED", Json::object(), 6).dump(), false);
  core_.OnPublish("events/json", BuildEvent("PLANNER_THING", Json{{"any", 1}}, 7).dump(), false);
  const Outbox out = core_.Take();
  EXPECT_EQ(seen.size(), 4u);
  EXPECT_EQ(out.mqtt.size(), 4u);
  ASSERT_EQ(out.logs.size(), 1u);
  EXPECT_EQ(out.logs[0].level, kLogError);
  EXPECT_NE(out.logs[0].text.find("odd"), std::string::npos);
}

// B11
TEST_F(Core, RobotStateLiveWithoutRetain) {
  RobotStateView state;
  state.current_state = "IDLE";
  core_.OnRobotState(state);
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(out.mqtt[0].topic, "robot_state/json");
  EXPECT_FALSE(out.mqtt[0].retain);
  EXPECT_NE(out.mqtt[0].payload.find(R"("current_state":"IDLE")"), std::string::npos);
}

// B13
TEST_F(Core, SensorsInfosOnChangeValuesLiveAndTheList) {
  SensorDescription battery;
  battery.id = "om_v_battery";
  battery.meaning = Meaning::kVoltage;
  SensorDescription state;
  state.id = "om_charge_state";
  state.is_text = true;
  core_.OnSensor(battery);
  core_.OnSensor(battery);
  core_.OnSensor(state);
  Outbox out = core_.Take();
  ASSERT_EQ(Topics(out), (std::vector<std::string>{"sensor_infos/json", "sensor_infos/json"}));
  EXPECT_TRUE(out.mqtt[1].retain);
  EXPECT_EQ(Json::parse(out.mqtt[1].payload)[0]["sensor_id"], "om_charge_state");

  core_.OnSensorValue("om_v_battery", 24.5, 10.0);
  core_.OnSensorValue("om_charge_state", "Fast Charge", 11.0);
  out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 2u);
  EXPECT_EQ(out.mqtt[0].topic, "sensors/om_v_battery/data");
  EXPECT_EQ(out.mqtt[0].payload, "24.500000");
  EXPECT_FALSE(out.mqtt[0].retain);
  EXPECT_EQ(out.mqtt[1].payload, "Fast Charge");

  const Json list = Call(core_, "sensors.list", Json{{"ids", {"om_v_battery"}}})["result"];
  ASSERT_EQ(list.size(), 1u);
  EXPECT_EQ(list[0]["value"], 24.5);
  EXPECT_EQ(list[0]["stamp"], 10.0);
  const Json bad = Call(core_, "sensors.list", Json{{"ids", "om_v_battery"}});
  EXPECT_EQ(bad["error"]["code"], kInvalidParams);
}

// B14
TEST_F(Core, ActionsOfAPrefixReplaceItsOldOnes) {
  core_.OnActions("mower_logic", {{"reset_emergency", "Release emergency stop", true}});
  core_.OnActions("mower_logic:mowing", {{"pause", "Pause", false}});
  core_.OnActions("mower_logic", {{"reset_emergency", "Release emergency stop", false}});
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 3u);
  EXPECT_TRUE(out.mqtt[2].retain);
  EXPECT_EQ(out.mqtt[2].payload,
            R"([{"action_id":"mower_logic/reset_emergency","action_name":"Release emergency stop","enabled":0},)"
            R"({"action_id":"mower_logic:mowing/pause","action_name":"Pause","enabled":0}])");
}

// B15
TEST_F(Core, ParamsOnlyOnChange) {
  core_.OnParams(R"({"node":"/n","params":{"a":1}})");
  core_.OnParams(R"({"node":"/n","params":{"a":1}})");
  core_.OnParams("broken");
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(out.mqtt[0].topic, "params/json");
  EXPECT_EQ(out.mqtt[0].payload, R"({"/n/a":1})");
  EXPECT_TRUE(out.mqtt[0].retain);
  EXPECT_EQ(out.mqtt[0].qos, 1);
}

// B16
TEST_F(Core, MapTextUnchangedEachTime) {
  core_.OnMap("{\n  \"areas\": []\n}");
  core_.OnMap("{\n  \"areas\": []\n}");
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 2u);
  EXPECT_EQ(out.mqtt[1].payload, "{\n  \"areas\": []\n}");
  EXPECT_TRUE(out.mqtt[1].retain);
}

// B17
TEST_F(Core, PublishedTextsGoOutUnchangedEventsAlsoToTheSink) {
  std::vector<Json> sunk;
  core_.SetEventSink([&](const Json& e) { sunk.push_back(e); });
  core_.OnPublish("events/json", R"({"id":"a","t":1.0,"type":"STATE","state":"IDLE"})", false);
  core_.OnPublish("events/json", "not json", false);
  core_.OnPublish("events/json", "[1]", false);
  core_.OnPublish("map_overlay/json", R"({"o":1})", true);
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 4u);
  EXPECT_EQ(out.mqtt[0].payload, R"({"id":"a","t":1.0,"type":"STATE","state":"IDLE"})");
  EXPECT_FALSE(out.mqtt[0].retain);
  EXPECT_EQ(out.mqtt[0].qos, 0);
  EXPECT_EQ(out.mqtt[1].payload, "not json");
  EXPECT_TRUE(out.mqtt[3].retain);
  EXPECT_EQ(out.mqtt[3].qos, 1);
  ASSERT_EQ(sunk.size(), 1u);
  EXPECT_EQ(sunk[0]["state"], "IDLE");
}

// B19
TEST(EventBuilder, IdTimeTypeThenTheDetailsInTheirOrder) {
  const Json e = BuildEvent("STATE", Json{{"state", "IDLE"}, {"job_id", "j"}, {"type", "X"}, {"t", 0}}, 12.5);
  EXPECT_EQ(e.size(), 5u);
  EXPECT_EQ(e.begin().key(), "id");
  EXPECT_EQ(std::next(e.begin(), 1).key(), "t");
  EXPECT_EQ(std::next(e.begin(), 2).key(), "type");
  EXPECT_EQ(std::next(e.begin(), 3).key(), "state");
  EXPECT_EQ(std::next(e.begin(), 4).key(), "job_id");
  EXPECT_EQ(e["type"], "STATE");
  EXPECT_DOUBLE_EQ(e["t"].get<double>(), 12.5);
  EXPECT_EQ(BuildEvent("A", nullptr, 0).size(), 3u);
  EXPECT_EQ(BuildEvent("A", Json::array({1}), 0).size(), 3u);
}

TEST(EventBuilder, RandomIdsOfLettersAndDigits) {
  const std::string id = RandomId();
  ASSERT_EQ(id.size(), 32u);
  for (const char c : id) EXPECT_TRUE(std::isalnum(static_cast<unsigned char>(c))) << c;
  EXPECT_NE(RandomId(), RandomId());
  EXPECT_EQ(RandomId(8).size(), 8u);
  EXPECT_EQ(RandomId(25).size(), 25u);
  std::set<char> seen;
  for (int i = 0; i < 200; ++i) {
    for (const char c : RandomId()) seen.insert(c);
  }
  EXPECT_EQ(seen.size(), 62u);
}

// B21
TEST_F(Core, WarningsOfTheNodesInLogsRecent) {
  core_.OnLog(1.0, kLogInfo, "n", "info");
  core_.OnLog(2.0, kLogWarn, "mowbite_logic", "docking.start refused");
  EXPECT_EQ(Call(core_, "logs.recent", Json{{"limit", 5}})["result"].dump(),
            R"([{"t":2.0,"level":"WARN","node":"mowbite_logic","msg":"docking.start refused"}])");
}

// B22
TEST(RestartWarning, TextOfTheNote) {
  EXPECT_EQ(RestartWarning(R"({"exit_code": -9, "restart": 1, "max_restarts": 3, "t": 1.0})"),
            "MowBite OS restarted after a crash: its container ended with exit code -9 (restart 1 of 3)");
  EXPECT_EQ(RestartWarning(R"({"exit_code": 0})"), "MowBite OS restarted after a crash: its container ended with exit code 0");
  EXPECT_EQ(RestartWarning("broken"), "MowBite OS restarted after a crash");
  EXPECT_EQ(RestartWarning(R"({"exit_code": 86, "restart": 2, "max_restarts": 3,
                               "stalled": {"node": "mowbite_bridge", "seconds": 10.24, "limit": 10}})"),
            "MowBite OS restarted after a crash: its container ended with exit code 86 (restart 2 of 3), ended by "
            "the watch: mowbite_bridge gave no sign for 10.2 s (limit 10.0 s)");
  EXPECT_EQ(RestartWarning(R"({"exit_code": 86, "stalled": {"node": "x", "seconds": "a"}})"),
            "MowBite OS restarted after a crash: its container ended with exit code 86, ended by the watch: x gave no "
            "sign for ? s (limit ? s)");
  EXPECT_EQ(RestartWarning(R"({"exit_code": -9, "stalled": null})"),
            "MowBite OS restarted after a crash: its container ended with exit code -9");
}

TEST(RollbackWarning, TextOfTheNote) {
  EXPECT_EQ(RollbackWarning(R"({"from": 3, "to": 2, "backup": "backup-3", "t": 1.0})"),
            "MowBite OS went back to an older image: data_dir is back at schema 2 from its backup, the changes since "
            "the update (schema 3) are in backup-3");
  EXPECT_EQ(RollbackWarning("broken"),
            "MowBite OS went back to an older image: data_dir is back at schema ? from its backup, the changes since "
            "the update (schema ?) are in ?");
}

// B23
TEST_F(Core, AFailingSinkBecomesAWarning) {
  core_.SetEventSink([](const Json&) { throw std::runtime_error("disk full"); });
  core_.OnPublish("events/json", R"({"type":"X"})", false);
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  ASSERT_EQ(out.logs.size(), 1u);
  EXPECT_EQ(out.logs[0].level, kLogWarn);
  EXPECT_EQ(out.logs[0].text, "Bridge: publish failed: disk full");
}

namespace {

// A connected core with a clock the test sets and its events kept.
class Timed : public ::testing::Test {
 protected:
  Timed() : core_(BridgeConfig{"1.2.3", "{}", [this] { return now_; }}) {
    core_.SetEventSink([this](const Json& event) { sunk_.push_back(event); });
    core_.Connected();
    core_.Take();
  }

  // The METHODS_CHANGED events a Tick sends after this many seconds.
  std::vector<Json> After(double seconds) {
    now_ += seconds;
    core_.Tick(1000.0 + now_);
    std::vector<Json> events;
    for (const auto& m : core_.Take().mqtt) {
      if (m.topic != "events/json") continue;
      EXPECT_FALSE(m.retain);
      events.push_back(Json::parse(m.payload));
    }
    return events;
  }

  // Forwards a call of this method and gives the id the node got.
  std::string Forward(const std::string& method, const Json& id) {
    core_.Take();
    core_.OnMqtt("rpc/request", Json{{"jsonrpc", "2.0"}, {"method", method}, {"id", id}}.dump(), false);
    const Outbox out = core_.Take();
    EXPECT_EQ(out.requests.size(), 1u) << method;
    return out.requests.empty() ? std::string("?") : out.requests[0].id;
  }

  std::vector<std::string> Methods() {
    return Call(core_, "rpc.methods")["result"].get<std::vector<std::string>>();
  }

  double now_ = 50.0;
  BridgeCore core_;
  std::vector<Json> sunk_;
};

bool Offers(const std::vector<std::string>& methods, const std::string& method) {
  return std::find(methods.begin(), methods.end(), method) != methods.end();
}

SensorDescription TextSensor(const std::string& id) {
  SensorDescription sensor;
  sensor.id = id;
  sensor.is_text = true;
  return sensor;
}

}  // namespace

// M5: the changes of one second in one event, as the catalog has it; the bridge's own methods are never news.
TEST_F(Timed, MethodsChangedTellsTheChangesOfOneSecondTogether) {
  core_.AddLocalMethod("events.history", [](const Json&) { return Json(); });
  EXPECT_TRUE(After(5.0).empty());

  core_.OnRpcMethods("mowbite_logic", {"params.set", "docking.start"});
  EXPECT_TRUE(After(0.4).empty());
  core_.OnRpcMethods("mowbite_planner", {"planner.plan"});
  EXPECT_TRUE(After(0.5).empty());
  std::vector<Json> events = After(0.1);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["type"], "METHODS_CHANGED");
  EXPECT_EQ(events[0]["added"], Json({"docking.start", "params.set", "planner.plan"}));
  EXPECT_EQ(events[0]["removed"], Json::array());
  EXPECT_EQ(events[0]["severity"], "info");
  EXPECT_EQ(events[0]["resume"], "auto");
  EXPECT_EQ(events[0]["t"], 1000.0 + now_);
  EXPECT_TRUE(mowbite_api::CheckEvent("METHODS_CHANGED", events[0]).empty());
  ASSERT_EQ(sunk_.size(), 1u);
  EXPECT_EQ(sunk_[0], events[0]);
  EXPECT_TRUE(After(5.0).empty());

  // One method less, one of the host more.
  core_.OnRpcMethods("mowbite_planner", {});
  core_.OnMqtt("mowbite/host/status", R"({"online": true, "methods": ["update.status"]})", true);
  events = After(1.0);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["added"], Json({"update.status"}));
  EXPECT_EQ(events[0]["removed"], Json({"planner.plan"}));

  // Back within the second: nothing to tell, and the next change has its own second.
  core_.OnRpcMethods("mowbite_light", {"light.preview"});
  core_.OnRpcMethods("mowbite_light", {});
  EXPECT_TRUE(After(0.9).empty());
  core_.OnRpcMethods("mowbite_light", {"light.preview"});
  EXPECT_TRUE(After(0.9).empty());
  events = After(0.1);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["added"], Json({"light.preview"}));
  EXPECT_EQ(sunk_.size(), 3u);
}

TEST(BridgeCoreConnection, MethodsChangedWaitsForTheBroker) {
  double now = 0.0;
  BridgeCore core(BridgeConfig{"1.2.3", "{}", [&now] { return now; }});
  std::vector<Json> sunk;
  core.SetEventSink([&sunk](const Json& event) { sunk.push_back(event); });
  core.OnRpcMethods("mowbite_logic", {"params.set"});
  now = 30.0;
  core.Tick(1.0);
  EXPECT_TRUE(core.Take().mqtt.empty());
  EXPECT_TRUE(sunk.empty());
  core.Connected();
  core.Take();
  core.OnRpcMethods("mowbite_map", {"map.replace"});
  core.Tick(2.0);
  const Outbox out = core.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(out.mqtt[0].topic, "events/json");
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["added"], Json({"map.replace", "params.set"}));
  EXPECT_EQ(sunk.size(), 1u);
}

// M5: a provider the graph lacks twice in a row is gone with its methods, its open calls and its sensors.
TEST_F(Timed, AProviderMissingTwiceInARowIsGone) {
  core_.OnRpcMethods("mowbite_camera", {"camera.list", "camera.stream"}, "aa");
  core_.OnRpcMethods("mowbite_logic", {"params.set"}, "bb");
  core_.OnSensor(TextSensor("vision_state"), "cc");
  core_.OnSensor(TextSensor("om_charge_state"), "bb");
  core_.OnSensor(TextSensor("from_a_test"));
  core_.OnSensorValue("vision_state", "running", 1.0);
  ASSERT_EQ(After(1.0).size(), 1u);
  const std::string camera_call = Forward("camera.stream", 7);
  const std::string logic_call = Forward("params.set", 8);

  // Missing once, there again, missing once: nothing happens.
  core_.OnProviders({"aa", "bb", "cc"});
  core_.OnProviders({"bb"});
  core_.OnProviders({"aa", "bb", "cc"});
  core_.OnProviders({"bb"});
  EXPECT_TRUE(core_.Take().mqtt.empty());
  EXPECT_TRUE(Offers(Methods(), "camera.stream"));

  core_.OnProviders({"bb"});
  Outbox out = core_.Take();
  ASSERT_EQ(Topics(out), (std::vector<std::string>{"rpc/response", "sensor_infos/json"}));
  const Json failed = Json::parse(out.mqtt[0].payload);
  EXPECT_EQ(failed["id"], 7);
  EXPECT_EQ(failed["error"]["code"], kMethodNotFound);
  EXPECT_TRUE(out.mqtt[1].retain);
  const Json infos = Json::parse(out.mqtt[1].payload);
  ASSERT_EQ(infos.size(), 2u);
  EXPECT_EQ(infos[0]["sensor_id"], "from_a_test");
  EXPECT_EQ(infos[1]["sensor_id"], "om_charge_state");
  ASSERT_EQ(out.logs.size(), 2u);
  EXPECT_EQ(out.logs[0].level, kLogWarn);
  EXPECT_EQ(out.logs[0].text, "mowbite_camera is gone, its 2 methods are no longer offered");
  EXPECT_EQ(out.logs[1].text, "Sensors no longer reported, their process is gone: vision_state");

  const std::vector<std::string> methods = Methods();
  EXPECT_FALSE(Offers(methods, "camera.list"));
  EXPECT_FALSE(Offers(methods, "camera.stream"));
  EXPECT_TRUE(Offers(methods, "params.set"));
  EXPECT_EQ(Call(core_, "camera.list")["error"]["code"], kMethodNotFound);
  EXPECT_EQ(Call(core_, "sensors.list", Json{{"ids", {"vision_state"}}})["result"], Json::array());
  const std::vector<Json> events = After(1.0);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["added"], Json::array());
  EXPECT_EQ(events[0]["removed"], Json({"camera.list", "camera.stream"}));

  // The call of the node that is still there gets its answer, the late one of the camera is dropped.
  core_.OnRpcResponse(logic_call, "true");
  core_.OnRpcResponse(camera_call, "{}");
  out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(Json::parse(out.mqtt[0].payload)["id"], 8);

  // A sensor without a known publisher stays whatever the graph says; the process that comes back is taken again.
  for (int i = 0; i < 3; ++i) core_.OnProviders({});
  EXPECT_EQ(Call(core_, "sensors.list")["result"].size(), 1u);
  core_.OnSensor(TextSensor("vision_state"), "cd");
  core_.OnRpcMethods("mowbite_camera", {"camera.list", "camera.stream"}, "ab");
  core_.OnProviders({"ab", "cd"});
  core_.OnProviders({"ab", "cd"});
  EXPECT_EQ(Call(core_, "sensors.list")["result"].size(), 2u);
  EXPECT_TRUE(Offers(Methods(), "camera.list"));
}

// A node that registers from another publisher started anew: what its old process was asked gets no answer.
TEST_F(Timed, ANewProcessOfANodeEndsTheCallsOfItsOldOne) {
  core_.OnRpcMethods("mowbite_camera", {"camera.stream"}, "aa");
  core_.OnRpcMethods("mowbite_logic", {"params.set"}, "bb");
  ASSERT_EQ(After(1.0).size(), 1u);
  Forward("camera.stream", "a");
  Forward("params.set", "b");

  // The same publisher once more, as DDS sends a latched message again.
  core_.OnRpcMethods("mowbite_camera", {"camera.stream"}, "aa");
  EXPECT_TRUE(core_.Take().mqtt.empty());

  core_.OnRpcMethods("mowbite_camera", {"camera.stream"}, "ab");
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 1u);
  const Json failed = Json::parse(out.mqtt[0].payload);
  EXPECT_EQ(failed["id"], "a");
  EXPECT_EQ(failed["error"]["code"], kMethodNotFound);
  EXPECT_EQ(failed["error"]["message"], "The node that answers \"camera.stream\" is gone");

  // The method is still offered: nothing to tell, and the new process gets the next call.
  EXPECT_TRUE(After(2.0).empty());
  EXPECT_NE(Forward("camera.stream", "c"), "?");
  // The old publisher leaving the graph afterwards changes nothing.
  core_.OnProviders({"ab", "bb"});
  core_.OnProviders({"ab", "bb"});
  EXPECT_TRUE(core_.Take().mqtt.empty());
  EXPECT_TRUE(Offers(Methods(), "camera.stream"));
}
