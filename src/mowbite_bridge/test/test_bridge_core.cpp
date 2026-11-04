#include <gtest/gtest.h>

#include <set>

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
  EXPECT_EQ(BridgeCore::CommandTopics(), (std::vector<std::string>{"action", "teleop", "rpc/request"}));
  core.Connected();
  Outbox out = core.Take();
  // Without sensors and map: none of them.
  EXPECT_EQ(Topics(out),
            (std::vector<std::string>{"capabilities/json", "version/json", "params/json", "actions/json"}));
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
                                                   "map_overlay/json", "other/json"}));
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
                             std::to_string(DataSchema()) + "}");
  core_.OnPlatform("airseekers_tron");
  EXPECT_EQ(Call(core_, "system.info")["result"]["platform"], "airseekers_tron");
  EXPECT_EQ(Call(core_, "rpc.methods", Json::array())["result"],
            Json({"logs.recent", "rpc.methods", "rpc.ping", "sensors.list", "system.info"}));
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
                  "sensors.list", "system.info"}));

  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"params.set","params":{"b":1,"a":[2]},"id":"7"})", false);
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"docking.start"})", false);
  out = core_.Take();
  EXPECT_TRUE(out.mqtt.empty());
  ASSERT_EQ(out.requests.size(), 2u);
  EXPECT_EQ(out.requests[0].method, "params.set");
  EXPECT_EQ(out.requests[0].params, R"({"b":1,"a":[2]})");
  EXPECT_EQ(out.requests[0].id, "7");
  EXPECT_EQ(out.requests[1].params, "");
  EXPECT_EQ(out.requests[1].id, "");

  // An id with a NUL byte is refused; the node never sees the request.
  core_.OnMqtt("rpc/request", R"({"jsonrpc":"2.0","method":"params.set","id":"a\u0000b"})", false);
  out = core_.Take();
  EXPECT_TRUE(out.requests.empty());
  ASSERT_EQ(out.mqtt.size(), 1u);
  EXPECT_EQ(out.mqtt[0].payload,
            R"({"jsonrpc":"2.0","error":{"code":-32600,"message":"The id must not contain a NUL byte"},"id":"a\u0000b"})");

  // An empty list removes the node's methods.
  core_.OnRpcMethods("mowbite_planner", {});
  EXPECT_EQ(Call(core_, "planner.plan", nullptr, "u1").dump(),
            R"({"jsonrpc":"2.0","error":{"code":-32601,"message":"Method \"planner.plan\" not found"},"id":"u1"})");
}

// B10
TEST_F(Core, AnswersOfTheNodes) {
  core_.OnRpcResponse("7", R"({"ok":true})");
  core_.OnRpcResponse("", R"("x")");
  core_.OnRpcResponse("8", "{broken");
  core_.OnRpcError("9", kInvalidRequest, "No docking station", R"({"reason":"NO_DOCK"})");
  core_.OnRpcError("10", kInvalidParams, "bad", "");
  core_.OnRpcError("11", kInternalError, "Gr\xe4" "fin", "not json");
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
  core_.OnActions("mower_logic", {{"reset_emergency", "Reset Emergency", true}});
  core_.OnActions("mower_logic:mowing", {{"pause", "Pause", false}});
  core_.OnActions("mower_logic", {{"reset_emergency", "Reset Emergency", false}});
  const Outbox out = core_.Take();
  ASSERT_EQ(out.mqtt.size(), 3u);
  EXPECT_TRUE(out.mqtt[2].retain);
  EXPECT_EQ(out.mqtt[2].payload,
            R"([{"action_id":"mower_logic/reset_emergency","action_name":"Reset Emergency","enabled":0},)"
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
