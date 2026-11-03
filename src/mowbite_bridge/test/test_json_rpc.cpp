#include <gtest/gtest.h>

#include "mowbite_bridge/json_rpc.h"

using namespace mowbite_bridge;

namespace {

Json ErrorOf(const std::string& payload) {
  const RpcReading r = ReadRequest(payload);
  EXPECT_EQ(r.kind, RpcReading::Kind::kError) << payload;
  return Json::parse(r.reply);
}

}  // namespace

// B7
TEST(ReadRequest, Call) {
  const RpcReading r = ReadRequest(R"({"jsonrpc":"2.0","method":"map.replace","params":[{"areas":[]}],"id":"7"})");
  ASSERT_EQ(r.kind, RpcReading::Kind::kCall);
  EXPECT_EQ(r.call.method, "map.replace");
  EXPECT_EQ(r.call.id, "7");
  EXPECT_TRUE(r.call.params.is_array());
}

TEST(ReadRequest, WithoutIdOrParams) {
  const RpcReading r = ReadRequest(R"({"jsonrpc":"2.0","method":"rpc.ping"})");
  ASSERT_EQ(r.kind, RpcReading::Kind::kCall);
  EXPECT_TRUE(r.call.id.is_null());
  EXPECT_TRUE(r.call.params.is_null());
  // params of any kind reach the method.
  EXPECT_EQ(ReadRequest(R"({"jsonrpc":"2.0","method":"rpc.ping","params":"x","id":"n3"})").kind,
            RpcReading::Kind::kCall);
}

TEST(ReadRequest, ParseErrors) {
  for (const std::string bad : {"{broken", "", "1e400", R"({"jsonrpc":"2.0","method":"x","id":"1","params":[1e400]})",
                                "{\"jsonrpc\":\"2.0\",\"method\":\"no.such\xe4\",\"id\":\"1\"}"}) {
    const Json e = ErrorOf(bad);
    EXPECT_EQ(e["error"]["code"], kParseError) << bad;
    EXPECT_EQ(e["error"]["message"], "Could not parse request JSON");
    EXPECT_TRUE(e["id"].is_null());
  }
}

TEST(ReadRequest, InvalidRequests) {
  const auto check = [](const std::string& payload, const std::string& message, const Json& id) {
    const Json e = ErrorOf(payload);
    EXPECT_EQ(e["jsonrpc"], "2.0");
    EXPECT_EQ(e["error"]["code"], kInvalidRequest) << payload;
    EXPECT_EQ(e["error"]["message"], message) << payload;
    EXPECT_EQ(e["id"], id) << payload;
    EXPECT_FALSE(e["error"].contains("data"));
  };
  check(R"([{"jsonrpc":"2.0","method":"rpc.ping","id":"b1"}])", "Request is not a JSON object", nullptr);
  check("\"text\"", "Request is not a JSON object", nullptr);
  check(R"({"method":"rpc.ping","id":"n1"})", "Invalid JSON-RPC version", nullptr);
  check(R"({"jsonrpc":"1.0","method":"rpc.ping","id":"n1"})", "Invalid JSON-RPC version", nullptr);
  check(R"({"jsonrpc":"2.0","method":7,"id":"n2"})", "Method is not a string", "n2");
  check(R"({"jsonrpc":"2.0","id":"d3"})", "Method is not a string", "d3");
  // The id comes back as it was sent.
  check(R"({"jsonrpc":"2.0","method":"rpc.ping","id":5})", "ID is not a string", 5);
  check(R"({"jsonrpc":"2.0","method":"rpc.ping","id":{"a":1}})", "ID is not a string", Json{{"a", 1}});
  check(R"({"jsonrpc":"2.0","method":"x","id":"1","params":)" + std::string(200, '[') + std::string(200, ']') + "}",
        "Request is nested too deep", nullptr);
}

TEST(ReadRequest, MetaAndExtAreNeverAnswered) {
  EXPECT_EQ(ReadRequest(R"({"jsonrpc":"2.0","method":"meta.version","id":"1"})").kind, RpcReading::Kind::kSilent);
  EXPECT_EQ(ReadRequest(R"({"jsonrpc":"2.0","method":"ext.anything","id":"1"})").kind, RpcReading::Kind::kSilent);
  EXPECT_EQ(ReadRequest(R"({"jsonrpc":"2.0","method":"meta.x","id":5})").kind, RpcReading::Kind::kSilent);
  EXPECT_EQ(ReadRequest(R"({"jsonrpc":"2.0","method":"metadata","id":"1"})").kind, RpcReading::Kind::kCall);
}

TEST(ReadRequest, DeepButAllowedNesting) {
  const std::string params = std::string(kMaxRequestDepth - 1, '[') + std::string(kMaxRequestDepth - 1, ']');
  EXPECT_EQ(ReadRequest(R"({"jsonrpc":"2.0","method":"x","id":"1","params":)" + params + "}").kind,
            RpcReading::Kind::kCall);
}

// B10
TEST(Replies, Result) {
  EXPECT_EQ(ResultReply("p1", "pong"), R"({"jsonrpc":"2.0","result":"pong","id":"p1"})");
  EXPECT_EQ(ResultReply(nullptr, "pong"), R"({"jsonrpc":"2.0","result":"pong","id":null})");
}

TEST(Replies, ResultFromNodeText) {
  const Json ok = Json::parse(ResultReplyFromText("9", R"({"b":1,"a":2})"));
  EXPECT_EQ(ok["result"].dump(), R"({"b":1,"a":2})");
  EXPECT_EQ(ok["id"], "9");
  const Json bad = Json::parse(ResultReplyFromText("9", "{broken"));
  EXPECT_EQ(bad["error"]["code"], kInternalError);
  EXPECT_NE(bad["error"]["message"].get<std::string>().find("parse error"), std::string::npos);
  EXPECT_EQ(Json::parse(ResultReplyFromText("9", ""))["error"]["code"], kInternalError);
}

TEST(Replies, ErrorWithAndWithoutData) {
  const Json refused = Json::parse(ErrorReply("7", kInvalidRequest, "No docking station",
                                              Json{{"reason", "NO_DOCK"}, {"message", "No docking station"}}));
  EXPECT_EQ(refused["error"]["code"], kInvalidRequest);
  EXPECT_EQ(refused["error"]["data"]["reason"], "NO_DOCK");
  EXPECT_FALSE(Json::parse(ErrorReply("7", kInvalidRequest, "busy"))["error"].contains("data"));
  EXPECT_EQ(ErrorReply("u1", kMethodNotFound, "Method \"x\" not found"),
            R"({"jsonrpc":"2.0","error":{"code":-32601,"message":"Method \"x\" not found"},"id":"u1"})");
}

TEST(Replies, BytesThatAreNotUtf8BecomeReplacements) {
  const Json error = Json::parse(ErrorReply(Json("i\xff"), kInvalidParams, "caster Gr\xe4" "fin"));
  EXPECT_EQ(error["error"]["message"], "caster Gr\xef\xbf\xbd" "fin");
  EXPECT_EQ(error["id"], "i\xef\xbf\xbd");
  EXPECT_EQ(Json::parse(ResultReply(Json("i\xff"), "x"))["id"], "i\xef\xbf\xbd");
  // The parser's words quote the bytes it read last.
  const Json bad = Json::parse(ResultReplyFromText("9", "{\"a\": \"Gr\xe4" "fin\"}"));
  EXPECT_EQ(bad["error"]["code"], kInternalError);
  EXPECT_NE(bad["error"]["message"].get<std::string>().find("Gr\xef\xbf\xbd"), std::string::npos)
      << bad["error"]["message"];
}
