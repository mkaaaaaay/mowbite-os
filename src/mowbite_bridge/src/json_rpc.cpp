#include "mowbite_bridge/json_rpc.h"

#include "mowbite_bridge/text.h"

namespace mowbite_bridge {

namespace {

std::string Dump(const Json& j) {
  return j.dump(-1, ' ', false, Json::error_handler_t::replace);
}

RpcReading Refuse(const Json& id, int code, const std::string& message) {
  RpcReading reading;
  reading.kind = RpcReading::Kind::kError;
  reading.reply = ErrorReply(id, code, message);
  return reading;
}

bool StartsWith(const std::string& text, const char* prefix) {
  return text.rfind(prefix, 0) == 0;
}

}  // namespace

RpcReading ReadRequest(std::string_view payload) {
  if (NestingDepth(payload) > kMaxRequestDepth) {
    return Refuse(nullptr, kInvalidRequest, "Request is nested too deep");
  }
  const Json request = Json::parse(payload.begin(), payload.end(), nullptr, false);
  if (request.is_discarded()) return Refuse(nullptr, kParseError, "Could not parse request JSON");
  if (!request.is_object()) return Refuse(nullptr, kInvalidRequest, "Request is not a JSON object");
  const auto version = request.find("jsonrpc");
  if (version == request.end() || *version != "2.0") {
    return Refuse(nullptr, kInvalidRequest, "Invalid JSON-RPC version");
  }
  const Json id = request.value("id", Json());
  const auto method = request.find("method");
  if (method == request.end() || !method->is_string()) {
    return Refuse(id, kInvalidRequest, "Method is not a string");
  }
  RpcReading reading;
  reading.call.method = method->get<std::string>();
  if (StartsWith(reading.call.method, "meta.") || StartsWith(reading.call.method, "ext.")) return reading;
  reading.kind = RpcReading::Kind::kCall;
  reading.call.params = request.value("params", Json());
  reading.call.id = id;
  return reading;
}

std::string ResultReply(const Json& id, const Json& result) {
  return Dump(Json{{"jsonrpc", "2.0"}, {"result", result}, {"id", id}});
}

std::string ResultReplyFromText(const Json& id, std::string_view result) {
  if (NestingDepth(result) > kMaxRequestDepth * 4) {
    return ErrorReply(id, kInternalError, "The result is nested too deep");
  }
  try {
    return ResultReply(id, Json::parse(result.begin(), result.end()));
  } catch (const Json::exception& e) {
    return ErrorReply(id, kInternalError, CleanUtf8(e.what()));
  }
}

std::string ErrorReply(const Json& id, int code, std::string_view message, const Json& data) {
  Json error = {{"code", code}, {"message", CleanUtf8(message)}};
  if (!data.is_null()) error["data"] = data;
  return Dump(Json{{"jsonrpc", "2.0"}, {"error", error}, {"id", id}});
}

}  // namespace mowbite_bridge
