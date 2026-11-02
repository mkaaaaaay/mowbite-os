#ifndef MOWBITE_BRIDGE_JSON_RPC_H
#define MOWBITE_BRIDGE_JSON_RPC_H

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

namespace mowbite_bridge {

using Json = nlohmann::ordered_json;

// Error codes of JSON-RPC 2.0.
constexpr int kParseError = -32700;
constexpr int kInvalidRequest = -32600;
constexpr int kMethodNotFound = -32601;
constexpr int kInvalidParams = -32602;
constexpr int kInternalError = -32603;

// Requests nested deeper than this are refused before they are parsed.
constexpr std::size_t kMaxRequestDepth = 64;

struct RpcCall {
  std::string method;
  // null when the request has none.
  Json params;
  // A string, or null when the request has none.
  Json id;
};

// What a payload on rpc/request asks for: a call, nothing (meta.* and ext.*), or an error to send back at once.
struct RpcReading {
  enum class Kind { kCall, kSilent, kError } kind = Kind::kSilent;
  RpcCall call;
  // The whole error answer for kError.
  std::string reply;
};

RpcReading ReadRequest(std::string_view payload);

// An answer with a result, compact; bytes that are not UTF-8 become U+FFFD.
std::string ResultReply(const Json& id, const Json& result);

// An answer with the result a node sent as JSON text; text that does not parse gives -32603 with the parser's words.
std::string ResultReplyFromText(const Json& id, std::string_view result);

// An error answer; data only when it is not null.
std::string ErrorReply(const Json& id, int code, std::string_view message, const Json& data = nullptr);

// Thrown by a local method to answer with this error.
struct RpcFailure : std::runtime_error {
  RpcFailure(int code, const std::string& message) : std::runtime_error(message), code(code) {}
  int code;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_JSON_RPC_H
