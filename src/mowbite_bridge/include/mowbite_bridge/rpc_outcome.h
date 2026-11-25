#ifndef MOWBITE_BRIDGE_RPC_OUTCOME_H
#define MOWBITE_BRIDGE_RPC_OUTCOME_H

#include <string>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// The answer of a node to a method: its result, or the code, message and data of the error.
struct RpcOutcome {
  bool ok = true;
  Json result;
  int code = 0;
  std::string message;
  // Details of an error for the caller, null for none.
  Json data;
};

RpcOutcome Ok(Json result);
RpcOutcome Error(int code, std::string message, Json data = Json());
RpcOutcome InvalidParams(std::string message);
// The data of a refusal: event, reason, message and the severity and resume mowbite_api::RateReason gives the
// reason in that event, then fields.
Json RefusalData(const std::string& event, const std::string& reason, const std::string& message,
                 const Json& fields = Json::object());
// A refusal as kInvalidRequest with RefusalData.
RpcOutcome Refusal(const std::string& event, const std::string& reason, const std::string& message,
                   const Json& fields = Json::object());
// The text of json for a message, invalid UTF-8 replaced.
std::string Dump(const Json& json);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_RPC_OUTCOME_H
