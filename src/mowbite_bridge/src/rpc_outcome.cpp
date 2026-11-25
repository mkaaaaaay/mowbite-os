#include "mowbite_bridge/rpc_outcome.h"

#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_bridge {

RpcOutcome Ok(Json result) {
  return {true, std::move(result), 0, "", Json()};
}

RpcOutcome Error(int code, std::string message, Json data) {
  return {false, Json(), code, std::move(message), std::move(data)};
}

RpcOutcome InvalidParams(std::string message) {
  return Error(kInvalidParams, std::move(message));
}

Json RefusalData(const std::string& event, const std::string& reason, const std::string& message, const Json& fields) {
  const mowbite_api::Rating rating = mowbite_api::RateReason(event, reason);
  Json data = {{"event", event},
               {"reason", reason},
               {"message", message},
               {"severity", rating.severity},
               {"resume", rating.resume}};
  for (const auto& [key, value] : fields.items()) {
    if (!data.contains(key)) data[key] = value;
  }
  return data;
}

RpcOutcome Refusal(const std::string& event, const std::string& reason, const std::string& message,
                   const Json& fields) {
  return Error(kInvalidRequest, message, RefusalData(event, reason, message, fields));
}

std::string Dump(const Json& json) {
  return json.dump(-1, ' ', false, Json::error_handler_t::replace);
}

}  // namespace mowbite_bridge
