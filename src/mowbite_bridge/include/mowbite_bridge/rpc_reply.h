#ifndef MOWBITE_BRIDGE_RPC_REPLY_H
#define MOWBITE_BRIDGE_RPC_REPLY_H

#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "mowbite_bridge/rpc_outcome.h"

namespace mowbite_bridge {

// The answer of a node to request id: the result on bridge/rpc/response, else the error on bridge/rpc/error.
inline void PublishOutcome(const std::string& id, const RpcOutcome& outcome,
                           rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>& response,
                           rclcpp::Publisher<mowbite_msgs::msg::RpcError>& error) {
  if (outcome.ok) {
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = Dump(outcome.result);
    msg.id = id;
    response.publish(msg);
    return;
  }
  mowbite_msgs::msg::RpcError msg;
  msg.code = outcome.code;
  msg.message = outcome.message;
  msg.id = id;
  if (!outcome.data.is_null()) msg.data = Dump(outcome.data);
  error.publish(msg);
}

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_RPC_REPLY_H
