#ifndef MOWBITE_CAMERA_HTTP_CLIENT_H
#define MOWBITE_CAMERA_HTTP_CLIENT_H

#include <cstdint>
#include <string>

namespace mowbite_camera {

struct HttpReply {
  // 0: no answer, error tells why.
  int status = 0;
  std::string body;
  std::string error;
};

// Who asks a server on 127.0.0.1, with HTTP Basic.
struct LocalServer {
  uint16_t port = 0;
  std::string user;
  std::string password;
};

// One request without a body to a server on this host, answered within timeout_s or given up. It is sent as
// HTTP/1.0, so the answer ends with its connection.
HttpReply LocalRequest(const LocalServer& server, const std::string& method, const std::string& target,
                       double timeout_s);

std::string Base64(const std::string& bytes);

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_HTTP_CLIENT_H
