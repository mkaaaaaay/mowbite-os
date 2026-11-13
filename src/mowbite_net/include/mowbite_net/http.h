#ifndef MOWBITE_NET_HTTP_H
#define MOWBITE_NET_HTTP_H

#include <memory>
#include <string>
#include <vector>

namespace mowbite_net {

// Each request gives up after this long (s), and an answer may hold this much.
constexpr long kHttpTimeout = 10;
constexpr size_t kMaxAnswerBytes = size_t{64} * 1024;

struct HttpRequest {
  std::string url;
  std::vector<std::string> headers;  // "Name: value"
  std::string body;                  // POST with a body, GET without
};

struct HttpResult {
  long status = 0;    // 0: no answer (no network, timeout, too large, TLS)
  std::string body;
  std::string error;  // why there was no answer; never the URL or a header
};

// The one way out to the network, replaced by a fake in the tests.
class Http {
 public:
  virtual ~Http() = default;
  virtual HttpResult Send(const HttpRequest& request) = 0;
};

// libcurl: http and https only, no redirects, certificates checked, kHttpTimeout, kMaxAnswerBytes.
std::unique_ptr<Http> MakeCurlHttp();
// Once per process before the first MakeCurlHttp, while there is one thread.
void InitCurl();

}  // namespace mowbite_net

#endif  // MOWBITE_NET_HTTP_H
