#include <curl/curl.h>

#include <string>

#include "mowbite_net/http.h"

namespace mowbite_net {

namespace {

struct Sink {
  std::string body;
  bool too_large = false;
};

size_t Collect(char* data, size_t size, size_t count, void* user) {
  auto* sink = static_cast<Sink*>(user);
  const size_t n = size * count;
  if (sink->body.size() + n > kMaxAnswerBytes) {
    sink->too_large = true;
    return 0;
  }
  sink->body.append(data, n);
  return n;
}

class CurlHttp : public Http {
 public:
  CurlHttp() : curl_(curl_easy_init()) {}
  ~CurlHttp() override {
    if (curl_) curl_easy_cleanup(curl_);
  }
  CurlHttp(const CurlHttp&) = delete;
  CurlHttp& operator=(const CurlHttp&) = delete;

  HttpResult Send(const HttpRequest& request) override {
    HttpResult result;
    if (!curl_) {
      result.error = "no curl handle";
      return result;
    }
    curl_easy_reset(curl_);
    Sink sink;
    curl_slist* headers = nullptr;
    for (const std::string& header : request.headers) headers = curl_slist_append(headers, header.c_str());
    curl_easy_setopt(curl_, CURLOPT_URL, request.url.c_str());
    curl_easy_setopt(curl_, CURLOPT_PROTOCOLS_STR, "http,https");
    curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(curl_, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, kHttpTimeout);
    curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT, kHttpTimeout);
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, Collect);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &sink);
    curl_easy_setopt(curl_, CURLOPT_USERAGENT, "mowbite-os");
    if (!request.body.empty()) {
      curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, request.body.c_str());
      curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, static_cast<long>(request.body.size()));
    }
    const CURLcode code = curl_easy_perform(curl_);
    curl_slist_free_all(headers);
    if (code != CURLE_OK) {
      result.error = sink.too_large ? "answer larger than 64 kB" : curl_easy_strerror(code);
      return result;
    }
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &result.status);
    result.body = std::move(sink.body);
    return result;
  }

 private:
  CURL* curl_;
};

}  // namespace

void InitCurl() {
  curl_global_init(CURL_GLOBAL_DEFAULT);
}

std::unique_ptr<Http> MakeCurlHttp() {
  return std::make_unique<CurlHttp>();
}

}  // namespace mowbite_net
