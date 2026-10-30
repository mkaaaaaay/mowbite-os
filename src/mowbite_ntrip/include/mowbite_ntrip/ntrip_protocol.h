#ifndef MOWBITE_NTRIP_NTRIP_PROTOCOL_H
#define MOWBITE_NTRIP_NTRIP_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mowbite_ntrip/ntrip_config.h"

namespace mowbite_ntrip {

std::string Base64(const std::string& text);

// The GET request for the mountpoint: NTRIP 2.0 as HTTP/1.1, NTRIP 1.0 as HTTP/1.0; Basic auth only with a user.
std::string BuildRequest(const NtripConfig& config);

// Reads the caster's answer: the status line and headers, then the corrections, chunked or raw.
class ResponseReader {
 public:
  enum class Result {
    kPending,       // header not complete yet
    kStreaming,     // 200: corrections follow
    kUnauthorized,  // 401: user or password refused
    kNoMountpoint,  // 404 or the source table instead of a stream
    kError,         // any other answer, or one that makes no sense
  };

  static constexpr size_t kMaxHeader = 8192;

  // Appends the correction bytes in data to body. Once the result is not kPending it stays.
  Result Feed(const uint8_t* data, size_t size, std::vector<uint8_t>& body);
  // HTTP status code, 0 before the header or for ICY.
  int status_code() const {
    return status_code_;
  }
  // The caster's status line for errors, printable and cut to 80 characters; empty otherwise.
  const std::string& status_line() const {
    return status_line_;
  }
  // The chunked stream sent its last chunk.
  bool ended() const {
    return ended_;
  }

 private:
  enum class Phase { kHeader, kRaw, kChunkSize, kChunkData, kChunkEnd, kDone };

  Result ParseHeader(std::vector<uint8_t>& body);
  void FeedBody(const uint8_t* data, size_t size, std::vector<uint8_t>& body);
  void Fail(Result result, const std::string& line);

  Phase phase_ = Phase::kHeader;
  Result result_ = Result::kPending;
  std::string header_;
  std::string chunk_line_;
  size_t chunk_left_ = 0;
  int status_code_ = 0;
  std::string status_line_;
  bool ended_ = false;
};

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_NTRIP_PROTOCOL_H
