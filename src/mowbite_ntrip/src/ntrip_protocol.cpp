#include "mowbite_ntrip/ntrip_protocol.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace mowbite_ntrip {

namespace {

constexpr char kUserAgent[] = "NTRIP MowBiteOS/1.0";

std::string Lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

// Printable ASCII only: the cut at 80 bytes can then split no character, and logs and JSON get no broken one.
std::string Printable(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (out.size() >= 80) break;
    const auto byte = static_cast<unsigned char>(c);
    out.push_back(byte >= ' ' && byte < 0x7F ? c : '?');
  }
  return out;
}

bool StartsWith(const std::string& s, const std::string& prefix) {
  return s.compare(0, prefix.size(), prefix) == 0;
}

}  // namespace

std::string Base64(const std::string& text) {
  static const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < text.size(); i += 3) {
    const uint32_t v = (static_cast<uint8_t>(text[i]) << 16) | (static_cast<uint8_t>(text[i + 1]) << 8) |
                       static_cast<uint8_t>(text[i + 2]);
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += kAlphabet[(v >> 6) & 63];
    out += kAlphabet[v & 63];
  }
  const size_t rest = text.size() - i;
  if (rest > 0) {
    uint32_t v = static_cast<uint8_t>(text[i]) << 16;
    if (rest == 2) v |= static_cast<uint8_t>(text[i + 1]) << 8;
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += rest == 2 ? kAlphabet[(v >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

std::string BuildRequest(const NtripConfig& config) {
  std::string request = "GET /" + config.mountpoint + (config.version == 1 ? " HTTP/1.0\r\n" : " HTTP/1.1\r\n");
  request += "Host: " + config.host + ":" + std::to_string(config.port) + "\r\n";
  if (config.version != 1) request += "Ntrip-Version: Ntrip/2.0\r\n";
  request += std::string("User-Agent: ") + kUserAgent + "\r\n";
  if (config.version == 1) request += "Accept: */*\r\n";
  if (!config.user.empty()) request += "Authorization: Basic " + Base64(config.user + ":" + config.password) + "\r\n";
  if (config.version != 1) request += "Connection: close\r\n";
  request += "\r\n";
  return request;
}

void ResponseReader::Fail(Result result, const std::string& line) {
  result_ = result;
  status_line_ = Printable(line);
  phase_ = Phase::kDone;
}

ResponseReader::Result ResponseReader::Feed(const uint8_t* data, size_t size, std::vector<uint8_t>& body) {
  if (phase_ == Phase::kHeader) {
    // The bytes past the end of the header go on to the body in ParseHeader.
    header_.append(reinterpret_cast<const char*>(data), size);
    return ParseHeader(body);
  }
  if (phase_ != Phase::kDone) FeedBody(data, size, body);
  return result_;
}

ResponseReader::Result ResponseReader::ParseHeader(std::vector<uint8_t>& body) {
  const size_t first_end = header_.find("\r\n");
  if (first_end == std::string::npos) {
    if (header_.size() > kMaxHeader) Fail(Result::kError, "answer without a line end");
    return result_;
  }
  const std::string first = header_.substr(0, first_end);
  // NTRIP 1.0: the corrections follow the status line directly.
  if (StartsWith(first, "ICY 200")) {
    result_ = Result::kStreaming;
    phase_ = Phase::kRaw;
    const std::string rest = header_.substr(first_end + 2);
    header_.clear();
    FeedBody(reinterpret_cast<const uint8_t*>(rest.data()), rest.size(), body);
    return result_;
  }
  if (StartsWith(first, "SOURCETABLE")) {
    Fail(Result::kNoMountpoint, first);
    return result_;
  }
  const size_t end = header_.find("\r\n\r\n");
  if (end == std::string::npos) {
    if (header_.size() > kMaxHeader) Fail(Result::kError, "header too long");
    return result_;
  }
  if (!StartsWith(first, "HTTP/1.")) {
    Fail(Result::kError, first);
    return result_;
  }
  const size_t space = first.find(' ');
  status_code_ = space == std::string::npos ? 0 : std::atoi(first.c_str() + space + 1);
  if (status_code_ == 401 || status_code_ == 403) {
    Fail(Result::kUnauthorized, first);
    return result_;
  }
  if (status_code_ == 404) {
    Fail(Result::kNoMountpoint, first);
    return result_;
  }
  if (status_code_ != 200) {
    Fail(Result::kError, first);
    return result_;
  }
  bool chunked = false;
  bool source_table = false;
  size_t pos = first_end + 2;
  while (pos < end) {
    const size_t line_end = header_.find("\r\n", pos);
    const std::string line = Lower(header_.substr(pos, line_end - pos));
    if (StartsWith(line, "transfer-encoding:") && line.find("chunked") != std::string::npos) chunked = true;
    if (StartsWith(line, "content-type:") && line.find("gnss/sourcetable") != std::string::npos) source_table = true;
    pos = line_end + 2;
  }
  if (source_table) {
    Fail(Result::kNoMountpoint, first);
    return result_;
  }
  result_ = Result::kStreaming;
  phase_ = chunked ? Phase::kChunkSize : Phase::kRaw;
  const std::string rest = header_.substr(end + 4);
  header_.clear();
  FeedBody(reinterpret_cast<const uint8_t*>(rest.data()), rest.size(), body);
  return result_;
}

void ResponseReader::FeedBody(const uint8_t* data, size_t size, std::vector<uint8_t>& body) {
  size_t i = 0;
  while (i < size && phase_ != Phase::kDone) {
    switch (phase_) {
      case Phase::kRaw:
        body.insert(body.end(), data + i, data + size);
        return;
      case Phase::kChunkSize: {
        const char c = static_cast<char>(data[i++]);
        if (c != '\n') {
          if (chunk_line_.size() > 64) {
            Fail(Result::kError, "bad chunk size");
            return;
          }
          chunk_line_.push_back(c);
          break;
        }
        // Size in hex, maybe with extensions after ';'.
        const std::string hex = chunk_line_.substr(0, chunk_line_.find_first_of(";\r"));
        chunk_line_.clear();
        char* end = nullptr;
        const unsigned long n = std::strtoul(hex.c_str(), &end, 16);
        if (hex.empty() || *end != '\0') {
          Fail(Result::kError, "bad chunk size");
          return;
        }
        if (n == 0) {
          ended_ = true;
          phase_ = Phase::kDone;
          return;
        }
        chunk_left_ = n;
        phase_ = Phase::kChunkData;
        break;
      }
      case Phase::kChunkData: {
        const size_t take = std::min(chunk_left_, size - i);
        body.insert(body.end(), data + i, data + i + take);
        i += take;
        chunk_left_ -= take;
        if (chunk_left_ == 0) phase_ = Phase::kChunkEnd;
        break;
      }
      case Phase::kChunkEnd:
        // The CR LF after the chunk data.
        if (data[i++] == '\n') phase_ = Phase::kChunkSize;
        break;
      case Phase::kHeader:
      case Phase::kDone: return;
    }
  }
}

}  // namespace mowbite_ntrip
