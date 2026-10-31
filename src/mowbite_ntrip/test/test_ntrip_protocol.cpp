#include <gtest/gtest.h>

#include <string>

#include "mowbite_ntrip/ntrip_protocol.h"

namespace mowbite_ntrip {
namespace {

using Result = ResponseReader::Result;

Result Feed(ResponseReader& reader, const std::string& text, std::vector<uint8_t>& body) {
  return reader.Feed(reinterpret_cast<const uint8_t*>(text.data()), text.size(), body);
}

std::string Text(const std::vector<uint8_t>& body) {
  return std::string(body.begin(), body.end());
}

TEST(Base64, KnownValues) {
  EXPECT_EQ(Base64(""), "");
  EXPECT_EQ(Base64("f"), "Zg==");
  EXPECT_EQ(Base64("fo"), "Zm8=");
  EXPECT_EQ(Base64("foo"), "Zm9v");
  EXPECT_EQ(Base64("foobar"), "Zm9vYmFy");
  EXPECT_EQ(Base64("Aladdin:open sesame"), "QWxhZGRpbjpvcGVuIHNlc2FtZQ==");
  EXPECT_EQ(Base64(std::string("\xff\xfe", 2)), "//4=");
}

TEST(Request, Ntrip2) {
  NtripConfig c;
  c.host = "caster.example.org";
  c.mountpoint = "VRS_TEST";
  c.user = "Aladdin";
  c.password = "open sesame";
  EXPECT_EQ(BuildRequest(c),
            "GET /VRS_TEST HTTP/1.1\r\n"
            "Host: caster.example.org:2101\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP MowBiteOS/1.0\r\n"
            "Authorization: Basic QWxhZGRpbjpvcGVuIHNlc2FtZQ==\r\n"
            "Connection: close\r\n"
            "\r\n");
}

TEST(Request, Ntrip1WithoutLogin) {
  NtripConfig c;
  c.host = "h";
  c.port = 80;
  c.mountpoint = "M";
  c.version = 1;
  EXPECT_EQ(BuildRequest(c),
            "GET /M HTTP/1.0\r\n"
            "Host: h:80\r\n"
            "User-Agent: NTRIP MowBiteOS/1.0\r\n"
            "Accept: */*\r\n"
            "\r\n");
}

TEST(Response, Ntrip2Chunked) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  EXPECT_EQ(Feed(reader, "HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n", body), Result::kPending);
  EXPECT_EQ(Feed(reader, "Transfer-Encoding: Chunked\r\nContent-Type: gnss/data\r\n\r\n5\r\nab", body),
            Result::kStreaming);
  EXPECT_EQ(Feed(reader, "cde\r\n3;ext=1\r\nfgh\r\n", body), Result::kStreaming);
  EXPECT_EQ(Text(body), "abcdefgh");
  EXPECT_EQ(reader.status_code(), 200);
  EXPECT_FALSE(reader.ended());
  Feed(reader, "0\r\n\r\n", body);
  EXPECT_TRUE(reader.ended());
}

TEST(Response, ChunkedByteByByte) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  const std::string all = "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nA\r\n0123456789\r\n2\r\nxy\r\n";
  Result r = Result::kPending;
  for (char c : all) r = Feed(reader, std::string(1, c), body);
  EXPECT_EQ(r, Result::kStreaming);
  EXPECT_EQ(Text(body), "0123456789xy");
}

TEST(Response, Ntrip2Raw) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  const std::string answer = "HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n";
  EXPECT_EQ(Feed(reader, answer + std::string("\xd3\x00", 2), body), Result::kStreaming);
  EXPECT_EQ(body.size(), 2u);
}

TEST(Response, Ntrip1Icy) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  EXPECT_EQ(Feed(reader, "ICY 2", body), Result::kPending);
  EXPECT_EQ(Feed(reader, std::string("00 OK\r\n\xd3\x00\x13", 10), body), Result::kStreaming);
  EXPECT_EQ(body.size(), 3u);
  EXPECT_EQ(reader.status_code(), 0);
}

TEST(Response, Unauthorized) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  EXPECT_EQ(Feed(reader, "HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic realm=\"/M\"\r\n\r\n", body),
            Result::kUnauthorized);
  EXPECT_EQ(reader.status_code(), 401);
  EXPECT_EQ(reader.status_line(), "HTTP/1.1 401 Unauthorized");
  // Stays so, whatever follows.
  EXPECT_EQ(Feed(reader, "more", body), Result::kUnauthorized);
  EXPECT_TRUE(body.empty());
}

TEST(Response, SourceTableMeansNoSuchMountpoint) {
  ResponseReader a, b, c;
  std::vector<uint8_t> body;
  EXPECT_EQ(Feed(a, "SOURCETABLE 200 OK\r\nServer: x\r\n", body), Result::kNoMountpoint);
  EXPECT_EQ(Feed(b, "HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\nSTR;", body), Result::kNoMountpoint);
  EXPECT_EQ(Feed(c, "HTTP/1.1 404 Not Found\r\n\r\n", body), Result::kNoMountpoint);
  EXPECT_TRUE(body.empty());
}

TEST(Response, OtherAnswers) {
  ResponseReader a, b, c;
  std::vector<uint8_t> body;
  EXPECT_EQ(Feed(a, "HTTP/1.1 503 Service Unavailable\r\n\r\n", body), Result::kError);
  EXPECT_EQ(a.status_line(), "HTTP/1.1 503 Service Unavailable");
  EXPECT_EQ(Feed(b, "SSH-2.0-OpenSSH\r\n\r\n", body), Result::kError);
  EXPECT_EQ(Feed(c, std::string(ResponseReader::kMaxHeader + 1, 'x'), body), Result::kError);
}

TEST(Response, StatusLineIsCutAndPrintable) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  Feed(reader, "HTTP/1.1 500 " + std::string(200, 'e') + "\x01\r\n\r\n", body);
  EXPECT_EQ(reader.status_line().size(), 80u);
}

TEST(Response, StatusLineIsAsciiOnly) {
  ResponseReader a, b;
  std::vector<uint8_t> body;
  // The cut after 80 bytes used to keep the first byte of this Ä.
  const std::string head = "HTTP/1.1 500 " + std::string(66, 'e');
  Feed(a, head + "\xc3\x84rger\r\n\r\n", body);
  EXPECT_EQ(a.status_line(), head + "?");
  Feed(b, "HTTP/1.1 503 Gr\xc3\xbc\xc3\x9f" "e \xe4\x7f\r\n\r\n", body);
  EXPECT_EQ(b.status_line(), "HTTP/1.1 503 Gr????e ??");
}

TEST(Response, BadChunkSize) {
  ResponseReader reader;
  std::vector<uint8_t> body;
  EXPECT_EQ(Feed(reader, "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nzz\r\n", body), Result::kError);
}

}  // namespace
}  // namespace mowbite_ntrip
