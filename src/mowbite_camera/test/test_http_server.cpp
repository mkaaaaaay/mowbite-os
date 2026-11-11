#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "http_test_client.h"
#include "mowbite_camera/http_server.h"

namespace mowbite_camera {
namespace {

constexpr char kGet[] = "GET /camera/cam.mjpg?t=abc HTTP/1.1\r\nHost: mower\r\n\r\n";

class HttpServerTest : public ::testing::Test {
 protected:
  HttpServerTest()
      : server_(
            [this](const HttpRequest& request) {
              requests_.push_back(request);
              return answer_;
            },
            [this](uint64_t connection) { gone_.push_back(connection); }, [this] { return now_; }) {}

  int Listen(const ListenerRules& rules = ListenerRules()) {
    std::string error;
    const int listener = server_.Listen("127.0.0.1", 0, rules, &error);
    EXPECT_GE(listener, 0) << error;
    return listener;
  }

  // Lets the server and the clients work until done, at most about 2 s.
  bool Until(const std::function<bool()>& done, const std::vector<TestClient*>& clients = {}) {
    for (int round = 0; round < 400; ++round) {
      server_.Poll(5);
      for (TestClient* client : clients) client->Take();
      if (done()) return true;
    }
    return false;
  }

  // A viewer of camera "cam" that has the head of the stream.
  void Watch(TestClient& client) {
    answer_ = {200, "", "cam"};
    client.Send(kGet);
    ASSERT_TRUE(Until([&] { return Has(client.in(), "\r\n\r\n"); }, {&client}));
  }

  double now_ = 1000.0;
  HttpAnswer answer_;
  std::vector<HttpRequest> requests_;
  std::vector<uint64_t> gone_;
  HttpServer server_;
};

TEST(HttpHead, TakesARequestApart) {
  HttpRequest request;
  size_t length = 7;
  ASSERT_TRUE(ParseHead(kGet, &request, &length));
  EXPECT_EQ(request.method, "GET");
  EXPECT_EQ(request.path, "/camera/cam.mjpg");
  EXPECT_EQ(request.query, "t=abc");
  EXPECT_EQ(length, 0u);
  ASSERT_TRUE(ParseHead("POST /auth HTTP/1.0\r\ncontent-LENGTH:  12 \r\n\r\n", &request, &length));
  EXPECT_EQ(request.method, "POST");
  EXPECT_EQ(request.path, "/auth");
  EXPECT_EQ(request.query, "");
  EXPECT_EQ(length, 12u);
  ASSERT_TRUE(ParseHead("GET /?a=1?b HTTP/1.1\r\n\r\n", &request, &length));
  EXPECT_EQ(request.path, "/");
  EXPECT_EQ(request.query, "a=1?b");
}

TEST(HttpHead, RefusesWhatIsNoRequestForIt) {
  HttpRequest request;
  size_t length = 0;
  for (const char* head :
       {"", "GET / HTTP/1.1", "get / HTTP/1.1\r\n\r\n", "GET  / HTTP/1.1\r\n\r\n", "GET / HTTP/2.0\r\n\r\n",
        "GET / HTTP/1.1 x\r\n\r\n", "GET camera HTTP/1.1\r\n\r\n", "GET /a\tb HTTP/1.1\r\n\r\n",
        "GET / HTTP/1.1\r\nno colon\r\n\r\n", "GET / HTTP/1.1\r\n: empty name\r\n\r\n",
        "POST / HTTP/1.1\r\nContent-Length: -1\r\n\r\n", "POST / HTTP/1.1\r\nContent-Length: 1e3\r\n\r\n",
        "POST / HTTP/1.1\r\nContent-Length: 1234567890\r\n\r\n", "POST / HTTP/1.1\r\nContent-Length:\r\n\r\n",
        "POST / HTTP/1.1\r\nContent-Length: 1\r\nContent-Length: 2\r\n\r\n",
        "POST / HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n", "VERYLONGMETHODNAMEX / HTTP/1.1\r\n\r\n"}) {
    EXPECT_FALSE(ParseHead(head, &request, &length)) << head;
  }
}

TEST(HttpHead, FindsAValueOfAQuery) {
  EXPECT_EQ(QueryValue("t=abc", "t"), "abc");
  EXPECT_EQ(QueryValue("a=1&t=x&b=2", "t"), "x");
  EXPECT_EQ(QueryValue("a=1&t=", "t"), "");
  EXPECT_EQ(QueryValue("tt=1&at=2", "t"), "");
  EXPECT_EQ(QueryValue("", "t"), "");
  EXPECT_EQ(QueryValue("t", "t"), "");
  EXPECT_EQ(QueryValue("t=a=b", "t"), "a=b");
}

TEST_F(HttpServerTest, AnswersARequestAndEndsTheConnection) {
  const int listener = Listen();
  answer_ = {404, "not here\n", ""};
  TestClient client(server_.Port(listener));
  ASSERT_TRUE(client.connected());
  client.Send(kGet);
  ASSERT_TRUE(Until([&] { return client.closed(); }, {&client}));
  EXPECT_EQ(client.in(), "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: 9\r\n"
                         "Cache-Control: no-store\r\nConnection: close\r\n\r\nnot here\n");
  ASSERT_EQ(requests_.size(), 1u);
  EXPECT_EQ(requests_[0].listener, listener);
  EXPECT_EQ(requests_[0].method, "GET");
  EXPECT_EQ(requests_[0].path, "/camera/cam.mjpg");
  EXPECT_EQ(requests_[0].query, "t=abc");
  EXPECT_TRUE(gone_.empty());
  EXPECT_EQ(server_.Connections(), 0u);
}

TEST_F(HttpServerTest, TakesARequestThatComesInPieces) {
  ListenerRules rules;
  rules.max_body = 64;
  const int listener = Listen(rules);
  TestClient client(server_.Port(listener));
  const std::string request = "POST /auth HTTP/1.1\r\nContent-Length: 11\r\n\r\nhello world";
  for (const char c : request) {
    client.Send(std::string(1, c));
    server_.Poll(1);
  }
  ASSERT_TRUE(Until([&] { return client.closed(); }, {&client}));
  ASSERT_EQ(requests_.size(), 1u);
  EXPECT_EQ(requests_[0].body, "hello world");
  EXPECT_TRUE(Has(client.in(), "HTTP/1.1 200 OK\r\n"));
  EXPECT_TRUE(Has(client.in(), "Content-Length: 0\r\n"));
}

TEST_F(HttpServerTest, RefusesALongHeadALongBodyAndWhatIsNoRequest) {
  ListenerRules rules;
  rules.max_body = 8;
  const int listener = Listen(rules);
  TestClient long_head(server_.Port(listener));
  long_head.Send("GET /" + std::string(rules.max_head, 'a'));
  TestClient long_body(server_.Port(listener));
  long_body.Send("POST /auth HTTP/1.1\r\nContent-Length: 9\r\n\r\n123456789");
  TestClient no_request(server_.Port(listener));
  no_request.Send("hello\r\n\r\n");
  ASSERT_TRUE(Until([&] { return long_head.closed() && long_body.closed() && no_request.closed(); },
                    {&long_head, &long_body, &no_request}));
  EXPECT_TRUE(Has(long_head.in(), "HTTP/1.1 431 "));
  EXPECT_TRUE(Has(long_body.in(), "HTTP/1.1 413 "));
  EXPECT_TRUE(Has(no_request.in(), "HTTP/1.1 400 "));
  EXPECT_TRUE(requests_.empty());
}

TEST_F(HttpServerTest, AHeadOfExactlyTheLimitIsTaken) {
  ListenerRules rules;
  const int listener = Listen(rules);
  const std::string line = "GET / HTTP/1.1\r\nX: ";
  TestClient client(server_.Port(listener));
  client.Send(line + std::string(rules.max_head - line.size() - 4, 'x') + "\r\n\r\n");
  ASSERT_TRUE(Until([&] { return client.closed(); }, {&client}));
  EXPECT_EQ(requests_.size(), 1u);
}

TEST_F(HttpServerTest, DropsARequestThatIsNotCompleteInTime) {
  const int listener = Listen();
  TestClient client(server_.Port(listener));
  client.Send("GET /camera/cam.mjpg?t=abc HTTP/1.1\r\n");
  ASSERT_TRUE(Until([&] { return server_.Connections() == 1; }));
  now_ += 4.9;
  server_.Poll(1);
  EXPECT_EQ(server_.Connections(), 1u);
  now_ += 0.2;
  ASSERT_TRUE(Until([&] { return client.closed(); }, {&client}));
  EXPECT_EQ(client.in(), "");
  EXPECT_TRUE(requests_.empty());
  EXPECT_TRUE(gone_.empty());
}

TEST_F(HttpServerTest, TakesOnlySoManyConnectionsWithoutARequest) {
  ListenerRules rules;
  rules.max_pending = 2;
  const int listener = Listen(rules);
  const int other = Listen(rules);
  TestClient first(server_.Port(listener));
  TestClient second(server_.Port(listener));
  ASSERT_TRUE(Until([&] { return server_.Connections() == 2; }));
  TestClient third(server_.Port(listener));
  ASSERT_TRUE(Until([&] { return third.closed(); }, {&third}));
  EXPECT_EQ(third.in(), "");
  // Every listener counts its own.
  TestClient elsewhere(server_.Port(other));
  ASSERT_TRUE(Until([&] { return server_.Connections() == 3; }));

  // A connection with its request makes room, also one that stays for pictures.
  Watch(first);
  TestClient fourth(server_.Port(listener));
  ASSERT_TRUE(Until([&] { return server_.Connections() == 4; }));
  fourth.Take();
  EXPECT_FALSE(fourth.closed());
}

TEST_F(HttpServerTest, SendsThePicturesOfItsCameraAsMultipart) {
  ListenerRules rules;
  rules.any_origin = true;
  const int listener = Listen(rules);
  TestClient client(server_.Port(listener));
  Watch(client);
  EXPECT_EQ(client.in(), "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                         "Cache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n");
  const size_t head = client.in().size();
  server_.Push("other", "XXXX");
  server_.Push("cam", "");
  server_.Push("cam", "JPEG1");
  ASSERT_TRUE(Until([&] { return Has(client.in(), "JPEG1\r\n"); }, {&client}));
  server_.Push("cam", "JPEG22");
  ASSERT_TRUE(Until([&] { return Has(client.in(), "JPEG22\r\n"); }, {&client}));
  EXPECT_EQ(client.in().substr(head),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 5\r\n\r\nJPEG1\r\n"
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 6\r\n\r\nJPEG22\r\n");
  EXPECT_FALSE(client.closed());

  client.Close();
  ASSERT_TRUE(Until([&] { return gone_.size() == 1; }));
  EXPECT_EQ(gone_[0], requests_[0].connection);
}

TEST_F(HttpServerTest, AViewerBusyWithAPictureGetsOnlyTheNewestAfterIt) {
  const int listener = Listen();
  TestClient client(server_.Port(listener));
  Watch(client);
  // More than the buffers of the connection hold: it does not get out while the client takes nothing.
  const std::string large(32u << 20, 'a');
  server_.Push("cam", large);
  for (int round = 0; round < 20; ++round) server_.Poll(1);
  server_.Push("cam", "SKIPPED");
  server_.Push("cam", "NEWEST");
  const std::string end = "Content-Length: 6\r\n\r\nNEWEST\r\n";
  const auto ends_with_newest = [&] {
    const std::string& in = client.in();
    return in.size() > end.size() && in.compare(in.size() - end.size(), end.size(), end) == 0;
  };
  ASSERT_TRUE(Until(ends_with_newest, {&client}));
  EXPECT_TRUE(Has(client.in(), "Content-Length: " + std::to_string(large.size())));
  EXPECT_FALSE(Has(client.in(), "SKIPPED"));
}

TEST_F(HttpServerTest, DropsAViewerThatDoesNotTakeAPictureInTime) {
  const int listener = Listen();
  TestClient client(server_.Port(listener));
  Watch(client);
  server_.Push("cam", std::string(32u << 20, 'a'));
  for (int round = 0; round < 20; ++round) server_.Poll(1);
  now_ += 4.9;
  server_.Poll(1);
  EXPECT_TRUE(gone_.empty());
  now_ += 0.2;
  server_.Poll(1);
  ASSERT_EQ(gone_.size(), 1u);
  EXPECT_EQ(server_.Connections(), 0u);
}

TEST_F(HttpServerTest, DropsAViewerTheCameraHasNoPictureFor) {
  const int listener = Listen();
  TestClient client(server_.Port(listener));
  Watch(client);
  now_ += 9.0;
  server_.Push("cam", "JPEG");
  ASSERT_TRUE(Until([&] { return Has(client.in(), "JPEG\r\n"); }, {&client}));
  now_ += 9.9;
  server_.Poll(1);
  EXPECT_TRUE(gone_.empty());
  now_ += 0.2;
  ASSERT_TRUE(Until([&] { return client.closed(); }, {&client}));
  EXPECT_EQ(gone_.size(), 1u);
}

TEST_F(HttpServerTest, DropsAViewerThatSendsMore) {
  const int listener = Listen();
  TestClient client(server_.Port(listener));
  Watch(client);
  client.Send("x");
  ASSERT_TRUE(Until([&] { return gone_.size() == 1; }));
}

TEST_F(HttpServerTest, CloseEndsAConnection) {
  const int listener = Listen();
  TestClient client(server_.Port(listener));
  TestClient stays(server_.Port(listener));
  Watch(client);
  Watch(stays);
  server_.Close(requests_[0].connection);
  ASSERT_TRUE(Until([&] { return client.closed(); }, {&client, &stays}));
  EXPECT_EQ(gone_, std::vector<uint64_t>{requests_[0].connection});
  EXPECT_FALSE(stays.closed());
  server_.Push("cam", "JPEG");
  ASSERT_TRUE(Until([&] { return Has(stays.in(), "JPEG\r\n"); }, {&stays}));
}

TEST_F(HttpServerTest, ATaskFromAnotherThreadWakesPoll) {
  using std::chrono::steady_clock;
  Listen();
  bool ran = false;
  const auto before = steady_clock::now();
  std::thread other([&] {
    std::this_thread::sleep_until(before + std::chrono::milliseconds(50));
    server_.Post([&] { ran = true; });
  });
  server_.Poll(5000);
  other.join();
  EXPECT_TRUE(ran);
  EXPECT_LT(steady_clock::now() - before, std::chrono::seconds(3));
}

TEST_F(HttpServerTest, TellsWhyItCannotListen) {
  std::string error;
  EXPECT_EQ(server_.Listen("localhost", 0, ListenerRules(), &error), -1);
  EXPECT_NE(error, "");
  const int listener = Listen();
  error.clear();
  HttpServer second([](const HttpRequest&) { return HttpAnswer(); }, [](uint64_t) {}, [] { return 0.0; });
  EXPECT_EQ(second.Listen("127.0.0.1", server_.Port(listener), ListenerRules(), &error), -1);
  EXPECT_NE(error, "");
}

}  // namespace
}  // namespace mowbite_camera
