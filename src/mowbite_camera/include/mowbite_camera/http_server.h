#ifndef MOWBITE_CAMERA_HTTP_SERVER_H
#define MOWBITE_CAMERA_HTTP_SERVER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace mowbite_camera {

// What one listener takes and how long it waits.
struct ListenerRules {
  // Longest head and longest body of a request, in bytes.
  size_t max_head = 2048;
  size_t max_body = 0;
  // A request has to be complete this long after its connection came, in s.
  double request_seconds = 5.0;
  // Connections without a complete request at once; one more is closed as it comes.
  int max_pending = 8;
  // Output the client does not take for this long ends the connection, in s.
  double send_seconds = 5.0;
  // A viewer the camera has no picture for this long is disconnected, in s.
  double picture_seconds = 10.0;
  // Answers carry Access-Control-Allow-Origin: *.
  bool any_origin = false;
};

struct HttpRequest {
  int listener = 0;
  uint64_t connection = 0;
  std::string method;
  // The target up to its '?', and what follows that.
  std::string path;
  std::string query;
  std::string body;
};

struct HttpAnswer {
  int status = 200;
  std::string body;
  // Not empty, with status 200: the connection stays and gets the pictures of this camera.
  std::string stream;
};

// The head of a request, with its empty line, into method, path and query; false when it is not a request this
// server takes. *content_length is 0 without a body.
bool ParseHead(const std::string& head, HttpRequest* request, size_t* content_length);

// The value of a key in a query like "a=1&b=2", empty without the key. Nothing is decoded.
std::string QueryValue(const std::string& query, const std::string& key);
// text without the blanks and tabs around it.
std::string Trim(const std::string& text);

// A small HTTP/1.1 server on IPv4 for a few clients: one request per connection, answered with a short text or
// with the pictures of a camera as multipart/x-mixed-replace until either side ends it. One thread drives it
// through Poll and calls Push and Close; Post reaches that thread from any other.
class HttpServer {
 public:
  using Handler = std::function<HttpAnswer(const HttpRequest&)>;
  using Gone = std::function<void(uint64_t connection)>;

  // handler answers every complete request, gone tells the end of a connection that got pictures; both run in the
  // thread of Poll. now is a steady clock in s.
  HttpServer(Handler handler, Gone gone, std::function<double()> now);
  ~HttpServer();
  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  // Listens on an address and a port (0: any free one). Its number for HttpRequest::listener, -1 with *error.
  int Listen(const std::string& address, uint16_t port, const ListenerRules& rules, std::string* error);
  // The port of a listener.
  uint16_t Port(int listener) const;

  // Waits up to timeout_ms for connections, input, room for output or a task, then serves what came.
  void Poll(int timeout_ms);
  // The newest picture of a camera for its viewers. One that is still taking an older picture gets this one after
  // it, unless a newer one comes first.
  void Push(const std::string& camera, const std::string& jpeg);
  // Ends a connection.
  void Close(uint64_t connection);
  // From any thread: task runs in the thread of Poll, which returns for it.
  void Post(std::function<void()> task);

  size_t Connections() const {
    return connections_.size();
  }

 private:
  enum class Phase { kRequest, kAnswer, kStream };
  struct Listener {
    int fd = -1;
    uint16_t port = 0;
    ListenerRules rules;
    int pending = 0;
    // After accept failed for want of a resource: not asked again before this.
    double retry = 0;
  };
  struct Connection {
    uint64_t id = 0;
    int fd = -1;
    int listener = 0;
    Phase phase = Phase::kRequest;
    bool closing = false;
    // When it came (kRequest) or its output began.
    double since = 0;
    std::string in;
    // The request once its head is complete, the place of its body in `in` and the length of that.
    bool parsed = false;
    HttpRequest request;
    size_t body_at = 0;
    size_t content_length = 0;
    std::string out;
    size_t sent = 0;
    // kStream: its camera, the newest picture that has not begun, and when the last picture came.
    std::string camera;
    std::string next;
    double last_picture = 0;
  };

  void Accept(Listener& listener, int number);
  void Read(Connection& connection);
  void Write(Connection& connection);
  void Reply(Connection& connection, int status, const std::string& body);
  void StartPicture(Connection& connection, const std::string& jpeg);
  void LeavePending(Connection& connection);
  double Deadline(const Connection& connection) const;
  void RunTasks();
  void Sweep();

  Handler handler_;
  Gone gone_;
  std::function<double()> now_;
  int wake_fd_ = -1;
  std::vector<Listener> listeners_;
  std::vector<Connection> connections_;
  uint64_t next_id_ = 1;
  std::mutex tasks_mutex_;
  std::vector<std::function<void()>> tasks_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_HTTP_SERVER_H
