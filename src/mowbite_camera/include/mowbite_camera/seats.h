#ifndef MOWBITE_CAMERA_SEATS_H
#define MOWBITE_CAMERA_SEATS_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mowbite_camera {

// Viewers at once, over all cameras and both protocols.
constexpr size_t kSeats = 2;
// How long a token waits for its connection, in s.
constexpr double kTokenSeconds = 30.0;
// A token is this many random bytes, written as hex digits.
constexpr size_t kTokenBytes = 16;

// Fills out with random bytes; false when the system gave none.
using RandomBytes = std::function<bool(uint8_t* out, size_t size)>;
// getrandom of the kernel.
bool SystemRandom(uint8_t* out, size_t size);
// size random bytes as hex digits, empty when the source gave none.
std::string RandomHex(const RandomBytes& random, size_t size);

// Whether two texts are the same, in a time that depends only on their lengths.
bool SameText(const std::string& a, const std::string& b);

// The place of one viewer.
struct Seat {
  // Only in memory, never in a log.
  std::string token;
  std::string camera_id;
  std::string protocol;
  // Its connection has to begin before this, on the clock of Grant.
  double expires = 0;
  // The connection that took it: one of the HTTP server (mjpeg) or a session of the stream server (webrtc); empty
  // until it began.
  std::string connection;
};

// The places of the viewers. A token holds one until its connection begins or the token expires, the connection
// holds it until it ends; a token is good for one connection.
class Seats {
 public:
  explicit Seats(RandomBytes random = SystemRandom);

  enum class Granted { kOk, kBusy, kNoRandom };
  // A place for one viewer of this camera, with its token.
  Granted Grant(const std::string& camera_id, const std::string& protocol, double now, std::string* token);
  // Whether the token still waits for its connection to this camera with this protocol.
  bool Waiting(const std::string& token, const std::string& camera_id, const std::string& protocol, double now) const;
  // The connection of the token begins: true for the first one in time, and again for that same connection.
  bool Connect(const std::string& token, const std::string& camera_id, const std::string& protocol,
               const std::string& connection, double now);
  // The connection ended and its place is free; false when it had none.
  bool Release(const std::string& protocol, const std::string& connection);
  // Tokens nobody connected with in time give their place back.
  void Expire(double now);
  // Frees the places of a camera, or of a protocol over all cameras. Returns those with a connection, which is to
  // be ended.
  std::vector<Seat> DropCamera(const std::string& camera_id);
  std::vector<Seat> DropProtocol(const std::string& protocol);

  const std::vector<Seat>& seats() const {
    return seats_;
  }

 private:
  const Seat* Find(const std::string& token, const std::string& camera_id, const std::string& protocol) const;
  std::vector<Seat> Drop(const std::function<bool(const Seat&)>& which);

  RandomBytes random_;
  std::vector<Seat> seats_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_SEATS_H
