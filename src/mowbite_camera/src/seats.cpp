#include "mowbite_camera/seats.h"

#include <sys/random.h>

#include <algorithm>
#include <cerrno>

namespace mowbite_camera {

bool SystemRandom(uint8_t* out, size_t size) {
  size_t have = 0;
  while (have < size) {
    const ssize_t n = ::getrandom(out + have, size - have, 0);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;
    have += static_cast<size_t>(n);
  }
  return true;
}

std::string RandomHex(const RandomBytes& random, size_t size) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::vector<uint8_t> bytes(size);
  if (!random || !random(bytes.data(), bytes.size())) return "";
  std::string text;
  text.reserve(2 * size);
  for (const uint8_t byte : bytes) {
    text.push_back(kDigits[byte >> 4]);
    text.push_back(kDigits[byte & 15]);
  }
  return text;
}

bool SameText(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  unsigned char differs = 0;
  for (size_t i = 0; i < a.size(); ++i) differs |= static_cast<unsigned char>(a[i] ^ b[i]);
  return differs == 0;
}

Seats::Seats(RandomBytes random) : random_(std::move(random)) {}

Seats::Granted Seats::Grant(const std::string& camera_id, const std::string& protocol, double now,
                            std::string* token) {
  Expire(now);
  if (seats_.size() >= kSeats) return Granted::kBusy;
  token->assign(RandomHex(random_, kTokenBytes));
  if (token->empty()) return Granted::kNoRandom;
  seats_.push_back({*token, camera_id, protocol, now + kTokenSeconds, ""});
  return Granted::kOk;
}

const Seat* Seats::Find(const std::string& token, const std::string& camera_id, const std::string& protocol) const {
  // Every seat is compared, so the time does not tell which one came close.
  const Seat* found = nullptr;
  for (const Seat& seat : seats_) {
    if (SameText(seat.token, token)) found = &seat;
  }
  if (!found || found->camera_id != camera_id || found->protocol != protocol) return nullptr;
  return found;
}

bool Seats::Waiting(const std::string& token, const std::string& camera_id, const std::string& protocol,
                    double now) const {
  const Seat* seat = Find(token, camera_id, protocol);
  return seat && seat->connection.empty() && now <= seat->expires;
}

bool Seats::Connect(const std::string& token, const std::string& camera_id, const std::string& protocol,
                    const std::string& connection, double now) {
  if (connection.empty()) return false;
  const Seat* seat = Find(token, camera_id, protocol);
  if (!seat) return false;
  if (!seat->connection.empty()) return seat->connection == connection;
  if (now > seat->expires) return false;
  seats_[static_cast<size_t>(seat - seats_.data())].connection = connection;
  return true;
}

bool Seats::Release(const std::string& protocol, const std::string& connection) {
  if (connection.empty()) return false;
  return !Drop([&](const Seat& seat) { return seat.protocol == protocol && seat.connection == connection; }).empty();
}

void Seats::Expire(double now) {
  Drop([now](const Seat& seat) { return seat.connection.empty() && now > seat.expires; });
}

std::vector<Seat> Seats::DropCamera(const std::string& camera_id) {
  return Drop([&](const Seat& seat) { return seat.camera_id == camera_id; });
}

std::vector<Seat> Seats::DropCamera(const std::string& camera_id, const std::string& protocol) {
  return Drop([&](const Seat& seat) { return seat.camera_id == camera_id && seat.protocol == protocol; });
}

std::vector<Seat> Seats::DropProtocol(const std::string& protocol) {
  return Drop([&](const Seat& seat) { return seat.protocol == protocol; });
}

std::vector<Seat> Seats::Drop(const std::function<bool(const Seat&)>& which) {
  std::vector<Seat> connected;
  for (const Seat& seat : seats_) {
    if (which(seat) && !seat.connection.empty()) connected.push_back(seat);
  }
  seats_.erase(std::remove_if(seats_.begin(), seats_.end(), which), seats_.end());
  return connected;
}

}  // namespace mowbite_camera
