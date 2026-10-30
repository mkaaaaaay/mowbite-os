#ifndef TRON_GNSS_BOARD_LINK_H
#define TRON_GNSS_BOARD_LINK_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace tron {

// Exactly these bytes switch the RTK board to network RTK; it compares the text, checksum included.
constexpr char kBoardRtkOn[] = "$GNRTK,ON*69\r\n";
constexpr char kBoardReset[] = "RTKRESET\r\n";

struct BoardConfig {
  // $GNRTK,ON again this often while network RTK is wanted; the board falls back to LoRa after about 6 s of silence.
  double keepalive_s = 4.0;
  // Network RTK is wanted while RTCM came within this many s; then the keep-alive stops and the board returns to LoRa.
  double active_window_s = 10.0;
  // The board reports ON -> OFF while RTCM is wanted: RTKRESET before the next ON, as the vendor does.
  bool reset_on_off = true;
  // RTCM goes to the port only while the board reports ON (in LoRa mode it does not pass the host's bytes).
  bool gate = true;
};

enum class BoardState { kUnknown, kOn, kOff };

// The handshake with the RTK board in front of the UM960, without port and clock. On activation only $GNRTK,ON (a reset
// would cost ~25 s of float), the keep-alive while RTCM comes, RTKRESET + ON after the board fell back on its own.
class BoardLink {
 public:
  explicit BoardLink(const BoardConfig& config) : config_(config) {
  }

  // The port was opened (again): the board's state is unknown, ON goes out with the next RTCM.
  void OnPortOpened();
  void OnBoardReport(bool on, double now);
  // RTCM of this size came from the NTRIP client. True if it may be written now, otherwise it is dropped and counted.
  bool OnRtcm(size_t bytes, double now);
  // Board commands to write now, empty if none.
  std::string Poll(double now);

  bool wanted(double now) const;
  BoardState state() const {
    return state_;
  }
  // s since RTCM was last let through, nullopt if never: the host's own age of the corrections.
  std::optional<double> host_correction_age(double now) const;
  uint64_t forwarded_bytes() const {
    return forwarded_bytes_;
  }
  uint64_t dropped_bytes() const {
    return dropped_bytes_;
  }
  uint64_t resets() const {
    return resets_;
  }

 private:
  BoardConfig config_;
  BoardState state_ = BoardState::kUnknown;
  bool active_ = false;
  bool pending_reset_ = false;
  double next_on_ = 0;
  std::optional<double> last_rtcm_;
  std::optional<double> last_forward_;
  uint64_t forwarded_bytes_ = 0;
  uint64_t dropped_bytes_ = 0;
  uint64_t resets_ = 0;
};

}  // namespace tron

#endif  // TRON_GNSS_BOARD_LINK_H
