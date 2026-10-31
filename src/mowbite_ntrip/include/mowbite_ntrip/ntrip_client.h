#ifndef MOWBITE_NTRIP_NTRIP_CLIENT_H
#define MOWBITE_NTRIP_NTRIP_CLIENT_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_ntrip/ntrip_config.h"
#include "mowbite_ntrip/ntrip_protocol.h"
#include "mowbite_ntrip/ntrip_source.h"
#include "mowbite_ntrip/rtcm.h"

namespace mowbite_ntrip {

struct ClientTiming {
  // Wait before the next attempt after a failure, doubled each time up to max_retry; whole frames reset it.
  double first_retry_s = 5.0;
  double max_retry_s = 300.0;
  // Wait after the caster refused the login: retrying at once gets an account locked.
  double rejected_retry_s = 300.0;
  // No byte from the caster for this long ends the connection (also while waiting for its answer).
  double data_timeout_s = 15.0;
  // GGA to the caster, the first right after its answer; VRS mountpoints need the rover's position.
  double gga_interval_s = 10.0;
  // No frame for this long after there were some gives the LOST event: max_correction_age, when the fix counts as float.
  double lost_after_s = 30.0;
};

enum class ClientState { kOff, kWaiting, kConnecting, kConnected, kRejected };

// Name in gnss/ntrip_status and the sensor: off, waiting, connecting, connected, rejected.
const char* StateName(ClientState state);

// For events/json: reason LOST, RESTORED, REJECTED or SOURCE_CHANGED with the fields of contract 3.1a.
struct ClientEvent {
  std::string reason;
  std::string message;
  std::string severity;
  // Empty for none: SOURCE_CHANGED holds nothing up.
  std::string resume;
  // REJECTED: the HTTP status; LOST: the limit in s. 0: not set.
  int code = 0;
  double limit = 0;
  // SOURCE_CHANGED: the source now in use and the one used before.
  std::optional<CorrectionSource> source = std::nullopt;
  std::optional<CorrectionSource> previous = std::nullopt;
};

// The NTRIP client without sockets and clocks: the node opens and closes the connection, passes the bytes and the
// monotonic time and sends what this class returns.
class NtripClient {
 public:
  explicit NtripClient(const ClientTiming& timing) : timing_(timing) {
  }

  // The access as read from the file; nullopt without a usable file. A changed access ends the wait of a refused login
  // or of failed attempts, and a running connection is closed to start again with it.
  void SetConfig(const std::optional<NtripConfig>& config, double now);
  const std::optional<NtripConfig>& config() const {
    return config_;
  }

  bool ShouldConnect(double now) const;
  // An attempt starts now; returns the request to send once the socket is connected.
  std::string BeginAttempt(double now);
  // Bytes from the caster. Whole frames with a valid CRC are appended to frames.
  void OnBytes(const uint8_t* data, size_t size, double now, std::vector<std::vector<uint8_t>>& frames);
  // Why the open connection should be closed now, if it should.
  std::optional<std::string> CloseReason(double now) const;
  // The connection is closed or could not be opened. error is a short text without access data, may be empty.
  void OnClosed(double now, const std::string& error);

  // The receiver's latest GGA sentence with a position, without line end.
  void SetGga(const std::string& sentence);
  // The GGA line to send now, with CR LF.
  std::optional<std::string> TakeGga(double now);

  // Checks for LOST; call about once a second.
  void Tick(double now);
  std::vector<ClientEvent> TakeEvents();

  // The source whose corrections were used last, as saved; nullopt before the first. The first frame of an access from
  // another source gives SOURCE_CHANGED once; the first source of all gives none.
  void SetLastSource(const std::optional<CorrectionSource>& source);
  // The source to save, once its first frame came and it differs from the last one in any way, also only in spelling.
  std::optional<CorrectionSource> TakeSourceToSave();

  ClientState state() const {
    return state_;
  }
  const std::string& last_error() const {
    return last_error_;
  }
  uint32_t attempts() const {
    return attempts_;
  }
  uint64_t bytes() const {
    return bytes_;
  }
  uint64_t frames() const {
    return frames_;
  }
  uint64_t crc_errors() const {
    return crc_errors_;
  }
  // s since the last frame, nullopt before the first.
  std::optional<double> last_frame_age(double now) const;
  // s until the next attempt while waiting or rejected, 0 otherwise.
  double retry_in(double now) const;

 private:
  // With the first frame of an access: SOURCE_CHANGED against the source used last, which it then replaces.
  void UseSource();

  ClientTiming timing_;
  std::optional<NtripConfig> config_;
  ClientState state_ = ClientState::kOff;
  // The socket is open (between BeginAttempt and OnClosed).
  bool socket_open_ = false;
  // The access changed while connected: the next attempt right after closing.
  bool restart_now_ = false;
  std::optional<std::string> close_reason_;
  ResponseReader reader_;
  RtcmFramer framer_;
  double next_attempt_ = 0;
  double retry_delay_ = 0;
  double last_data_ = 0;
  std::optional<double> last_frame_;
  std::optional<double> last_gga_sent_;
  std::string gga_;
  std::string last_error_;
  uint32_t attempts_ = 0;
  uint64_t bytes_ = 0;
  uint64_t frames_ = 0;
  uint64_t crc_errors_ = 0;
  bool lost_reported_ = false;
  bool rejected_reported_ = false;
  std::optional<CorrectionSource> last_source_;
  std::optional<CorrectionSource> source_to_save_;
  // The frames of this access were compared with last_source_.
  bool source_checked_ = false;
  std::vector<ClientEvent> events_;
};

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_NTRIP_CLIENT_H
