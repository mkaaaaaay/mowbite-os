#ifndef XCORE_DRIVER_WIRE_H
#define XCORE_DRIVER_WIRE_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xcore {

using Bytes = std::vector<uint8_t>;

constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kHeaderSize = 24;
// One UDP datagram over Ethernet without fragments: 1472 bytes, the header included.
constexpr size_t kMaxDatagram = 1472;
constexpr size_t kMaxPayload = kMaxDatagram - kHeaderSize;

enum class MessageType : uint8_t {
  kData = 0x01,
  kConfigurationRequest = 0x02,
  kClaim = 0x03,
  kHeartbeat = 0x04,
  kTransaction = 0x05,
  kRpcRequest = 0x06,
  kRpcResponse = 0x07,
  kLog = 0x7F,
  kAdvertisement = 0x80,
};

// The board sets this flag in its first messages after a start.
constexpr uint8_t kFlagReboot = 0x01;

// arg1 of a claim: the request of the host, the answer of the service.
constexpr uint8_t kClaimRequest = 0;
constexpr uint8_t kClaimAck = 1;
// arg1 of a transaction: values (inputs or outputs) or the registers of the configuration.
constexpr uint8_t kTransactionData = 0;
constexpr uint8_t kTransactionConfig = 1;

struct Header {
  uint8_t version = kProtocolVersion;
  MessageType type = MessageType::kData;
  uint8_t flags = 0;
  uint16_t service_id = 0;
  uint8_t arg1 = 0;
  uint16_t arg2 = 0;
  uint16_t sequence = 0;
  uint64_t timestamp = 0;
};

struct Packet {
  Header header;
  Bytes payload;
};

// Header and payload in one datagram; empty if the payload is longer than kMaxPayload.
Bytes EncodePacket(const Header& header, const Bytes& payload);
// nullopt unless the datagram holds a header of the known version and exactly the payload length it names.
std::optional<Packet> DecodePacket(const uint8_t* data, size_t size);

// A section of a transaction: the id of an input, output or register and its raw bytes.
struct Section {
  uint16_t id = 0;
  Bytes value;
};

// Each section as an 8-byte descriptor (u16 id, u16 0, u32 length) and its bytes.
Bytes EncodeSections(const std::vector<Section>& sections);
// nullopt if a descriptor or a value runs past the end.
std::optional<std::vector<Section>> DecodeSections(const Bytes& payload);

// Little-endian values without padding, as the services lay them out.
class Writer {
 public:
  void U8(uint8_t value);
  void I8(int8_t value);
  void U16(uint16_t value);
  void U32(uint32_t value);
  void U64(uint64_t value);
  void F32(float value);
  void F64(double value);
  void Raw(const uint8_t* data, size_t size);
  const Bytes& bytes() const {
    return out_;
  }
  Bytes Take() {
    return std::move(out_);
  }

 private:
  Bytes out_;
};

// Reads little-endian values; a read past the end sets failed() and returns 0.
class Reader {
 public:
  Reader(const uint8_t* data, size_t size) : data_(data), size_(size) {
  }
  explicit Reader(const Bytes& bytes) : Reader(bytes.data(), bytes.size()) {
  }

  uint8_t U8();
  int8_t I8();
  uint16_t U16();
  uint32_t U32();
  uint64_t U64();
  float F32();
  double F64();
  size_t remaining() const {
    return failed_ ? 0 : size_ - at_;
  }
  bool failed() const {
    return failed_;
  }

 private:
  bool Take(size_t n);

  const uint8_t* data_;
  size_t size_;
  size_t at_ = 0;
  bool failed_ = false;
};

// Values of a known shape, as a service sends or expects them.
Bytes EncodeDoubles(const std::vector<double>& values);
std::optional<std::vector<double>> DecodeDoubles(const Bytes& value, size_t count);
// Text of a char[N] value: up to the first null byte or the end.
std::string DecodeText(const Bytes& value);

}  // namespace xcore

#endif  // XCORE_DRIVER_WIRE_H
