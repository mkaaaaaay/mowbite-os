#ifndef XCORE_DRIVER_SCHEMA_H
#define XCORE_DRIVER_SCHEMA_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "xcore_driver/cbor.h"

namespace xcore {

// The type of a value as a service description names it: a scalar, an array of up to count scalars, or a blob.
// Enum types of the description count as their base type.
struct ValueType {
  enum class Base { kU8, kI8, kU16, kI16, kU32, kI32, kU64, kI64, kF32, kF64, kChar, kBlob };
  Base base = Base::kU8;
  // 0 for a scalar.
  size_t count = 0;

  size_t element_size() const;
  // Largest value in bytes; blobs have no fixed size and give 0.
  size_t max_size() const;
  bool operator==(const ValueType& other) const {
    return base == other.base && count == other.count;
  }
  bool operator!=(const ValueType& other) const {
    return !(*this == other);
  }
};

// "uint8_t", "double[6]", "char[25]", "blob"; nullopt for anything else.
std::optional<ValueType> ParseValueType(const std::string& text);
// A value of this length fits the type: a scalar exactly, an array a whole number of elements up to count.
bool FitsType(const ValueType& type, size_t length);
// "double[6]" and so on, the way a description names the type.
std::string TypeText(const ValueType& type);

struct Channel {
  uint16_t id = 0;
  std::string name;
  ValueType type;
};

struct ServiceDescription {
  std::string type;
  uint32_t version = 0;
  std::vector<Channel> inputs;
  std::vector<Channel> outputs;
  std::vector<Channel> registers;
};

// The description as the service announces it; enum names resolve to their base type. nullopt if type or version is
// missing or a channel has no id or a type the driver cannot read.
std::optional<ServiceDescription> ParseDescription(const Cbor& desc);

struct Announcement {
  uint16_t service_id = 0;
  std::string ip;
  uint16_t port = 0;
  ServiceDescription description;
};

// The CBOR payload of an announcement; nullopt unless sid, endpoint (ip, port) and desc are all there and valid.
std::optional<Announcement> ParseAnnouncement(const uint8_t* data, size_t size);

// What the driver relies on in a service: its type and version, and the channels it uses by id and type. The
// announced description may carry more channels.
using ExpectedService = ServiceDescription;

// The description as CBOR, the way a service announces it (for the simulated board and tests).
Cbor DescriptionToCbor(const ServiceDescription& description);
// Payload of an announcement.
std::vector<uint8_t> EncodeAnnouncement(const Announcement& announcement);

// Differences that keep the driver from using the service, empty if it may claim it.
std::vector<std::string> CheckDescription(const ExpectedService& expected, const ServiceDescription& announced);

}  // namespace xcore

#endif  // XCORE_DRIVER_SCHEMA_H
