#ifndef XCORE_DRIVER_CBOR_H
#define XCORE_DRIVER_CBOR_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace xcore {

// A decoded CBOR item (RFC 8949). Tags are dropped, undefined reads as null.
class Cbor {
 public:
  enum class Kind { kUnsigned, kNegative, kBytes, kText, kArray, kMap, kBool, kNull, kFloat };

  static Cbor Unsigned(uint64_t value);
  // value < 0
  static Cbor Negative(int64_t value);
  static Cbor Integer(int64_t value);
  static Cbor Text(std::string text);
  static Cbor ByteString(std::vector<uint8_t> bytes);
  static Cbor Array(std::vector<Cbor> items);
  static Cbor Map(std::vector<std::pair<Cbor, Cbor>> entries);
  static Cbor Bool(bool value);
  static Cbor Null();
  static Cbor Float(double value);

  Kind kind() const {
    return kind_;
  }
  bool IsInteger() const {
    return kind_ == Kind::kUnsigned || kind_ == Kind::kNegative;
  }
  // nullopt unless an integer in the range of int64.
  std::optional<int64_t> AsInt() const;
  // Integers and floats.
  std::optional<double> AsNumber() const;
  // Text and byte strings.
  const std::string* AsString() const;
  const std::vector<Cbor>& items() const {
    return items_;
  }
  const std::vector<std::pair<Cbor, Cbor>>& entries() const {
    return entries_;
  }
  bool bool_value() const {
    return bool_;
  }
  // The stored values by kind, 0 for another kind.
  uint64_t unsigned_value() const {
    return unsigned_;
  }
  int64_t negative_value() const {
    return negative_;
  }
  double float_value() const {
    return float_;
  }
  // The value of a map entry whose key is this text, nullptr if there is none or this is no map.
  const Cbor* Find(const std::string& key) const;

 private:
  Kind kind_ = Kind::kNull;
  uint64_t unsigned_ = 0;
  int64_t negative_ = 0;
  double float_ = 0;
  bool bool_ = false;
  std::string string_;
  std::vector<Cbor> items_;
  std::vector<std::pair<Cbor, Cbor>> entries_;
};

// One item that spans exactly size bytes; nullopt on malformed, truncated or too deeply nested input.
std::optional<Cbor> DecodeCbor(const uint8_t* data, size_t size);
// Definite lengths, shortest heads, floats as double.
std::vector<uint8_t> EncodeCbor(const Cbor& item);

}  // namespace xcore

#endif  // XCORE_DRIVER_CBOR_H
