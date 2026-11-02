#include "xcore_driver/cbor.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace xcore {

namespace {

constexpr int kMaxDepth = 16;
constexpr uint8_t kBreak = 0xFF;

class Decoder {
 public:
  Decoder(const uint8_t* data, size_t size) : data_(data), size_(size) {
  }

  std::optional<Cbor> Item(int depth);
  bool AtEnd() const {
    return at_ == size_;
  }

 private:
  bool Byte(uint8_t& out);
  // The argument of a head; indefinite is set for additional info 31.
  bool Argument(uint8_t info, uint64_t& value, bool& indefinite);
  bool Chunk(uint64_t length, std::string& out);
  std::optional<Cbor> String(uint8_t major, uint64_t length, bool indefinite);
  bool PeekBreak();

  const uint8_t* data_;
  size_t size_;
  size_t at_ = 0;
};

bool Decoder::Byte(uint8_t& out) {
  if (at_ >= size_) return false;
  out = data_[at_++];
  return true;
}

bool Decoder::Argument(uint8_t info, uint64_t& value, bool& indefinite) {
  indefinite = false;
  if (info < 24) {
    value = info;
    return true;
  }
  if (info == 31) {
    indefinite = true;
    return true;
  }
  if (info > 27) return false;
  int bytes = 1 << (info - 24);
  value = 0;
  for (int i = 0; i < bytes; i++) {
    uint8_t b;
    if (!Byte(b)) return false;
    value = (value << 8) | b;
  }
  return true;
}

bool Decoder::Chunk(uint64_t length, std::string& out) {
  if (length > size_ - at_) return false;
  out.append(reinterpret_cast<const char*>(data_ + at_), static_cast<size_t>(length));
  at_ += static_cast<size_t>(length);
  return true;
}

bool Decoder::PeekBreak() {
  if (at_ < size_ && data_[at_] == kBreak) {
    at_++;
    return true;
  }
  return false;
}

std::optional<Cbor> Decoder::String(uint8_t major, uint64_t length, bool indefinite) {
  std::string text;
  if (!indefinite) {
    if (!Chunk(length, text)) return std::nullopt;
  } else {
    while (!PeekBreak()) {
      uint8_t head;
      if (!Byte(head) || (head >> 5) != major) return std::nullopt;
      uint64_t chunk_length;
      bool chunk_indefinite;
      if (!Argument(head & 0x1F, chunk_length, chunk_indefinite) || chunk_indefinite) return std::nullopt;
      if (!Chunk(chunk_length, text)) return std::nullopt;
    }
  }
  if (major == 3) return Cbor::Text(std::move(text));
  return Cbor::ByteString(std::vector<uint8_t>(text.begin(), text.end()));
}

double HalfToDouble(uint16_t half) {
  int exponent = (half >> 10) & 0x1F;
  int mantissa = half & 0x3FF;
  double value;
  if (exponent == 0) {
    value = std::ldexp(mantissa, -24);
  } else if (exponent == 31) {
    value = mantissa == 0 ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::quiet_NaN();
  } else {
    value = std::ldexp(mantissa + 1024, exponent - 25);
  }
  return (half & 0x8000) ? -value : value;
}

std::optional<Cbor> Decoder::Item(int depth) {
  if (depth > kMaxDepth) return std::nullopt;
  uint8_t head;
  if (!Byte(head)) return std::nullopt;
  uint8_t major = head >> 5;
  uint8_t info = head & 0x1F;
  if (major == 7) {
    switch (info) {
      case 20:
        return Cbor::Bool(false);
      case 21:
        return Cbor::Bool(true);
      case 22:
      case 23:
        return Cbor::Null();
      case 25: {
        uint64_t bits;
        bool indefinite;
        if (!Argument(info, bits, indefinite)) return std::nullopt;
        return Cbor::Float(HalfToDouble(static_cast<uint16_t>(bits)));
      }
      case 26: {
        uint64_t bits;
        bool indefinite;
        if (!Argument(info, bits, indefinite)) return std::nullopt;
        uint32_t b32 = static_cast<uint32_t>(bits);
        float f;
        std::memcpy(&f, &b32, sizeof(f));
        return Cbor::Float(f);
      }
      case 27: {
        uint64_t bits;
        bool indefinite;
        if (!Argument(info, bits, indefinite)) return std::nullopt;
        double d;
        std::memcpy(&d, &bits, sizeof(d));
        return Cbor::Float(d);
      }
      default:
        return std::nullopt;
    }
  }
  uint64_t argument;
  bool indefinite;
  if (!Argument(info, argument, indefinite)) return std::nullopt;
  switch (major) {
    case 0:
      if (indefinite) return std::nullopt;
      return Cbor::Unsigned(argument);
    case 1:
      if (indefinite || argument > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) return std::nullopt;
      return Cbor::Negative(-1 - static_cast<int64_t>(argument));
    case 2:
    case 3:
      return String(major, argument, indefinite);
    case 4: {
      // Every item takes at least one byte, which bounds a definite count that lies.
      if (!indefinite && argument > size_ - at_) return std::nullopt;
      std::vector<Cbor> items;
      while (indefinite ? !PeekBreak() : items.size() < argument) {
        auto item = Item(depth + 1);
        if (!item) return std::nullopt;
        items.push_back(std::move(*item));
      }
      return Cbor::Array(std::move(items));
    }
    case 5: {
      if (!indefinite && argument > (size_ - at_) / 2) return std::nullopt;
      std::vector<std::pair<Cbor, Cbor>> entries;
      while (indefinite ? !PeekBreak() : entries.size() < argument) {
        auto key = Item(depth + 1);
        if (!key) return std::nullopt;
        auto value = Item(depth + 1);
        if (!value) return std::nullopt;
        entries.emplace_back(std::move(*key), std::move(*value));
      }
      return Cbor::Map(std::move(entries));
    }
    case 6:
      if (indefinite) return std::nullopt;
      return Item(depth + 1);
    default:
      return std::nullopt;
  }
}

void Head(std::vector<uint8_t>& out, uint8_t major, uint64_t value) {
  uint8_t m = static_cast<uint8_t>(major << 5);
  if (value < 24) {
    out.push_back(m | static_cast<uint8_t>(value));
    return;
  }
  int bytes;
  if (value <= 0xFF) {
    out.push_back(m | 24);
    bytes = 1;
  } else if (value <= 0xFFFF) {
    out.push_back(m | 25);
    bytes = 2;
  } else if (value <= 0xFFFFFFFFull) {
    out.push_back(m | 26);
    bytes = 4;
  } else {
    out.push_back(m | 27);
    bytes = 8;
  }
  for (int i = bytes - 1; i >= 0; i--) out.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

void Encode(const Cbor& item, std::vector<uint8_t>& out) {
  switch (item.kind()) {
    case Cbor::Kind::kUnsigned:
      Head(out, 0, static_cast<uint64_t>(*item.AsInt()));
      break;
    case Cbor::Kind::kNegative:
      Head(out, 1, static_cast<uint64_t>(-1 - *item.AsInt()));
      break;
    case Cbor::Kind::kBytes:
    case Cbor::Kind::kText: {
      const std::string& s = *item.AsString();
      Head(out, item.kind() == Cbor::Kind::kText ? 3 : 2, s.size());
      out.insert(out.end(), s.begin(), s.end());
      break;
    }
    case Cbor::Kind::kArray:
      Head(out, 4, item.items().size());
      for (const auto& i : item.items()) Encode(i, out);
      break;
    case Cbor::Kind::kMap:
      Head(out, 5, item.entries().size());
      for (const auto& e : item.entries()) {
        Encode(e.first, out);
        Encode(e.second, out);
      }
      break;
    case Cbor::Kind::kBool:
      out.push_back(item.bool_value() ? 0xF5 : 0xF4);
      break;
    case Cbor::Kind::kNull:
      out.push_back(0xF6);
      break;
    case Cbor::Kind::kFloat: {
      double d = *item.AsNumber();
      uint64_t bits;
      std::memcpy(&bits, &d, sizeof(bits));
      out.push_back(0xFB);
      for (int i = 7; i >= 0; i--) out.push_back(static_cast<uint8_t>(bits >> (8 * i)));
      break;
    }
  }
}

}  // namespace

Cbor Cbor::Unsigned(uint64_t value) {
  Cbor c;
  c.kind_ = Kind::kUnsigned;
  c.unsigned_ = value;
  return c;
}

Cbor Cbor::Negative(int64_t value) {
  Cbor c;
  c.kind_ = Kind::kNegative;
  c.negative_ = value;
  return c;
}

Cbor Cbor::Integer(int64_t value) {
  return value < 0 ? Negative(value) : Unsigned(static_cast<uint64_t>(value));
}

Cbor Cbor::Text(std::string text) {
  Cbor c;
  c.kind_ = Kind::kText;
  c.string_ = std::move(text);
  return c;
}

Cbor Cbor::ByteString(std::vector<uint8_t> bytes) {
  Cbor c;
  c.kind_ = Kind::kBytes;
  c.string_.assign(bytes.begin(), bytes.end());
  return c;
}

Cbor Cbor::Array(std::vector<Cbor> items) {
  Cbor c;
  c.kind_ = Kind::kArray;
  c.items_ = std::move(items);
  return c;
}

Cbor Cbor::Map(std::vector<std::pair<Cbor, Cbor>> entries) {
  Cbor c;
  c.kind_ = Kind::kMap;
  c.entries_ = std::move(entries);
  return c;
}

Cbor Cbor::Bool(bool value) {
  Cbor c;
  c.kind_ = Kind::kBool;
  c.bool_ = value;
  return c;
}

Cbor Cbor::Null() {
  return Cbor();
}

Cbor Cbor::Float(double value) {
  Cbor c;
  c.kind_ = Kind::kFloat;
  c.float_ = value;
  return c;
}

std::optional<int64_t> Cbor::AsInt() const {
  if (kind_ == Kind::kNegative) return negative_;
  if (kind_ == Kind::kUnsigned && unsigned_ <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
    return static_cast<int64_t>(unsigned_);
  }
  return std::nullopt;
}

std::optional<double> Cbor::AsNumber() const {
  if (kind_ == Kind::kFloat) return float_;
  if (kind_ == Kind::kUnsigned) return static_cast<double>(unsigned_);
  if (kind_ == Kind::kNegative) return static_cast<double>(negative_);
  return std::nullopt;
}

const std::string* Cbor::AsString() const {
  return (kind_ == Kind::kText || kind_ == Kind::kBytes) ? &string_ : nullptr;
}

const Cbor* Cbor::Find(const std::string& key) const {
  for (const auto& e : entries_) {
    if (e.first.kind_ == Kind::kText && e.first.string_ == key) return &e.second;
  }
  return nullptr;
}

std::optional<Cbor> DecodeCbor(const uint8_t* data, size_t size) {
  Decoder decoder(data, size);
  auto item = decoder.Item(0);
  if (!item || !decoder.AtEnd()) return std::nullopt;
  return item;
}

std::vector<uint8_t> EncodeCbor(const Cbor& item) {
  std::vector<uint8_t> out;
  Encode(item, out);
  return out;
}

}  // namespace xcore
