#ifndef MOWBITE_JOURNAL_CODEC_H
#define MOWBITE_JOURNAL_CODEC_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the journal is written little endian");

// std::optional as null or its value; nlohmann 3.11 has no serializer of its own for it.
namespace nlohmann {
template <typename T>
struct adl_serializer<std::optional<T>> {
  static void to_json(json& j, const std::optional<T>& value) {
    if (value) {
      j = *value;
    } else {
      j = nullptr;
    }
  }
  static void from_json(const json& j, std::optional<T>& value) {
    if (j.is_null()) {
      value.reset();
    } else {
      value = j.get<T>();
    }
  }
};
}  // namespace nlohmann

namespace mowbite_journal {

using json = nlohmann::json;

// The bytes of one record while it is made; it keeps its memory, so after the largest record nothing is allocated.
class Out {
 public:
  void Clear() {
    size_ = 0;
  }
  void Put(const void* data, size_t size) {
    if (size_ + size > bytes_.size()) bytes_.resize(std::max({size_ + size, 2 * bytes_.size(), size_t{256}}));
    if (size > 0) std::memcpy(bytes_.data() + size_, data, size);
    size_ += size;
  }
  void PutU32(uint32_t value) {
    Put(&value, sizeof(value));
  }
  void SetU32(size_t offset, uint32_t value) {
    std::memcpy(bytes_.data() + offset, &value, sizeof(value));
  }
  const uint8_t* data() const {
    return bytes_.data();
  }
  size_t size() const {
    return size_;
  }

 private:
  std::vector<uint8_t> bytes_;
  size_t size_ = 0;
};

// Reads what Out wrote; a record that ends too early throws.
class In {
 public:
  In(const uint8_t* data, size_t size) : data_(data), size_(size) {}
  void Get(void* data, size_t size) {
    const uint8_t* from = Take(size);
    if (size > 0) std::memcpy(data, from, size);
  }
  uint32_t GetU32() {
    uint32_t value;
    Get(&value, sizeof(value));
    return value;
  }
  const uint8_t* Take(size_t size) {
    if (size > size_ - offset_) throw std::runtime_error("record ends early");
    const uint8_t* at = data_ + offset_;
    offset_ += size;
    return at;
  }
  size_t left() const {
    return size_ - offset_;
  }
  const uint8_t* here() const {
    return data_ + offset_;
  }

 private:
  const uint8_t* data_;
  size_t size_;
  size_t offset_ = 0;
};

// Plain structs copied byte for byte, for the cores whose calls come from I/O threads: no allocation, no CBOR. Only
// for arguments: their padding is copied as it is, so a result must not be one of them. MOWBITE_JOURNAL_POD(Type).
template <typename T>
struct Pod : std::false_type {};

// Own encodings for types that are neither plain nor worth CBOR on an I/O thread (specialize Write and Read).
template <typename T>
struct Codec;

namespace detail {

template <typename T, typename = void>
struct HasCodec : std::false_type {};
template <typename T>
struct HasCodec<T, std::void_t<decltype(Codec<T>::Write)>> : std::true_type {};

template <typename T>
struct IsVector : std::false_type {};
template <typename T, typename A>
struct IsVector<std::vector<T, A>> : std::true_type {};
template <typename T>
struct IsOptional : std::false_type {};
template <typename T>
struct IsOptional<std::optional<T>> : std::true_type {};
template <typename T>
struct IsPair : std::false_type {};
template <typename A, typename B>
struct IsPair<std::pair<A, B>> : std::true_type {};
template <typename T>
struct IsArray : std::false_type {};
template <typename T, size_t N>
struct IsArray<std::array<T, N>> : std::true_type {};
template <typename T>
struct IsVariant : std::false_type {};
template <typename... A>
struct IsVariant<std::variant<A...>> : std::true_type {};

template <typename T>
constexpr bool kRaw = std::is_arithmetic_v<T> || std::is_enum_v<T> || Pod<T>::value;

inline std::vector<uint8_t>& CborScratch() {
  thread_local std::vector<uint8_t> bytes;
  return bytes;
}

}  // namespace detail

template <typename T>
void Write(Out& out, const T& value);
template <typename T>
T Read(In& in);

namespace detail {

template <typename T, size_t... I>
T ReadVariant(In& in, size_t index, std::index_sequence<I...>) {
  T value;
  const bool known = ((index == I && (value = Read<std::variant_alternative_t<I, T>>(in), true)) || ...);
  if (!known) throw std::runtime_error("variant index beyond its types");
  return value;
}

}  // namespace detail

// Numbers and enums as their bytes, strings and containers with a u32 count, a variant as the index of its type and
// its value, an empty struct as nothing, everything else as CBOR of its to_json.
template <typename T>
void Write(Out& out, const T& value) {
  if constexpr (detail::kRaw<T>) {
    static_assert(std::is_trivially_copyable_v<T>, "a raw journal type has to be trivially copyable");
    out.Put(&value, sizeof(T));
  } else if constexpr (std::is_same_v<T, std::string>) {
    out.PutU32(static_cast<uint32_t>(value.size()));
    out.Put(value.data(), value.size());
  } else if constexpr (detail::IsVector<T>::value) {
    out.PutU32(static_cast<uint32_t>(value.size()));
    if constexpr (detail::kRaw<typename T::value_type>) {
      out.Put(value.data(), value.size() * sizeof(typename T::value_type));
    } else {
      for (const auto& element : value) Write(out, element);
    }
  } else if constexpr (detail::IsOptional<T>::value) {
    const uint8_t present = value.has_value();
    out.Put(&present, 1);
    if (value) Write(out, *value);
  } else if constexpr (detail::IsPair<T>::value) {
    Write(out, value.first);
    Write(out, value.second);
  } else if constexpr (detail::IsArray<T>::value) {
    for (const auto& element : value) Write(out, element);
  } else if constexpr (detail::IsVariant<T>::value) {
    static_assert(std::variant_size_v<T> <= 255, "the index of a variant is one byte");
    const auto index = static_cast<uint8_t>(value.index());
    out.Put(&index, 1);
    std::visit([&out](const auto& alternative) { Write(out, alternative); }, value);
  } else if constexpr (std::is_empty_v<T>) {
  } else if constexpr (detail::HasCodec<T>::value) {
    Codec<T>::Write(out, value);
  } else {
    // to_cbor and never dump: CBOR keeps NaN and infinity.
    auto& bytes = detail::CborScratch();
    bytes.clear();
    json::to_cbor(json(value), bytes);
    out.PutU32(static_cast<uint32_t>(bytes.size()));
    out.Put(bytes.data(), bytes.size());
  }
}

template <typename T>
T Read(In& in) {
  if constexpr (detail::kRaw<T>) {
    T value;
    in.Get(&value, sizeof(T));
    return value;
  } else if constexpr (std::is_same_v<T, std::string>) {
    const uint32_t size = in.GetU32();
    const uint8_t* bytes = in.Take(size);
    return std::string(reinterpret_cast<const char*>(bytes), size);
  } else if constexpr (detail::IsVector<T>::value) {
    using E = typename T::value_type;
    const uint32_t count = in.GetU32();
    T value;
    if constexpr (detail::kRaw<E>) {
      if (count > in.left() / sizeof(E)) throw std::runtime_error("count beyond the record");
      value.resize(count);
      in.Get(value.data(), count * sizeof(E));
    } else {
      if (count > in.left()) throw std::runtime_error("count beyond the record");
      value.reserve(count);
      for (uint32_t i = 0; i < count; ++i) value.push_back(Read<E>(in));
    }
    return value;
  } else if constexpr (detail::IsOptional<T>::value) {
    uint8_t present;
    in.Get(&present, 1);
    if (!present) return T();
    return T(Read<typename T::value_type>(in));
  } else if constexpr (detail::IsPair<T>::value) {
    auto first = Read<typename T::first_type>(in);
    auto second = Read<typename T::second_type>(in);
    return T(std::move(first), std::move(second));
  } else if constexpr (detail::IsArray<T>::value) {
    T value;
    for (auto& element : value) element = Read<typename T::value_type>(in);
    return value;
  } else if constexpr (detail::IsVariant<T>::value) {
    uint8_t index;
    in.Get(&index, 1);
    return detail::ReadVariant<T>(in, index, std::make_index_sequence<std::variant_size_v<T>>());
  } else if constexpr (std::is_empty_v<T>) {
    return T();
  } else if constexpr (detail::HasCodec<T>::value) {
    return Codec<T>::Read(in);
  } else {
    const uint32_t size = in.GetU32();
    const uint8_t* bytes = in.Take(size);
    return json::from_cbor(bytes, bytes + size).get<T>();
  }
}

}  // namespace mowbite_journal

#define MOWBITE_JOURNAL_POD(Type)                                     \
  template <>                                                         \
  struct mowbite_journal::Pod<Type> : std::true_type {                \
    static_assert(std::is_trivially_copyable_v<Type>, #Type " is not plain"); \
  }

#endif  // MOWBITE_JOURNAL_CODEC_H
