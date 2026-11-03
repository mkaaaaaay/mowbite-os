#include "xcore_driver/wire.h"

#include <cstring>

namespace xcore {

Bytes EncodePacket(const Header& header, const Bytes& payload) {
  if (payload.size() > kMaxPayload) return {};
  Writer w;
  w.U8(header.version);
  w.U8(static_cast<uint8_t>(header.type));
  w.U8(header.flags);
  w.U8(0);
  w.U16(header.service_id);
  w.U8(header.arg1);
  w.U8(0);
  w.U16(header.arg2);
  w.U16(header.sequence);
  w.U64(header.timestamp);
  w.U32(static_cast<uint32_t>(payload.size()));
  w.Raw(payload.data(), payload.size());
  return w.Take();
}

std::optional<Packet> DecodePacket(const uint8_t* data, size_t size) {
  if (size < kHeaderSize || size > kMaxDatagram) return std::nullopt;
  Reader r(data, kHeaderSize);
  Packet packet;
  Header& h = packet.header;
  h.version = r.U8();
  h.type = static_cast<MessageType>(r.U8());
  h.flags = r.U8();
  r.U8();
  h.service_id = r.U16();
  h.arg1 = r.U8();
  r.U8();
  h.arg2 = r.U16();
  h.sequence = r.U16();
  h.timestamp = r.U64();
  uint32_t length = r.U32();
  if (h.version != kProtocolVersion || length != size - kHeaderSize) return std::nullopt;
  packet.payload.assign(data + kHeaderSize, data + size);
  return packet;
}

Bytes EncodeSections(const std::vector<Section>& sections) {
  Writer w;
  for (const auto& section : sections) {
    w.U16(section.id);
    w.U16(0);
    w.U32(static_cast<uint32_t>(section.value.size()));
    w.Raw(section.value.data(), section.value.size());
  }
  return w.Take();
}

std::optional<std::vector<Section>> DecodeSections(const Bytes& payload) {
  std::vector<Section> sections;
  size_t at = 0;
  while (at < payload.size()) {
    if (payload.size() - at < 8) return std::nullopt;
    Reader r(payload.data() + at, 8);
    Section section;
    section.id = r.U16();
    r.U16();
    uint32_t length = r.U32();
    at += 8;
    if (length > payload.size() - at) return std::nullopt;
    section.value.assign(payload.begin() + static_cast<std::ptrdiff_t>(at),
                         payload.begin() + static_cast<std::ptrdiff_t>(at + length));
    at += length;
    sections.push_back(std::move(section));
  }
  return sections;
}

void Writer::U8(uint8_t value) {
  out_.push_back(value);
}

void Writer::I8(int8_t value) {
  out_.push_back(static_cast<uint8_t>(value));
}

void Writer::U16(uint16_t value) {
  for (int i = 0; i < 2; i++) out_.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

void Writer::U32(uint32_t value) {
  for (int i = 0; i < 4; i++) out_.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

void Writer::U64(uint64_t value) {
  for (int i = 0; i < 8; i++) out_.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

void Writer::F32(float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  U32(bits);
}

void Writer::F64(double value) {
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  U64(bits);
}

void Writer::Raw(const uint8_t* data, size_t size) {
  out_.insert(out_.end(), data, data + size);
}

bool Reader::Take(size_t n) {
  if (failed_ || size_ - at_ < n) {
    failed_ = true;
    return false;
  }
  return true;
}

uint8_t Reader::U8() {
  if (!Take(1)) return 0;
  return data_[at_++];
}

int8_t Reader::I8() {
  return static_cast<int8_t>(U8());
}

uint16_t Reader::U16() {
  if (!Take(2)) return 0;
  uint16_t value = static_cast<uint16_t>(data_[at_] | (data_[at_ + 1] << 8));
  at_ += 2;
  return value;
}

uint32_t Reader::U32() {
  if (!Take(4)) return 0;
  uint32_t value = 0;
  for (int i = 0; i < 4; i++) value |= static_cast<uint32_t>(data_[at_ + i]) << (8 * i);
  at_ += 4;
  return value;
}

uint64_t Reader::U64() {
  if (!Take(8)) return 0;
  uint64_t value = 0;
  for (int i = 0; i < 8; i++) value |= static_cast<uint64_t>(data_[at_ + i]) << (8 * i);
  at_ += 8;
  return value;
}

float Reader::F32() {
  uint32_t bits = U32();
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double Reader::F64() {
  uint64_t bits = U64();
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

Bytes EncodeDoubles(const std::vector<double>& values) {
  Writer w;
  for (double v : values) w.F64(v);
  return w.Take();
}

std::optional<std::vector<double>> DecodeDoubles(const Bytes& value, size_t count) {
  if (value.size() != count * 8) return std::nullopt;
  Reader r(value);
  std::vector<double> out(count);
  for (auto& v : out) v = r.F64();
  return out;
}

std::string DecodeText(const Bytes& value) {
  size_t end = 0;
  while (end < value.size() && value[end] != 0) end++;
  return std::string(value.begin(), value.begin() + static_cast<std::ptrdiff_t>(end));
}

}  // namespace xcore
