#include <gtest/gtest.h>

#include "xcore_driver/wire.h"

using namespace xcore;

TEST(Wire, HeaderLayoutIsLittleEndianWithoutPadding) {
  Header h;
  h.type = MessageType::kClaim;
  h.flags = kFlagReboot;
  h.service_id = 0x0102;
  h.arg1 = 0x03;
  h.arg2 = 0x0405;
  h.sequence = 0x0607;
  h.timestamp = 0x08090A0B0C0D0E0Full;
  Bytes packet = EncodePacket(h, {0xAA, 0xBB});
  Bytes expected{0x01, 0x03, 0x01, 0x00, 0x02, 0x01, 0x03, 0x00, 0x05, 0x04, 0x07, 0x06,
                 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x02, 0x00, 0x00, 0x00, 0xAA, 0xBB};
  EXPECT_EQ(packet, expected);
}

TEST(Wire, RoundTrip) {
  Header h;
  h.type = MessageType::kTransaction;
  h.service_id = 7;
  h.arg1 = kTransactionConfig;
  h.sequence = 65535;
  h.timestamp = 123456789;
  Bytes payload(100, 0x5A);
  Bytes packet = EncodePacket(h, payload);
  auto decoded = DecodePacket(packet.data(), packet.size());
  ASSERT_TRUE(decoded);
  EXPECT_EQ(decoded->header.type, MessageType::kTransaction);
  EXPECT_EQ(decoded->header.service_id, 7);
  EXPECT_EQ(decoded->header.arg1, kTransactionConfig);
  EXPECT_EQ(decoded->header.sequence, 65535);
  EXPECT_EQ(decoded->header.timestamp, 123456789u);
  EXPECT_EQ(decoded->payload, payload);
}

TEST(Wire, RejectsLengthThatDoesNotMatch) {
  Bytes packet = EncodePacket(Header{}, {1, 2, 3});
  EXPECT_TRUE(DecodePacket(packet.data(), packet.size()));
  Bytes longer = packet;
  longer.push_back(0);
  EXPECT_FALSE(DecodePacket(longer.data(), longer.size()));
  EXPECT_FALSE(DecodePacket(packet.data(), packet.size() - 1));
  EXPECT_FALSE(DecodePacket(packet.data(), kHeaderSize - 1));
}

TEST(Wire, RejectsOtherVersion) {
  Header h;
  h.version = 2;
  Bytes packet = EncodePacket(h, {});
  EXPECT_FALSE(DecodePacket(packet.data(), packet.size()));
}

TEST(Wire, PayloadLimitIsOneDatagram) {
  EXPECT_EQ(EncodePacket(Header{}, Bytes(kMaxPayload, 0)).size(), kMaxDatagram);
  EXPECT_TRUE(EncodePacket(Header{}, Bytes(kMaxPayload + 1, 0)).empty());
}

TEST(Wire, SectionsRoundTrip) {
  std::vector<Section> sections{{0, {1, 2, 3}}, {5, {}}, {0xFFFF, Bytes(10, 9)}};
  Bytes payload = EncodeSections(sections);
  EXPECT_EQ(payload.size(), 3 * 8 + 3 + 0 + 10u);
  // Descriptor: id, two zero bytes, length.
  EXPECT_EQ(Bytes(payload.begin(), payload.begin() + 8), (Bytes{0, 0, 0, 0, 3, 0, 0, 0}));
  auto decoded = DecodeSections(payload);
  ASSERT_TRUE(decoded);
  ASSERT_EQ(decoded->size(), 3u);
  EXPECT_EQ((*decoded)[0].value, (Bytes{1, 2, 3}));
  EXPECT_TRUE((*decoded)[1].value.empty());
  EXPECT_EQ((*decoded)[2].id, 0xFFFF);
  EXPECT_TRUE(DecodeSections({}));
}

TEST(Wire, SectionsRejectTruncation) {
  Bytes payload = EncodeSections({{1, {1, 2, 3, 4}}});
  payload.pop_back();
  EXPECT_FALSE(DecodeSections(payload));
  EXPECT_FALSE(DecodeSections(Bytes(7, 0)));
}

TEST(Wire, ReaderFailsPastTheEnd) {
  Bytes bytes{1, 2, 3};
  Reader r(bytes);
  EXPECT_EQ(r.U16(), 0x0201);
  EXPECT_EQ(r.U16(), 0);
  EXPECT_TRUE(r.failed());
  EXPECT_EQ(r.U8(), 0);
}

TEST(Wire, Values) {
  Writer w;
  w.F32(1.5f);
  w.F64(-2.25);
  w.I8(-3);
  Reader r(w.bytes());
  EXPECT_FLOAT_EQ(r.F32(), 1.5f);
  EXPECT_DOUBLE_EQ(r.F64(), -2.25);
  EXPECT_EQ(r.I8(), -3);
  EXPECT_EQ(r.remaining(), 0u);
  auto doubles = DecodeDoubles(EncodeDoubles({1, 2, 3}), 3);
  ASSERT_TRUE(doubles);
  EXPECT_EQ((*doubles)[2], 3);
  EXPECT_FALSE(DecodeDoubles(EncodeDoubles({1, 2}), 3));
  EXPECT_EQ(DecodeText({'F', 'I', 'X', 0, 'x'}), "FIX");
  EXPECT_EQ(DecodeText({'F', 'L', 'O', 'A', 'T'}), "FLOAT");
}
