#include <gtest/gtest.h>

#include <cmath>

#include "xcore_driver/cbor.h"

using namespace xcore;

namespace {

std::optional<Cbor> Decode(std::initializer_list<int> bytes) {
  std::vector<uint8_t> data;
  for (int b : bytes) data.push_back(static_cast<uint8_t>(b));
  return DecodeCbor(data.data(), data.size());
}

}  // namespace

// Examples of RFC 8949, appendix A.
TEST(Cbor, Integers) {
  EXPECT_EQ(Decode({0x00})->AsInt(), 0);
  EXPECT_EQ(Decode({0x17})->AsInt(), 23);
  EXPECT_EQ(Decode({0x18, 0x18})->AsInt(), 24);
  EXPECT_EQ(Decode({0x19, 0x03, 0xe8})->AsInt(), 1000);
  EXPECT_EQ(Decode({0x1a, 0x00, 0x0f, 0x42, 0x40})->AsInt(), 1000000);
  EXPECT_EQ(Decode({0x20})->AsInt(), -1);
  EXPECT_EQ(Decode({0x38, 0x63})->AsInt(), -100);
  auto big = Decode({0x1b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff});
  ASSERT_TRUE(big);
  EXPECT_FALSE(big->AsInt());
}

TEST(Cbor, Floats) {
  EXPECT_EQ(Decode({0xf9, 0x3c, 0x00})->AsNumber(), 1.0);
  EXPECT_EQ(Decode({0xf9, 0x7b, 0xff})->AsNumber(), 65504.0);
  EXPECT_EQ(Decode({0xf9, 0xc4, 0x00})->AsNumber(), -4.0);
  EXPECT_EQ(Decode({0xf9, 0x00, 0x01})->AsNumber(), 5.960464477539063e-8);
  EXPECT_EQ(Decode({0xfa, 0x47, 0xc3, 0x50, 0x00})->AsNumber(), 100000.0);
  EXPECT_EQ(Decode({0xfb, 0x3f, 0xf1, 0x99, 0x99, 0x99, 0x99, 0x99, 0x9a})->AsNumber(), 1.1);
  EXPECT_TRUE(std::isinf(*Decode({0xf9, 0x7c, 0x00})->AsNumber()));
}

TEST(Cbor, SimpleValues) {
  EXPECT_EQ(Decode({0xf4})->kind(), Cbor::Kind::kBool);
  EXPECT_FALSE(Decode({0xf4})->bool_value());
  EXPECT_TRUE(Decode({0xf5})->bool_value());
  EXPECT_EQ(Decode({0xf6})->kind(), Cbor::Kind::kNull);
  EXPECT_EQ(Decode({0xf7})->kind(), Cbor::Kind::kNull);
  EXPECT_FALSE(Decode({0xf0}));
}

TEST(Cbor, Strings) {
  EXPECT_EQ(*Decode({0x64, 0x49, 0x45, 0x54, 0x46})->AsString(), "IETF");
  EXPECT_EQ(Decode({0x44, 0x01, 0x02, 0x03, 0x04})->kind(), Cbor::Kind::kBytes);
  EXPECT_EQ(*Decode({0x7f, 0x65, 0x73, 0x74, 0x72, 0x65, 0x61, 0x64, 0x6d, 0x69, 0x6e, 0x67, 0xff})->AsString(),
            "streaming");
  // A chunk of another major type inside an indefinite text.
  EXPECT_FALSE(Decode({0x7f, 0x41, 0x61, 0xff}));
}

TEST(Cbor, ArraysAndMaps) {
  auto array = Decode({0x83, 0x01, 0x82, 0x02, 0x03, 0x82, 0x04, 0x05});
  ASSERT_TRUE(array);
  ASSERT_EQ(array->items().size(), 3u);
  EXPECT_EQ(array->items()[1].items()[1].AsInt(), 3);
  auto map = Decode({0xa2, 0x61, 0x61, 0x01, 0x61, 0x62, 0x82, 0x02, 0x03});
  ASSERT_TRUE(map);
  EXPECT_EQ(map->Find("a")->AsInt(), 1);
  EXPECT_EQ(map->Find("b")->items().size(), 2u);
  EXPECT_EQ(map->Find("c"), nullptr);
  auto indefinite = Decode({0xbf, 0x63, 0x46, 0x75, 0x6e, 0xf5, 0x63, 0x41, 0x6d, 0x74, 0x21, 0xff});
  ASSERT_TRUE(indefinite);
  EXPECT_TRUE(indefinite->Find("Fun")->bool_value());
  EXPECT_EQ(indefinite->Find("Amt")->AsInt(), -2);
  EXPECT_EQ(Decode({0x9f, 0xff})->items().size(), 0u);
}

TEST(Cbor, TagsAreDropped) {
  EXPECT_EQ(Decode({0xc1, 0x1a, 0x51, 0x4b, 0x67, 0xb0})->AsInt(), 1363896240);
}

TEST(Cbor, RejectsMalformedInput) {
  EXPECT_FALSE(Decode({}));
  EXPECT_FALSE(Decode({0x19, 0x03}));
  EXPECT_FALSE(Decode({0x64, 0x49, 0x45}));
  EXPECT_FALSE(Decode({0x82, 0x01}));
  EXPECT_FALSE(Decode({0x9f, 0x01}));
  EXPECT_FALSE(Decode({0x01, 0x02}));
  EXPECT_FALSE(Decode({0x1c}));
  // A count far beyond the input.
  EXPECT_FALSE(Decode({0x9b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01}));
  EXPECT_FALSE(Decode({0xff}));
}

TEST(Cbor, LimitsNesting) {
  std::vector<uint8_t> deep(40, 0x81);
  deep.push_back(0x00);
  EXPECT_FALSE(DecodeCbor(deep.data(), deep.size()));
  std::vector<uint8_t> shallow(10, 0x81);
  shallow.push_back(0x00);
  EXPECT_TRUE(DecodeCbor(shallow.data(), shallow.size()));
}

TEST(Cbor, EncodeRoundTrip) {
  Cbor item = Cbor::Map({{Cbor::Text("sid"), Cbor::Unsigned(6)},
                         {Cbor::Text("endpoint"),
                          Cbor::Map({{Cbor::Text("ip"), Cbor::Text("172.16.78.150")},
                                     {Cbor::Text("port"), Cbor::Unsigned(50123)}})},
                         {Cbor::Text("neg"), Cbor::Integer(-70000)},
                         {Cbor::Text("f"), Cbor::Float(0.25)},
                         {Cbor::Text("raw"), Cbor::ByteString({0, 255})},
                         {Cbor::Text("list"), Cbor::Array({Cbor::Bool(true), Cbor::Null()})}});
  auto bytes = EncodeCbor(item);
  auto decoded = DecodeCbor(bytes.data(), bytes.size());
  ASSERT_TRUE(decoded);
  EXPECT_EQ(decoded->Find("sid")->AsInt(), 6);
  EXPECT_EQ(*decoded->Find("endpoint")->Find("ip")->AsString(), "172.16.78.150");
  EXPECT_EQ(decoded->Find("endpoint")->Find("port")->AsInt(), 50123);
  EXPECT_EQ(decoded->Find("neg")->AsInt(), -70000);
  EXPECT_EQ(decoded->Find("f")->AsNumber(), 0.25);
  EXPECT_EQ(decoded->Find("raw")->AsString()->size(), 2u);
  EXPECT_EQ(decoded->Find("list")->items()[1].kind(), Cbor::Kind::kNull);
  // Shortest heads.
  EXPECT_EQ(EncodeCbor(Cbor::Unsigned(1000)), (std::vector<uint8_t>{0x19, 0x03, 0xe8}));
  EXPECT_EQ(EncodeCbor(Cbor::Integer(-100)), (std::vector<uint8_t>{0x38, 0x63}));
}

TEST(Cbor, EncodesUnsignedBeyondInt64) {
  EXPECT_EQ(EncodeCbor(Cbor::Unsigned(UINT64_MAX)),
            (std::vector<uint8_t>{0x1b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}));
}
