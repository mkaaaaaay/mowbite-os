#include <gtest/gtest.h>

#include "xcore_driver/schema.h"
#include "xcore_driver/services.h"

using namespace xcore;

TEST(Schema, ValueTypes) {
  auto d6 = ParseValueType("double[6]");
  ASSERT_TRUE(d6);
  EXPECT_EQ(d6->base, ValueType::Base::kF64);
  EXPECT_EQ(d6->count, 6u);
  EXPECT_EQ(d6->max_size(), 48u);
  EXPECT_EQ(ParseValueType("uint16_t")->max_size(), 2u);
  EXPECT_EQ(ParseValueType("char[25]")->max_size(), 25u);
  EXPECT_EQ(ParseValueType("blob")->base, ValueType::Base::kBlob);
  EXPECT_FALSE(ParseValueType("blob[3]"));
  EXPECT_FALSE(ParseValueType("double[]"));
  EXPECT_FALSE(ParseValueType("double[0]"));
  EXPECT_FALSE(ParseValueType("double[6"));
  EXPECT_FALSE(ParseValueType("[6]"));
  EXPECT_FALSE(ParseValueType("bool"));
  EXPECT_FALSE(ParseValueType(""));
  EXPECT_EQ(TypeText(*d6), "double[6]");
}

TEST(Schema, FitsType) {
  EXPECT_TRUE(FitsType(*ParseValueType("float"), 4));
  EXPECT_FALSE(FitsType(*ParseValueType("float"), 8));
  // Arrays may come shorter, in whole elements.
  EXPECT_TRUE(FitsType(*ParseValueType("uint8_t[255]"), 17));
  EXPECT_FALSE(FitsType(*ParseValueType("uint8_t[255]"), 256));
  EXPECT_TRUE(FitsType(*ParseValueType("double[6]"), 48));
  EXPECT_FALSE(FitsType(*ParseValueType("double[6]"), 47));
  EXPECT_TRUE(FitsType(*ParseValueType("blob"), 1000));
}

TEST(Schema, AnnouncementRoundTrip) {
  Announcement a;
  a.service_id = kDiffDriveService;
  a.ip = "172.16.78.150";
  a.port = 51234;
  a.description = ExpectedDiffDrive();
  auto bytes = EncodeAnnouncement(a);
  auto parsed = ParseAnnouncement(bytes.data(), bytes.size());
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->service_id, kDiffDriveService);
  EXPECT_EQ(parsed->ip, "172.16.78.150");
  EXPECT_EQ(parsed->port, 51234);
  EXPECT_TRUE(CheckDescription(ExpectedDiffDrive(), parsed->description).empty());
}

TEST(Schema, EnumTypesResolveToTheirBase) {
  Cbor desc = Cbor::Map(
      {{Cbor::Text("type"), Cbor::Text("GpsService")},
       {Cbor::Text("version"), Cbor::Unsigned(1)},
       {Cbor::Text("registers"),
        Cbor::Array({Cbor::Map({{Cbor::Text("id"), Cbor::Unsigned(1)},
                                {Cbor::Text("name"), Cbor::Text("Protocol")},
                                {Cbor::Text("type"), Cbor::Text("ProtocolType")}})})},
       {Cbor::Text("enums"), Cbor::Array({Cbor::Map({{Cbor::Text("id"), Cbor::Text("ProtocolType")},
                                                     {Cbor::Text("base_type"), Cbor::Text("uint8_t")}})})}});
  auto parsed = ParseDescription(desc);
  ASSERT_TRUE(parsed);
  ASSERT_EQ(parsed->registers.size(), 1u);
  EXPECT_EQ(parsed->registers[0].type, *ParseValueType("uint8_t"));
  EXPECT_TRUE(parsed->inputs.empty());
}

TEST(Schema, RejectsIncompleteAnnouncements) {
  auto encode = [](Cbor c) { return EncodeCbor(c); };
  Cbor desc = DescriptionToCbor(ExpectedImu());
  Cbor endpoint = Cbor::Map({{Cbor::Text("ip"), Cbor::Text("172.16.78.150")}, {Cbor::Text("port"), Cbor::Unsigned(1)}});
  auto ok = encode(Cbor::Map({{Cbor::Text("sid"), Cbor::Unsigned(4)}, {Cbor::Text("endpoint"), endpoint},
                              {Cbor::Text("desc"), desc}}));
  EXPECT_TRUE(ParseAnnouncement(ok.data(), ok.size()));
  auto no_desc = encode(Cbor::Map({{Cbor::Text("sid"), Cbor::Unsigned(4)}, {Cbor::Text("endpoint"), endpoint}}));
  EXPECT_FALSE(ParseAnnouncement(no_desc.data(), no_desc.size()));
  auto bad_port = encode(Cbor::Map(
      {{Cbor::Text("sid"), Cbor::Unsigned(4)},
       {Cbor::Text("endpoint"), Cbor::Map({{Cbor::Text("ip"), Cbor::Text("x")}, {Cbor::Text("port"), Cbor::Unsigned(70000)}})},
       {Cbor::Text("desc"), desc}}));
  EXPECT_FALSE(ParseAnnouncement(bad_port.data(), bad_port.size()));
  auto bad_sid = encode(Cbor::Map({{Cbor::Text("sid"), Cbor::Integer(-1)}, {Cbor::Text("endpoint"), endpoint},
                                   {Cbor::Text("desc"), desc}}));
  EXPECT_FALSE(ParseAnnouncement(bad_sid.data(), bad_sid.size()));
  Cbor unknown_type = Cbor::Map({{Cbor::Text("type"), Cbor::Text("X")}, {Cbor::Text("version"), Cbor::Unsigned(1)},
                                 {Cbor::Text("outputs"), Cbor::Array({Cbor::Map({{Cbor::Text("id"), Cbor::Unsigned(0)},
                                                                                 {Cbor::Text("type"), Cbor::Text("Foo")}})})}});
  EXPECT_FALSE(ParseDescription(unknown_type));
  std::vector<uint8_t> garbage{0xa1, 0x63};
  EXPECT_FALSE(ParseAnnouncement(garbage.data(), garbage.size()));
}

TEST(Schema, CheckFindsDifferences) {
  ServiceDescription announced = ExpectedPower();
  EXPECT_TRUE(CheckDescription(ExpectedPower(), announced).empty());
  // Extra channels do not matter.
  announced.outputs.push_back({42, "extra", *ParseValueType("float")});
  EXPECT_TRUE(CheckDescription(ExpectedPower(), announced).empty());
  announced.outputs[0].type = *ParseValueType("double");
  announced.registers.pop_back();
  auto problems = CheckDescription(ExpectedPower(), announced);
  ASSERT_EQ(problems.size(), 2u);
  EXPECT_NE(problems[0].find("is double, expected float"), std::string::npos);
  EXPECT_NE(problems[1].find("missing"), std::string::npos);
  announced = ExpectedPower();
  announced.version = 2;
  problems = CheckDescription(ExpectedPower(), announced);
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_EQ(problems[0], "service is PowerService v2, expected PowerService v1");
}

TEST(Schema, EveryClaimedServiceIsKnown) {
  for (uint16_t id : {kEmergencyService, kDiffDriveService, kMowerService, kImuService, kPowerService, kGpsService,
                      kInputService, kMetaService}) {
    ASSERT_NE(ExpectedFor(id), nullptr) << id;
  }
  EXPECT_EQ(ExpectedFor(kHighLevelService), nullptr);
  EXPECT_EQ(ExpectedFor(kBmsService), nullptr);
}
