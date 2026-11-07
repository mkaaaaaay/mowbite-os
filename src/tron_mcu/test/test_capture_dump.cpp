#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "tron_mcu/capture_dump.h"
#include "tron_mcu/mcu_codec.h"

using namespace tron;

namespace {

constexpr uint8_t kFromMcu = static_cast<uint8_t>(PacketType::kMcuToHost);

std::string Hex(const std::vector<uint8_t>& bytes) {
  std::string out;
  char pair[3];
  for (uint8_t b : bytes) {
    std::snprintf(pair, sizeof(pair), "%02x", b);
    out += pair;
  }
  return out;
}

SubPacket Speed(float linear, float angular) {
  SubPacket packet{kFromMcu, static_cast<uint8_t>(Module::kSpeed), std::vector<uint8_t>(8)};
  std::memcpy(packet.payload.data(), &linear, 4);
  std::memcpy(packet.payload.data() + 4, &angular, 4);
  return packet;
}

// bumper, rain, lift, stop, power_off, battery_gate, cutter_size, press_module, bumper_right, bumper_left
SubPacket Sensor(std::vector<uint8_t> bytes) {
  return {kFromMcu, static_cast<uint8_t>(Module::kSensor), std::move(bytes)};
}

SubPacket Motors(int16_t cutter, int16_t left, int16_t right) {
  SubPacket packet{kFromMcu, static_cast<uint8_t>(Module::kMotors), std::vector<uint8_t>(32)};
  const int16_t rpm[4] = {cutter, left, right, 0};
  for (int i = 0; i < 4; ++i) std::memcpy(packet.payload.data() + 8 * i, &rpm[i], 2);
  return packet;
}

std::vector<std::string> Dump(const std::string& capture, bool* ok = nullptr) {
  std::istringstream in(capture);
  std::ostringstream out;
  const bool result = DumpCapture(in, out);
  if (ok) *ok = result;
  std::vector<std::string> lines;
  std::istringstream text(out.str());
  for (std::string line; std::getline(text, line);) lines.push_back(line);
  return lines;
}

}  // namespace

TEST(CaptureDump, DecodesSpeedSensorAndMotors) {
  const auto frame = EncodeFrame({Speed(0.05f, -0.25f), Sensor({0, 0, 1, 0, 0, 0, 2, 0, 0, 1})});
  const auto motors = EncodeFrame({Motors(0, -12, 13)});
  bool ok = false;
  const auto lines = Dump("10.5 " + Hex(frame) + "\n10.75 " + Hex(motors) + "\n", &ok);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_TRUE(ok);
  EXPECT_EQ(lines[0], "{\"t\":10.500000,\"speed\":[0.05,-0.25]}");
  EXPECT_EQ(lines[1],
            "{\"t\":10.500000,\"sensor\":{\"bumper\":0,\"rain\":0,\"lift\":1,\"stop\":0,\"bumper_left\":1,"
            "\"bumper_right\":0}}");
  EXPECT_EQ(lines[2], "{\"t\":10.750000,\"motors\":{\"cutter\":0,\"left\":-12,\"right\":13}}");
  EXPECT_EQ(lines[3],
            "{\"summary\":{\"lines\":2,\"bad_lines\":0,\"bytes\":" + std::to_string(frame.size() + motors.size()) +
                ",\"frames\":2,\"checksum_errors\":0,\"length_errors\":0,\"escape_errors\":0,\"overflows\":0,"
                "\"dropped_bytes\":0}}");
}

TEST(CaptureDump, FrameSplitOverChunksTakesTheTimeOfItsLastChunk) {
  const std::string hex = Hex(EncodeFrame({Speed(0.0f, 0.0f)}));
  const auto lines = Dump("1.0 " + hex.substr(0, 6) + "\n1.25 " + hex.substr(6) + "\n");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "{\"t\":1.250000,\"speed\":[0,0]}");
}

TEST(CaptureDump, CountsChecksumErrorsAndBytesOutsideFrames) {
  auto frame = EncodeFrame({Speed(0.1f, 0.0f)});
  frame[5] ^= 0x01;  // first payload byte
  const auto good = EncodeFrame({Speed(0.0f, 0.0f)});
  const auto lines = Dump("2.0 0102" + Hex(frame) + Hex(good) + "\n");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "{\"t\":2.000000,\"speed\":[0,0]}");
  EXPECT_NE(lines[1].find("\"frames\":1,\"checksum_errors\":1,"), std::string::npos);
  EXPECT_NE(lines[1].find("\"dropped_bytes\":2}"), std::string::npos);
}

TEST(CaptureDump, OtherModulesByNumberAndShortPayloadsAsInvalid) {
  SubPacket version{kFromMcu, static_cast<uint8_t>(Module::kVersion), {0, 6, 36, 0, 6, 34, 0, 0, 0}};
  SubPacket short_speed{kFromMcu, static_cast<uint8_t>(Module::kSpeed), {1, 2, 3}};
  const auto lines = Dump("3 " + Hex(EncodeFrame({version, short_speed})) + "\n");
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "{\"t\":3.000000,\"module\":12}");
  EXPECT_EQ(lines[1], "{\"t\":3.000000,\"module\":4,\"invalid\":true}");
}

TEST(CaptureDump, HostPacketsOnTheLineAreShownByType) {
  const auto lines = Dump("4 " + Hex(EncodeFrame({ChargePacket(false)})) + "\n");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "{\"t\":4.000000,\"type\":0,\"module\":3}");
}

TEST(CaptureDump, UnreadableLinesAreCountedAndFail) {
  const std::string hex = Hex(EncodeFrame({Speed(0.0f, 0.0f)}));
  bool ok = true;
  const auto lines = Dump("x " + hex + "\n5 abc\n5 zz\n5 " + hex + " extra\n\n6 " + hex + "\n", &ok);
  EXPECT_FALSE(ok);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "{\"t\":6.000000,\"speed\":[0,0]}");
  EXPECT_NE(lines[1].find("\"lines\":5,\"bad_lines\":4,"), std::string::npos);
}

TEST(CaptureDump, EmptyCaptureGivesOnlyTheSummary) {
  bool ok = false;
  const auto lines = Dump("", &ok);
  EXPECT_TRUE(ok);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0].rfind("{\"summary\":{\"lines\":0,", 0), 0u);
}
