#include <gtest/gtest.h>

#include <cmath>

#include "tron_mcu/mcu_codec.h"

using namespace tron;

namespace {

std::vector<uint8_t> Bytes(std::initializer_list<int> values) {
  std::vector<uint8_t> out;
  for (int v : values) out.push_back(static_cast<uint8_t>(v));
  return out;
}

std::vector<std::vector<SubPacket>> ParseAll(FrameParser& parser, const std::vector<uint8_t>& bytes) {
  return parser.Push(bytes.data(), bytes.size());
}

}  // namespace

TEST(Framing, ChargeFrameMatchesLayout) {
  // total_len 4 (sub_len, type, module, payload), checksum 4+1+0+3+1.
  EXPECT_EQ(EncodeFrame({ChargePacket(true)}), Bytes({0xA5, 0x04, 0x01, 0x00, 0x03, 0x01, 0x09, 0x5A}));
}

TEST(Framing, EscapesSpecialBytes) {
  SubPacket packet{0x08, 19, Bytes({0xA5, 0x5A, 0xF5})};
  EXPECT_EQ(EncodeFrame({packet}),
            Bytes({0xA5, 0x06, 0x03, 0x08, 0x13, 0xF5, 0x01, 0xF5, 0x02, 0xF5, 0x03, 0x18, 0x5A}));
}

TEST(Framing, EscapesChecksum) {
  for (int b = 0; b < 256; b++) {
    SubPacket packet{0x08, 19, Bytes({b})};
    std::vector<uint8_t> inner{4, 1, 0x08, 19, static_cast<uint8_t>(b)};
    if (Checksum(inner.data(), inner.size()) != kFrameStart) continue;
    std::vector<uint8_t> frame = EncodeFrame({packet});
    ASSERT_EQ(frame[frame.size() - 3], kEscape);
    ASSERT_EQ(frame[frame.size() - 2], 0x01);
    FrameParser parser;
    auto frames = ParseAll(parser, frame);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0][0].payload, packet.payload);
    return;
  }
  FAIL() << "no payload byte gives checksum 0xA5";
}

TEST(Framing, RejectsOversizedFrame) {
  SubPacket packet{0, 19, std::vector<uint8_t>(253, 0)};
  EXPECT_TRUE(EncodeFrame({packet}).empty());
  EXPECT_TRUE(EncodeFrame({}).empty());
  packet.payload.resize(252);
  EXPECT_FALSE(EncodeFrame({packet}).empty());
}

TEST(Parser, RoundTripMultipleSubPackets) {
  std::vector<SubPacket> packets{ChargePacket(false), SensorClearPacket(), *SpeedPacket(0.1f, -0.2f)};
  FrameParser parser;
  auto frames = ParseAll(parser, EncodeFrame(packets));
  ASSERT_EQ(frames.size(), 1u);
  ASSERT_EQ(frames[0].size(), 3u);
  for (size_t i = 0; i < packets.size(); i++) {
    EXPECT_EQ(frames[0][i].type, packets[i].type);
    EXPECT_EQ(frames[0][i].module, packets[i].module);
    EXPECT_EQ(frames[0][i].payload, packets[i].payload);
  }
}

TEST(Parser, ByteByByteAndGarbage) {
  std::vector<uint8_t> stream = Bytes({0x00, 0x5A, 0x13, 0xF5});
  std::vector<uint8_t> frame = EncodeFrame({SubPacket{0x08, 19, Bytes({0xA5, 0x5A, 0xF5})}});
  stream.insert(stream.end(), frame.begin(), frame.end());
  stream.insert(stream.end(), frame.begin(), frame.end());
  FrameParser parser;
  size_t count = 0;
  for (uint8_t b : stream) count += parser.Push(&b, 1).size();
  EXPECT_EQ(count, 2u);
  EXPECT_EQ(parser.Stats().frames, 2u);
  EXPECT_EQ(parser.Stats().dropped_bytes, 4u);
}

TEST(Parser, ChecksumErrorThenResync) {
  std::vector<uint8_t> bad = EncodeFrame({ChargePacket(true)});
  bad[bad.size() - 2] ^= 0x01;
  std::vector<uint8_t> good = EncodeFrame({ChargePacket(false)});
  bad.insert(bad.end(), good.begin(), good.end());
  FrameParser parser;
  auto frames = ParseAll(parser, bad);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0][0].payload, Bytes({0}));
  EXPECT_EQ(parser.Stats().checksum_errors, 1u);
}

TEST(Parser, StartByteRestartsTruncatedFrame) {
  std::vector<uint8_t> frame = EncodeFrame({ChargePacket(true)});
  std::vector<uint8_t> stream(frame.begin(), frame.begin() + 4);
  stream.insert(stream.end(), frame.begin(), frame.end());
  FrameParser parser;
  EXPECT_EQ(ParseAll(parser, stream).size(), 1u);
  EXPECT_EQ(parser.Stats().dropped_bytes, 4u);
}

TEST(Parser, RejectsBadLengths) {
  FrameParser parser;
  // total_len says 5, only 4 bytes follow.
  EXPECT_TRUE(ParseAll(parser, Bytes({0xA5, 0x05, 0x01, 0x00, 0x03, 0x01, 0x0A, 0x5A})).empty());
  // sub_len 2 runs past total_len.
  EXPECT_TRUE(ParseAll(parser, Bytes({0xA5, 0x04, 0x02, 0x00, 0x03, 0x01, 0x0A, 0x5A})).empty());
  // Empty frame.
  EXPECT_TRUE(ParseAll(parser, Bytes({0xA5, 0x00, 0x00, 0x5A})).empty());
  EXPECT_EQ(parser.Stats().length_errors, 3u);
}

TEST(Parser, RejectsUnknownEscape) {
  FrameParser parser;
  EXPECT_TRUE(ParseAll(parser, Bytes({0xA5, 0x04, 0x01, 0x00, 0x03, 0xF5, 0x04, 0x0C, 0x5A})).empty());
  EXPECT_EQ(parser.Stats().escape_errors, 1u);
}

TEST(Parser, DropsOverlongFrame) {
  FrameParser parser(16);
  std::vector<uint8_t> stream{kFrameStart};
  stream.insert(stream.end(), 40, 0x11);
  stream.push_back(kFrameEnd);
  std::vector<uint8_t> good = EncodeFrame({ChargePacket(true)});
  stream.insert(stream.end(), good.begin(), good.end());
  EXPECT_EQ(ParseAll(parser, stream).size(), 1u);
  EXPECT_EQ(parser.Stats().overflows, 1u);
}

TEST(Commands, SensorClearIsEightZeros) {
  SubPacket clear = SensorClearPacket();
  EXPECT_EQ(clear.type, static_cast<uint8_t>(PacketType::kHostToMcu));
  EXPECT_EQ(clear.module, static_cast<uint8_t>(Module::kSensor));
  EXPECT_EQ(clear.payload, std::vector<uint8_t>(8, 0));
  EXPECT_EQ(clear.payload[4], 0);
}

TEST(Commands, PowerOffSetsOnlyByteFour) {
  SubPacket off = PowerOffPacket();
  EXPECT_EQ(off.module, static_cast<uint8_t>(Module::kSensor));
  EXPECT_EQ(off.payload, Bytes({0, 0, 0, 0, 1, 0, 0, 0}));
}

TEST(Commands, CutterHeightAlwaysInRange) {
  for (int mm : {0, 1, 29, 30, 55, 90, 91, 1000, 65535}) {
    CutterCommand command;
    command.height_mm = static_cast<uint16_t>(mm);
    SubPacket packet = CutterPacket(command);
    ASSERT_EQ(packet.payload.size(), 12u);
    uint16_t height = packet.payload[10] | (packet.payload[11] << 8);
    EXPECT_GE(height, kMinCutterHeightMm) << mm;
    EXPECT_LE(height, kMaxCutterHeightMm) << mm;
    if (mm >= 30 && mm <= 90) {
      EXPECT_EQ(height, mm);
    }
  }
}

TEST(Commands, CutterLayout) {
  CutterCommand command;
  command.blade_on = true;
  command.blade_speed_percent = 250;
  command.height_mm = 50;
  EXPECT_EQ(CutterPacket(command).payload, Bytes({1, 0, 100, 0, 0, 0, 0, 0, 0, 0, 50, 0}));
  // Default command: blade off, deck up.
  EXPECT_EQ(CutterPacket(CutterCommand{}).payload, Bytes({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 90, 0}));
}

TEST(Commands, SpeedIsClampedAndFinite) {
  auto packet = SpeedPacket(2.0f, -5.0f);
  ASSERT_TRUE(packet);
  auto decoded = DecodeSpeed(packet->payload);
  ASSERT_TRUE(decoded);
  EXPECT_FLOAT_EQ(decoded->linear, kMaxLinearSpeed);
  EXPECT_FLOAT_EQ(decoded->angular, -kMaxAngularSpeed);
  EXPECT_FALSE(SpeedPacket(NAN, 0.0f));
  EXPECT_FALSE(SpeedPacket(0.0f, INFINITY));
  EXPECT_EQ(SpeedPacket(0.0f, 0.0f)->payload, std::vector<uint8_t>(8, 0));
}

TEST(Commands, ZeroSpeedIsTheSpeedPacketOfZero) {
  const SubPacket zero = ZeroSpeedPacket();
  const auto speed = SpeedPacket(0.0f, 0.0f);
  ASSERT_TRUE(speed);
  EXPECT_EQ(zero.type, speed->type);
  EXPECT_EQ(zero.module, speed->module);
  EXPECT_EQ(zero.payload, std::vector<uint8_t>(8, 0));
  EXPECT_EQ(EncodeFrame({zero}), EncodeFrame({*speed}));
}

TEST(Commands, SpeedIsLittleEndianFloat) {
  // 0.25f = 0x3E800000
  EXPECT_EQ(SpeedPacket(0.25f, 0.0f)->payload, Bytes({0x00, 0x00, 0x80, 0x3E, 0, 0, 0, 0}));
}

TEST(Commands, Heartbeat) {
  WallClock now{2026, 10, 28, 14, 5, 9, 0x01F4};
  SubPacket packet = HeartbeatPacket(now);
  EXPECT_EQ(packet.type, static_cast<uint8_t>(PacketType::kHeartbeat));
  EXPECT_EQ(packet.payload, Bytes({26, 10, 28, 14, 5, 9, 0x01, 0xF4}));
}

TEST(Commands, ImuScalingAndLayout) {
  ImuCounts imu = ImuCountsFromSi(90.0, -180.0, 0.0, {0.0, 0.0, 1.0}, {0.0, 0.0, 4000.0});
  EXPECT_EQ(imu.pitch, 16384);
  EXPECT_EQ(imu.roll, -32768);
  EXPECT_EQ(imu.acc[2], 2048);
  EXPECT_EQ(imu.gyro[2], 32767);
  SubPacket packet = ImuPacket(imu);
  EXPECT_EQ(packet.module, static_cast<uint8_t>(Module::kImu));
  ASSERT_EQ(packet.payload.size(), 18u);
  EXPECT_EQ(packet.payload[0], 0x00);
  EXPECT_EQ(packet.payload[1], 0x40);
  EXPECT_EQ(packet.payload[10], 0x00);
  EXPECT_EQ(packet.payload[11], 0x08);
}

TEST(Telemetry, BatteryFromCapture) {
  // Captured at rest, off dock: 19.7 V, 0.7 A discharging, 31 %, 32 degC.
  auto battery = DecodeBattery(Bytes({0xC5, 0x00, 0x07, 0x00, 0x1F, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00}));
  ASSERT_TRUE(battery);
  EXPECT_NEAR(battery->voltage_v, 19.7, 1e-9);
  EXPECT_NEAR(battery->current_a, 0.7, 1e-9);
  EXPECT_EQ(battery->percentage, 31);
  EXPECT_EQ(battery->dock_ok, 0);
  EXPECT_EQ(battery->temperature_c, 32);
  EXPECT_EQ(battery->error_bits, 0u);
  EXPECT_NEAR(DecodeBattery(Bytes({0, 0, 0xF6, 0xFF, 0, 1, 0, 0, 0, 0, 0x80}))->current_a, -1.0, 1e-9);
  EXPECT_EQ(DecodeBattery(Bytes({0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0x80}))->error_bits, 0x80000000u);
  EXPECT_FALSE(DecodeBattery(Bytes({0xC5})));
}

TEST(Telemetry, MotorsFromCapture) {
  auto motors = DecodeMotors(Bytes({0x00, 0x00, 0x00, 0x00, 0xB1, 0x07, 0x1E, 0x00,  //
                                    0x00, 0x00, 0xFE, 0x0D, 0xFE, 0x31, 0x28, 0x01,  //
                                    0x00, 0x00, 0x97, 0xFD, 0xFE, 0x31, 0x22, 0x01,  //
                                    0xEF, 0x4C, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x00}));
  ASSERT_TRUE(motors);
  EXPECT_EQ(motors->cutter.voltage_raw, 1969);
  EXPECT_EQ(motors->cutter.temperature_c, 30);
  // The right drive motor comes first.
  EXPECT_EQ(motors->right.current_raw, 3582);
  EXPECT_EQ(motors->right.voltage_raw, 12798);
  EXPECT_EQ(motors->right.temperature_c, 40);
  EXPECT_EQ(motors->right.status, 1);
  EXPECT_EQ(motors->left.current_raw, -617);
  EXPECT_EQ(motors->left.temperature_c, 34);
}

// Measured 07.11.: forwards the MCU reports -3/-4 rpm, on a left turn -3 first and +4 second.
TEST(Telemetry, MotorsBySidePositiveForwards) {
  auto frame = [](int16_t first, int16_t second) {
    std::vector<uint8_t> payload(32, 0);
    payload[8] = static_cast<uint8_t>(first & 0xFF);
    payload[9] = static_cast<uint8_t>((first >> 8) & 0xFF);
    payload[16] = static_cast<uint8_t>(second & 0xFF);
    payload[17] = static_cast<uint8_t>((second >> 8) & 0xFF);
    return *DecodeMotors(payload);
  };
  auto forwards = frame(-3, -4);
  EXPECT_EQ(forwards.right.rpm, 3);
  EXPECT_EQ(forwards.left.rpm, 4);
  auto backwards = frame(3, 4);
  EXPECT_EQ(backwards.right.rpm, -3);
  EXPECT_EQ(backwards.left.rpm, -4);
  // Left turn: the left wheel turns back, the right one forwards.
  auto left_turn = frame(-3, 4);
  EXPECT_EQ(left_turn.right.rpm, 3);
  EXPECT_EQ(left_turn.left.rpm, -4);
}

TEST(Telemetry, SensorInfo) {
  auto info = DecodeSensorInfo(Bytes({0, 0, 1, 0, 0, 0, 2, 0, 1, 0}));
  ASSERT_TRUE(info);
  EXPECT_TRUE(info->lift);
  EXPECT_FALSE(info->stop);
  EXPECT_EQ(info->cutter_size, 2);
  EXPECT_TRUE(info->bumper_right);
  EXPECT_FALSE(info->bumper_left);
  EXPECT_FALSE(DecodeSensorInfo(Bytes({0, 0, 0, 0, 0, 0, 0, 0})));
}

TEST(Telemetry, VersionsUptimeLog) {
  auto versions = DecodeVersionInfo(Bytes({0, 6, 36, 0, 6, 34, 0, 0, 0}));
  ASSERT_TRUE(versions);
  EXPECT_EQ(versions->cutter_board.patch, 36);
  EXPECT_EQ(versions->chassis_board.patch, 34);
  EXPECT_EQ(DecodeBmsVersion(Bytes({1, 18, 0}))->minor, 18);
  EXPECT_EQ(*DecodeUptime(Bytes({0x58, 0x02, 0x00, 0x00})), 600u);
  EXPECT_EQ(DecodeLog(Bytes({'o', 'k', 0, 'x'})), "ok");
}
