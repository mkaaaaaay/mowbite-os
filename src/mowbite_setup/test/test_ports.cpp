#include <gtest/gtest.h>

#include "mowbite_api/setup.h"
#include "mowbite_setup/ports.h"

namespace mowbite_setup {
namespace {

using mowbite_serial::Heard;
using mowbite_serial::Holder;
using mowbite_serial::Kind;
using mowbite_serial::Listened;
using mowbite_serial::PortInfo;

// A mower with the MCU on ttyS9 (driver), a free ttyS4, a USB adapter and a ttyS1 another program holds.
PortsView View() {
  PortsView view;
  view.ports = {{"/dev/ttyS1", {}, "dw-apb-uart", std::nullopt},
                {"/dev/ttyS4", {}, "dw-apb-uart", std::nullopt},
                {"/dev/ttyS9", {}, "dw-apb-uart", std::nullopt},
                {"/dev/ttyUSB0",
                 {"/dev/serial/by-id/usb-FTDI_A1-if00-port0"},
                 "ftdi_sio",
                 mowbite_serial::UsbInfo{"0403", "6001", "FTDI", "FT232R USB UART"}}};
  view.driver_ports = {{"/dev/ttyS9", "tron_mcu"}};
  view.holders = [](const std::string& path) {
    return path == "/dev/ttyS1" ? std::vector<Holder>{{42, "minicom"}} : std::vector<Holder>{};
  };
  view.resting = true;
  return view;
}

std::optional<Refusal> Check(const json& params, const PortsView& view = View(), ListenRequest* out = nullptr) {
  ListenRequest request;
  auto refusal = CheckListen(params, view, &request);
  if (out) *out = request;
  return refusal;
}

TEST(Ports, AnswerTellsUseAndUsb) {
  const json answer = PortsAnswer(View());
  ASSERT_EQ(answer["ports"].size(), 4u);
  EXPECT_FALSE(answer["ports"][0]["free"]);
  EXPECT_EQ(answer["ports"][0]["held_by"][0]["name"], "minicom");
  EXPECT_TRUE(answer["ports"][1]["free"]);
  EXPECT_TRUE(answer["ports"][1]["usb"].is_null());
  EXPECT_EQ(answer["ports"][2]["node"], "tron_mcu");
  EXPECT_FALSE(answer["ports"][2]["free"]);
  EXPECT_EQ(answer["ports"][3]["usb"]["vendor_id"], "0403");
  EXPECT_EQ(answer["ports"][3]["by_id"][0], "/dev/serial/by-id/usb-FTDI_A1-if00-port0");
  EXPECT_TRUE(answer["resting"]);
  EXPECT_FALSE(answer["listening"]);
}

TEST(Ports, WithoutScanOnlyDriverAndExtraPortsAreListed) {
  PortsView view = View();
  view.driver_ports = {{"/dev/pts/7", "tron_mcu"}};
  view.ports = ListedPorts(false, view.driver_ports, {"/dev/pts/9"});
  const json answer = PortsAnswer(view);
  ASSERT_EQ(answer["ports"].size(), 2u);
  EXPECT_EQ(answer["ports"][0]["path"], "/dev/pts/7");
  EXPECT_EQ(answer["ports"][0]["node"], "tron_mcu");
  EXPECT_FALSE(answer["ports"][0]["free"]);
  EXPECT_EQ(answer["ports"][1]["path"], "/dev/pts/9");
  EXPECT_TRUE(answer["ports"][1]["free"]);
}

TEST(Ports, ADriverPortNamedByItsLinkIsFound) {
  PortsView view = View();
  view.driver_ports = {{"/dev/serial/by-id/usb-FTDI_A1-if00-port0", "mowbite_nmea"}};
  EXPECT_EQ(NodeOf(view.ports[3], view.driver_ports), "mowbite_nmea");
  EXPECT_EQ(NodeOf(view.ports[1], view.driver_ports), "");
}

TEST(Ports, ListenTakesAFreePortWithDefaults) {
  ListenRequest request;
  EXPECT_FALSE(Check({{"port", "/dev/ttyS4"}}, View(), &request));
  EXPECT_EQ(request.port, "/dev/ttyS4");
  EXPECT_EQ(request.seconds, mowbite_api::kSetupListenDefaultS);
  EXPECT_EQ(request.bauds, mowbite_serial::ListenBauds());
  EXPECT_FALSE(Check({{"port", "/dev/ttyUSB0"}, {"seconds", 30}, {"baud", 9600}}, View(), &request));
  EXPECT_EQ(request.seconds, 30);
  EXPECT_EQ(request.bauds, std::vector<int>{9600});
}

TEST(Ports, ListenRefusesWhatItMayNotTouch) {
  EXPECT_EQ(Check({{"port", "/dev/ttyS9"}})->reason, "PORT_IN_USE");
  EXPECT_EQ(Check({{"port", "/dev/ttyS9"}})->data["node"], "tron_mcu");
  EXPECT_EQ(Check({{"port", "/dev/ttyS1"}})->data["by"][0], "minicom");
  EXPECT_EQ(Check({{"port", "/dev/ttyS7"}})->reason, "PORT_UNKNOWN");
  PortsView moving = View();
  moving.resting = false;
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}}, moving)->reason, "BUSY");
  PortsView listening = View();
  listening.listening = "/dev/ttyUSB0";
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}}, listening)->reason, "LISTENING");
}

TEST(Ports, ListenParamsOutsideTheirRangeAreInvalid) {
  EXPECT_EQ(Check(json::object())->field, "port");
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}, {"seconds", 0}})->field, "seconds");
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}, {"seconds", mowbite_api::kSetupListenMaxS + 1}})->field, "seconds");
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}, {"seconds", 2.5}})->field, "seconds");
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}, {"baud", 4800}})->field, "baud");
  EXPECT_EQ(Check({{"port", "/dev/ttyS4"}, {"write", true}})->field, "write");
}

TEST(Ports, AnswerOfARecognizedReceiver) {
  Listened listened;
  listened.baud = 115200;
  listened.heard.kind = Kind::kNmea;
  listened.heard.bytes = 1000;
  listened.heard.seconds = 2;
  listened.heard.nmea_lines = 10;
  listened.heard.talkers = {"GN"};
  listened.heard.sentences = {"GGA", "RMC"};
  listened.heard.gga_hz = 1;
  listened.heard.fix = 4;
  Heard quiet;
  listened.tried = {{9600, quiet}, {115200, listened.heard}};
  json answer;
  EXPECT_FALSE(ListenAnswer("/dev/ttyS4", listened, &answer));
  EXPECT_EQ(answer["baud"], 115200);
  EXPECT_EQ(answer["kind"], "nmea");
  EXPECT_DOUBLE_EQ(answer["bytes_per_s"].get<double>(), 500.0);
  EXPECT_EQ(answer["nmea"]["fix"], 4);
  EXPECT_TRUE(answer["wit"].is_null());
  EXPECT_TRUE(answer["rtcm"].is_null());
  ASSERT_EQ(answer["tried"].size(), 2u);
  EXPECT_EQ(answer["tried"][0]["kind"], "silent");
}

TEST(Ports, AnswerOfNothingAndOfAPortThatDidNotOpen) {
  Listened listened;
  json answer;
  EXPECT_FALSE(ListenAnswer("/dev/ttyS4", listened, &answer));
  EXPECT_TRUE(answer["baud"].is_null());
  EXPECT_EQ(answer["kind"], "silent");
  EXPECT_TRUE(answer["nmea"].is_null());
  listened.error = "open /dev/ttyS4: Device or resource busy";
  const auto refused = ListenAnswer("/dev/ttyS4", listened, &answer);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->reason, "PORT_NOT_OPENED");
}

}  // namespace
}  // namespace mowbite_setup
