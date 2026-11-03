#include <gtest/gtest.h>

#include "xcore_driver/udp.h"

using namespace xcore;

TEST(Udp, Ipv4Text) {
  uint32_t ip = 0;
  ASSERT_TRUE(ParseIpv4("172.16.78.1", ip));
  EXPECT_EQ(ip, 0xAC104E01u);
  EXPECT_EQ(Ipv4Text(ip), "172.16.78.1");
  EXPECT_FALSE(ParseIpv4("172.16.78", ip));
  EXPECT_FALSE(ParseIpv4("eth0", ip));
}

TEST(Udp, LoopbackRoundTrip) {
  uint32_t loopback = 0x7F000001;
  std::string error;
  UdpSocket a, b;
  ASSERT_TRUE(a.Bind(loopback, 0, false, error)) << error;
  ASSERT_TRUE(b.Bind(loopback, 0, false, error)) << error;
  ASSERT_NE(b.LocalPort(), 0);
  uint8_t data[3] = {1, 2, 3};
  ASSERT_TRUE(a.Send(data, sizeof(data), {loopback, b.LocalPort()}));
  uint8_t buffer[16];
  Endpoint from;
  long n = b.Receive(buffer, sizeof(buffer), from, 1000);
  ASSERT_EQ(n, 3);
  EXPECT_EQ(buffer[2], 3);
  EXPECT_EQ(from.port, a.LocalPort());
  EXPECT_EQ(from.ip, loopback);
  EXPECT_EQ(b.Receive(buffer, sizeof(buffer), from, 10), 0);
  // A datagram longer than the buffer reports its full size.
  uint8_t big[40] = {};
  ASSERT_TRUE(a.Send(big, sizeof(big), {loopback, b.LocalPort()}));
  EXPECT_EQ(b.Receive(buffer, sizeof(buffer), from, 1000), 40);
}

TEST(Udp, SharedPort) {
  uint32_t any = 0;
  std::string error;
  UdpSocket a, b;
  ASSERT_TRUE(a.Bind(any, 0, true, error)) << error;
  ASSERT_TRUE(b.Bind(any, a.LocalPort(), true, error)) << error;
  UdpSocket c;
  EXPECT_FALSE(c.Bind(any, a.LocalPort(), false, error));
}

TEST(Udp, ForbiddenPortsAreNeverSentTo) {
  EXPECT_TRUE(ForbiddenPort(8007));
  EXPECT_TRUE(ForbiddenPort(10000));
  EXPECT_TRUE(ForbiddenPort(65102));
  EXPECT_TRUE(ForbiddenPort(65103));
  EXPECT_TRUE(ForbiddenPort(65104));
  EXPECT_FALSE(ForbiddenPort(4242));
  EXPECT_FALSE(ForbiddenPort(49154));
  uint32_t loopback = 0x7F000001;
  std::string error;
  UdpSocket a;
  ASSERT_TRUE(a.Bind(loopback, 0, false, error)) << error;
  uint8_t text[5] = {'R', 'E', 'S', 'E', 'T'};
  EXPECT_FALSE(a.Send(text, sizeof(text), {loopback, 8007}));
  EXPECT_FALSE(a.Send(text, sizeof(text), {loopback, 65103}));
}

TEST(Udp, InterfaceAddress) {
  uint32_t ip = 0;
  ASSERT_TRUE(InterfaceAddress("lo", ip));
  EXPECT_EQ(ip, 0x7F000001u);
  EXPECT_FALSE(InterfaceAddress("no-such-interface", ip));
}
