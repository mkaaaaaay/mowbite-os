#include <gtest/gtest.h>

#include "xcore_driver/socket_table.h"

using namespace xcore;

namespace {

constexpr uint32_t kBindIp = 0xAC104E01;  // 172.16.78.1

const char kUdp[] =
    "   sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref pointer drops\n"
    "  1: 014E10AC:C350 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1001 2 0 0\n"
    "  2: 00000000:1092 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1002 2 0 0\n"
    "  3: 014E10AC:0043 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1003 2 0 0\n"
    "  4: 014E10AC:0035 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1004 2 0 0\n"
    "  5: 00000000:D431 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1005 2 0 0\n"
    "  6: 0100007F:C351 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1006 2 0 0\n"
    "  7: 004EFFE9:1092 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 1007 2 0 0\n";

const char kUdp6[] =
    "  sl  local_address                         remote_address                        st tx_queue rx_queue tr "
    "tm->when retrnsmt   uid  timeout inode ref pointer drops\n"
    "  1: 00000000000000000000000000000000:1092 00000000000000000000000000000000:0000 07 00000000:00000000 00:00000000 "
    "00000000     0        0 2001 2 0 0\n"
    "  2: 0000000000000000FFFF0000014E10AC:9C40 00000000000000000000000000000000:0000 07 00000000:00000000 00:00000000 "
    "00000000     0        0 2002 2 0 0\n"
    "  3: 00000000000000000000000000000000:9C41 00000000000000000000000000000000:0000 07 00000000:00000000 00:00000000 "
    "00000000     0        0 2003 2 0 0\n";

}  // namespace

TEST(SocketTable, FindsSocketsAtTheBindAddressAndTheAnnouncementPort) {
  auto found = ForeignSockets(kUdp, kBindIp, 4242, {});
  ASSERT_EQ(found.size(), 3u);
  // A host's service socket at 172.16.78.1:50000.
  EXPECT_EQ(found[0].ip, kBindIp);
  EXPECT_EQ(found[0].port, 50000);
  EXPECT_EQ(found[0].inode, 1001u);
  // Anything at port 4242, on the wildcard or the group.
  EXPECT_EQ(found[1].port, 4242);
  EXPECT_EQ(found[1].inode, 1002u);
  EXPECT_EQ(found[2].inode, 1007u);
  EXPECT_EQ(ForeignSocketText(found[0]), "udp 172.16.78.1:50000 (inode 1001)");
}

TEST(SocketTable, LeavesOutOwnSocketsSystemServicesAndOthers) {
  // DHCP (67) and DNS (53) on the bind address, a client on the wildcard and one on loopback count as no host.
  auto found = ForeignSockets(kUdp, kBindIp, 4242, {1001, 1002, 1007});
  EXPECT_TRUE(found.empty());
}

TEST(SocketTable, ReadsIpv6WithWildcardAndMappedAddresses) {
  auto found = ForeignSockets(kUdp6, kBindIp, 4242, {});
  ASSERT_EQ(found.size(), 2u);
  EXPECT_TRUE(found[0].ipv6);
  EXPECT_EQ(found[0].port, 4242);
  EXPECT_EQ(found[1].ip, kBindIp);
  EXPECT_EQ(found[1].port, 40000);
}

TEST(SocketTable, IgnoresBrokenLines) {
  EXPECT_TRUE(ForeignSockets("", kBindIp, 4242, {}).empty());
  EXPECT_TRUE(ForeignSockets("  1: 014E10AC 00000000:0000 07 0 0 0 0 0 1\n", kBindIp, 4242, {}).empty());
  EXPECT_TRUE(ForeignSockets("  1: 014E10AC:ZZ 00000000:0000 07 0 0 0 0 0 1\n", kBindIp, 4242, {}).empty());
  EXPECT_TRUE(ForeignSockets("  1: 014E10AC:C350 00000000:0000 07 0 0 0 0 0 x\n", kBindIp, 4242, {}).empty());
}
