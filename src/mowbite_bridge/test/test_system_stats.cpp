#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "mowbite_bridge/system_stats.h"

using namespace mowbite_bridge;
namespace fs = std::filesystem;

class SystemStatsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("mowbite_stats_" + std::to_string(::getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "proc/net");
    fs::create_directories(dir_ / "thermal");
    sources_ = {(dir_ / "proc").string(), (dir_ / "thermal").string(), dir_.string(), dir_.string()};
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }
  void Put(const std::string& path, const std::string& text) {
    fs::create_directories((dir_ / path).parent_path());
    std::ofstream(dir_ / path) << text;
  }
  void Zone(int n, const std::string& type, const std::string& milli) {
    Put("thermal/thermal_zone" + std::to_string(n) + "/type", type + "\n");
    Put("thermal/thermal_zone" + std::to_string(n) + "/temp", milli + "\n");
  }

  fs::path dir_;
  SystemSources sources_;
};

TEST_F(SystemStatsTest, ValuesInTheUnitsOfSystemCgi) {
  Put("proc/meminfo", "MemTotal:        3995000 kB\nMemFree:  100 kB\nMemAvailable:    2100000 kB\n");
  Put("proc/loadavg", "1.25 0.80 0.40 2/310 4242\n");
  Put("proc/uptime", "86400.73 300000.00\n");
  Put("proc/cpuinfo", "processor\t: 0\nBogoMIPS\t: 48.00\n\nprocessor\t: 1\n\nprocessor\t: 2\n");
  Zone(0, "gpu-thermal", "41000");
  Zone(1, "soc-thermal", "47500");
  Put("proc/net/wireless",
      "Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
      " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
      " wlan0: 0000   58.  -52.  -256        0      0      0      0      0        0\n");
  const Json s = SystemStats(sources_);
  EXPECT_EQ(s["mem_total"], 3995000ull * 1024);
  EXPECT_EQ(s["mem_available"], 2100000ull * 1024);
  EXPECT_DOUBLE_EQ(s["load"].get<double>(), 1.25);
  EXPECT_EQ(s["uptime"], 86400);
  EXPECT_EQ(s["cpus"], 3);
  EXPECT_DOUBLE_EQ(s["cpu_temp"].get<double>(), 47.5);
  EXPECT_EQ(s["wifi_iface"], "wlan0");
  EXPECT_EQ(s["wifi_signal"], -52);
  EXPECT_GT(s["disk_total"].get<uint64_t>(), 0u);
  EXPECT_LE(s["disk_free"].get<uint64_t>(), s["disk_total"].get<uint64_t>());
  EXPECT_EQ(s["data_total"], s["disk_total"]);
  EXPECT_FALSE(s.contains("wifi_ssid"));
}

TEST_F(SystemStatsTest, WhatCannotBeReadIsLeftOut) {
  sources_.data = "";
  const Json s = SystemStats(sources_);
  for (const char* key : {"mem_total", "mem_available", "load", "uptime", "cpus", "cpu_temp", "wifi_iface",
                          "wifi_signal", "net_iface", "net_ip", "data_total", "data_free"}) {
    EXPECT_FALSE(s.contains(key)) << key;
  }
  EXPECT_TRUE(s.contains("disk_total"));
  sources_.root = (dir_ / "missing").string();
  EXPECT_FALSE(SystemStats(sources_).contains("disk_total"));
}

TEST_F(SystemStatsTest, TheFirstZoneWithoutOneOfTheCpu) {
  Zone(0, "gpu-thermal", "bad");
  Zone(1, "npu-thermal", "39000");
  Zone(2, "center-thermal", "45000");
  EXPECT_DOUBLE_EQ(SystemStats(sources_)["cpu_temp"].get<double>(), 39.0);
}

TEST_F(SystemStatsTest, ASignalCountedFromZero) {
  Put("proc/net/wireless",
      "Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
      " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
      "  wlan1: 0000   40.  196.  0        0      0      0      0      0        0\n");
  const Json s = SystemStats(sources_);
  EXPECT_EQ(s["wifi_iface"], "wlan1");
  EXPECT_EQ(s["wifi_signal"], -60);
}

TEST_F(SystemStatsTest, TheDefaultRouteWithTheLowestMetric) {
  Put("proc/net/route",
      "Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT\n"
      "eth9\t0002A8C0\t00000000\t0001\t0\t0\t0\t00FFFFFF\t0\t0\t0\n"
      "eth9\t00000000\t0102A8C0\t0003\t0\t0\t600\t00000000\t0\t0\t0\n"
      "lo\t00000000\t0102A8C0\t0003\t0\t0\t100\t00000000\t0\t0\t0\n");
  const Json s = SystemStats(sources_);
  EXPECT_EQ(s["net_iface"], "lo");
  EXPECT_EQ(s["net_ip"], "127.0.0.1");
}
