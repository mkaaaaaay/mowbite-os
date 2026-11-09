#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>

#include "test_util.h"
#include "tron_vision/budget.h"

namespace tron_vision {
namespace {

TEST(Budget, ResidentBytesFromStatm) {
  test::TempDir dir;
  const auto page = static_cast<size_t>(sysconf(_SC_PAGESIZE));
  EXPECT_EQ(ResidentBytes(dir.Write("statm", "5000 1000 300 10 0 900 0\n")), 1000 * page);
  EXPECT_EQ(ResidentBytes(dir.Path() + "/none"), 0U);
  EXPECT_GT(ResidentBytes(), 0U);
}

TEST(Budget, MemAvailable) {
  test::TempDir dir;
  EXPECT_EQ(MemAvailableKb(dir.Write("meminfo", "MemTotal: 3990000 kB\nMemFree: 100 kB\nMemAvailable: 3710000 kB\n")),
            3710000);
  EXPECT_EQ(MemAvailableKb(dir.Write("other", "MemTotal: 1 kB\n")), -1);
}

TEST(Budget, ThermalZoneByType) {
  test::TempDir dir;
  std::filesystem::create_directories(dir.Path() + "/thermal_zone0");
  std::filesystem::create_directories(dir.Path() + "/thermal_zone3");
  dir.Write("thermal_zone0/type", "bigcore0-thermal\n");
  dir.Write("thermal_zone3/type", "soc-thermal\n");
  dir.Write("thermal_zone3/temp", "30538\n");
  const std::string temp = FindThermalZone(dir.Path(), "soc-thermal");
  EXPECT_EQ(temp, dir.Path() + "/thermal_zone3/temp");
  double c = 0.0;
  ASSERT_TRUE(ReadCelsius(temp, c));
  EXPECT_DOUBLE_EQ(c, 30.538);
  EXPECT_EQ(FindThermalZone(dir.Path(), "npu-thermal"), "");
  EXPECT_FALSE(ReadCelsius(dir.Path() + "/none", c));
}

TEST(Budget, ThermalGateHysteresis) {
  ThermalGate gate(80.0, 75.0);
  EXPECT_FALSE(gate.Paused(79.9));
  EXPECT_TRUE(gate.Paused(80.0));
  EXPECT_TRUE(gate.Paused(76.0));
  EXPECT_TRUE(gate.Paused(75.0));
  EXPECT_FALSE(gate.Paused(74.9));
  EXPECT_FALSE(gate.Paused(79.0));
}

TEST(Budget, LogGateOncePerInterval) {
  LogGate gate(60.0);
  int suppressed = -1;
  EXPECT_TRUE(gate.Allow("a", 0.0, suppressed));
  EXPECT_EQ(suppressed, 0);
  EXPECT_FALSE(gate.Allow("a", 10.0, suppressed));
  EXPECT_FALSE(gate.Allow("a", 59.0, suppressed));
  EXPECT_TRUE(gate.Allow("b", 30.0, suppressed));
  EXPECT_TRUE(gate.Allow("a", 60.0, suppressed));
  EXPECT_EQ(suppressed, 2);
  EXPECT_FALSE(gate.Allow("a", 61.0, suppressed));
}

}  // namespace
}  // namespace tron_vision
