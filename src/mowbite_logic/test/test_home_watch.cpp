#include <gtest/gtest.h>

#include "mowbite_logic/home_watch.h"

using namespace mowbite_logic;

namespace {

HomeSample Good(double t) {
  HomeSample s;
  s.time = t;
  s.telemetry_ok = true;
  s.battery_ok = true;
  s.battery_voltage = 24.0;
  s.battery_temperature = 25;
  s.blade_ok = true;
  s.blade_temperature = 30;
  return s;
}

HomeSample Volts(double t, double v) {
  HomeSample s = Good(t);
  s.battery_voltage = v;
  return s;
}

using Due = std::vector<HomeReason>;

}  // namespace

TEST(HomeWatch, CriticalAfterItsHoldTimeWithoutABreak) {
  HomeWatch watch;
  watch.OnSample(Volts(0, 19.5));
  watch.OnSample(Volts(1, 19.5));
  EXPECT_FALSE(watch.Holds(HomeReason::kBatteryCritical));
  // A dip under load ends at once.
  watch.OnSample(Volts(1.5, 19.8));
  watch.OnSample(Volts(2.5, 19.5));
  watch.OnSample(Volts(4.0, 19.5));
  EXPECT_FALSE(watch.Holds(HomeReason::kBatteryCritical));
  watch.OnSample(Volts(4.5, 19.5));
  EXPECT_TRUE(watch.Holds(HomeReason::kBatteryCritical));
  EXPECT_EQ(watch.Due(), Due{HomeReason::kBatteryCritical});
}

TEST(HomeWatch, LowByTheHighestVoltageOfAFullWindow) {
  HomeWatch watch;
  // Only with samples over the whole window.
  for (int t = 0; t < 20; ++t) watch.OnSample(Volts(t, 20.8));
  EXPECT_FALSE(watch.Holds(HomeReason::kBatteryLow));
  watch.OnSample(Volts(20, 20.8));
  EXPECT_TRUE(watch.Holds(HomeReason::kBatteryLow));
  EXPECT_DOUBLE_EQ(watch.WindowVoltage(), 20.8);

  // One sample at or above empty within the window keeps it from counting, for 20 s.
  HomeWatch other;
  for (int t = 0; t <= 20; ++t) other.OnSample(Volts(t, t == 10 ? 21.0 : 20.8));
  EXPECT_FALSE(other.Holds(HomeReason::kBatteryLow));
  for (int t = 21; t < 31; ++t) other.OnSample(Volts(t, 20.8));
  EXPECT_FALSE(other.Holds(HomeReason::kBatteryLow));
  other.OnSample(Volts(31, 20.8));
  EXPECT_TRUE(other.Holds(HomeReason::kBatteryLow));
}

TEST(HomeWatch, AGapStartsEveryRuleAnew) {
  HomeConfig config;
  config.rain_mode = 1;
  HomeWatch watch(config);
  for (int t = 0; t < 19; ++t) {
    HomeSample s = Volts(t, 20.8);
    s.rain = true;
    watch.OnSample(s);
  }
  // A sample without telemetry is a gap: neither low nor wet.
  HomeSample lost = Volts(19, 20.8);
  lost.telemetry_ok = false;
  lost.rain = true;
  watch.OnSample(lost);
  for (int t = 20; t < 39; ++t) {
    HomeSample s = Volts(t, 20.8);
    s.rain = true;
    watch.OnSample(s);
  }
  EXPECT_TRUE(watch.Due().empty());
  // So does a time between two samples over max_gap.
  HomeWatch late;
  late.OnSample(Volts(0, 19.0));
  late.OnSample(Volts(1.5, 19.0));
  late.OnSample(Volts(4.0, 19.0));
  EXPECT_FALSE(late.Holds(HomeReason::kBatteryCritical));
  late.OnSample(Volts(6.0, 19.0));
  EXPECT_TRUE(late.Holds(HomeReason::kBatteryCritical));
  // Without a battery report the battery rules start anew, the others go on.
  HomeSample unknown = Volts(7, 19.0);
  unknown.battery_ok = false;
  late.OnSample(unknown);
  EXPECT_FALSE(late.Holds(HomeReason::kBatteryCritical));
}

TEST(HomeWatch, NoBatteryLevelForAFullWindowCountsAsLow) {
  HomeWatch watch;
  for (int t = 0; t < 20; ++t) {
    HomeSample s = Good(t);
    s.battery_ok = false;
    watch.OnSample(s);
  }
  EXPECT_FALSE(watch.Holds(HomeReason::kBatteryLow));
  HomeSample s = Good(20);
  s.battery_ok = false;
  watch.OnSample(s);
  EXPECT_TRUE(watch.Holds(HomeReason::kBatteryLow));
  EXPECT_TRUE(watch.BatteryUnreported());
  // A report ends it.
  watch.OnSample(Good(21));
  EXPECT_FALSE(watch.Holds(HomeReason::kBatteryLow));
  EXPECT_FALSE(watch.BatteryUnreported());
}

TEST(HomeWatch, HotForItsTimeOnlyWithAReportAndALimit) {
  HomeConfig config;
  config.motor_hot_temperature = 70;
  HomeWatch watch(config);
  for (double t = 0; t <= 10; t += 1) {
    HomeSample s = Good(t);
    s.blade_temperature = 70;
    // Off at 0.
    s.battery_temperature = 80;
    watch.OnSample(s);
    EXPECT_EQ(watch.Holds(HomeReason::kMotorHot), t >= 3) << t;
  }
  EXPECT_FALSE(watch.Holds(HomeReason::kBatteryHot));
  // A blade without a report is no hot blade.
  HomeSample silent = Good(11);
  silent.blade_ok = false;
  silent.blade_temperature = 90;
  watch.OnSample(silent);
  EXPECT_FALSE(watch.Holds(HomeReason::kMotorHot));

  config.battery_hot_temperature = 55;
  watch.SetConfig(config);
  for (double t = 20; t <= 23; t += 1) {
    HomeSample s = Good(t);
    s.battery_temperature = 55;
    watch.OnSample(s);
  }
  EXPECT_TRUE(watch.Holds(HomeReason::kBatteryHot));
  EXPECT_FALSE(watch.Holds(HomeReason::kMotorHot));
}

TEST(HomeWatch, RainOnlyWithARainModeAndInEverySample) {
  HomeWatch off;
  for (double t = 0; t <= 300; t += 1) {
    HomeSample s = Good(t);
    s.rain = true;
    off.OnSample(s);
  }
  EXPECT_TRUE(off.Due().empty());

  HomeConfig config;
  config.rain_mode = 1;
  config.rain_check_seconds = 30;
  HomeWatch watch(config);
  for (double t = 0; t < 30; t += 1) {
    HomeSample s = Good(t);
    s.rain = t != 5;
    watch.OnSample(s);
  }
  // Dry once at 5 s: wet since 6 s.
  EXPECT_FALSE(watch.Holds(HomeReason::kRain));
  for (double t = 30; t < 37; t += 1) {
    HomeSample s = Good(t);
    s.rain = true;
    watch.OnSample(s);
  }
  EXPECT_TRUE(watch.Holds(HomeReason::kRain));
}

TEST(HomeWatch, TheOrderOfTheReasons) {
  HomeConfig config;
  config.battery_hot_temperature = 55;
  config.rain_mode = 3;
  HomeWatch watch(config);
  for (double t = 0; t <= 25; t += 1) {
    HomeSample s = Volts(t, 19.0);
    s.battery_temperature = 60;
    s.blade_temperature = 90;
    s.rain = true;
    watch.OnSample(s);
  }
  EXPECT_EQ(watch.Due(), (Due{HomeReason::kBatteryCritical, HomeReason::kBatteryHot, HomeReason::kBatteryLow,
                              HomeReason::kMotorHot, HomeReason::kRain}));
  watch.Reset();
  EXPECT_TRUE(watch.Due().empty());
}
