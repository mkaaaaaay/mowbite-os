#include <gtest/gtest.h>

#include "mowbite_net/weather.h"

using namespace mowbite_net;

namespace {

const char kAnswer[] = R"({"current": {"precipitation": 0.2, "temperature_2m": 7.4},
  "minutely_15": {"precipitation": [0.1, 0.1, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0]}})";

}  // namespace

TEST(Weather, Url) {
  EXPECT_EQ(WeatherUrl("https://api.open-meteo.com", 52.123, 9.987, 3),
            "https://api.open-meteo.com/v1/forecast?latitude=52.12&longitude=9.99&current=precipitation,"
            "temperature_2m&minutely_15=precipitation&forecast_minutely_15=12");
}

TEST(Weather, ParsesRainPerHourAndTemperature) {
  const WeatherData d = ParseWeather(kAnswer, 3);
  ASSERT_TRUE(d.ok);
  EXPECT_TRUE(d.has_rain);
  EXPECT_DOUBLE_EQ(d.rain_now_mm, 0.2);
  ASSERT_EQ(d.rain_hour_mm.size(), 3u);
  EXPECT_NEAR(d.rain_hour_mm[0], 0.2, 1e-9);
  EXPECT_NEAR(d.rain_hour_mm[1], 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(d.rain_hour_mm[2], 0);
  EXPECT_TRUE(d.has_temp);
  EXPECT_DOUBLE_EQ(d.temp_c, 7.4);
}

TEST(Weather, BrokenOrPartAnswers) {
  EXPECT_FALSE(ParseWeather("<html>", 3).ok);
  EXPECT_FALSE(ParseWeather("{}", 3).ok);
  // Too few quarters or a gap in them: no rain, the temperature still counts.
  WeatherData d = ParseWeather(R"({"current": {"precipitation": 0, "temperature_2m": 3},
      "minutely_15": {"precipitation": [0, null, 0, 0]}})", 1);
  EXPECT_TRUE(d.ok);
  EXPECT_FALSE(d.has_rain);
  EXPECT_TRUE(d.has_temp);
  d = ParseWeather(R"({"current": {"precipitation": 0}, "minutely_15": {"precipitation": [0, 0, 0, 0]}})", 2);
  EXPECT_FALSE(d.ok);
  d = ParseWeather(R"({"current": {"precipitation": -1}, "minutely_15": {"precipitation": [0, 0, 0, 0]}})", 1);
  EXPECT_FALSE(d.has_rain);
}

TEST(Weather, CacheKeepsFifteenMinutesForThePlace) {
  WeatherCache cache;
  double age = -1;
  EXPECT_TRUE(cache.MayFetch(0));
  cache.Fetched(52.12, 9.99, 3, 100, ParseWeather(kAnswer, 3));
  std::optional<WeatherData> d = cache.Get(52.12, 9.99, 2, 400, &age);
  ASSERT_TRUE(d);
  EXPECT_DOUBLE_EQ(age, 300);
  EXPECT_EQ(d->rain_hour_mm.size(), 2u);
  EXPECT_FALSE(cache.Get(52.13, 9.99, 2, 400, &age));
  EXPECT_FALSE(cache.Get(52.12, 9.99, 2, 100 + kWeatherKeep, &age));
  EXPECT_TRUE(cache.MayFetch(101));
}

TEST(Weather, AFailedFetchWaitsAMinute) {
  WeatherCache cache;
  cache.Fetched(52.12, 9.99, 3, 100, WeatherData());
  double age = 0;
  EXPECT_FALSE(cache.Get(52.12, 9.99, 1, 101, &age));
  EXPECT_FALSE(cache.MayFetch(150));
  EXPECT_TRUE(cache.MayFetch(160));
}
