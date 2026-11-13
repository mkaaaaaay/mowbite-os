#include "mowbite_net/weather.h"

#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>

namespace mowbite_net {

namespace {

constexpr int kQuarters = 4;

bool Number(const nlohmann::json& j) {
  return j.is_number() && std::isfinite(j.get<double>());
}

}  // namespace

std::string WeatherUrl(const std::string& server, double latitude, double longitude, int hours) {
  char query[200];
  std::snprintf(query, sizeof(query),
                "/v1/forecast?latitude=%.2f&longitude=%.2f&current=precipitation,temperature_2m"
                "&minutely_15=precipitation&forecast_minutely_15=%d",
                latitude, longitude, hours * kQuarters);
  return server + query;
}

WeatherData ParseWeather(const std::string& body, int hours) {
  WeatherData data;
  const nlohmann::json j = nlohmann::json::parse(body, nullptr, false);
  if (!j.is_object()) return data;
  const nlohmann::json current = j.value("current", nlohmann::json());
  if (current.is_object()) {
    if (current.contains("temperature_2m") && Number(current["temperature_2m"])) {
      data.has_temp = true;
      data.temp_c = current["temperature_2m"].get<double>();
    }
  }
  const nlohmann::json minutely = j.value("minutely_15", nlohmann::json());
  const nlohmann::json quarters = minutely.is_object() ? minutely.value("precipitation", nlohmann::json()) : nullptr;
  const bool now = current.is_object() && current.contains("precipitation") && Number(current["precipitation"]);
  if (now && quarters.is_array() && quarters.size() >= static_cast<size_t>(hours * kQuarters)) {
    bool all = true;
    std::vector<double> per_hour(static_cast<size_t>(hours), 0.0);
    for (size_t i = 0; i < per_hour.size() * kQuarters; ++i) {
      if (!Number(quarters[i]) || quarters[i].get<double>() < 0) {
        all = false;
        break;
      }
      per_hour[i / kQuarters] += quarters[i].get<double>();
    }
    if (all && current["precipitation"].get<double>() >= 0) {
      data.has_rain = true;
      data.rain_now_mm = current["precipitation"].get<double>();
      data.rain_hour_mm = per_hour;
    }
  }
  data.ok = data.has_rain || data.has_temp;
  return data;
}

std::optional<WeatherData> WeatherCache::Get(double latitude, double longitude, int hours, double mono,
                                             double* age) const {
  if (!ok_ || mono - at_ >= kWeatherKeep || mono < at_) return std::nullopt;
  if (std::fabs(latitude - latitude_) > 0.005 || std::fabs(longitude - longitude_) > 0.005 || hours > hours_) {
    return std::nullopt;
  }
  *age = mono - at_;
  WeatherData data = data_;
  if (data.has_rain) data.rain_hour_mm.resize(static_cast<size_t>(hours));
  return data;
}

bool WeatherCache::MayFetch(double mono) const {
  return !fetched_ || last_ok_ || mono - *fetched_ >= kWeatherRetry || mono < *fetched_;
}

void WeatherCache::Fetched(double latitude, double longitude, int hours, double mono, const WeatherData& data) {
  fetched_ = mono;
  last_ok_ = data.ok;
  if (!data.ok) return;
  ok_ = true;
  latitude_ = latitude;
  longitude_ = longitude;
  hours_ = hours;
  at_ = mono;
  data_ = data;
}

}  // namespace mowbite_net
