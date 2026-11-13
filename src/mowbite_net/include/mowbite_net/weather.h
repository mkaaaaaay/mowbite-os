#ifndef MOWBITE_NET_WEATHER_H
#define MOWBITE_NET_WEATHER_H

#include <optional>
#include <string>
#include <vector>

namespace mowbite_net {

// At most one fetch in this long (s); a request within gets the stored answer with its age.
constexpr double kWeatherKeep = 900.0;
// A failed fetch is not tried again sooner (s).
constexpr double kWeatherRetry = 60.0;
// Every fetch asks for the most hours a request may want, so the stored answer serves all.
constexpr int kWeatherHours = 3;

struct WeatherData {
  bool ok = false;
  bool has_rain = false;
  double rain_now_mm = 0;
  std::vector<double> rain_hour_mm;
  bool has_temp = false;
  double temp_c = 0;
};

constexpr char kWeatherServer[] = "https://api.open-meteo.com";

// The forecast of open-meteo at server for the place (deg, 0.01) and the coming hours.
std::string WeatherUrl(const std::string& server, double latitude, double longitude, int hours);
// The answer of open-meteo; ok false when it holds neither rain nor temperature.
WeatherData ParseWeather(const std::string& body, int hours);

// The last good answer, by place and hours, and when the last fetch was.
class WeatherCache {
 public:
  // The stored answer when it fits and is younger than kWeatherKeep, with its age in s.
  std::optional<WeatherData> Get(double latitude, double longitude, int hours, double mono, double* age) const;
  // Whether a fetch may go out now.
  bool MayFetch(double mono) const;
  void Fetched(double latitude, double longitude, int hours, double mono, const WeatherData& data);

 private:
  std::optional<double> fetched_;
  bool last_ok_ = false;
  bool ok_ = false;
  double latitude_ = 0;
  double longitude_ = 0;
  int hours_ = 0;
  double at_ = 0;
  WeatherData data_;
};

}  // namespace mowbite_net

#endif  // MOWBITE_NET_WEATHER_H
