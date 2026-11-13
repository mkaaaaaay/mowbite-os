# net/weather: the answer of mowbite_net to a WeatherRequest with the same id.

int64 id
# false: no usable answer (no network, timeout, broken reply).
bool ok
# Age of the data in s.
float64 age_s
bool has_rain
float64 rain_now_mm
# Rain in each of the coming hours.
float64[] rain_hour_mm
bool has_temp
float64 temp_c
