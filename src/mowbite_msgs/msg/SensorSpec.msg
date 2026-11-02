# bridge/sensor_info, latched: what a sensor reports on bridge/sensor_value. A new spec of an id replaces the old one.
string id
string name

uint8 KIND_NUMBER=0
uint8 KIND_TEXT=1
uint8 kind

uint8 MEANING_UNKNOWN=0
uint8 MEANING_VOLTAGE=1
uint8 MEANING_CURRENT=2
uint8 MEANING_TEMPERATURE=3
uint8 MEANING_REVOLUTIONS=4
uint8 MEANING_DISTANCE=5
uint8 meaning
string unit

# Limits of a number, NaN where there is none. The range counts only with both ends.
float64 min
float64 max
float64 critical_low
float64 critical_high
