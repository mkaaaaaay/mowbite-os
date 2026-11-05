#include "mowbite_logic/home_watch.h"

#include <algorithm>

namespace mowbite_logic {

namespace {

// Since when a condition holds without a break.
void Track(std::optional<double>* since, bool holds, double now) {
  if (!holds) {
    since->reset();
  } else if (!*since) {
    *since = now;
  }
}

bool Held(const std::optional<double>& since, double now, double time) {
  return since && now - *since >= time;
}

}  // namespace

void HomeWatch::OnSample(const HomeSample& sample) {
  if (!sample.telemetry_ok || (last_ && sample.time - *last_ > config_.max_gap)) Reset();
  if (!sample.telemetry_ok) return;
  last_ = sample.time;
  now_ = sample.time;
  if (sample.battery_ok) {
    Track(&critical_since_, sample.battery_voltage < config_.critical_voltage, now_);
    Track(&battery_hot_since_,
          config_.battery_hot_temperature > 0 && sample.battery_temperature >= config_.battery_hot_temperature, now_);
    if (!window_since_) window_since_ = now_;
    window_.emplace_back(now_, sample.battery_voltage);
    while (!window_.empty() && window_.front().first < now_ - config_.low_window) window_.pop_front();
  } else {
    critical_since_.reset();
    battery_hot_since_.reset();
    window_since_.reset();
    window_.clear();
  }
  Track(&motor_hot_since_, sample.blade_ok && sample.blade_temperature >= config_.motor_hot_temperature, now_);
  Track(&wet_since_, config_.rain_mode != 0 && sample.rain, now_);
}

bool HomeWatch::Holds(HomeReason reason) const {
  switch (reason) {
    case HomeReason::kBatteryCritical: return Held(critical_since_, now_, config_.critical_time);
    case HomeReason::kBatteryHot: return Held(battery_hot_since_, now_, config_.hot_time);
    case HomeReason::kBatteryLow:
      return Held(window_since_, now_, config_.low_window) && WindowVoltage() < config_.empty_voltage;
    case HomeReason::kMotorHot: return Held(motor_hot_since_, now_, config_.hot_time);
    case HomeReason::kRain: return Held(wet_since_, now_, config_.rain_check_seconds);
  }
  return false;
}

std::vector<HomeReason> HomeWatch::Due() const {
  std::vector<HomeReason> due;
  for (HomeReason reason : {HomeReason::kBatteryCritical, HomeReason::kBatteryHot, HomeReason::kBatteryLow,
                            HomeReason::kMotorHot, HomeReason::kRain}) {
    if (Holds(reason)) due.push_back(reason);
  }
  return due;
}

double HomeWatch::WindowVoltage() const {
  double highest = 0;
  for (const auto& sample : window_) highest = std::max(highest, sample.second);
  return highest;
}

void HomeWatch::Reset() {
  last_.reset();
  critical_since_.reset();
  battery_hot_since_.reset();
  motor_hot_since_.reset();
  wet_since_.reset();
  window_since_.reset();
  window_.clear();
}

void HomeWatch::SetConfig(const HomeConfig& config) {
  config_ = config;
  Reset();
}

}  // namespace mowbite_logic
