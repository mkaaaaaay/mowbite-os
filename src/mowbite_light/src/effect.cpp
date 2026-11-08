#include "mowbite_light/effect.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace mowbite_light {

namespace {

// The darkest step of a pulse, of its colour.
constexpr double kPulseFloor = 0.05;

const Rgb kWhite{255, 255, 255};
const Rgb kYellow{255, 160, 0};
const Rgb kGreen{0, 255, 0};
const Rgb kRed{255, 0, 0};
const Rgb kBlue{0, 80, 255};

Rgb Scale(Rgb c, double f) {
  f = std::clamp(f, 0.0, 1.0);
  return {static_cast<uint8_t>(std::lround(c.r * f)), static_cast<uint8_t>(std::lround(c.g * f)),
          static_cast<uint8_t>(std::lround(c.b * f))};
}

Rgb Mix(Rgb a, Rgb b, double f) {
  auto channel = [f](uint8_t x, uint8_t y) { return static_cast<uint8_t>(std::lround(x + (y - x) * f)); };
  return {channel(a.r, b.r), channel(a.g, b.g), channel(a.b, b.b)};
}

// 0 <= result < 1: how far t is into its period.
double Phase(double t, double period) {
  if (period <= 0) return 0;
  const double p = std::fmod(t / period, 1.0);
  return p < 0 ? p + 1 : p;
}

std::string Hex(Rgb c) {
  char text[8];
  std::snprintf(text, sizeof(text), "#%02x%02x%02x", c.r, c.g, c.b);
  return text;
}

std::optional<Rgb> ParseHex(const Json& value) {
  if (!value.is_string()) return std::nullopt;
  const std::string text = value.get<std::string>();
  if (text.size() != 7 || text[0] != '#') return std::nullopt;
  unsigned r = 0, g = 0, b = 0;
  for (size_t i = 1; i < 7; ++i) {
    if (!std::isxdigit(static_cast<unsigned char>(text[i]))) return std::nullopt;
  }
  if (std::sscanf(text.c_str() + 1, "%2x%2x%2x", &r, &g, &b) != 3) return std::nullopt;
  return Rgb{static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)};
}

Effect Make(EffectKind kind, std::vector<Rgb> colors, double period) {
  return {kind, std::move(colors), period, 1.0};
}

}  // namespace

const std::vector<std::string>& EffectNames() {
  static const std::vector<std::string> names = {"off", "solid", "blink", "pulse", "chase", "gradient"};
  return names;
}

std::vector<Rgb> Render(const Effect& effect, size_t n, double t) {
  std::vector<Rgb> leds(n);
  if (effect.colors.empty() || n == 0) return leds;
  const Rgb first = effect.colors[0];
  const Rgb second = effect.colors.size() > 1 ? effect.colors[1] : Rgb{};
  const double phase = Phase(t, effect.period);
  switch (effect.kind) {
    case EffectKind::kOff:
      break;
    case EffectKind::kSolid:
      std::fill(leds.begin(), leds.end(), first);
      break;
    case EffectKind::kBlink:
      std::fill(leds.begin(), leds.end(), phase < 0.5 ? first : second);
      break;
    case EffectKind::kPulse: {
      const double level = kPulseFloor + (1 - kPulseFloor) * 0.5 * (1 - std::cos(2 * M_PI * phase));
      std::fill(leds.begin(), leds.end(), Scale(first, level));
      break;
    }
    case EffectKind::kChase: {
      const size_t run = std::max<size_t>(1, n / 5);
      const size_t head = static_cast<size_t>(phase * n);
      for (size_t i = 0; i < n; ++i) leds[i] = (i + n - head) % n < run ? first : second;
      break;
    }
    case EffectKind::kGradient: {
      const size_t parts = effect.colors.size();
      for (size_t i = 0; i < n; ++i) {
        // Along the chain and back to the first colour, so that a moving gradient has no seam.
        const double x = std::fmod(static_cast<double>(i) / n + phase, 1.0) * parts;
        const size_t k = static_cast<size_t>(x) % parts;
        leds[i] = Mix(effect.colors[k], effect.colors[(k + 1) % parts], x - std::floor(x));
      }
      break;
    }
  }
  for (auto& c : leds) c = Scale(c, effect.brightness);
  return leds;
}

Json ToJson(const Effect& effect) {
  Json colors = Json::array();
  for (const auto& c : effect.colors) colors.push_back(Hex(c));
  return Json{{"effect", EffectNames()[static_cast<size_t>(effect.kind)]},
              {"colors", colors},
              {"period", effect.period},
              {"brightness", effect.brightness}};
}

std::optional<Effect> EffectFromJson(const Json& value, std::string* error) {
  if (!value.is_object()) {
    *error = "an effect is an object";
    return std::nullopt;
  }
  Effect effect;
  const auto& names = EffectNames();
  const auto kind = value.find("effect");
  const auto name =
      kind != value.end() && kind->is_string() ? std::find(names.begin(), names.end(), *kind) : names.end();
  if (name == names.end()) {
    *error = "effect is one of off, solid, blink, pulse, chase, gradient";
    return std::nullopt;
  }
  effect.kind = static_cast<EffectKind>(name - names.begin());
  const Json colors = value.value("colors", Json::array());
  if (!colors.is_array() || colors.size() > kMaxColors || (colors.empty() && effect.kind != EffectKind::kOff)) {
    *error = "colors are 1 to 4 texts #rrggbb";
    return std::nullopt;
  }
  for (const auto& c : colors) {
    const auto rgb = ParseHex(c);
    if (!rgb) {
      *error = "colors are 1 to 4 texts #rrggbb";
      return std::nullopt;
    }
    effect.colors.push_back(*rgb);
  }
  const Json period = value.value("period", Json(1.0));
  const double p = period.is_number() ? period.get<double>() : -1.0;
  if (!((p >= kMinPeriod && p <= kMaxPeriod) || (p == 0 && effect.kind == EffectKind::kGradient))) {
    *error = "period is 0.2 to 20 s, for a gradient also 0";
    return std::nullopt;
  }
  effect.period = p;
  const Json brightness = value.value("brightness", Json(1.0));
  if (!brightness.is_number() || !(brightness.get<double>() >= 0 && brightness.get<double>() <= 1)) {
    *error = "brightness is 0 to 1";
    return std::nullopt;
  }
  effect.brightness = brightness.get<double>();
  return effect;
}

const std::vector<std::string>& ProfileStates() {
  static const std::vector<std::string> states = {"IDLE",     "MOWING",   "PAUSED",  "DOCKING",    "UNDOCKING",
                                                  "AREA_RECORDING", "CHARGING", "CHARGED", "BATTERY_LOW"};
  return states;
}

Profile DefaultProfile() {
  return {{"IDLE", Make(EffectKind::kPulse, {kWhite}, 3.0)},
          {"MOWING", Make(EffectKind::kSolid, {kWhite}, 1.0)},
          {"PAUSED", Make(EffectKind::kSolid, {kYellow}, 1.0)},
          {"DOCKING", Make(EffectKind::kSolid, {kGreen}, 1.0)},
          {"UNDOCKING", Make(EffectKind::kBlink, {kWhite}, 1.0)},
          {"AREA_RECORDING", Make(EffectKind::kPulse, {kBlue}, 2.0)},
          {"CHARGING", Make(EffectKind::kChase, {kGreen}, 2.0)},
          {"CHARGED", Make(EffectKind::kSolid, {kGreen}, 1.0)},
          {"BATTERY_LOW", Make(EffectKind::kPulse, {kRed}, 3.0)}};
}

Effect EmergencyEffect() {
  return Make(EffectKind::kBlink, {kRed}, 0.5);
}

Json ToJson(const Profile& profile) {
  Json out = Json::object();
  for (const auto& state : ProfileStates()) {
    const auto it = profile.find(state);
    if (it != profile.end()) out[state] = ToJson(it->second);
  }
  return out;
}

std::optional<Profile> ProfileFromJson(const Json& value, std::string* error) {
  if (!value.is_object()) {
    *error = "the profile is an object of states";
    return std::nullopt;
  }
  Profile profile = DefaultProfile();
  for (const auto& [state, effect_json] : value.items()) {
    if (!profile.count(state)) {
      *error = "unknown state " + state;
      return std::nullopt;
    }
    std::string why;
    const auto effect = EffectFromJson(effect_json, &why);
    if (!effect) {
      *error = state + ": " + why;
      return std::nullopt;
    }
    profile[state] = *effect;
  }
  return profile;
}

std::string ShownState(const std::string& current_state, bool is_charging, double battery, double low_battery,
                       double charged_from) {
  if (is_charging) return battery >= charged_from ? "CHARGED" : "CHARGING";
  if (current_state == "IDLE" && battery > 0 && battery < low_battery) return "BATTERY_LOW";
  const auto& states = ProfileStates();
  return std::find(states.begin(), states.end(), current_state) != states.end() ? current_state : "IDLE";
}

}  // namespace mowbite_light
