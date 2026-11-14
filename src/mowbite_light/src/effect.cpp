#include "mowbite_light/effect.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <string_view>

namespace mowbite_light {

namespace {

// The battery from which charging shows CHARGED.
constexpr double kChargedFrom = 0.99;
// The darkest step of a pulse, of its colour.
constexpr double kPulseFloor = 0.05;
// One flash and the gap after it, of the period.
constexpr double kFlashShare = 0.08;
// Every this many LEDs one sparkles, on average.
constexpr uint32_t kSparkleEvery = 5;

// Per kind in the order of EffectKind: its name, whether it takes no colours and whether it may stand still (period 0).
struct KindRule {
  const char* name;
  bool colorless;
  bool still;
};
constexpr KindRule kKinds[] = {
    {"off", true, false},      {"solid", false, false},   {"blink", false, false},  {"pulse", false, false},
    {"chase", false, false},   {"gradient", false, true}, {"fade", false, false},   {"comet", false, false},
    {"scanner", false, false}, {"rainbow", true, true},   {"sparkle", false, false}, {"flash", false, false},
    {"level", false, true}};
static_assert(std::size(kKinds) == static_cast<size_t>(EffectKind::kLevel) + 1, "one rule per EffectKind");

const KindRule& Rule(EffectKind kind) {
  return kKinds[static_cast<size_t>(kind)];
}

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

// 0 at phase 0, 1 at 0.5 and 0 again at 1, along a cosine.
double Swell(double phase) {
  return 0.5 * (1 - std::cos(2 * M_PI * phase));
}

// The share of its colour a pulse shows at phase.
double Breath(double phase) {
  return kPulseFloor + (1 - kPulseFloor) * Swell(phase);
}

// The colour at hue h (0..1 is once round the circle, red at 0) with full saturation and value.
Rgb Hue(double h) {
  const double x = (h - std::floor(h)) * 6;
  auto channel = [x](double center) {
    double d = std::fabs(x - center);
    d = std::min(d, 6 - d);
    return static_cast<uint8_t>(std::lround(255 * std::clamp(2 - d, 0.0, 1.0)));
  };
  return {channel(0), channel(2), channel(4)};
}

// A well mixed number from a and b, the same on every machine.
uint32_t Hash(uint32_t a, uint32_t b) {
  uint32_t h = a * 0x9e3779b1u + b * 0x632be5abu;
  h ^= h >> 16;
  h *= 0x9e3779b1u;
  h ^= h >> 13;
  return h;
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

// True when all of text is one decimal number.
bool Index(std::string_view text, size_t* value) {
  const char* end = text.data() + text.size();
  const auto [last, error] = std::from_chars(text.data(), end, *value);
  return error == std::errc() && last == end;
}

// The colours of one run of n LEDs.
std::vector<Rgb> Along(const Effect& effect, size_t n, double t, double level) {
  std::vector<Rgb> leds(n);
  if (n == 0 || (effect.colors.empty() && !Rule(effect.kind).colorless)) return leds;
  const Rgb first = effect.colors.empty() ? Rgb{} : effect.colors[0];
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
    case EffectKind::kPulse:
      std::fill(leds.begin(), leds.end(), Scale(first, Breath(phase)));
      break;
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
    case EffectKind::kFade:
      std::fill(leds.begin(), leds.end(), Mix(first, second, Swell(phase)));
      break;
    case EffectKind::kComet: {
      const size_t tail = std::max<size_t>(2, n / 3);
      const size_t head = static_cast<size_t>(phase * n);
      for (size_t i = 0; i < n; ++i) {
        const size_t behind = (head + n - i) % n;
        leds[i] = behind <= tail ? Mix(first, second, static_cast<double>(behind) / (tail + 1)) : second;
      }
      break;
    }
    case EffectKind::kScanner: {
      const double dot = (1 - std::fabs(2 * phase - 1)) * (n - 1);
      for (size_t i = 0; i < n; ++i) leds[i] = Mix(second, first, std::max(0.0, 1 - std::fabs(i - dot)));
      break;
    }
    case EffectKind::kRainbow:
      for (size_t i = 0; i < n; ++i) leds[i] = Hue(static_cast<double>(i) / n - phase);
      break;
    case EffectKind::kSparkle: {
      const double slot = effect.period > 0 ? std::floor(t / effect.period) : 0;
      const auto slot_index = static_cast<uint32_t>(static_cast<int64_t>(slot));
      const Rgb spark = Mix(second, first, Swell(phase));
      for (size_t i = 0; i < n; ++i) {
        leds[i] = Hash(static_cast<uint32_t>(i), slot_index) % kSparkleEvery == 0 ? spark : second;
      }
      break;
    }
    case EffectKind::kFlash: {
      const bool on = phase < kFlashShare || (phase >= 2 * kFlashShare && phase < 3 * kFlashShare);
      std::fill(leds.begin(), leds.end(), on ? first : second);
      break;
    }
    case EffectKind::kLevel: {
      const double filled = std::clamp(level, 0.0, 1.0);
      const size_t lit = filled > 0 ? std::max<size_t>(1, static_cast<size_t>(std::lround(filled * n))) : 0;
      for (size_t i = 0; i < n; ++i) leds[i] = i < lit ? first : second;
      if (lit > 0 && effect.period > 0) leds[lit - 1] = Mix(second, first, Breath(phase));
      break;
    }
  }
  for (auto& c : leds) c = Scale(c, effect.brightness);
  return leds;
}

}  // namespace

const std::vector<std::string>& EffectNames() {
  static const std::vector<std::string> names = [] {
    std::vector<std::string> all;
    for (const KindRule& rule : kKinds) all.emplace_back(rule.name);
    return all;
  }();
  return names;
}

std::vector<Rgb> Render(const Effect& effect, size_t n, double t) {
  return Along(effect, n, t, 1.0);
}

std::optional<std::vector<Strip>> ParseStrips(const std::vector<std::string>& texts, size_t leds, std::string* error) {
  std::vector<Strip> strips;
  std::vector<bool> used(leds);
  for (const auto& text : texts) {
    Strip strip;
    const std::string_view view(text);
    const size_t dash = view.find('-');
    if (dash == std::string_view::npos || !Index(view.substr(0, dash), &strip.first) ||
        !Index(view.substr(dash + 1), &strip.last)) {
      *error = "strip " + text + " is not first-last";
      return std::nullopt;
    }
    if (strip.first >= leds || strip.last >= leds) {
      *error = "strip " + text + " is beyond the " + std::to_string(leds) + " LEDs";
      return std::nullopt;
    }
    const size_t low = std::min(strip.first, strip.last);
    for (size_t i = low; i < low + strip.Length(); ++i) {
      if (used[i]) {
        *error = "strip " + text + " overlaps another at " + std::to_string(i);
        return std::nullopt;
      }
      used[i] = true;
    }
    strips.push_back(strip);
  }
  return strips;
}

std::vector<Rgb> Render(const Effect& effect, const std::vector<Strip>& strips, size_t n, double t, double level) {
  if (strips.empty()) return Along(effect, n, t, level);
  std::vector<Rgb> leds(n);
  for (const auto& strip : strips) {
    const bool forward = strip.first <= strip.last;
    const auto along = Along(effect, strip.Length(), t, level);
    for (size_t k = 0; k < along.size(); ++k) {
      const size_t i = forward ? strip.first + k : strip.first - k;
      if (i < n) leds[i] = along[k];
    }
  }
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
    *error = "effect is one of " + names[0];
    for (size_t i = 1; i < names.size(); ++i) *error += ", " + names[i];
    return std::nullopt;
  }
  effect.kind = static_cast<EffectKind>(name - names.begin());
  const Json colors = value.value("colors", Json::array());
  if (!colors.is_array() || colors.size() > kMaxColors || (colors.empty() && !Rule(effect.kind).colorless)) {
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
  if (!((p >= kMinPeriod && p <= kMaxPeriod) || (p == 0 && Rule(effect.kind).still))) {
    *error = "period is 0.2 to 20 s, for gradient, rainbow and level also 0";
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
  static const std::vector<std::string> states = {"IDLE",        "MOWING",         "PAUSED",   "DOCKING",
                                                  "UNDOCKING",   "AREA_RECORDING", "CHARGING", "CHARGED",
                                                  "BATTERY_LOW", "SCHEDULE_START"};
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
          {"BATTERY_LOW", Make(EffectKind::kPulse, {kRed}, 3.0)},
          {"SCHEDULE_START", Make(EffectKind::kBlink, {kYellow}, 0.5)}};
}

Effect EmergencyEffect() {
  return Make(EffectKind::kBlink, {kRed}, 0.5);
}

Effect StopReadyEffect() {
  return Make(EffectKind::kSolid, {kGreen}, 1.0);
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

std::string ShownState(const std::string& current_state, bool is_charging, double battery, double low_battery) {
  if (is_charging) return battery >= kChargedFrom ? "CHARGED" : "CHARGING";
  if (current_state == "IDLE" && battery > 0 && battery < low_battery) return "BATTERY_LOW";
  const auto& states = ProfileStates();
  return std::find(states.begin(), states.end(), current_state) != states.end() ? current_state : "IDLE";
}

}  // namespace mowbite_light
