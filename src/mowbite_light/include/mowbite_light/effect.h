#ifndef MOWBITE_LIGHT_EFFECT_H
#define MOWBITE_LIGHT_EFFECT_H

#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_light/ws2812.h"

namespace mowbite_light {

using Json = nlohmann::ordered_json;

// The bounds of an effect.
constexpr size_t kMaxColors = 4;
constexpr double kMinPeriod = 0.2;
constexpr double kMaxPeriod = 20.0;

enum class EffectKind {
  kOff,
  kSolid,
  kBlink,
  kPulse,
  kChase,
  kGradient,
  kFade,
  kComet,
  kScanner,
  kRainbow,
  kSparkle,
  kFlash,
  kLevel
};

// What a state shows: solid, blink (on for half of period s, then the second colour or dark), pulse (breathing once per
// period), chase (a run of light along the chain once per period, on the second colour or dark), gradient (the
// colours spread along the chain, moving once per period, still with period 0), fade (all from the first colour to the
// second or dark and back once per period), comet (a head with a fading tail along the chain once per period), scanner
// (a dot there and back once per period), rainbow (the colour circle along the chain, moving once per period, still
// with period 0, without colours), sparkle (single LEDs glowing up and fading, a new choice every period), flash (two
// short flashes at the start of every period) and level (the chain filled to the level, its top breathing once per
// period, still with period 0). Comet, scanner, sparkle, flash and level lie on the second colour or dark.
struct Effect {
  EffectKind kind = EffectKind::kOff;
  std::vector<Rgb> colors;
  double period = 1.0;
  // 0..1 of what the limits allow.
  double brightness = 1.0;
};

const std::vector<std::string>& EffectNames();

// A run of LEDs that chase and gradient follow from first to last, backwards when last < first.
struct Strip {
  size_t first = 0;
  size_t last = 0;
  size_t Length() const { return (first <= last ? last - first : first - last) + 1; }
};

// The strips from texts "first-last" of a chain of leds; nullopt and why for one that is malformed, beyond the chain
// or overlaps another.
std::optional<std::vector<Strip>> ParseStrips(const std::vector<std::string>& texts, size_t leds, std::string* error);

// The colours of n LEDs t s after the start, level full.
std::vector<Rgb> Render(const Effect& effect, size_t n, double t);
// The same on every strip at once, the LEDs outside them dark; no strips: the whole chain as one. level (0..1,
// clamped) is what the effect level fills.
std::vector<Rgb> Render(const Effect& effect, const std::vector<Strip>& strips, size_t n, double t,
                        double level = 1.0);

Json ToJson(const Effect& effect);
// nullopt and why for a value that is no effect: 1 to 4 colours as #rrggbb (off and rainbow also none), period 0.2 to
// 20 s (gradient, rainbow and level also 0), brightness 0 to 1.
std::optional<Effect> EffectFromJson(const Json& value, std::string* error);

// The states of a profile in the order the app shows them: robot_state with PAUSED, then charging and the battery.
const std::vector<std::string>& ProfileStates();
using Profile = std::map<std::string, Effect>;
Profile DefaultProfile();
// The signal of an emergency, the same on every mower and not part of a profile.
Effect EmergencyEffect();
// In an emergency: STOP was held long enough, letting go releases. Like the emergency, not part of a profile.
Effect StopReadyEffect();

Json ToJson(const Profile& profile);
// Missing states keep the default; nullopt and why for an unknown state or an effect that is none.
std::optional<Profile> ProfileFromJson(const Json& value, std::string* error);

// The profile state for robot_state: charging (CHARGED from a battery of 0.99 on), BATTERY_LOW while IDLE below
// low_battery, else current_state, IDLE for one the profile does not know.
std::string ShownState(const std::string& current_state, bool is_charging, double battery, double low_battery);

}  // namespace mowbite_light

#endif  // MOWBITE_LIGHT_EFFECT_H
