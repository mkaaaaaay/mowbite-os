// The step platform and the choice of the mower: setup.platforms and setup.platform.set.
#include <algorithm>
#include <utility>

#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

void Program::SetPlatforms(json platforms, std::string chosen, bool fixed) {
  platforms_ = std::move(platforms);
  chosen_ = std::move(chosen);
  fixed_ = fixed;
  MarkPlatform();
}

json Program::Platforms(const std::map<std::string, std::pair<std::string, int>>& images) const {
  json platforms = platforms_;
  for (json& platform : platforms) platform["image"] = ImageOf(images, platform.value("image", ""), false);
  return {{"platforms", std::move(platforms)}, {"current", chosen_}, {"fixed", fixed_}};
}

void Program::MarkPlatform() {
  for (const Step& step : list_.steps) {
    if (step.check == "platform") records_.at(step.id).state = platform_.empty() ? "open" : "ok";
  }
}

std::optional<Refusal> Program::ChoosePlatform(const json& params, double wall, json* result) {
  if (auto refusal = Common(params, nullptr)) return refusal;
  for (const auto& item : params.items()) {
    if (item.key() != "platform" && item.key() != "version") {
      return BadParam(item.key(), item.key() + " is no param of setup.platform.set");
    }
  }
  if (!params.contains("platform") || !params["platform"].is_string()) {
    return BadParam("platform", "platform is missing or no text");
  }
  const std::string name = params["platform"].get<std::string>();
  if (fixed_) return Refuse("PLATFORM_FIXED", "The installation runs this mower as " + chosen_);
  const bool known = std::any_of(platforms_.begin(), platforms_.end(),
                                 [&name](const json& entry) { return entry.value("name", "") == name; });
  if (!known) return Refuse("UNKNOWN_PLATFORM", "The image has no platform " + name, {{"platform", name}});
  if (name == chosen_) {
    *result = {{"restart", false}};
    return std::nullopt;
  }
  if (run_) return Refuse("BUSY", StepOf(*run_).id + " is running", {{"step", StepOf(*run_).id}});
  if (!platform_.empty()) {
    if (hardware_.hold) return Refuse("BUSY", "An update holds the mower", {{"state", "HOLD"}});
    if (world_.state != "IDLE") return Refuse("BUSY", "The mower is not in IDLE", {{"state", world_.state}});
    // Another mower: nothing measured on this one counts, the lock stays closed until its summary passes.
    restart_wanted_ = false;
    own_.reset();
    release_.reset();
    ResetSettings(wall);
    for (auto& [id, record] : records_) record = Record();
    reached_.clear();
    CloseWithReset(wall);
    MarkPlatform();
    MarkHardware();
  }
  events_.push_back({"SETUP", {{"what", "platform"}, {"platform", name}, {"from", chosen_}}});
  choice_ = name;
  Changed();
  *result = {{"restart", true}};
  return std::nullopt;
}

std::optional<std::string> Program::TakePlatform() {
  auto choice = std::move(choice_);
  choice_.reset();
  return choice;
}

}  // namespace mowbite_setup
