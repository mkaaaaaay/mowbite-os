#include "mowbite_setup/steps.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <regex>
#include <set>
#include <stdexcept>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_setup/layer.h"

namespace mowbite_setup {

namespace {

using mowbite_api::FindSetupName;
using mowbite_api::SetupName;
using mowbite_api::Texts;

const char* const kLanguages[] = {"de", "en"};

// A problem of the list, with the step it is in.
struct Wrong : std::runtime_error {
  using std::runtime_error::runtime_error;
};

bool IsId(const std::string& text) {
  static const std::regex pattern(mowbite_api::kSetupIdPattern);
  return std::regex_match(text, pattern);
}

bool IsImageName(const std::string& text) {
  static const std::regex pattern(mowbite_api::kSetupImagePattern);
  return std::regex_match(text, pattern);
}

template <typename List>
bool Has(const List& list, const std::string& value) {
  return std::find(std::begin(list), std::end(list), value) != std::end(list);
}

void OnlyKeys(const json& object, const std::set<std::string>& keys, const std::string& where) {
  for (const auto& item : object.items()) {
    if (!keys.count(item.key())) throw Wrong(where + ": " + item.key() + " is no field of it");
  }
}

std::string TextOf(const json& object, const char* key, const std::string& where, bool required = false) {
  if (!object.contains(key)) {
    if (required) throw Wrong(where + ": " + key + " is missing");
    return "";
  }
  if (!object[key].is_string()) throw Wrong(where + ": " + key + " is no text");
  return object[key].get<std::string>();
}

std::vector<std::string> TextsOf(const json& object, const char* key, const std::string& where) {
  std::vector<std::string> out;
  if (!object.contains(key)) return out;
  if (!object[key].is_array()) throw Wrong(where + ": " + key + " is no list");
  for (const auto& item : object[key]) {
    if (!item.is_string()) throw Wrong(where + ": " + key + " lists texts");
    out.push_back(item.get<std::string>());
  }
  return out;
}

// base with the name, help and fix the list gives on top, by language; other languages or fields are a slip.
json Overlaid(json base, const json& over, const std::string& where) {
  if (!base.is_object()) base = json::object();
  if (over.is_null()) return base;
  if (!over.is_object()) throw Wrong(where + ": texts is no object by language");
  for (const auto& language : over.items()) {
    if (!Has(kLanguages, language.key())) throw Wrong(where + ": texts in " + language.key() + ", only de and en");
    if (!language.value().is_object()) throw Wrong(where + ": texts." + language.key() + " is no object");
    json& texts = base[language.key()];
    if (!texts.is_object()) texts = {{"name", ""}, {"help", ""}, {"fix", ""}};
    for (const auto& field : language.value().items()) {
      if (field.key() != "name" && field.key() != "help" && field.key() != "fix") {
        throw Wrong(where + ": texts have name, help and fix, not " + field.key());
      }
      if (!field.value().is_string()) throw Wrong(where + ": texts." + field.key() + " is no text");
      texts[field.key()] = field.value();
    }
  }
  return base;
}

void NeedNames(const json& texts, const std::string& where) {
  for (const char* language : kLanguages) {
    if (!texts.contains(language) || texts[language].value("name", "").empty()) {
      throw Wrong(where + ": no name in " + language);
    }
  }
}

json TextsOrNull(const json& object) {
  return object.contains("texts") ? object["texts"] : json();
}

// A point or reason of the catalog by its id, or the list's own as {id, texts}.
Named NamedOf(const json& item, const std::vector<SetupName>& catalog, bool own, const std::string& where) {
  if (item.is_string()) {
    const SetupName* name = FindSetupName(catalog, item.get<std::string>());
    if (!name) throw Wrong(where + ": " + item.get<std::string>() + " is not in the catalog");
    return {name->id, Texts(name->texts)};
  }
  if (!own || !item.is_object()) throw Wrong(where + ": an entry is no name of the catalog");
  OnlyKeys(item, {"id", "texts"}, where);
  Named named{TextOf(item, "id", where, true), Overlaid(json::object(), TextsOrNull(item), where)};
  if (!IsId(named.id)) throw Wrong(where + ": " + named.id + " is no id");
  if (const SetupName* name = FindSetupName(catalog, named.id)) {
    named.texts = Overlaid(Texts(name->texts), TextsOrNull(item), where);
  }
  NeedNames(named.texts, where + "." + named.id);
  return named;
}

void ReadParams(const mowbite_api::SetupCheck& check, const json& params, const std::string& where, Step* step) {
  if (!params.is_object()) throw Wrong(where + ": params is no object");
  for (const auto& item : params.items()) {
    const bool known = std::any_of(check.params.begin(), check.params.end(),
                                   [&item](const auto& param) { return param.name == item.key(); });
    if (!known) throw Wrong(where + ": " + item.key() + " is no param of " + check.name);
  }
  for (const auto& param : check.params) {
    const std::string name = where + "." + param.name;
    if (!params.contains(param.name)) {
      if (param.required) throw Wrong(name + " is missing");
      if (param.type == mowbite_api::Type::kNumber) step->numbers[param.name] = param.def;
      if (param.type == mowbite_api::Type::kString) step->words[param.name] = param.word;
      if (param.type == mowbite_api::Type::kArray) step->lists[param.name] = {};
      continue;
    }
    const json& value = params[param.name];
    if (param.type == mowbite_api::Type::kNumber) {
      if (!value.is_number()) throw Wrong(name + " is no number");
      const double number = value.get<double>();
      if (!(number >= param.min && number <= param.max)) {
        throw Wrong(name + " is outside of " + std::to_string(param.min) + " to " + std::to_string(param.max));
      }
      step->numbers[param.name] = number;
    } else if (param.type == mowbite_api::Type::kString) {
      if (!value.is_string()) throw Wrong(name + " is no text");
      const std::string text = value.get<std::string>();
      if (!param.values.empty() && !Has(param.values, text)) throw Wrong(name + ": " + text + " is none of its values");
      step->words[param.name] = text;
    } else {
      const std::vector<std::string> list = TextsOf(params, param.name.c_str(), where);
      for (const auto& text : list) {
        if (!param.values.empty() && !Has(param.values, text)) {
          throw Wrong(name + ": " + text + " is none of its values");
        }
      }
      step->lists[param.name] = list;
    }
  }
}

void ReadParts(const json& entry, const std::string& where, Step* step) {
  const bool given = entry.contains("parts");
  if (given && !entry["parts"].is_array()) throw Wrong(where + ": parts is no list");
  if (step->check == "input") {
    const std::string& role = step->Word("role");
    const SetupName* short_mode = FindSetupName(mowbite_api::SetupModes(), "short");
    json parts = given ? entry["parts"] : json::array({json{{"id", role}}});
    for (const auto& item : parts) {
      if (!item.is_object()) throw Wrong(where + ": a part is no object");
      OnlyKeys(item, {"id", "mode", "role", "texts"}, where + ".parts");
      Part part;
      part.id = TextOf(item, "id", where + ".parts", true);
      part.mode = item.contains("mode") ? TextOf(item, "mode", where) : "press";
      part.role = item.contains("role") ? TextOf(item, "role", where) : role;
      if (!FindSetupName(mowbite_api::SetupModes(), part.mode)) throw Wrong(where + ": " + part.mode + " is no mode");
      const SetupName* part_role = FindSetupName(mowbite_api::SetupRoles(), part.role);
      if (!part_role) throw Wrong(where + ": " + part.role + " is no role");
      json base = part.mode == "short" ? Texts(short_mode->texts) : Texts(part_role->texts);
      part.texts = Overlaid(std::move(base), TextsOrNull(item), where + ".parts." + part.id);
      step->parts.push_back(std::move(part));
    }
  } else if (step->check == "inputs_assign") {
    const SetupName* assign = FindSetupName(mowbite_api::SetupModes(), "assign");
    for (const auto& item : given ? entry["parts"] : json::array()) {
      if (!item.is_object()) throw Wrong(where + ": a part is no object");
      OnlyKeys(item, {"id", "role", "delay_ms", "texts"}, where + ".parts");
      Part part;
      part.id = TextOf(item, "id", where + ".parts", true);
      part.mode = assign->id;
      part.role = TextOf(item, "role", where + ".parts", true);
      const auto* role = FindSetupName(mowbite_api::SetupRoles(), part.role);
      // A line is one input of hw/status: the bumper as a whole is none.
      if (!role || part.role == "bumper") throw Wrong(where + ": " + part.role + " is no role of a line");
      if (item.contains("delay_ms")) {
        const json& delay = item["delay_ms"];
        if (!delay.is_number_integer() || delay.get<int64_t>() < 0 ||
            delay.get<int64_t>() > mowbite_api::kInputsMaxDelayMs) {
          throw Wrong(where + ".parts." + part.id + ": delay_ms is no whole number the board takes");
        }
        part.delay_ms = static_cast<int>(delay.get<int64_t>());
      }
      // The name of the role, help and fix of the mode, then what the list brings.
      json base = Texts(assign->texts);
      const json named = Texts(role->texts);
      for (const char* language : kLanguages) base[language]["name"] = named[language]["name"];
      part.texts = Overlaid(std::move(base), TextsOrNull(item), where + ".parts." + part.id);
      step->parts.push_back(std::move(part));
    }
  } else if (step->check == "stop_moving") {
    // One part by input to trigger while the wheels turn, each of a role; the stop button alone without parts.
    json parts = given ? entry["parts"] : json::array({json{{"id", "stop"}, {"role", "stop"}}});
    for (const auto& item : parts) {
      if (!item.is_object()) throw Wrong(where + ": a part is no object");
      OnlyKeys(item, {"id", "role", "texts"}, where + ".parts");
      Part part;
      part.id = TextOf(item, "id", where + ".parts", true);
      part.mode = "press";
      part.role = TextOf(item, "role", where + ".parts", true);
      const SetupName* role = FindSetupName(mowbite_api::SetupRoles(), part.role);
      if (!role) throw Wrong(where + ": " + part.role + " is no role");
      part.texts = Overlaid(Texts(role->texts), TextsOrNull(item), where + ".parts." + part.id);
      step->parts.push_back(std::move(part));
    }
  } else if (step->check == "wheels") {
    if (given) throw Wrong(where + ": the parts of wheels are fixed");
    for (const auto& name : mowbite_api::SetupWheelParts()) {
      step->parts.push_back({name.id, "", "", "", Texts(name.texts)});
    }
  } else if (step->check == "ground") {
    if (given) throw Wrong(where + ": the parts of ground are fixed");
    // stop and bumper wait for their input as a part of stop_moving does.
    for (const auto& name : mowbite_api::SetupGroundParts()) {
      const bool press = name.id == "stop" || name.id == "bumper";
      step->parts.push_back({name.id, press ? "press" : "", press ? name.id : "", "", Texts(name.texts)});
    }
  } else if (step->check == "imu") {
    const std::vector<std::string> moves =
        given ? TextsOf(entry, "parts", where) : std::vector<std::string>{"front_up", "left_up", "turn_left"};
    for (const auto& move : moves) {
      const SetupName* name = FindSetupName(mowbite_api::SetupMoves(), move);
      if (!name) throw Wrong(where + ": " + move + " is no move");
      step->parts.push_back({move, "", "", move, Texts(name->texts)});
    }
  } else if (given) {
    throw Wrong(where + ": parts go with input, inputs_assign, imu, stop_moving, wheels and ground only");
  }
  if (step->parts.size() > static_cast<size_t>(mowbite_api::kSetupMaxParts)) throw Wrong(where + ": too many parts");
  if ((step->check == "input" || step->check == "inputs_assign" || step->check == "imu" ||
       step->check == "stop_moving") &&
      step->parts.empty()) {
    throw Wrong(where + ": no part");
  }
  std::set<std::string> ids;
  for (const Part& part : step->parts) {
    if (!IsId(part.id)) throw Wrong(where + ": " + part.id + " is no id of a part");
    if (!ids.insert(part.id).second) throw Wrong(where + ": the part " + part.id + " comes twice");
    NeedNames(part.texts, where + ".parts." + part.id);
  }
}

std::string GroupOfRole(const std::string& role) {
  const auto* known = FindSetupName(mowbite_api::SetupRoles(), role);
  return known ? known->group : "";
}

// The texts a step of this check has before the list adds its own.
json DefaultTexts(const mowbite_api::SetupCheck& check, const Step& step) {
  if (check.name == "input") return Texts(FindSetupName(mowbite_api::SetupRoles(), step.Word("role"))->texts);
  if (check.name == "done_when") {
    return Texts(FindSetupName(mowbite_api::SetupConditions(), step.Word("condition"))->texts);
  }
  return Texts(check.texts);
}

Step ReadStep(const json& entry, const std::vector<Step>& before, size_t* stage_index) {
  if (!entry.is_object()) throw Wrong("a step is no object");
  Step step;
  step.id = TextOf(entry, "id", "a step", true);
  if (!IsId(step.id)) throw Wrong(step.id + " is no id of a step");
  const std::string where = step.id;
  OnlyKeys(entry,
           {"id", "stage", "check", "params", "parts", "confirm", "needs", "skippable", "skip_reasons", "image",
            "accent", "symbol", "mark", "open", "texts", "note", "value_texts"},
           where);
  for (const Step& other : before) {
    if (other.id == step.id) throw Wrong(where + " comes twice");
  }
  step.stage = TextOf(entry, "stage", where, true);
  const auto& stages = mowbite_api::SetupStages();
  const auto stage = std::find_if(stages.begin(), stages.end(), [&](const auto& s) { return s.id == step.stage; });
  if (stage == stages.end()) throw Wrong(where + ": " + step.stage + " is no stage");
  const size_t index = static_cast<size_t>(stage - stages.begin());
  if (index < *stage_index) throw Wrong(where + ": its stage comes before the one of the step ahead");
  *stage_index = index;

  step.check = TextOf(entry, "check", where, true);
  const mowbite_api::SetupCheck* check = mowbite_api::FindSetupCheck(step.check);
  if (!check) throw Wrong(where + ": " + step.check + " is no check");
  step.kind = check->kind;
  step.start = check->start;
  step.automatic = check->automatic;
  ReadParams(*check, entry.contains("params") ? entry["params"] : json::object(), where, &step);

  if (step.check == "input" && step.Word("emergency").empty()) {
    step.words["emergency"] = FindSetupName(mowbite_api::SetupRoles(), step.Word("role"))->emergency;
  }
  if (step.check == "number" && !(step.Number("min") < step.Number("max"))) {
    throw Wrong(where + ": min is not below max");
  }
  if (step.check == "choice" || step.check == "number") {
    if (!IsId(step.Word("name"))) throw Wrong(where + ": " + step.Word("name") + " is no name of an input");
  }
  if (step.check == "choice") {
    const auto& values = step.lists.at("values");
    if (values.empty() || values.size() > static_cast<size_t>(mowbite_api::kSetupMaxInputValues)) {
      throw Wrong(where + ": a choice has 1 to " + std::to_string(mowbite_api::kSetupMaxInputValues) + " values");
    }
    const json none = json::object();
    const json& texts = entry.contains("value_texts") ? entry["value_texts"] : none;
    if (!texts.is_object()) throw Wrong(where + ": value_texts is no object");
    for (const auto& value : values) {
      if (!IsId(value)) throw Wrong(where + ": " + value + " is no id of a value");
      Named named{value, Overlaid(json::object(), texts.contains(value) ? texts[value] : json(), where)};
      NeedNames(named.texts, where + ".value_texts." + value);
      step.choices.push_back(std::move(named));
    }
  } else if (entry.contains("value_texts")) {
    throw Wrong(where + ": value_texts go with choice only");
  }
  if (step.check == "geometry") {
    auto& keys = step.lists["keys"];
    const auto& all = mowbite_api::GeometryKeys();
    if (keys.empty()) {
      for (const auto& key : all) {
        if (key.settable) keys.push_back(key.name);
      }
    }
    for (const auto& name : keys) {
      const auto key = std::find_if(all.begin(), all.end(), [&](const auto& entry) { return entry.name == name; });
      if (key == all.end() || !key->settable) throw Wrong(where + ": " + name + " is no key the user sets");
    }
  }
  ReadParts(entry, where, &step);

  for (const auto& need : TextsOf(entry, "needs", where)) {
    if (std::none_of(before.begin(), before.end(), [&](const Step& other) { return other.id == need; })) {
      throw Wrong(where + " needs " + need + ", which is no step ahead of it");
    }
    step.needs.push_back(need);
  }
  if (entry.contains("skippable")) {
    if (!entry["skippable"].is_boolean()) throw Wrong(where + ": skippable is no boolean");
    step.skippable = entry["skippable"].get<bool>();
  }
  if (step.skippable) {
    json reasons = json::array();
    if (entry.contains("skip_reasons")) {
      reasons = entry["skip_reasons"];
    } else {
      for (const auto& reason : mowbite_api::SetupSkipReasons()) reasons.push_back(reason.id);
    }
    if (!reasons.is_array() || reasons.empty()) throw Wrong(where + ": skip_reasons is no list of reasons");
    for (const auto& reason : reasons) {
      step.skip_reasons.push_back(NamedOf(reason, mowbite_api::SetupSkipReasons(), false, where + ".skip_reasons"));
    }
  } else if (entry.contains("skip_reasons")) {
    throw Wrong(where + ": skip_reasons without skippable");
  }
  if (entry.contains("confirm")) {
    if (!step.start) throw Wrong(where + ": confirm goes with a step setup.start begins");
    if (!entry["confirm"].is_array()) throw Wrong(where + ": confirm is no list");
    for (const auto& point : entry["confirm"]) {
      step.confirm.push_back(NamedOf(point, mowbite_api::SetupPoints(), true, where + ".confirm"));
      const auto same = [&](const Named& other) { return other.id == step.confirm.back().id; };
      if (std::count_if(step.confirm.begin(), step.confirm.end(), same) > 1) throw Wrong(where + ": a point twice");
    }
  }
  if (step.kind == "drive") {
    // The wheels turn only with the mower jacked up and a hand at a STOP button, on the ground with the way clear and
    // the user next to it, and with the inputs checked standing.
    const bool ground = step.check == "ground";
    for (const char* point : ground ? std::vector<const char*>{"area_clear"}
                                    : std::vector<const char*>{"raised", "hand_at_stop"}) {
      const auto has = [point](const Named& named) { return named.id == point; };
      if (std::none_of(step.confirm.begin(), step.confirm.end(), has)) {
        throw Wrong(where + ": a step that moves the wheels confirms " + point);
      }
    }
    for (const char* group : {"stop", "lift", "bumper"}) {
      const auto checks = [group](const Step& other) {
        return other.check == "input" && GroupOfRole(other.Word("role")) == group;
      };
      if (std::none_of(before.begin(), before.end(), checks)) {
        throw Wrong(where + ": a step that moves the wheels comes after a step input of " + group);
      }
    }
    if (step.lists.count("emergency") && step.lists.at("emergency").empty()) {
      throw Wrong(where + ": emergency names no emergency");
    }
  }
  step.open = TextOf(entry, "open", where);
  if (step.open.empty()) step.open = check->open;
  if (step.open.empty() && step.check == "done_when") {
    step.open = FindSetupName(mowbite_api::SetupConditions(), step.Word("condition"))->open;
  }
  if (!step.open.empty() && !Has(mowbite_api::SetupOpens(), step.open)) throw Wrong(where + ": open is no screen");
  step.image = TextOf(entry, "image", where);
  step.accent = TextOf(entry, "accent", where);
  if (!step.image.empty() && !IsImageName(step.image)) throw Wrong(where + ": " + step.image + " is no image name");
  if (!step.accent.empty() && (step.image.empty() || !IsImageName(step.accent))) {
    throw Wrong(where + ": accent needs an image and an image name");
  }
  step.symbol = TextOf(entry, "symbol", where);
  if (!step.symbol.empty() && !Has(mowbite_api::SetupSymbols(), step.symbol)) {
    throw Wrong(where + ": " + step.symbol + " is no symbol");
  }
  step.mark = TextsOf(entry, "mark", where);
  for (const auto& mark : step.mark) {
    if (!IsId(mark)) throw Wrong(where + ": " + mark + " is no id to mark");
  }
  if (step.mark.size() > 16) throw Wrong(where + ": too many marks");

  step.texts = Overlaid(DefaultTexts(*check, step), TextsOrNull(entry), where);
  if (entry.contains("note")) {
    const json& note = entry["note"];
    if (!note.is_object()) throw Wrong(where + ": note is no object by language");
    OnlyKeys(note, {"de", "en"}, where + ".note");
    for (const char* language : kLanguages) {
      const std::string text = TextOf(note, language, where + ".note", true);
      json& texts = step.texts[language];
      if (!texts.is_object()) texts = {{"name", ""}, {"help", ""}, {"fix", ""}};
      const std::string help = texts.value("help", "");
      texts["help"] = help.empty() ? text : help + " " + text;
    }
  }
  NeedNames(step.texts, where);
  return step;
}

std::string Lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}

}  // namespace

const Step* StepList::Find(const std::string& id) const {
  for (const Step& step : steps) {
    if (step.id == id) return &step;
  }
  return nullptr;
}

std::vector<std::string> StepList::Images() const {
  std::vector<std::string> names;
  for (const Step& step : steps) {
    for (const std::string* name : {&step.image, &step.accent}) {
      if (!name->empty() && !Has(names, *name)) names.push_back(*name);
    }
  }
  return names;
}

std::optional<StepList> ParseSteps(const json& list, std::string* why) {
  try {
    if (!list.is_object()) throw Wrong("the step list is no object");
    OnlyKeys(list, {"catalog", "restart", "steps"}, "the step list");
    if (!list.contains("catalog") || !list["catalog"].is_number_integer() || list["catalog"].get<int64_t>() < 1 ||
        list["catalog"].get<int64_t>() > 1000000) {
      throw Wrong("catalog is no whole number from 1");
    }
    if (!list.contains("steps") || !list["steps"].is_array() || list["steps"].empty()) {
      throw Wrong("steps is no list of steps");
    }
    if (list["steps"].size() > static_cast<size_t>(mowbite_api::kSetupMaxSteps)) throw Wrong("too many steps");
    StepList out;
    out.catalog = static_cast<int>(list["catalog"].get<int64_t>());
    out.restart = TextOf(list, "restart", "the step list");
    if (!out.restart.empty() && !FindSetupName(mowbite_api::SetupRestarts(), out.restart)) {
      throw Wrong("restart is no way to restart");
    }
    size_t stage_index = 0;
    for (const auto& entry : list["steps"]) out.steps.push_back(ReadStep(entry, out.steps, &stage_index));
    for (const Step& step : out.steps) {
      if (step.check != "inputs_assign") continue;
      if (out.restart.empty()) throw Wrong(step.id + " needs restart at the top of the list");
      if (!Layer::IsNode(step.Word("node"))) throw Wrong(step.id + ": node is no name of a node");
    }
    return out;
  } catch (const Wrong& wrong) {
    if (why) *why = wrong.what();
    return std::nullopt;
  }
}

std::string CheckImage(const std::string& svg) {
  if (svg.size() > static_cast<size_t>(mowbite_api::kSetupImageBytes)) {
    return "larger than " + std::to_string(mowbite_api::kSetupImageBytes) + " bytes";
  }
  const std::string text = Lower(svg);
  if (text.find("<svg") == std::string::npos) return "no SVG";
  for (const char* banned : {"<script", "<image", "<text", "<foreignobject", "<style", "style=", "href", "url(",
                             "<!doctype", "<!entity", "<use"}) {
    if (text.find(banned) != std::string::npos) return std::string("it has ") + banned;
  }
  static const std::regex paint(R"((fill|stroke)\s*=\s*["']([^"']*)["'])");
  std::set<std::string> colours;
  for (std::sregex_iterator it(text.begin(), text.end(), paint), end; it != end; ++it) {
    const std::string colour = (*it)[2];
    if (colour != "none" && colour != "currentcolor") colours.insert(colour);
  }
  if (colours.size() > 1) return "more than one colour";
  return "";
}

std::string LoadImage(const std::string& folder, const std::string& name, std::string* bytes) {
  std::ifstream in(folder + "/" + name, std::ios::binary);
  if (!in) return "it is missing";
  // One byte more than an image may have: CheckImage tells one that is too large without all of it read.
  bytes->resize(static_cast<size_t>(mowbite_api::kSetupImageBytes) + 1);
  in.read(bytes->data(), static_cast<std::streamsize>(bytes->size()));
  bytes->resize(static_cast<size_t>(in.gcount()));
  if (in.bad()) return "it cannot be read";
  return CheckImage(*bytes);
}

unsigned EmergencyBit(const std::string& name) {
  for (const auto& entry : mowbite_api::EmergencyBits()) {
    if (entry.name == name) return 1u << entry.bit;
  }
  return 0;
}

}  // namespace mowbite_setup
