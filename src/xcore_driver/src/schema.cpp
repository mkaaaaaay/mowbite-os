#include "xcore_driver/schema.h"

#include <cctype>
#include <map>

namespace xcore {

namespace {

std::optional<ValueType::Base> ParseBase(const std::string& name) {
  static const std::map<std::string, ValueType::Base> kBases{
      {"uint8_t", ValueType::Base::kU8},   {"int8_t", ValueType::Base::kI8},   {"uint16_t", ValueType::Base::kU16},
      {"int16_t", ValueType::Base::kI16},  {"uint32_t", ValueType::Base::kU32}, {"int32_t", ValueType::Base::kI32},
      {"uint64_t", ValueType::Base::kU64}, {"int64_t", ValueType::Base::kI64}, {"float", ValueType::Base::kF32},
      {"double", ValueType::Base::kF64},   {"char", ValueType::Base::kChar},   {"blob", ValueType::Base::kBlob},
  };
  auto it = kBases.find(name);
  if (it == kBases.end()) return std::nullopt;
  return it->second;
}

// Splits "name[count]" into name and count; count 0 without brackets.
bool SplitArray(const std::string& text, std::string& name, size_t& count) {
  auto open = text.find('[');
  if (open == std::string::npos) {
    name = text;
    count = 0;
    return !text.empty();
  }
  if (text.back() != ']' || open == 0 || open + 2 > text.size() - 1) return false;
  name = text.substr(0, open);
  std::string digits = text.substr(open + 1, text.size() - open - 2);
  if (digits.empty() || digits.size() > 5) return false;
  for (char c : digits) {
    if (!std::isdigit(static_cast<unsigned char>(c))) return false;
  }
  count = std::stoul(digits);
  return count > 0;
}

// Enum ids of the description and their base types.
std::map<std::string, std::string> Enums(const Cbor& desc) {
  std::map<std::string, std::string> enums;
  const Cbor* list = desc.Find("enums");
  if (!list || list->kind() != Cbor::Kind::kArray) return enums;
  for (const auto& e : list->items()) {
    const Cbor* id = e.Find("id");
    const Cbor* base = e.Find("base_type");
    if (id && base && id->kind() == Cbor::Kind::kText && base->kind() == Cbor::Kind::kText) {
      enums[*id->AsString()] = *base->AsString();
    }
  }
  return enums;
}

bool ParseChannels(const Cbor& desc, const char* key, const std::map<std::string, std::string>& enums,
                   std::vector<Channel>& out) {
  const Cbor* list = desc.Find(key);
  if (!list) return true;
  if (list->kind() != Cbor::Kind::kArray) return false;
  for (const auto& item : list->items()) {
    const Cbor* id = item.Find("id");
    const Cbor* type = item.Find("type");
    if (!id || !type || type->kind() != Cbor::Kind::kText) return false;
    auto id_value = id->AsInt();
    if (!id_value || *id_value < 0 || *id_value > 0xFFFF) return false;
    std::string type_text = *type->AsString();
    std::string name;
    size_t count;
    if (!SplitArray(type_text, name, count)) return false;
    auto e = enums.find(name);
    if (e != enums.end()) type_text = count ? e->second + type_text.substr(name.size()) : e->second;
    auto parsed = ParseValueType(type_text);
    if (!parsed) return false;
    Channel channel;
    channel.id = static_cast<uint16_t>(*id_value);
    channel.type = *parsed;
    const Cbor* channel_name = item.Find("name");
    if (channel_name && channel_name->kind() == Cbor::Kind::kText) channel.name = *channel_name->AsString();
    out.push_back(std::move(channel));
  }
  return true;
}

const Channel* FindChannel(const std::vector<Channel>& channels, uint16_t id) {
  for (const auto& c : channels) {
    if (c.id == id) return &c;
  }
  return nullptr;
}

}  // namespace

std::string TypeText(const ValueType& type) {
  static const char* kNames[] = {"uint8_t", "int8_t",  "uint16_t", "int16_t", "uint32_t", "int32_t",
                                 "uint64_t", "int64_t", "float",    "double",  "char",     "blob"};
  std::string text = kNames[static_cast<int>(type.base)];
  if (type.count) text += "[" + std::to_string(type.count) + "]";
  return text;
}

namespace {

void CheckChannels(const char* kind, const std::vector<Channel>& expected, const std::vector<Channel>& announced,
                   std::vector<std::string>& problems) {
  for (const auto& e : expected) {
    const Channel* a = FindChannel(announced, e.id);
    if (!a) {
      problems.push_back(std::string(kind) + " " + std::to_string(e.id) + " (" + e.name + ") missing");
    } else if (a->type != e.type) {
      problems.push_back(std::string(kind) + " " + std::to_string(e.id) + " (" + e.name + ") is " + TypeText(a->type) +
                         ", expected " + TypeText(e.type));
    }
  }
}

}  // namespace

size_t ValueType::element_size() const {
  switch (base) {
    case Base::kU8:
    case Base::kI8:
    case Base::kChar:
    case Base::kBlob:
      return 1;
    case Base::kU16:
    case Base::kI16:
      return 2;
    case Base::kU32:
    case Base::kI32:
    case Base::kF32:
      return 4;
    case Base::kU64:
    case Base::kI64:
    case Base::kF64:
      return 8;
  }
  return 1;
}

size_t ValueType::max_size() const {
  if (base == Base::kBlob) return 0;
  return element_size() * (count ? count : 1);
}

std::optional<ValueType> ParseValueType(const std::string& text) {
  std::string name;
  size_t count;
  if (!SplitArray(text, name, count)) return std::nullopt;
  auto base = ParseBase(name);
  if (!base || (*base == ValueType::Base::kBlob && count)) return std::nullopt;
  ValueType type;
  type.base = *base;
  type.count = count;
  return type;
}

bool FitsType(const ValueType& type, size_t length) {
  if (type.base == ValueType::Base::kBlob) return true;
  if (!type.count) return length == type.element_size();
  return length % type.element_size() == 0 && length <= type.max_size();
}

std::optional<ServiceDescription> ParseDescription(const Cbor& desc) {
  if (desc.kind() != Cbor::Kind::kMap) return std::nullopt;
  const Cbor* type = desc.Find("type");
  const Cbor* version = desc.Find("version");
  if (!type || type->kind() != Cbor::Kind::kText || !version) return std::nullopt;
  auto version_value = version->AsInt();
  if (!version_value || *version_value < 0 || *version_value > 0xFFFFFFFF) return std::nullopt;
  ServiceDescription out;
  out.type = *type->AsString();
  out.version = static_cast<uint32_t>(*version_value);
  auto enums = Enums(desc);
  if (!ParseChannels(desc, "inputs", enums, out.inputs) || !ParseChannels(desc, "outputs", enums, out.outputs) ||
      !ParseChannels(desc, "registers", enums, out.registers)) {
    return std::nullopt;
  }
  return out;
}

std::optional<Announcement> ParseAnnouncement(const uint8_t* data, size_t size) {
  auto root = DecodeCbor(data, size);
  if (!root || root->kind() != Cbor::Kind::kMap) return std::nullopt;
  const Cbor* sid = root->Find("sid");
  const Cbor* endpoint = root->Find("endpoint");
  const Cbor* desc = root->Find("desc");
  if (!sid || !endpoint || !desc) return std::nullopt;
  const Cbor* ip = endpoint->Find("ip");
  const Cbor* port = endpoint->Find("port");
  if (!ip || ip->kind() != Cbor::Kind::kText || !port) return std::nullopt;
  auto sid_value = sid->AsInt();
  auto port_value = port->AsInt();
  if (!sid_value || *sid_value < 0 || *sid_value > 0xFFFF) return std::nullopt;
  if (!port_value || *port_value <= 0 || *port_value > 0xFFFF) return std::nullopt;
  auto description = ParseDescription(*desc);
  if (!description) return std::nullopt;
  Announcement out;
  out.service_id = static_cast<uint16_t>(*sid_value);
  out.ip = *ip->AsString();
  out.port = static_cast<uint16_t>(*port_value);
  out.description = std::move(*description);
  return out;
}

std::vector<std::string> CheckDescription(const ExpectedService& expected, const ServiceDescription& announced) {
  std::vector<std::string> problems;
  if (announced.type != expected.type || announced.version != expected.version) {
    problems.push_back("service is " + announced.type + " v" + std::to_string(announced.version) + ", expected " +
                       expected.type + " v" + std::to_string(expected.version));
    return problems;
  }
  CheckChannels("input", expected.inputs, announced.inputs, problems);
  CheckChannels("output", expected.outputs, announced.outputs, problems);
  CheckChannels("register", expected.registers, announced.registers, problems);
  return problems;
}

Cbor DescriptionToCbor(const ServiceDescription& description) {
  auto channels = [](const std::vector<Channel>& list) {
    std::vector<Cbor> items;
    items.reserve(list.size());
    for (const auto& c : list) {
      items.push_back(Cbor::Map({{Cbor::Text("id"), Cbor::Unsigned(c.id)},
                                 {Cbor::Text("name"), Cbor::Text(c.name)},
                                 {Cbor::Text("type"), Cbor::Text(TypeText(c.type))}}));
    }
    return Cbor::Array(std::move(items));
  };
  return Cbor::Map({{Cbor::Text("type"), Cbor::Text(description.type)},
                    {Cbor::Text("version"), Cbor::Unsigned(description.version)},
                    {Cbor::Text("inputs"), channels(description.inputs)},
                    {Cbor::Text("outputs"), channels(description.outputs)},
                    {Cbor::Text("registers"), channels(description.registers)}});
}

std::vector<uint8_t> EncodeAnnouncement(const Announcement& announcement) {
  return EncodeCbor(Cbor::Map({{Cbor::Text("sid"), Cbor::Unsigned(announcement.service_id)},
                               {Cbor::Text("endpoint"), Cbor::Map({{Cbor::Text("ip"), Cbor::Text(announcement.ip)},
                                                                   {Cbor::Text("port"), Cbor::Unsigned(announcement.port)}})},
                               {Cbor::Text("desc"), DescriptionToCbor(announcement.description)}}));
}

}  // namespace xcore
