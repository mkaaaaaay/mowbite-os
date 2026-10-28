// SHA-256 of a text, for the version check of map/replace.
#pragma once

#include <string>

namespace mowbite_map {

// 64 lower-case hex characters, empty if the digest cannot be made.
std::string Sha256Hex(const std::string& data);

// Whether the text is a SHA-256 as Sha256Hex writes it.
bool IsSha256Hex(const std::string& text);

}  // namespace mowbite_map
