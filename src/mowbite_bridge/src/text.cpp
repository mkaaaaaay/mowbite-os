#include "mowbite_bridge/text.h"

#include <algorithm>
#include <cctype>

namespace mowbite_bridge {

namespace {

constexpr char kReplacement[] = "\xef\xbf\xbd";

bool IsContinuation(unsigned char c) {
  return (c & 0xC0) == 0x80;
}

// Length of the valid sequence at text[i], or 0 together with how many bytes one U+FFFD stands for.
std::size_t ValidSequence(std::string_view text, std::size_t i, std::size_t* broken) {
  const auto at = [&](std::size_t k) { return static_cast<unsigned char>(text[k]); };
  const unsigned char lead = at(i);
  *broken = 1;
  if (lead < 0x80) return 1;
  std::size_t length = 0;
  unsigned char low = 0x80, high = 0xBF;  // range of the second byte
  if (lead >= 0xC2 && lead <= 0xDF) {
    length = 2;
  } else if (lead >= 0xE0 && lead <= 0xEF) {
    length = 3;
    if (lead == 0xE0) low = 0xA0;
    if (lead == 0xED) high = 0x9F;
  } else if (lead >= 0xF0 && lead <= 0xF4) {
    length = 4;
    if (lead == 0xF0) low = 0x90;
    if (lead == 0xF4) high = 0x8F;
  } else {
    return 0;
  }
  std::size_t k = 1;
  for (; k < length && i + k < text.size(); ++k) {
    const unsigned char c = at(i + k);
    const bool ok = k == 1 ? (c >= low && c <= high) : IsContinuation(c);
    if (!ok) break;
  }
  if (k == length) return length;
  // A valid start that breaks off counts as one replacement.
  *broken = k;
  return 0;
}

bool StartsWithNoCase(std::string_view text, std::size_t at, std::string_view word) {
  if (at + word.size() > text.size()) return false;
  for (std::size_t k = 0; k < word.size(); ++k) {
    if (std::tolower(static_cast<unsigned char>(text[at + k])) != word[k]) return false;
  }
  return true;
}

bool IsSpace(char c) {
  return std::isspace(static_cast<unsigned char>(c)) != 0;
}

}  // namespace

std::string CleanUtf8(std::string_view text) {
  std::string out;
  out.reserve(text.size());
  std::size_t i = 0;
  while (i < text.size()) {
    std::size_t broken = 1;
    const std::size_t n = ValidSequence(text, i, &broken);
    if (n > 0) {
      out.append(text.substr(i, n));
      i += n;
    } else {
      out += kReplacement;
      i += broken;
    }
  }
  return out;
}

std::size_t NestingDepth(std::string_view text) {
  std::size_t depth = 0, deepest = 0;
  bool in_string = false, escaped = false;
  for (const char c : text) {
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
    } else if (c == '"') {
      in_string = true;
    } else if (c == '[' || c == '{') {
      deepest = std::max(deepest, ++depth);
    } else if ((c == ']' || c == '}') && depth > 0) {
      --depth;
    }
  }
  return deepest;
}

std::string HideSecrets(std::string_view text) {
  std::string out(text);
  // user:password@ after "://", up to the first '/', '@' or space.
  for (std::size_t scheme = out.find("://"); scheme != std::string::npos; scheme = out.find("://", scheme + 3)) {
    const std::size_t begin = scheme + 3;
    std::size_t end = begin;
    while (end < out.size() && out[end] != '/' && out[end] != '@' && !IsSpace(out[end])) ++end;
    if (end < out.size() && out[end] == '@' && end > begin) out.replace(begin, end - begin, "***");
  }
  // The value after a key word and = or :.
  for (const std::string_view word : {"password", "passwd", "token", "secret"}) {
    for (std::size_t at = 0; at < out.size(); ++at) {
      if (!StartsWithNoCase(out, at, word)) continue;
      std::size_t k = at + word.size();
      while (k < out.size() && IsSpace(out[k])) ++k;
      if (k >= out.size() || (out[k] != '=' && out[k] != ':')) continue;
      ++k;
      while (k < out.size() && IsSpace(out[k])) ++k;
      std::size_t end = k;
      while (end < out.size() && !IsSpace(out[end])) ++end;
      if (end > k) out.replace(k, end - k, "***");
      at = k + 2;
    }
  }
  return out;
}

}  // namespace mowbite_bridge
