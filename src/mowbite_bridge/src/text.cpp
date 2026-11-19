#include "mowbite_bridge/text.h"

#include <algorithm>
#include <cctype>
#include <fstream>

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

bool IsNameChar(char c) {
  return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

// In lower case; a name that holds one of them has a secret value.
constexpr std::string_view kSecretWords[] = {"password", "passwd", "token", "secret", "api_key", "private_key"};

// Length of the secret word at text[at], 0 for none.
std::size_t SecretWordAt(std::string_view text, std::size_t at) {
  for (const std::string_view word : kSecretWords) {
    if (StartsWithNoCase(text, at, word)) return word.size();
  }
  return 0;
}

}  // namespace

bool IsSecretName(std::string_view name) {
  for (std::size_t at = 0; at < name.size(); ++at) {
    if (SecretWordAt(name, at) != 0) return true;
  }
  return false;
}

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
  // user:password@ after "://", up to the last '@' before a space or quote: a password may hold '/' and '@'.
  for (std::size_t scheme = out.find("://"); scheme != std::string::npos; scheme = out.find("://", scheme + 3)) {
    const std::size_t begin = scheme + 3;
    std::size_t end = begin;
    std::size_t at = std::string::npos;
    while (end < out.size() && !IsSpace(out[end]) && out[end] != '"' && out[end] != '\'') {
      if (out[end] == '@') at = end;
      ++end;
    }
    if (at != std::string::npos && at > begin) out.replace(begin, at - begin, "***");
  }
  // The credentials of an Authorization header: "Basic x" or "Bearer x".
  for (const std::string_view scheme : {std::string_view("basic"), std::string_view("bearer")}) {
    for (std::size_t at = 0; at < out.size(); ++at) {
      if (!StartsWithNoCase(out, at, scheme) || (at > 0 && IsNameChar(out[at - 1]))) continue;
      std::size_t k = at + scheme.size();
      if (k >= out.size() || !IsSpace(out[k])) continue;
      while (k < out.size() && IsSpace(out[k])) ++k;
      std::size_t end = k;
      while (end < out.size() && !IsSpace(out[end]) && out[end] != '"' && out[end] != '\'') ++end;
      if (end > k) out.replace(k, end - k, "***");
      at = k;
    }
  }
  // The value after a secret name and = or :, the name and a quoted value each in " or ' as in JSON.
  for (std::size_t at = 0; at < out.size(); ++at) {
    const std::size_t word = SecretWordAt(out, at);
    if (word == 0) continue;
    std::size_t k = at + word;
    while (k < out.size() && IsNameChar(out[k])) ++k;
    if (k < out.size() && (out[k] == '"' || out[k] == '\'')) ++k;
    while (k < out.size() && IsSpace(out[k])) ++k;
    if (k >= out.size() || (out[k] != '=' && out[k] != ':')) continue;
    ++k;
    while (k < out.size() && IsSpace(out[k])) ++k;
    std::size_t end = k;
    if (k < out.size() && (out[k] == '"' || out[k] == '\'')) {
      const char quote = out[k];
      end = ++k;
      while (end < out.size() && out[end] != quote) end += out[end] == '\\' ? 2 : 1;
      end = std::min(end, out.size());
    } else {
      while (end < out.size() && !IsSpace(out[end])) ++end;
    }
    if (end > k) out.replace(k, end - k, "***");
    at = k + 2;
  }
  return out;
}

bool AppendLines(const std::string& path, const std::string& lines) {
  bool torn = false;
  {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (in && in.tellg() > 0) {
      in.seekg(-1, std::ios::end);
      torn = in.get() != '\n';
    }
  }
  std::ofstream out(path, std::ios::app | std::ios::binary);
  if (!out) return false;
  if (torn) out << '\n';
  out << lines;
  out.flush();
  return static_cast<bool>(out);
}

}  // namespace mowbite_bridge
