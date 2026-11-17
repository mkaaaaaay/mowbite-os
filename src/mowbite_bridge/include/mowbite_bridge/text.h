#ifndef MOWBITE_BRIDGE_TEXT_H
#define MOWBITE_BRIDGE_TEXT_H

#include <string>
#include <string_view>

namespace mowbite_bridge {

// The text as valid UTF-8: every broken or overlong sequence, surrogate and lone byte becomes one U+FFFD.
std::string CleanUtf8(std::string_view text);

// Deepest nesting of [ and { outside of strings, without parsing; tells a payload too deep to parse safely.
std::size_t NestingDepth(std::string_view text);

// A parameter or field name whose value is secret: it holds password, passwd, token, secret, api_key or private_key.
bool IsSecretName(std::string_view name);

// Hides user:password@ of a URL and the value after a secret name in a log line, "quoted" up to its closing quote.
std::string HideSecrets(std::string_view text);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_TEXT_H
