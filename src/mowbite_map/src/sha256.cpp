#include "mowbite_map/sha256.h"

#include <openssl/evp.h>

namespace mowbite_map {

std::string Sha256Hex(const std::string& data) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_Digest(data.data(), data.size(), digest, &length, EVP_sha256(), nullptr) != 1 || length != 32) return "";
  static const char digits[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(64);
  for (unsigned int i = 0; i < length; ++i) {
    hex.push_back(digits[digest[i] >> 4]);
    hex.push_back(digits[digest[i] & 0x0f]);
  }
  return hex;
}

bool IsSha256Hex(const std::string& text) {
  if (text.size() != 64) return false;
  for (const char c : text) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

}  // namespace mowbite_map
