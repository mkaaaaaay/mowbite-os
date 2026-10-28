#include <gtest/gtest.h>

#include <string>

#include "mowbite_map/sha256.h"

using mowbite_map::IsSha256Hex;
using mowbite_map::Sha256Hex;

// The vectors are the ones of FIPS 180-4.
TEST(Sha256, MatchesTheKnownVectors) {
  EXPECT_EQ(Sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(Sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(Sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(Sha256Hex(std::string(1000000, 'a')), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, HashesBytesIncludingZeros) {
  const std::string with_zero("a\0b", 3);
  EXPECT_NE(Sha256Hex(with_zero), Sha256Hex("a"));
  EXPECT_EQ(Sha256Hex(with_zero).size(), 64u);
}

TEST(Sha256, RecognisesTheFormatItWrites) {
  EXPECT_TRUE(IsSha256Hex(Sha256Hex("abc")));
  EXPECT_FALSE(IsSha256Hex(""));
  EXPECT_FALSE(IsSha256Hex(std::string(63, 'a')));
  EXPECT_FALSE(IsSha256Hex(std::string(65, 'a')));
  EXPECT_FALSE(IsSha256Hex(std::string(64, 'A')));
  EXPECT_FALSE(IsSha256Hex(std::string(63, 'a') + "g"));
}
