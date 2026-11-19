#include <gtest/gtest.h>

#include "mowbite_bridge/text.h"

using namespace mowbite_bridge;

namespace {
const std::string kR = "\xef\xbf\xbd";
}

TEST(CleanUtf8, KeepsValidText) {
  const std::string text = "Gr\xc3\xa4" "fin \xe2\x82\xac \xf0\x9f\x8c\xb1 plain";
  EXPECT_EQ(CleanUtf8(text), text);
  EXPECT_EQ(CleanUtf8(""), "");
}

TEST(CleanUtf8, LoneAndBrokenBytesBecomeOneReplacementEach) {
  // A lone 0xE4 (Latin-1), an Ä cut off at the end, an overlong '/', a surrogate, bytes above U+10FFFF.
  EXPECT_EQ(CleanUtf8("Gr\xe4" "fin"), "Gr" + kR + "fin");
  EXPECT_EQ(CleanUtf8("cut \xc3"), "cut " + kR);
  EXPECT_EQ(CleanUtf8("\xc0\xaf ok \xc3\x84"), kR + kR + " ok \xc3\x84");
  EXPECT_EQ(CleanUtf8("\xed\xa0\x80"), kR + kR + kR);
  EXPECT_EQ(CleanUtf8("\xf4\x90\x80\x80"), kR + kR + kR + kR);
  EXPECT_EQ(CleanUtf8("\xff\xfe"), kR + kR);
  // A valid start of a three byte sign that breaks off is one replacement.
  EXPECT_EQ(CleanUtf8("\xe2\x82x"), kR + "x");
}

TEST(NestingDepth, CountsBracketsOutsideStrings) {
  EXPECT_EQ(NestingDepth(R"({"a":"[[[[","b":[{}]})"), 3u);
  EXPECT_EQ(NestingDepth(R"(["\"[["])"), 1u);
  EXPECT_EQ(NestingDepth("plain"), 0u);
  EXPECT_EQ(NestingDepth(std::string(500, '[')), 500u);
}

TEST(HideSecrets, LoginsInUrlsAndValuesAfterKeyWords) {
  EXPECT_EQ(HideSecrets("ntrip://user:secret@caster:2101/MOUNT"), "ntrip://***@caster:2101/MOUNT");
  EXPECT_EQ(HideSecrets("password=hunter2 token: abc"), "password=*** token: ***");
  EXPECT_EQ(HideSecrets("Password = x1 SECRET:y"), "Password = *** SECRET:***");
  // Nothing to hide.
  EXPECT_EQ(HideSecrets("http://caster:2101/MOUNT and a@b"), "http://caster:2101/MOUNT and a@b");
  EXPECT_EQ(HideSecrets("password missing"), "password missing");
  // A password with '/' or '@', and the credentials of an Authorization header.
  EXPECT_EQ(HideSecrets("ntrip://user:pa/ss@caster/MOUNT x"), "ntrip://***@caster/MOUNT x");
  EXPECT_EQ(HideSecrets("ntrip://user:p@ss@host:2101/M"), "ntrip://***@host:2101/M");
  EXPECT_EQ(HideSecrets("Authorization: Basic dXNlcjpwYXNz"), "Authorization: Basic ***");
  EXPECT_EQ(HideSecrets("header bearer abc.def end"), "header bearer *** end");
  EXPECT_EQ(HideSecrets("basically fine"), "basically fine");
}

TEST(HideSecrets, QuotedNamesAndValuesAsInJson) {
  EXPECT_EQ(HideSecrets(R"({"password": "two words", "user": "u"})"), R"({"password": "***", "user": "u"})");
  EXPECT_EQ(HideSecrets(R"({"api_token":"a\"b c","n":1})"), R"({"api_token":"***","n":1})");
  EXPECT_EQ(HideSecrets("client_secret='x y' private_key: k1 api_key=k2"),
            "client_secret='***' private_key: *** api_key=***");
  EXPECT_EQ(HideSecrets(R"({"token": "open)"), R"({"token": "***)");
  EXPECT_EQ(HideSecrets(R"({"geometry_key": "g"})"), R"({"geometry_key": "g"})");
}
