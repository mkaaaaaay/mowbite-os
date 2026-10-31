#include <gtest/gtest.h>

#include "mowbite_ntrip/ntrip_access.h"

namespace mowbite_ntrip {
namespace {

using json = nlohmann::ordered_json;

const char kSecret[] = "s3cr3t-of-the-test";

NtripConfig Stored() {
  NtripConfig config;
  config.host = "sapos.example";
  config.port = 2101;
  config.mountpoint = "VRS_A";
  config.user = "rover7";
  config.password = kSecret;
  return config;
}

TEST(NtripAccess, MethodsOnlyReadWithoutWriting) {
  EXPECT_EQ(AccessMethods(false), std::vector<std::string>({"ntrip.get"}));
  EXPECT_EQ(AccessMethods(true), std::vector<std::string>({"ntrip.get", "ntrip.set", "ntrip.delete"}));
}

TEST(NtripAccess, SummaryWithoutUserAndPassword) {
  AccessFile file;
  file.exists = true;
  file.config = Stored();
  const json summary = AccessSummary(file);
  EXPECT_EQ(summary, json::parse(R"({"configured": true, "host": "sapos.example", "port": 2101,
                                    "mountpoint": "VRS_A", "version": 2, "has_user": true, "has_password": true})"));
  EXPECT_EQ(summary.dump().find("rover7"), std::string::npos);
  EXPECT_EQ(summary.dump().find(kSecret), std::string::npos);
  file.config->user.clear();
  file.config->password.clear();
  EXPECT_EQ(AccessSummary(file)["has_user"], false);
  EXPECT_EQ(AccessSummary(file)["has_password"], false);
}

TEST(NtripAccess, SummaryWithoutOrWithABadFile) {
  EXPECT_EQ(AccessSummary(AccessFile{}), json::parse(R"({"configured": false})"));
  AccessFile bad;
  bad.exists = true;
  bad.error = "line 3, port: not a port number";
  EXPECT_EQ(AccessSummary(bad), json::parse(R"({"configured": false, "error": "line 3, port: not a port number"})"));
}

TEST(NtripAccess, NewAccessWithDefaults) {
  const AccessChange change = ChangeAccess(
      json::parse(R"({"host": "h.example", "mountpoint": "/M1", "user": "u", "password": "p"})"), std::nullopt);
  ASSERT_TRUE(change.config) << change.error;
  EXPECT_EQ(change.config->host, "h.example");
  EXPECT_EQ(change.config->port, 2101);
  EXPECT_EQ(change.config->mountpoint, "M1");
  EXPECT_EQ(change.config->version, 2);
  EXPECT_EQ(change.config->user, "u");
  EXPECT_EQ(change.config->password, "p");
  // Empty texts: a caster without login.
  const AccessChange open = ChangeAccess(
      json::parse(R"({"host": "h", "mountpoint": "M", "port": 2102, "version": 1, "user": "", "password": ""})"),
      std::nullopt);
  ASSERT_TRUE(open.config) << open.error;
  EXPECT_EQ(open.config->port, 2102);
  EXPECT_EQ(open.config->version, 1);
  EXPECT_TRUE(open.config->user.empty());
}

TEST(NtripAccess, LeftOutLoginStaysForTheSameCaster) {
  const AccessChange change =
      ChangeAccess(json::parse(R"({"host": "sapos.example", "mountpoint": "VRS_B", "version": 1})"), Stored());
  ASSERT_TRUE(change.config) << change.error;
  EXPECT_EQ(change.config->mountpoint, "VRS_B");
  EXPECT_EQ(change.config->version, 1);
  EXPECT_EQ(change.config->user, "rover7");
  EXPECT_EQ(change.config->password, kSecret);
  // A new password alone keeps the user.
  const AccessChange renewed =
      ChangeAccess(json::parse(R"({"host": "sapos.example", "mountpoint": "VRS_A", "password": "new"})"), Stored());
  ASSERT_TRUE(renewed.config) << renewed.error;
  EXPECT_EQ(renewed.config->user, "rover7");
  EXPECT_EQ(renewed.config->password, "new");
}

TEST(NtripAccess, StoredLoginNeverGoesToAnotherCaster) {
  for (const char* params : {R"({"host": "evil.example", "mountpoint": "VRS_A"})",
                             R"({"host": "sapos.example", "port": 2102, "mountpoint": "VRS_A"})",
                             R"({"host": "SAPOS.example", "mountpoint": "VRS_A", "user": "rover7"})"}) {
    const AccessChange change = ChangeAccess(json::parse(params), Stored());
    EXPECT_FALSE(change.config) << params;
    EXPECT_NE(change.error.find("needed for a new caster"), std::string::npos) << change.error;
  }
  // Without a stored access as well.
  const AccessChange first = ChangeAccess(json::parse(R"({"host": "h", "mountpoint": "M"})"), std::nullopt);
  EXPECT_FALSE(first.config);
  EXPECT_EQ(first.error, "user: needed for a new caster (\"\" for none)");
}

TEST(NtripAccess, Refusals) {
  struct Case {
    const char* params;
    const char* error;
  } cases[] = {
      {"null", "params: an object with host, port, mountpoint, version, user and password"},
      {R"(["h", "M"])", "params: an object with host, port, mountpoint, version, user and password"},
      {R"({"mountpoint": "M", "user": "", "password": ""})", "host: missing"},
      {R"({"host": "h", "user": "", "password": ""})", "mountpoint: missing"},
      {R"({"host": 7, "mountpoint": "M", "user": "", "password": ""})", "host: not a text"},
      {R"({"host": "h b", "mountpoint": "M", "user": "", "password": ""})", "host: empty or with spaces"},
      {R"({"host": "h", "mountpoint": "/", "user": "", "password": ""})", "mountpoint: empty or with spaces"},
      {R"({"host": "h", "mountpoint": "M", "port": "2101", "user": "", "password": ""})", "port: not a port number"},
      {R"({"host": "h", "mountpoint": "M", "port": 2101.5, "user": "", "password": ""})", "port: not a port number"},
      {R"({"host": "h", "mountpoint": "M", "port": 0, "user": "", "password": ""})", "port: not a port number"},
      {R"({"host": "h", "mountpoint": "M", "port": 65536, "user": "", "password": ""})", "port: not a port number"},
      {R"({"host": "h", "mountpoint": "M", "version": 3, "user": "", "password": ""})", "version: 1 or 2"},
      {R"({"host": "h", "mountpoint": "M", "version": true, "user": "", "password": ""})", "version: 1 or 2"},
      {R"({"host": "h", "mountpoint": "M", "user": null, "password": ""})", "user: not a text"},
      {R"({"host": "h", "mountpoint": "M", "user": "", "password": 1234})", "password: not a text"},
      {R"({"host": "h", "mountpoint": "M", "user": "", "password": "a\nb"})", "password: control characters"},
  };
  for (const auto& c : cases) {
    const AccessChange change = ChangeAccess(json::parse(c.params), Stored());
    EXPECT_FALSE(change.config) << c.params;
    EXPECT_EQ(change.error, c.error) << c.params;
  }
}

TEST(NtripAccess, RefusalsNeverShowAValue) {
  const std::string long_secret = std::string(kSecret) + std::string(300, 'x');
  for (const json& params : {json{{"host", "h"}, {"mountpoint", "M"}, {"user", ""}, {"password", long_secret}},
                             json{{"host", "h"}, {"mountpoint", "M"}, {"user", long_secret}, {"password", ""}},
                             json{{"host", "h"}, {"mountpoint", "M"}, {"user", ""}, {kSecret, "x"}},
                             json{{"host", "h"}, {"mountpoint", "M"}, {"user", ""}, {"password", 1}, {"pw", kSecret}}}) {
    const AccessChange change = ChangeAccess(params, Stored());
    EXPECT_FALSE(change.config) << params.dump();
    EXPECT_EQ(change.error.find(kSecret), std::string::npos) << change.error;
    EXPECT_FALSE(change.error.empty());
  }
}

}  // namespace
}  // namespace mowbite_ntrip
