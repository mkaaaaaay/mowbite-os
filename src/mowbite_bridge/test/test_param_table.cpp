#include <gtest/gtest.h>

#include <vector>

#include "mowbite_bridge/param_table.h"

using namespace mowbite_bridge;

// B15
TEST(ParamTable, EmptyByDefault) {
  EXPECT_EQ(ParamTable().Text(), "{}");
  EXPECT_EQ(ParamTable("not json").Text(), "{}");
  EXPECT_EQ(ParamTable("[1]").Text(), "{}");
}

TEST(ParamTable, FlatKeysWithTheNodeSortedAcrossNodes) {
  ParamTable table;
  EXPECT_TRUE(table.Apply(R"({"node":"/xbot_positioning","params":{"antenna_offset_y":0.0,"antenna_offset_x":0.18}})"));
  EXPECT_TRUE(table.Apply(R"({"node":"tron_gnss/","params":{"datum_lat":52.5,"datum_long":9}})"));
  EXPECT_EQ(table.Text(),
            R"({"/tron_gnss/datum_lat":52.5,"/tron_gnss/datum_long":9,)"
            R"("/xbot_positioning/antenna_offset_x":0.18,"/xbot_positioning/antenna_offset_y":0.0})");
}

TEST(ParamTable, OnlyAChangeCounts) {
  ParamTable table;
  const std::string message = R"({"node":"/n","params":{"a":1,"b":[]}})";
  EXPECT_TRUE(table.Apply(message));
  EXPECT_FALSE(table.Apply(message));
  EXPECT_TRUE(table.Apply(R"({"node":"/n","params":{"a":2,"b":[]}})"));
}

TEST(ParamTable, ANewListOfANodeReplacesItsOldOne) {
  ParamTable table;
  table.Apply(R"({"node":"/n","params":{"a":1,"b":2}})");
  table.Apply(R"({"node":"/m","params":{"c":3}})");
  EXPECT_TRUE(table.Apply(R"({"node":"/n","params":{"b":5}})"));
  EXPECT_EQ(table.Text(), R"({"/m/c":3,"/n/b":5})");
}

TEST(ParamTable, PasswordsAreNull) {
  ParamTable table;
  table.Apply(R"({"node":"/ntrip","params":{"password":"geheim","caster_Password":"x2","user":"x"}})");
  EXPECT_EQ(table.Text(), R"({"/ntrip/caster_Password":null,"/ntrip/password":null,"/ntrip/user":"x"})");
}

TEST(ParamTable, MessagesOfAnotherShapeChangeNothing) {
  ParamTable table(R"({"/a/b":1})");
  for (const std::string& bad : std::vector<std::string>{
           "nope", "[]", R"({"node":"/n"})", R"({"params":{}})", R"({"node":5,"params":{}})",
        R"({"node":"/n","params":[1]})", R"({"node":"","params":{"a":1}})", R"({"node":"///","params":{"a":1}})",
        R"({"node":"/n","params":)" + std::string(100, '[') + std::string(100, ']') + "}"}) {
    EXPECT_FALSE(table.Apply(bad)) << bad;
  }
  EXPECT_EQ(table.Text(), R"({"/a/b":1})");
}

TEST(ParamTable, TheStartObjectStaysUnderTheNodes) {
  ParamTable table(R"({"/mower_logic/x":1,"/n/a":0})");
  table.Apply(R"({"node":"/n","params":{"a":true}})");
  EXPECT_EQ(table.Text(), R"({"/mower_logic/x":1,"/n/a":true})");
}
