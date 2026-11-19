#include <gtest/gtest.h>
#include <stdlib.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>

#include "mowbite_calibration/result.h"

namespace mowbite_calibration {
namespace {

Ending Done() {
  Ending ending;
  ending.id = "calib-1793707200";
  ending.time = 1793707470.0;
  ending.reason = "DONE";
  ending.message = "The calibration drive ran to its end";
  ending.recording = "/data/calibration/recordings/calib-20261103T120000Z.jsonl.gz";
  return ending;
}

json Value(const std::string& name, double value, double old, bool ok) {
  return json{{"name", name}, {"value", value}, {"unit", "m"},   {"old", old},
              {"sigma", 0.004}, {"quality", ok ? "good" : "poor"}, {"ok", ok}, {"warning", nullptr}};
}

json Evaluation() {
  return json{{"ready", true},
              {"version", 1},
              {"based_on", {{"m", 12.0}, {"turns", 6}, {"runs", 1}}},
              {"warnings", json::array()},
              {"values", json::array({Value("ticks_per_m", 941.0, 1000.0, true),
                                      Value("wheel_distance_m", 0.396, 0.395, true),
                                      Value("antenna_offset_x", 0.097, 0.0, true),
                                      Value("antenna_offset_y", 0.001, 0.0, true),
                                      Value("imu_roll", 0.035, 0.0, true), Value("imu_pitch", -0.026, 0.0, false)})},
              {"info", {{"gyro_scale", 1.01}, {"gnss_delay_s", 0.1}}}};
}

TEST(Result, EveryValueGetsItsKeyOfTheGeometry) {
  const json result = Compose(Done(), Evaluation());
  EXPECT_EQ(result["status"], "done");
  EXPECT_EQ(result["id"], "calib-1793707200");
  EXPECT_EQ(result["recording"], Done().recording);
  EXPECT_EQ(result["reason"], "DONE");
  ASSERT_EQ(result["values"].size(), 6u);
  const std::vector<std::string> keys = {"ticks_per_m", "track", "antenna_x", "antenna_y", "imu_roll", "imu_pitch"};
  for (size_t i = 0; i < keys.size(); i++) EXPECT_EQ(result["values"][i]["key"], keys[i]);
  EXPECT_EQ(result["values"][0]["old"], 1000.0);
  EXPECT_EQ(result["info"]["gyro_scale"], 1.01);
  EXPECT_EQ(result["based_on"]["turns"], 6);
  EXPECT_EQ(result["summary"], "5 of 6 values ok");
  // The antenna moves by 9.7 cm: the station has to be recorded again.
  EXPECT_TRUE(result["station_again"].get<bool>());
  EXPECT_EQ(GeometryKey("imu_yaw"), "");
}

TEST(Result, TheStationStaysWithASmallOrRejectedChangeOfTheAntenna) {
  json evaluation = Evaluation();
  evaluation["values"][2] = Value("antenna_offset_x", 0.015, 0.0, true);
  EXPECT_FALSE(Compose(Done(), evaluation)["station_again"].get<bool>());
  evaluation["values"][2] = Value("antenna_offset_x", 0.2, 0.0, false);
  EXPECT_FALSE(Compose(Done(), evaluation)["station_again"].get<bool>());
  evaluation["values"][3] = Value("antenna_offset_y", -0.03, 0.0, true);
  EXPECT_TRUE(Compose(Done(), evaluation)["station_again"].get<bool>());
  // A value without numbers is no reason.
  evaluation["values"][3]["value"] = nullptr;
  EXPECT_FALSE(Compose(Done(), evaluation)["station_again"].get<bool>());
}

TEST(Result, FieldsOfAnotherTypeCountAsMissing) {
  json evaluation = Evaluation();
  evaluation["values"][0]["name"] = 7;
  evaluation["values"][1]["ok"] = "yes";
  evaluation["values"][2]["ok"] = 1;
  json result;
  ASSERT_NO_THROW(result = Compose(Done(), evaluation));
  EXPECT_EQ(result["status"], "done");
  EXPECT_EQ(result["values"][0]["key"], nullptr);
  EXPECT_FALSE(result["station_again"].get<bool>());
  EXPECT_EQ(result["summary"], "3 of 6 values ok");
  evaluation["ready"] = "true";
  ASSERT_NO_THROW(result = Compose(Done(), evaluation));
  EXPECT_EQ(result["status"], "failed");
}

TEST(Result, StatusWithoutEvaluationOrWithoutAWholeDrive) {
  const json missing = Compose(Done(), json());
  EXPECT_EQ(missing["status"], "not_evaluated");
  EXPECT_EQ(missing["values"], json::array());
  EXPECT_EQ(missing["recording"], Done().recording);
  EXPECT_EQ(missing["summary"], "Not evaluated: the recording is kept for later");

  const json rejected = Compose(Done(), json{{"error", "no gps lines"}});
  EXPECT_EQ(rejected["status"], "failed");
  EXPECT_EQ(rejected["error"], "no gps lines");
  EXPECT_EQ(rejected["summary"], "The evaluation found no usable values");

  json short_data = Evaluation();
  short_data["ready"] = false;
  EXPECT_EQ(Compose(Done(), short_data)["status"], "failed");

  Ending stopped = Done();
  stopped.reason = "NO_RTK";
  const json ended = Compose(stopped, json());
  EXPECT_EQ(ended["status"], "failed");
  EXPECT_EQ(ended["summary"], "The drive stopped: NO_RTK");
}

TEST(Result, WrittenWhole) {
  char pattern[] = "/tmp/mowbite_result_XXXXXX";
  const std::string folder = mkdtemp(pattern);
  const std::string path = folder + "/calibration/result.json";
  std::string why;
  ASSERT_TRUE(WriteWhole(path, "{\"a\": 1}\n", &why)) << why;
  ASSERT_TRUE(WriteWhole(path, "{\"a\": 2}\n", &why)) << why;
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  EXPECT_EQ(text.str(), "{\"a\": 2}\n");
  // Nothing is left beside it.
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(folder + "/calibration"),
                          std::filesystem::directory_iterator()),
            1);
  EXPECT_FALSE(WriteWhole("/proc/no/such/result.json", "x", &why));
  EXPECT_FALSE(why.empty());
  std::filesystem::remove_all(folder);
}

}  // namespace
}  // namespace mowbite_calibration
