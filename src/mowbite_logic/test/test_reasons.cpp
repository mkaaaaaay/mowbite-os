#include <gtest/gtest.h>

#include "mowbite_logic/reasons.h"

namespace mowbite_logic {
namespace {

void ExpectRating(const std::string& event, const std::string& code, const std::string& severity,
                  const std::string& resume, const std::string& action = "") {
  const Rating rating = RateReason(event, code, action);
  EXPECT_EQ(rating.severity, severity) << event << " " << code;
  EXPECT_EQ(rating.resume, resume) << event << " " << code;
}

TEST(Reasons, RefusalsAlwaysWaitForTheUser) {
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED", "RECORDING_REFUSED"}) {
    ExpectRating(event, "BUSY", "info", "user");
    ExpectRating(event, "RAIN", "info", "user");
    ExpectRating(event, "EMERGENCY", "error", "user");
    ExpectRating(event, "NO_SAFETY_MEASUREMENT", "error", "user");
    ExpectRating(event, "MAP_MISMATCH", "error", "user");
    ExpectRating(event, "POSITION_ACCURACY", "warning", "user");
    ExpectRating(event, "BATTERY_LOW", "warning", "user");
  }
  ExpectRating("MOWING_REFUSED", "PLANNER_MISSING", "error", "user");
  ExpectRating("RECORDING_REFUSED", "SAVE_FAILED", "error", "user");
  ExpectRating("RECORDING_REFUSED", "STILL_SAVING", "info", "user");
}

TEST(Reasons, TheDockingStationWaitsForTheUser) {
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED"}) {
    ExpectRating(event, "REFERENCE_MISMATCH", "warning", "user");
    ExpectRating(event, "NO_DOCK", "warning", "user");
    ExpectRating(event, "NO_DOCK_CONTACT", "warning", "user");
    ExpectRating(event, "ON_DOCK", "info", "user");
  }
  ExpectRating("RECORDING_REFUSED", "NO_DOCK_CONTACT", "warning", "user", "record_dock");
  ExpectRating("RECORDING_REFUSED", "NO_HEADING", "warning", "user", "record_dock");
}

TEST(Reasons, AutomaticPointsGoOnOnceTheyAreAccurate) {
  ExpectRating("RECORDING_REFUSED", "POSITION_ACCURACY", "warning", "auto", "point");
  ExpectRating("RECORDING_REFUSED", "POSITION_ACCURACY", "warning", "user", "collect_point");
}

TEST(Reasons, PausesForThePositionGoOnByThemselves) {
  for (const char* code : {"NO_POSITION", "POSE_LOST", "POSITION_ACCURACY", "HEADING"}) {
    ExpectRating("STATE", code, "warning", "auto");
  }
  ExpectRating("STATE", "MANUAL", "info", "user");
  for (const char* code : {"EMERGENCY", "CROSS_TRACK", "TIMEOUT", "PLANNING_FAILED", "BLADE_REFUSED", "MAP_MISMATCH"}) {
    ExpectRating("STATE", code, "error", "user");
  }
}

TEST(Reasons, DriveEnds) {
  for (const char* code : {"DONE", "ALREADY_THERE", "STOPPED", "CANCELED"}) ExpectRating("DRIVE", code, "info", "user");
  ExpectRating("DRIVE", "POSE_LOST", "warning", "user");
  ExpectRating("DRIVE", "TRANSIT_FAILED", "error", "user");
  ExpectRating("DRIVE", "OUTSIDE", "error", "user");
}

TEST(Reasons, OtherEvents) {
  ExpectRating("POSITION_JUMP", "OUTLIERS", "warning", "auto");
  ExpectRating("AREA_SKIPPED", "MANUAL", "info", "auto");
}

TEST(Reasons, PutReasonAddsFieldsMessageAndRating) {
  json details = {{"action", "point"}};
  PutReason(&details, "RECORDING_REFUSED",
            Reason("POSITION_ACCURACY", "Position accuracy 0.30 m").With("accuracy", 0.3).With("limit", 0.1));
  EXPECT_EQ(details["reason"], "POSITION_ACCURACY");
  EXPECT_DOUBLE_EQ(details["accuracy"].get<double>(), 0.3);
  EXPECT_DOUBLE_EQ(details["limit"].get<double>(), 0.1);
  EXPECT_EQ(details["message"], "Position accuracy 0.30 m");
  EXPECT_EQ(details["severity"], "warning");
  EXPECT_EQ(details["resume"], "auto");
}

TEST(Reasons, MapReasonFromTheDatumState) {
  EXPECT_EQ(MapReason(MapDatum::kUnknown, "").code, "MAP_UNKNOWN");
  EXPECT_EQ(MapReason(MapDatum::kEmpty, "").code, "NO_MOWING_AREA");
  EXPECT_EQ(MapReason(MapDatum::kMismatch, "far away").code, "MAP_MISMATCH");
  EXPECT_EQ(MapReason(MapDatum::kMismatch, "far away").message, "far away");
  const Reason unbound = MapReason(MapDatum::kUnbound, "");
  EXPECT_EQ(unbound.code, "MAP_MISMATCH");
  EXPECT_EQ(unbound.fields["datum_state"], "unbound");
}

}  // namespace
}  // namespace mowbite_logic
