#include <gtest/gtest.h>

#include <cmath>

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
    ExpectRating(event, "HOLD", "info", "user");
    ExpectRating(event, "MAP_MISMATCH", "error", "user");
    ExpectRating(event, "POSITION_ACCURACY", "warning", "user");
    // A reason the catalog does not list for the event waits for the user as well.
    ExpectRating(event, "BATTERY_LOW", "warning", "user");
  }
  // The recorder refuses for the hold, the map and its points, never for the emergency or the safety measurement.
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED"}) {
    ExpectRating(event, "BUSY", "info", "user");
    ExpectRating(event, "EMERGENCY", "error", "user");
    ExpectRating(event, "NO_SAFETY_MEASUREMENT", "error", "user");
  }
  ExpectRating("MOWING_REFUSED", "RAIN", "info", "user");
  ExpectRating("MOWING_REFUSED", "PLANNER_MISSING", "error", "user");
  ExpectRating("RECORDING_REFUSED", "SAVE_FAILED", "error", "user");
  ExpectRating("RECORDING_REFUSED", "STILL_SAVING", "info", "user");
}

TEST(Reasons, TheDockingStationWaitsForTheUser) {
  for (const char* event : {"MOWING_REFUSED", "DRIVE_REFUSED"}) {
    ExpectRating(event, "REFERENCE_MISMATCH", "warning", "user");
    ExpectRating(event, "NO_DOCK", "warning", "user");
    ExpectRating(event, "NO_DOCK_CONTACT", "warning", "user");
    ExpectRating(event, "DOCK_UNREACHABLE", "warning", "user");
    ExpectRating(event, "ON_DOCK", "info", "user");
  }
  ExpectRating("RECORDING_REFUSED", "NO_DOCK_CONTACT", "warning", "user", "record_dock");
  ExpectRating("RECORDING_REFUSED", "NO_HEADING", "warning", "user", "record_dock");
  // OpenMower's reason values.
  ExpectRating("UNDOCKING_FAILED", "path_failed", "error", "user");
  ExpectRating("UNDOCKING_FAILED", "no_gps", "error", "user");
  for (const char* code : {"approach_failed", "dock_failed", "charge_failed"}) {
    ExpectRating("DOCKING_RETRY", code, "warning", "auto");
    ExpectRating("DOCKING_FAILED", code, "error", "user");
  }
  // Charging that stopped on the station.
  ExpectRating("CHARGING_LOST", "NO_DOCK_CONTACT", "warning", "user");
  ExpectRating("CHARGING_LOST", "NO_CHARGE_CURRENT", "warning", "user");
  // docking.start on the contacts.
  ExpectRating("DRIVE_REFUSED", "ALREADY_THERE", "info", "user");
}

TEST(Reasons, ARefusedResetWaitsForTheUser) {
  ExpectRating("EMERGENCY_RESET_REFUSED", "STILL_TILTED", "warning", "user");
  for (const char* code : {"TILT_UNKNOWN", "FOREIGN_HOST", "DRIVER_MISSING", "SAFETY_MISSING", "NO_ANSWER"}) {
    ExpectRating("EMERGENCY_RESET_REFUSED", code, "error", "user");
  }
}

TEST(Reasons, ARefusedResetCarriesItsNumbers) {
  json details = json::object();
  PutReason(&details, "EMERGENCY_RESET_REFUSED", ResetRefusal("STILL_TILTED", "still tilted", 31.5, 26));
  EXPECT_EQ(details["reason"], "STILL_TILTED");
  EXPECT_EQ(details["angle"], 31.5);
  EXPECT_EQ(details["limit"], 26.0);
  EXPECT_EQ(details["message"], "still tilted");
  EXPECT_EQ(details["severity"], "warning");
  EXPECT_EQ(details["resume"], "user");
  Reason unknown = ResetRefusal("TILT_UNKNOWN", "no IMU", std::nan(""), std::nan(""));
  EXPECT_TRUE(unknown.fields.empty());
  EXPECT_EQ(ResetRefusal("TILT_UNKNOWN", "no IMU", 1.25, std::nan("")).fields["seconds"], 1.25);
  EXPECT_TRUE(ResetRefusal("FOREIGN_HOST", "udp 192.0.2.7:5000", 0, 0).fields.empty());
  EXPECT_EQ(ResetRefusal("", "", 0, 0).code, "NO_ANSWER");
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

TEST(Reasons, APathTheLogicStoppedTellsTheCause) {
  const Reason tilt = Reason("EMERGENCY", "Emergency: LATCH, TILT").With("limit", 31.0);
  // Whether the cancel reached the follower first or it stopped for the same emergency itself: the same text.
  for (const char* code : {"CANCELED", "EMERGENCY"}) {
    const Reason why = FollowerEndReason(code, "x", tilt);
    EXPECT_EQ(why.code, "EMERGENCY") << code;
    EXPECT_EQ(why.message, "Emergency: LATCH, TILT (x)") << code;
    EXPECT_DOUBLE_EQ(why.fields["limit"].get<double>(), 31.0) << code;
  }
  const Reason skip("STOPPED", "Skipped by the app");
  EXPECT_EQ(FollowerEndReason("CANCELED", "canceled", skip).message, "Skipped by the app (canceled)");
  // Without a cancel of the logic, or stopped for something else than the cause: the follower's own result.
  for (const auto& cancel : {std::optional<Reason>(), std::optional<Reason>(skip)}) {
    const Reason why = FollowerEndReason("EMERGENCY", "emergency", cancel);
    EXPECT_EQ(why.code, "EMERGENCY");
    EXPECT_EQ(why.message, "emergency");
  }
  EXPECT_EQ(FollowerEndReason("CROSS_TRACK", "off the path", tilt).code, "CROSS_TRACK");
  EXPECT_EQ(FollowerEndReason("CANCELED", "canceled", std::nullopt).code, "CANCELED");
  EXPECT_EQ(FollowerEndReason("DONE", "done", std::nullopt).code, "DONE");
}

}  // namespace
}  // namespace mowbite_logic
