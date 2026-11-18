#include <gtest/gtest.h>

#include <vector>

#include "mowbite_logic/obstacle_part.h"

using namespace mowbite_logic;

namespace {

constexpr int kSlow = 1;
constexpr int kStop = 2;

ObstacleTrackSample Track(uint32_t id, const char* kind, int zone, double ahead = 0.7, bool in_view = true) {
  return {id, kind, true, in_view, ahead, zone};
}

ObstacleSample Seen(std::vector<ObstacleTrackSample> tracks, bool sight = true) {
  return {sight, std::move(tracks)};
}

ObstaclePart Armed() {
  ObstaclePart part;
  part.SetMode(2, 2);
  return part;
}

// Status at 10 Hz from `from` to `to` (s, both in).
void Feed(ObstaclePart& part, const ObstacleSample& sample, double from, double to, bool driving = false) {
  for (double t = from; t <= to + 1e-9; t += 0.1) {
    part.OnStatus(sample, t, driving);
    part.Tick(t);
  }
}

}  // namespace

TEST(ObstaclePart, OnlyStopOnAPlatformProvenForItArmsTheHold) {
  ObstaclePart part;
  part.SetMode(2, 1);
  EXPECT_EQ(part.mode(), 1);
  part.OnStatus(Seen({Track(1, "PERSON", kStop)}), 0, true);
  EXPECT_FALSE(part.Holding());
  part.SetMode(3, 3);
  // Avoid acts as stop until the mower drives around obstacles.
  EXPECT_EQ(part.mode(), 2);
  part.OnStatus(Seen({Track(1, "PERSON", kStop)}), 0.1, true);
  EXPECT_TRUE(part.Holding());
  part.SetMode(1, 3);
  EXPECT_FALSE(part.Holding());
}

TEST(ObstaclePart, ShowNotesWhereItWouldHaveStoppedWhileDriving) {
  ObstaclePart part;
  part.SetMode(1, 2);
  part.OnStatus(Seen({Track(1, "OBJECT", kStop), Track(2, "ANIMAL", kSlow, 1.5)}), 0, false);
  EXPECT_TRUE(part.TakeNotes().empty());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop), Track(2, "ANIMAL", kSlow, 1.5)}), 0.1, true);
  part.OnStatus(Seen({Track(1, "OBJECT", kStop), Track(2, "ANIMAL", kSlow, 1.5)}), 0.2, true);
  const auto notes = part.TakeNotes();
  ASSERT_EQ(notes.size(), 1u);
  EXPECT_EQ(notes[0].track_id, 1u);
  EXPECT_EQ(notes[0].action, "would_stop");
  EXPECT_FALSE(part.Holding());
  EXPECT_FALSE(part.Slow());
}

TEST(ObstaclePart, AConfirmedThingInTheStopZoneHolds) {
  ObstaclePart part = Armed();
  ObstacleTrackSample candidate = Track(1, "PERSON", kStop);
  candidate.confirmed = false;
  part.OnStatus(Seen({candidate}), 0, true);
  EXPECT_FALSE(part.Holding());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop), Track(2, "PERSON", kStop, 0.9)}), 0.1, true);
  EXPECT_TRUE(part.Holding());
  const Reason why = part.Why();
  EXPECT_EQ(why.code, "OBSTACLE");
  EXPECT_EQ(why.fields["kind"], "PERSON");
  const auto notes = part.TakeNotes();
  ASSERT_EQ(notes.size(), 2u);
  EXPECT_EQ(notes[0].action, "stopped");
}

TEST(ObstaclePart, ALivingThingAheadSlowsTheMower) {
  ObstaclePart part = Armed();
  part.OnStatus(Seen({Track(1, "ANIMAL", kSlow, 1.5)}), 0, true);
  EXPECT_TRUE(part.Slow());
  EXPECT_FALSE(part.Holding());
  EXPECT_EQ(part.TakeNotes()[0].action, "slowed");
  part.Tick(1.2);
  EXPECT_FALSE(part.Slow());
}

TEST(ObstaclePart, GoesOnOnceTheZoneIsSeenFreeForTenSeconds) {
  ObstaclePart part = Armed();
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.7)}), 0, true);
  ASSERT_TRUE(part.Holding());
  Feed(part, Seen({}), 0.1, 10.0);
  EXPECT_TRUE(part.Holding());
  part.OnStatus(Seen({}), 10.15, false);
  EXPECT_FALSE(part.Holding());
}

TEST(ObstaclePart, ThirtySecondsAfterAThingRightAtTheNose) {
  ObstaclePart part = Armed();
  part.OnStatus(Seen({Track(1, "ANIMAL", kStop, 0.7)}), 0, true);
  part.OnStatus(Seen({Track(1, "ANIMAL", kStop, 0.3)}), 0.1, true);
  Feed(part, Seen({}), 0.2, 30.1);
  EXPECT_TRUE(part.Holding());
  part.OnStatus(Seen({}), 30.25, false);
  EXPECT_FALSE(part.Holding());
}

TEST(ObstaclePart, APlaceOutOfViewOrWithoutSightIsNeverFree) {
  ObstaclePart part = Armed();
  // The thing stays in the stop zone out of view: the hold stays as long as it does.
  Feed(part, Seen({Track(1, "ANIMAL", kStop, 0.1, false)}), 0, 600);
  EXPECT_TRUE(part.Holding());
  // Gone from the tracks, but the sensor sees nothing: the wait does not run.
  Feed(part, Seen({}, false), 600.1, 700);
  EXPECT_TRUE(part.Holding());
  // Sight again: the wait starts from there.
  Feed(part, Seen({}), 700.1, 709.9);
  EXPECT_TRUE(part.Holding());
  Feed(part, Seen({}), 710.0, 710.3);
  EXPECT_FALSE(part.Holding());
}

TEST(ObstaclePart, AStatusThatStopsComingStartsNothingAndFreesNothing) {
  ObstaclePart part = Armed();
  part.OnStatus(Seen({}), 0, true);
  part.Tick(5);
  EXPECT_FALSE(part.Holding());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop)}), 5, true);
  part.OnStatus(Seen({}), 5.1, true);
  // The node of the obstacles stopped: the wait for a free zone ends with its last status.
  part.Tick(6.2);
  part.Tick(30);
  EXPECT_TRUE(part.Holding());
  part.OnStatus(Seen({}), 31, false);
  part.OnStatus(Seen({}), 40.9, false);
  EXPECT_TRUE(part.Holding());
  part.OnStatus(Seen({}), 41, false);
  EXPECT_FALSE(part.Holding());
}

TEST(ObstaclePart, TheUserLetsItGoPastWhatNoSensorSees) {
  ObstaclePart part = Armed();
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.2, true)}), 0, true);
  EXPECT_FALSE(part.Waive());
  EXPECT_TRUE(part.Holding());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.2, false)}), 0.1, true);
  EXPECT_TRUE(part.Waive());
  EXPECT_FALSE(part.Holding());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.2, false)}), 0.2, true);
  EXPECT_FALSE(part.Holding());
  // Seen again, it holds again; so does a new thing.
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.2, true)}), 0.3, true);
  EXPECT_TRUE(part.Holding());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.2, false)}), 0.4, true);
  EXPECT_TRUE(part.Waive());
  part.OnStatus(Seen({Track(1, "OBJECT", kStop, 0.2, false), Track(2, "PERSON", kStop, 0.8, false)}), 0.5, true);
  EXPECT_TRUE(part.Holding());
}
