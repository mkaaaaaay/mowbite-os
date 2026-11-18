#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mowbite_logic/obstacle_part.h"

using namespace mowbite_logic;

namespace {

constexpr int kNone = 0;
constexpr int kSlow = 1;
constexpr int kStop = 2;

ObstacleTrackSample Track(uint32_t id, const char* kind, int zone, double ahead = 0.7, bool in_view = true) {
  ObstacleTrackSample track;
  track.id = id;
  track.kind = kind;
  track.confirmed = true;
  track.in_view = in_view;
  track.ahead = ahead;
  track.zone = zone;
  return track;
}

ObstacleSample Seen(std::vector<ObstacleTrackSample> tracks, bool sight = true) {
  return {sight, std::move(tracks)};
}

// A thing at (x, y) in the map, 0.1 m round with 0.05 m spread.
ObstacleTrackSample At(uint32_t id, const char* kind, double x, double y, int zone = kStop, bool in_view = true) {
  ObstacleTrackSample track = Track(id, kind, zone, 0.7, in_view);
  track.x = x;
  track.y = y;
  track.radius = 0.1;
  track.sigma = 0.05;
  return track;
}

ObstaclePart Avoiding() {
  ObstaclePart part;
  part.SetMode(3, 3);
  return part;
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
  part.SetMode(3, 2);
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
  Feed(part, Seen({Track(1, "ANIMAL", kStop, 0.7, false)}), 0, 600);
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

TEST(ObstaclePart, AnObjectThatStandsThreeSecondsIsDrivenAround) {
  ObstaclePart part = Avoiding();
  EXPECT_EQ(part.mode(), 3);
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 2.9);
  ASSERT_TRUE(part.Holding());
  EXPECT_TRUE(part.AvoidDue(2.95).empty());
  const auto due = part.AvoidDue(3.0);
  ASSERT_EQ(due.size(), 1u);
  EXPECT_EQ(due[0].track_id, 1u);
  EXPECT_DOUBLE_EQ(due[0].x, 5);
  // Its radius, twice its spread and 0.3 m between it and the body on the way around; its radius and 0.1 m, the spread
  // within, on a lane passing it.
  EXPECT_NEAR(due[0].r, 0.1 + 2 * 0.05 + 0.3, 1e-9);
  EXPECT_NEAR(due[0].pass_r, 0.1 + 0.1, 1e-9);
}

TEST(ObstaclePart, APersonOrAnimalIsNeverDrivenAround) {
  for (const char* kind : {"PERSON", "ANIMAL", "UNKNOWN"}) {
    ObstaclePart part = Avoiding();
    Feed(part, Seen({At(1, "OBJECT", 5, 0), At(2, kind, 5, 1)}), 0, 600);
    EXPECT_TRUE(part.Holding()) << kind;
    EXPECT_TRUE(part.AvoidDue(600).empty()) << kind;
  }
}

TEST(ObstaclePart, AThingThatMovesStandsAnew) {
  ObstaclePart part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 2.0);
  Feed(part, Seen({At(1, "OBJECT", 5.3, 0)}), 2.1, 4.9);
  EXPECT_TRUE(part.AvoidDue(4.9).empty());
  Feed(part, Seen({At(1, "OBJECT", 5.3, 0)}), 5.0, 5.2);
  EXPECT_EQ(part.AvoidDue(5.2).size(), 1u);
}

TEST(ObstaclePart, BelowAvoidOrWithoutAFreshStatusNothingIsDrivenAround) {
  ObstaclePart part;
  part.SetMode(3, 2);
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 10);
  EXPECT_TRUE(part.AvoidDue(10).empty());
  part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 10);
  part.Tick(11.2);
  EXPECT_TRUE(part.AvoidDue(11.2).empty());
}

TEST(ObstaclePart, InsideItsCircleAnObjectHoldsNoMoreAPersonStillDoes) {
  ObstaclePart part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  part.Avoid(part.AvoidDue(3.0));
  EXPECT_FALSE(part.Holding());
  ASSERT_EQ(part.circles().size(), 1u);
  Feed(part, Seen({At(1, "OBJECT", 5.05, 0)}), 3.1, 10, true);
  EXPECT_FALSE(part.Holding());
  // Its track ends out of view; a new track of the same thing is the circle's.
  part.OnStatus(Seen({At(1, "OBJECT", 5.05, 0, kStop, false)}), 10.05, true);
  Feed(part, Seen({At(7, "OBJECT", 5, 0.1)}), 10.1, 12, true);
  EXPECT_FALSE(part.Holding());
  EXPECT_EQ(part.circles()[0].follows, 7u);
  part.OnStatus(Seen({At(7, "OBJECT", 5, 0.1), At(8, "PERSON", 5, 0.2)}), 12.1, true);
  EXPECT_TRUE(part.Holding());
  // So does an object beside it that its circle does not take in.
  ObstaclePart other = Avoiding();
  Feed(other, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  other.Avoid(other.AvoidDue(3.0));
  other.OnStatus(Seen({At(1, "OBJECT", 5, 0), At(2, "OBJECT", 5, 0.6)}), 3.1, true);
  EXPECT_TRUE(other.Holding());
}

TEST(ObstaclePart, ACircleGoesOnceItsPlaceIsSeenFree) {
  ObstaclePart part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  part.Avoid(part.AvoidDue(3.0));
  part.TakeNotes();
  // Its track ended out of view: the circle stays without one.
  part.OnStatus(Seen({At(1, "OBJECT", 5, 0, kNone, false)}), 3.1, true);
  part.OnStatus(Seen({}), 3.2, true);
  ASSERT_EQ(part.circles().size(), 1u);
  EXPECT_EQ(part.circles()[0].follows, 0u);
  EXPECT_FALSE(part.TakeCirclesGone());
  // Found again and then seen free: it goes, and its entry loses it.
  part.OnStatus(Seen({At(3, "OBJECT", 5, 0, kNone)}), 3.3, true);
  part.OnStatus(Seen({}), 3.4, true);
  EXPECT_TRUE(part.circles().empty());
  EXPECT_TRUE(part.TakeCirclesGone());
  const auto notes = part.TakeNotes();
  ASSERT_EQ(notes.size(), 1u);
  EXPECT_EQ(notes[0].track_id, 1u);
  EXPECT_EQ(notes[0].action, "avoided");
  EXPECT_EQ(notes[0].keep_out_r, 0.0);
}

TEST(ObstaclePart, ACircleGoesAfterHalfAnHourBelowAvoidAndWithTheJob) {
  ObstaclePart part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  part.Avoid(part.AvoidDue(3.0));
  part.Tick(1802.9);
  EXPECT_EQ(part.circles().size(), 1u);
  part.Tick(1803.0);
  EXPECT_TRUE(part.circles().empty());
  part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  part.Avoid(part.AvoidDue(3.0));
  part.SetMode(2, 3);
  EXPECT_TRUE(part.circles().empty());
  part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  part.Avoid(part.AvoidDue(3.0));
  part.ClearCircles();
  EXPECT_TRUE(part.circles().empty());
  EXPECT_TRUE(part.TakeCirclesGone());
}

TEST(ObstaclePart, WithoutAWayAroundTheHoldWaitsLikeStop) {
  ObstaclePart part = Avoiding();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 0, 3.0);
  part.AvoidRefused();
  Feed(part, Seen({At(1, "OBJECT", 5, 0)}), 3.1, 60);
  EXPECT_TRUE(part.Holding());
  EXPECT_TRUE(part.AvoidDue(60).empty());
  // A new hold asks again.
  Feed(part, Seen({}), 60.1, 70.2);
  EXPECT_FALSE(part.Holding());
  Feed(part, Seen({At(2, "OBJECT", 6, 0)}), 70.3, 73.4);
  EXPECT_EQ(part.AvoidDue(73.4).size(), 1u);
}

TEST(ObstaclePart, TheKeepOutRingHoldsTheCircle) {
  const AvoidCircle circle{1, 1, 2.0, -1.0, 0.5, 0.3, 0.0};
  for (const bool inner : {false, true}) {
    const double r = inner ? 0.3 : 0.5;
    const Ring ring = KeepOutRing(circle, inner);
    ASSERT_GE(ring.size(), 16u);
    for (size_t i = 0; i < ring.size(); ++i) {
      const Point2& a = ring[i];
      const Point2& b = ring[(i + 1) % ring.size()];
      // The middle of each edge touches the circle, the corners lie outside.
      EXPECT_NEAR(std::hypot((a.x + b.x) / 2 - 2.0, (a.y + b.y) / 2 + 1.0), r, 1e-9);
      EXPECT_GT(std::hypot(a.x - 2.0, a.y + 1.0), r);
    }
  }
}
