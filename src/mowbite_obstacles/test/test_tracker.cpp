#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_obstacles/tracker.h"

namespace mowbite_obstacles {
namespace {

constexpr double kDeg = M_PI / 180.0;

// The view of a camera on the nose: 43 deg to each side, from 0.4 to 4 m.
ViewSector Front() {
  return {0.466, 0.0, 43.0 * kDeg, -43.0 * kDeg, 0.4, 4.0};
}

Find Thing(Kind kind, double x, double y, double score = 0.8) {
  Find find;
  find.kind = kind;
  find.class_name = kind == Kind::kPerson ? "person" : kind == Kind::kAnimal ? "hedgehog" : "ball";
  find.score = kind == Kind::kUnknown ? 1.0 : score;
  find.x = x;
  find.y = y;
  find.radius = 0.1;
  find.height = 0.2;
  find.cov_xx = 0.01;
  find.cov_yy = 0.0025;
  find.box_w = 20;
  find.box_h = 30;
  return find;
}

Frame At(double stamp, std::vector<Find> finds = {}) {
  Frame frame;
  frame.source = "stereo";
  frame.stamp = stamp;
  frame.sight = true;
  frame.view = Front();
  frame.finds = std::move(finds);
  return frame;
}

class TrackerTest : public ::testing::Test {
 protected:
  TrackerTest() {
    tracker.SetSources({"stereo"});
    tracker.SetBody({0.605, 0.12, 0.235});
  }

  // The mower stands here from now on.
  void Stand(double x, double y, double heading) { pose = {0.0, x, y, heading, 0.0}; }

  // A frame with the pose of the mower at its stamp.
  FrameResult Feed(double stamp, std::vector<Find> finds = {}) {
    pose.stamp = stamp;
    tracker.OnPose(pose);
    return tracker.OnFrame(At(stamp, std::move(finds)), stamp + 0.05);
  }

  Tracker tracker;
  Pose pose;
};

TEST_F(TrackerTest, AFindGoesIntoTheMapWithThePoseOfItsFrame) {
  Tracker moving;
  moving.SetSources({"stereo"});
  moving.OnPose({1.0, 10.0, 5.0, 90.0 * kDeg, 0.0});
  moving.OnPose({1.2, 10.0, 5.2, 90.0 * kDeg, 0.0});
  moving.OnFrame(At(1.1, {Thing(Kind::kObject, 2.0, 0.5)}), 1.15);
  ASSERT_EQ(moving.tracks().size(), 1u);
  const Track& track = moving.tracks().begin()->second;
  EXPECT_NEAR(track.x, 9.5, 1e-9);
  EXPECT_NEAR(track.y, 7.1, 1e-9);
  // Its spread along the view of the camera lies along y of the map now.
  EXPECT_NEAR(track.cov_xx, 0.0025, 1e-12);
  EXPECT_NEAR(track.cov_yy, 0.01, 1e-12);
}

TEST_F(TrackerTest, AFrameWithoutAPoseNearItsStampIsLeftOut) {
  Tracker lost;
  lost.SetSources({"stereo"});
  lost.OnPose({1.0, 0.0, 0.0, 0.0, 0.0});
  const FrameResult result = lost.OnFrame(At(2.0, {Thing(Kind::kObject, 2.0, 0.0)}), 2.0);
  EXPECT_TRUE(result.no_pose);
  EXPECT_TRUE(lost.tracks().empty());
}

TEST_F(TrackerTest, AFrameOfAnotherSensorIsIgnored) {
  Frame frame = At(1.0, {Thing(Kind::kObject, 2.0, 0.0)});
  frame.source = "rear";
  EXPECT_FALSE(tracker.OnFrame(frame, 1.0).used);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST_F(TrackerTest, TwoFindsInARowConfirmAThing) {
  EXPECT_TRUE(Feed(1.0, {Thing(Kind::kObject, 2.0, 0.0)}).confirmed.empty());
  const FrameResult second = Feed(1.2, {Thing(Kind::kObject, 2.02, 0.01)});
  ASSERT_EQ(second.confirmed.size(), 1u);
  EXPECT_TRUE(tracker.TrackOf(second.confirmed[0])->confirmed);
  EXPECT_TRUE(Feed(1.4, {Thing(Kind::kObject, 2.0, 0.0)}).confirmed.empty());
  EXPECT_EQ(tracker.tracks().size(), 1u);
}

TEST_F(TrackerTest, TwoOfThreeFramesConfirm) {
  Feed(1.0, {Thing(Kind::kObject, 2.0, 0.0)});
  Feed(1.2);
  EXPECT_EQ(Feed(1.4, {Thing(Kind::kObject, 2.0, 0.0)}).confirmed.size(), 1u);
}

TEST_F(TrackerTest, OneFindInThreeFramesIsDropped) {
  Feed(1.0, {Thing(Kind::kObject, 2.0, 0.0)});
  Feed(1.2);
  const FrameResult third = Feed(1.4);
  EXPECT_EQ(third.ended.size(), 1u);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST_F(TrackerTest, AFindBelowItsScoreDoesNotCount) {
  Feed(1.0, {Thing(Kind::kPerson, 2.0, 0.0, 0.3)});
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST_F(TrackerTest, ThingsOfOtherKindsStaySeparate) {
  Feed(1.0, {Thing(Kind::kPerson, 2.0, 0.0), Thing(Kind::kObject, 2.1, 0.0)});
  EXPECT_EQ(tracker.tracks().size(), 2u);
}

TEST_F(TrackerTest, AClassOfTheModelNamesAThingSeenByDepthAlone) {
  Feed(1.0, {Thing(Kind::kUnknown, 2.0, 0.0)});
  Feed(1.2, {Thing(Kind::kAnimal, 2.0, 0.0)});
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks().begin()->second.kind, Kind::kAnimal);
  EXPECT_EQ(tracker.tracks().begin()->second.class_name, "hedgehog");
}

TEST_F(TrackerTest, AWalkingPersonKeepsItsTrack) {
  for (int i = 0; i < 10; ++i) Feed(1.0 + 0.2 * i, {Thing(Kind::kPerson, 3.5 - 0.25 * i, 0.5)});
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_NEAR(tracker.tracks().begin()->second.x, 3.5 - 0.25 * 9, 0.1);
}

TEST_F(TrackerTest, AThingEndsOnlyOnceItsPlaceIsSeenFreeLongEnough) {
  Feed(1.0, {Thing(Kind::kObject, 2.0, 0.0)});
  Feed(1.2, {Thing(Kind::kObject, 2.0, 0.0)});
  for (double t = 1.4; t < 2.3; t += 0.2) EXPECT_TRUE(Feed(t).ended.empty()) << t;
  EXPECT_EQ(Feed(2.4).ended.size(), 1u);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST_F(TrackerTest, AThingTooNearToSeeIsNotSeenFree) {
  Feed(1.0, {Thing(Kind::kObject, 2.0, 0.0)});
  Feed(1.2, {Thing(Kind::kObject, 2.0, 0.0)});
  // The mower drove up: the thing lies 0.2 m ahead of the camera, under its view.
  Stand(1.3, 0.0, 0.0);
  for (double t = 1.4; t < 5.0; t += 0.2) Feed(t);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_FALSE(tracker.tracks().begin()->second.in_view);
  // Right at the nose it lies in the stop zone: out of view it never runs out of time there.
  EXPECT_TRUE(tracker.Tick(20.0).empty());
  EXPECT_TRUE(tracker.Tick(31.3).empty());
  // Once the mower left it behind, it does.
  Stand(-2.0, 0.0, 0.0);
  pose.stamp = 31.4;
  tracker.OnPose(pose);
  EXPECT_EQ(tracker.Tick(31.4).size(), 1u);
}

TEST_F(TrackerTest, AThingOutOfViewAndOutOfTheZonesRunsOutOfTime) {
  Feed(1.0, {Thing(Kind::kObject, 3.5, 0.0)});
  Feed(1.2, {Thing(Kind::kObject, 3.5, 0.0)});
  // Turned away: the thing lies behind the view, farther than any zone.
  Stand(0.0, 0.0, M_PI);
  for (double t = 1.4; t < 3.0; t += 0.2) Feed(t);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.Tick(31.3).size(), 1u);
}

TEST_F(TrackerTest, EachTrackCarriesItsZone) {
  Find ball = Thing(Kind::kObject, 0.605 + 0.1 + 0.5, 0.0);
  Find dog = Thing(Kind::kAnimal, 0.605 + 0.1 + 1.5, 0.0);
  ball.cov_xx = ball.cov_yy = dog.cov_xx = dog.cov_yy = 0.0;
  dog.y = 0.6;
  for (const double t : {1.0, 1.2}) {
    pose.stamp = t;
    tracker.OnPose(pose);
    tracker.OnFrame(At(t, {ball, dog}), t);
  }
  const Status status = tracker.StatusAt(1.3);
  ASSERT_EQ(status.tracks.size(), 2u);
  for (const TrackView& view : status.tracks) {
    EXPECT_EQ(view.zone, view.track->kind == Kind::kObject ? Zone::kStop : Zone::kSlow);
  }
  EXPECT_TRUE(status.stop);
  EXPECT_EQ(status.stop_kind, Kind::kObject);
  EXPECT_TRUE(status.slow);
}

TEST_F(TrackerTest, WithoutSightNothingIsSeenFree) {
  Feed(1.0, {Thing(Kind::kObject, 2.0, 0.0)});
  Feed(1.2, {Thing(Kind::kObject, 2.0, 0.0)});
  for (double t = 1.4; t < 5.0; t += 0.2) {
    Frame frame = At(t);
    frame.sight = false;
    frame.reason = "TOO_DARK";
    tracker.OnFrame(frame, t);
  }
  EXPECT_EQ(tracker.tracks().size(), 1u);
  const Status status = tracker.StatusAt(5.0);
  EXPECT_FALSE(status.sight);
  EXPECT_EQ(status.reason, "TOO_DARK");
  EXPECT_EQ(status.source, "stereo");
}

TEST_F(TrackerTest, ASensorWithoutFramesHasNoSight) {
  const Status status = tracker.StatusAt(0.5);
  EXPECT_FALSE(status.sight);
  EXPECT_EQ(status.reason, "NO_SIGNAL");
  Feed(1.6);
  EXPECT_TRUE(tracker.StatusAt(1.7).sight);
  EXPECT_FALSE(tracker.StatusAt(2.7).sight);
}

TEST_F(TrackerTest, WithoutSensorsThereIsSight) {
  Tracker none;
  EXPECT_TRUE(none.StatusAt(100.0).sight);
}

TEST_F(TrackerTest, AnUnconfirmedThingNeverStops) {
  Feed(1.0, {Thing(Kind::kPerson, 1.0, 0.0)});
  const Status status = tracker.StatusAt(1.1);
  EXPECT_FALSE(status.stop);
  ASSERT_EQ(status.tracks.size(), 1u);
}

// Confirms one thing in base_link of the mower at this pose and returns the status.
Status Confirmed(Tracker& tracker, const Find& find, Pose pose = {}) {
  for (const double t : {1.0, 1.2}) {
    pose.stamp = t;
    tracker.OnPose(pose);
    tracker.OnFrame(At(t, {find}), t);
  }
  return tracker.StatusAt(1.3);
}

TEST_F(TrackerTest, AnObjectRightAheadStops) {
  Find ball = Thing(Kind::kObject, 0.605 + 0.1 + 0.8, 0.0);
  ball.cov_xx = ball.cov_yy = 0.0;
  const Status status = Confirmed(tracker, ball);
  EXPECT_TRUE(status.stop);
  EXPECT_EQ(status.stop_kind, Kind::kObject);
  EXPECT_FALSE(status.slow);
  ASSERT_EQ(status.tracks.size(), 1u);
  EXPECT_NEAR(status.tracks[0].ahead, 0.8, 1e-9);
}

TEST_F(TrackerTest, AnObjectBesideTheCorridorDoesNotStop) {
  Find ball = Thing(Kind::kObject, 1.5, 0.235 + 0.15 + 0.1 + 0.05);
  ball.cov_xx = ball.cov_yy = 0.0;
  EXPECT_FALSE(Confirmed(tracker, ball).stop);
}

TEST_F(TrackerTest, AnObjectFartherThanItsZoneDoesNotStop) {
  Find ball = Thing(Kind::kObject, 0.605 + 0.1 + 1.2, 0.0);
  ball.cov_xx = ball.cov_yy = 0.0;
  EXPECT_FALSE(Confirmed(tracker, ball).stop);
}

TEST_F(TrackerTest, TheSpreadOfAThingCountsNearer) {
  Find ball = Thing(Kind::kObject, 0.605 + 0.1 + 1.0, 0.0);
  ball.cov_xx = 0.09;
  ball.cov_yy = 0.0;
  EXPECT_TRUE(Confirmed(tracker, ball).stop);
}

TEST_F(TrackerTest, APersonStopsFartherAndSlowsFartherStill) {
  Find person = Thing(Kind::kPerson, 0.605 + 0.1 + 0.9, 0.5);
  person.cov_xx = person.cov_yy = 0.0;
  Status status = Confirmed(tracker, person);
  EXPECT_TRUE(status.stop);
  EXPECT_EQ(status.stop_kind, Kind::kPerson);

  Tracker far;
  far.SetSources({"stereo"});
  far.SetBody({0.605, 0.12, 0.235});
  person.x = 0.605 + 0.1 + 1.8;
  status = Confirmed(far, person);
  EXPECT_FALSE(status.stop);
  EXPECT_TRUE(status.slow);
}

TEST_F(TrackerTest, AnAnimalBesideTheBodyStops) {
  Find animal = Thing(Kind::kAnimal, 0.3, 0.235 + 0.1 + 0.8);
  animal.cov_xx = animal.cov_yy = 0.0;
  // Outside the view of the camera, but a find is a find.
  const Status status = Confirmed(tracker, animal);
  EXPECT_TRUE(status.stop);
  EXPECT_FALSE(status.slow);
}

TEST_F(TrackerTest, DepthAloneStopsOnlyWhenAsked) {
  Find lump = Thing(Kind::kUnknown, 1.3, 0.0);
  lump.cov_xx = lump.cov_yy = 0.0;
  EXPECT_FALSE(Confirmed(tracker, lump).stop);
  TrackerConfig config;
  config.unknown_stops = true;
  Tracker strict(config);
  strict.SetSources({"stereo"});
  strict.SetBody({0.605, 0.12, 0.235});
  EXPECT_TRUE(Confirmed(strict, lump).stop);
}

TEST_F(TrackerTest, TheNearestOfSeveralStopsIsNamed) {
  Find near = Thing(Kind::kObject, 1.2, 0.0);
  Find person = Thing(Kind::kPerson, 2.0, 0.3);
  near.cov_xx = near.cov_yy = person.cov_xx = person.cov_yy = 0.0;
  Feed(1.0, {near, person});
  Feed(1.2, {near, person});
  const Status status = tracker.StatusAt(1.3);
  EXPECT_TRUE(status.stop);
  EXPECT_EQ(status.stop_kind, Kind::kObject);
}

TEST_F(TrackerTest, TheZonesFollowTheHeadingOfTheMower) {
  Tracker turned;
  turned.SetSources({"stereo"});
  turned.SetBody({0.605, 0.12, 0.235});
  Find ball = Thing(Kind::kObject, 1.3, 0.0);
  ball.cov_xx = ball.cov_yy = 0.0;
  EXPECT_TRUE(Confirmed(turned, ball, {0.0, 4.0, 4.0, 90.0 * kDeg, 0.0}).stop);
  // Now it faces the other way: the ball lies behind it.
  turned.OnPose({1.4, 4.0, 4.0, -90.0 * kDeg, 0.0});
  EXPECT_FALSE(turned.StatusAt(1.5).stop);
}

TEST(Inside, ADiscCountsOnlyWhollyInsideTheSector) {
  const ViewSector view = Front();
  EXPECT_TRUE(Inside(view, 2.0, 0.0, 0.1));
  EXPECT_FALSE(Inside(view, 0.466 + 0.45, 0.0, 0.1)) << "reaches under range_min";
  EXPECT_FALSE(Inside(view, 0.466 + 3.95, 0.0, 0.1)) << "reaches past range_max";
  const double edge = 43.0 * kDeg;
  EXPECT_FALSE(Inside(view, 0.466 + 2.0 * std::cos(edge), 2.0 * std::sin(edge), 0.05));
  EXPECT_TRUE(Inside(view, 0.466 + 2.0 * std::cos(edge - 0.1), 2.0 * std::sin(edge - 0.1), 0.05));
}

TEST(Inside, ASectorAcrossTheBackWorks) {
  const ViewSector rear{-0.2, 0.0, -150.0 * kDeg, 150.0 * kDeg, 0.3, 3.0};
  EXPECT_TRUE(Inside(rear, -2.0, 0.0, 0.1));
  EXPECT_FALSE(Inside(rear, 2.0, 0.0, 0.1));
  EXPECT_FALSE(Inside(rear, -0.2, 2.0, 0.1));
}

TEST(Kinds, NamesGoBothWays) {
  for (Kind kind : {Kind::kPerson, Kind::kAnimal, Kind::kObject, Kind::kUnknown}) {
    Kind parsed;
    ASSERT_TRUE(ParseKind(KindName(kind), &parsed));
    EXPECT_EQ(parsed, kind);
  }
  Kind parsed;
  EXPECT_FALSE(ParseKind("dog", &parsed));
}

}  // namespace
TEST_F(TrackerTest, AThingTheEdgeOfTheViewCutsIsNotSeenFree) {
  // At 37 deg from the camera its disc lies in the view, with the margin and its spread it reaches past 43 deg.
  const double d = 1.5, angle = 37.0 * kDeg;
  Find dog = Thing(Kind::kAnimal, 0.466 + d * std::cos(angle), d * std::sin(angle));
  Feed(1.0, {dog});
  Feed(1.2, {dog});
  for (double t = 1.4; t < 5.0; t += 0.2) EXPECT_TRUE(Feed(t).ended.empty()) << t;
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_FALSE(tracker.tracks().begin()->second.in_view);
}

TEST_F(TrackerTest, AThingThatStopsLeavesTheZoneOnlyWellBeyondItsEdge) {
  // A ball right on the edge of the stop zone of objects, 0.8 m ahead of the nose.
  Find ball = Thing(Kind::kObject, 0.605 + 0.1 + 0.79, 0.0);
  ball.cov_xx = ball.cov_yy = 0.0;
  ASSERT_TRUE(Confirmed(tracker, ball).stop);
  tracker.Tick(1.3);
  // The pose wanders back by 5 cm: it still stops.
  pose = {1.4, -0.05, 0.0, 0.0, 0.0};
  tracker.OnPose(pose);
  tracker.Tick(1.4);
  EXPECT_TRUE(tracker.StatusAt(1.4).stop);
  // Beyond the release it lets go, and needs the zone itself again to stop.
  pose = {1.5, -0.25, 0.0, 0.0, 0.0};
  tracker.OnPose(pose);
  tracker.Tick(1.5);
  EXPECT_FALSE(tracker.StatusAt(1.5).stop);
  pose = {1.6, -0.05, 0.0, 0.0, 0.0};
  tracker.OnPose(pose);
  tracker.Tick(1.6);
  EXPECT_FALSE(tracker.StatusAt(1.6).stop);
}

}  // namespace mowbite_obstacles
