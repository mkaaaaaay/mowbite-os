#include <gtest/gtest.h>

#include <algorithm>

#include "tron_vision/dock_exposure.h"

namespace tron_vision {
namespace {

DockExposureLimits Limits() {
  DockExposureLimits limits;
  limits.range = {2, 50, 16, 255};
  return limits;
}

DockExposureState Settled(int exposure, int gain) {
  DockExposureState state;
  state.setting = {exposure, gain};
  state.since_change = 100;
  return state;
}

MarkerDetection Marker(double white, double saturated = 0.0) {
  MarkerDetection seen;
  seen.result = MarkerResult::kFound;
  seen.side_px = 80.0;
  seen.white = white;
  seen.saturated = saturated;
  return seen;
}

MarkerDetection Nothing(MarkerResult result = MarkerResult::kNoMarker, double image_saturated = 0.0) {
  MarkerDetection seen;
  seen.result = result;
  seen.image_saturated = image_saturated;
  return seen;
}

double Light(const DockExposureState& state) {
  return static_cast<double>(state.setting.exposure) * state.setting.gain;
}

TEST(DockExposure, WhiteInsideTheBandChangesNothing) {
  const DockExposureState next = NextDockExposure(Settled(20, 16), Marker(185.0), Limits());
  EXPECT_EQ(next.setting.exposure, 20);
  EXPECT_EQ(next.setting.gain, 16);
  EXPECT_EQ(next.since_change, 101);
}

TEST(DockExposure, FollowsTheWhiteOfTheMarkerNotTheImage) {
  // A dark image around a glowing marker: only the white of the marker counts.
  MarkerDetection seen = Marker(240.0);
  seen.image_mean = 6.0;
  DockExposureState next = NextDockExposure(Settled(40, 16), seen, Limits());
  EXPECT_EQ(next.setting.exposure, 28);  // 40 * 170 / 240
  EXPECT_EQ(next.setting.gain, 16);
  EXPECT_EQ(next.since_change, 0);
  // Too dim: at most twice the light, the exposure first, the gain only above the longest exposure.
  next = NextDockExposure(Settled(20, 16), Marker(60.0), Limits());
  EXPECT_EQ(next.setting.exposure, 40);
  EXPECT_EQ(next.setting.gain, 16);
  next = NextDockExposure(Settled(40, 16), Marker(60.0), Limits());
  EXPECT_EQ(next.setting.exposure, 50);
  EXPECT_EQ(next.setting.gain, 26);  // 40 * 16 * 2 / 50
}

TEST(DockExposure, SaturatedWhiteGoesDownAFullStep) {
  // The value read from saturated white says nothing about how far it is over.
  const DockExposureState next = NextDockExposure(Settled(40, 16), Marker(180.0, 0.5), Limits());
  EXPECT_EQ(next.setting.exposure, 20);
  MarkerDetection seen = Marker(250.0, 1.0);
  seen.result = MarkerResult::kTooBright;
  EXPECT_EQ(NextDockExposure(Settled(40, 16), seen, Limits()).setting.exposure, 20);
}

TEST(DockExposure, WaitsUntilANewSettingShows) {
  DockExposureState state = Settled(40, 16);
  state.since_change = 0;
  for (int i = 0; i < 2; i++) {
    state = NextDockExposure(state, Marker(250.0), Limits());
    EXPECT_EQ(state.setting.exposure, 40);
  }
  state = NextDockExposure(state, Marker(250.0), Limits());
  EXPECT_EQ(state.setting.exposure, 27);
}

TEST(DockExposure, SearchBeginsAfterSomeImagesWithoutAMarker) {
  DockExposureState state = Settled(20, 16);
  for (int i = 0; i < 4; i++) {
    state = NextDockExposure(state, Nothing(), Limits());
    EXPECT_EQ(state.setting.exposure, 20) << i;
  }
  state = NextDockExposure(state, Nothing(), Limits());
  EXPECT_EQ(state.setting.exposure, 10);
  // A marker in between starts the count again.
  state.since_change = 100;
  state = NextDockExposure(state, Marker(170.0), Limits());
  EXPECT_EQ(state.without_marker, 0);
  state = NextDockExposure(state, Nothing(), Limits());
  EXPECT_EQ(state.setting.exposure, 10);
}

TEST(DockExposure, SearchStartsAwayFromWhatIsWrongWithTheImage) {
  DockExposureState state = Settled(20, 16);
  state.without_marker = 4;
  state.search_down = true;
  EXPECT_EQ(NextDockExposure(state, Nothing(MarkerResult::kTooDark), Limits()).setting.exposure, 40);
  // Dark image with a saturated spot, the glowing marker at night: down first.
  EXPECT_EQ(NextDockExposure(state, Nothing(MarkerResult::kTooDark, 0.002), Limits()).setting.exposure, 10);
  state.search_down = false;
  EXPECT_EQ(NextDockExposure(state, Nothing(MarkerResult::kTooBright), Limits()).setting.exposure, 10);
  // Nothing wrong with the image: on in the direction of the last search.
  EXPECT_EQ(NextDockExposure(state, Nothing(), Limits()).setting.exposure, 40);
}

TEST(DockExposure, SearchWalksThroughTheWholeRangeAndBack) {
  DockExposureState state = Settled(20, 16);
  double least = Light(state);
  double most = Light(state);
  int turns = 0;
  bool down = state.search_down;
  for (int i = 0; i < 200; i++) {
    // The image stays too dark all the time: the hint counts only when the search begins.
    state = NextDockExposure(state, Nothing(MarkerResult::kTooDark, 0.002), Limits());
    least = std::min(least, Light(state));
    most = std::max(most, Light(state));
    if (state.search_down != down) turns++;
    down = state.search_down;
  }
  EXPECT_EQ(least, 2.0 * 16.0);
  EXPECT_EQ(most, 50.0 * 255.0);
  EXPECT_GE(turns, 3);
}

TEST(DockExposure, AMarkerCutOffOrNotAloneLeavesTheSettingAlone) {
  for (const MarkerResult result : {MarkerResult::kClipped, MarkerResult::kAmbiguous}) {
    DockExposureState state = Settled(20, 16);
    state.without_marker = 3;
    for (int i = 0; i < 20; i++) state = NextDockExposure(state, Nothing(result), Limits());
    EXPECT_EQ(state.setting.exposure, 20);
    EXPECT_EQ(state.setting.gain, 16);
    EXPECT_EQ(state.without_marker, 0);
  }
}

TEST(DockExposure, WithoutAValidRangeNothingChanges) {
  DockExposureLimits limits;
  const DockExposureState next = NextDockExposure(Settled(20, 16), Marker(60.0), limits);
  EXPECT_EQ(next.setting.exposure, 20);
  EXPECT_EQ(next.setting.gain, 16);
}

TEST(DockExposure, ASettingOutsideTheRangeIsBroughtIn) {
  const DockExposureState next = NextDockExposure(Settled(500, 1), Marker(60.0), Limits());
  EXPECT_EQ(next.setting.exposure, 50);
  EXPECT_EQ(next.setting.gain, 32);  // 50 * 16 doubled
}

}  // namespace
}  // namespace tron_vision
