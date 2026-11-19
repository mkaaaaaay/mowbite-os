#include <gtest/gtest.h>

#include "mowbite_obstacles/obstacle_list.h"

namespace mowbite_obstacles {
namespace {

Track Confirmed(uint32_t id, Kind kind, double x, double y, double first, double last) {
  Track track;
  track.id = id;
  track.kind = kind;
  track.class_name = kind == Kind::kPerson ? "person" : "ball";
  track.score = 0.8;
  track.x = x;
  track.y = y;
  track.radius = 0.1;
  track.confirmed = true;
  track.first_seen = first;
  track.last_seen = last;
  track.source = "stereo";
  return track;
}

Photo PhotoOf(uint32_t id, int64_t bytes) {
  Photo photo;
  photo.name = "o" + std::to_string(id) + ".jpg";
  photo.sha256 = std::string(64, 'a');
  photo.bytes = bytes;
  photo.width = 640;
  photo.height = 480;
  photo.box_w = 20;
  photo.box_h = 30;
  return photo;
}

TEST(ObstacleList, AConfirmedTrackIsANewEntry) {
  ObstacleList list;
  const auto seen = list.OnConfirmed(Confirmed(7, Kind::kObject, 1.234, 2.0, 100.0, 100.4));
  EXPECT_TRUE(seen.added);
  const Entry* entry = list.Get(seen.id);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->t, 100.0);
  EXPECT_EQ(entry->last_t, 100.4);
  EXPECT_DOUBLE_EQ(entry->x, 1.23);
  EXPECT_EQ(entry->camera_id, "stereo");
  EXPECT_EQ(list.EntryOf(7), seen.id);
  EXPECT_EQ(list.OnConfirmed(Confirmed(7, Kind::kObject, 1.2, 2.0, 100.0, 101.0)).id, seen.id);
}

TEST(ObstacleList, AnObjectAtThePlaceOfAnEntryIsThatEntry) {
  ObstacleList list;
  const auto first = list.OnConfirmed(Confirmed(1, Kind::kObject, 1.0, 1.0, 100.0, 101.0));
  list.ForgetTrack(1);
  const auto again = list.OnConfirmed(Confirmed(2, Kind::kObject, 1.3, 1.2, 5000.0, 5001.0));
  EXPECT_FALSE(again.added);
  EXPECT_EQ(again.id, first.id);
  EXPECT_EQ(list.Get(first.id)->count, 2);
  EXPECT_EQ(list.Get(first.id)->last_t, 5001.0);
  EXPECT_EQ(list.Get(first.id)->t, 100.0);
  EXPECT_TRUE(list.OnConfirmed(Confirmed(3, Kind::kObject, 2.0, 1.0, 5000.0, 5001.0)).added);
  EXPECT_TRUE(list.OnConfirmed(Confirmed(4, Kind::kPerson, 1.0, 1.0, 5000.0, 5001.0)).added);
}

TEST(ObstacleList, APersonWithinHalfAMinuteIsTheSameEntry) {
  ObstacleList list;
  const auto first = list.OnConfirmed(Confirmed(1, Kind::kPerson, 1.0, 1.0, 100.0, 101.0));
  EXPECT_EQ(list.OnConfirmed(Confirmed(2, Kind::kPerson, 9.0, 9.0, 120.0, 125.0)).id, first.id);
  EXPECT_DOUBLE_EQ(list.Get(first.id)->x, 9.0);
  EXPECT_TRUE(list.OnConfirmed(Confirmed(3, Kind::kPerson, 9.0, 9.0, 160.0, 160.0)).added);
}

TEST(ObstacleList, AnEntryFollowsItsTrack) {
  ObstacleList list;
  Track track = Confirmed(1, Kind::kPerson, 1.0, 1.0, 100.0, 100.0);
  const auto seen = list.OnConfirmed(track);
  EXPECT_FALSE(list.Follow(track));
  track.x = 2.0;
  track.last_seen = 103.0;
  EXPECT_TRUE(list.Follow(track));
  EXPECT_EQ(list.Get(seen.id)->last_t, 103.0);
  EXPECT_DOUBLE_EQ(list.Get(seen.id)->x, 2.0);
}

TEST(ObstacleList, ThePhotoOfAnEntryStays) {
  ObstacleList list;
  const auto seen = list.OnConfirmed(Confirmed(1, Kind::kObject, 1.0, 1.0, 100.0, 100.0));
  EXPECT_TRUE(list.SetPhoto(seen.id, PhotoOf(seen.id, 1000)));
  EXPECT_FALSE(list.SetPhoto(seen.id, PhotoOf(99, 1000)));
  EXPECT_EQ(list.ByPhoto("o1.jpg"), seen.id);
  EXPECT_EQ(list.ByPhoto("../list.json"), 0u);
}

TEST(ObstacleList, TheLeastRecentlySeenGoFirst) {
  ListLimits limits;
  limits.entries = 2;
  ObstacleList list(limits);
  list.OnConfirmed(Confirmed(1, Kind::kObject, 0.0, 0.0, 100.0, 300.0));
  list.OnConfirmed(Confirmed(2, Kind::kObject, 5.0, 0.0, 200.0, 200.0));
  list.SetPhoto(2, PhotoOf(2, 10));
  list.OnConfirmed(Confirmed(3, Kind::kObject, 9.0, 0.0, 250.0, 250.0));
  const std::vector<std::string> gone = list.Prune(0.0);
  EXPECT_EQ(gone, std::vector<std::string>{"o2.jpg"});
  EXPECT_NE(list.Get(1), nullptr);
  EXPECT_EQ(list.Get(2), nullptr);
  EXPECT_EQ(list.EntryOf(2), 0u);
}

TEST(ObstacleList, PhotosCountToTheBytes) {
  ListLimits limits;
  limits.bytes = 3 * kEntryBytes + 1500;
  ObstacleList list(limits);
  for (uint32_t i = 1; i <= 3; ++i) {
    list.OnConfirmed(Confirmed(i, Kind::kObject, 2.0 * i, 0.0, 100.0 * i, 100.0 * i));
    list.SetPhoto(i, PhotoOf(i, 1000));
  }
  EXPECT_EQ(list.Prune(0.0).size(), 2u);
  EXPECT_EQ(list.size(), 1u);
  EXPECT_NE(list.Get(3), nullptr);
}

TEST(ObstacleList, OldEntriesGoOnlyWithAClock) {
  ObstacleList list;
  list.OnConfirmed(Confirmed(1, Kind::kObject, 0.0, 0.0, 1.0e9, 1.0e9));
  list.OnConfirmed(Confirmed(2, Kind::kObject, 5.0, 0.0, 1.0e9 + 13 * 86400.0, 1.0e9 + 13 * 86400.0));
  list.Prune(0.0);
  EXPECT_EQ(list.size(), 2u);
  list.Prune(1.0e9 + 14.5 * 86400.0);
  EXPECT_EQ(list.size(), 1u);
  EXPECT_NE(list.Get(2), nullptr);
}

TEST(ObstacleList, RemoveTakesOneOrAll) {
  ObstacleList list;
  list.OnConfirmed(Confirmed(1, Kind::kObject, 0.0, 0.0, 1.0, 1.0));
  list.OnConfirmed(Confirmed(2, Kind::kObject, 5.0, 0.0, 2.0, 2.0));
  list.SetPhoto(1, PhotoOf(1, 10));
  EXPECT_EQ(list.Remove(1), std::vector<std::string>{"o1.jpg"});
  EXPECT_EQ(list.size(), 1u);
  EXPECT_TRUE(list.Remove(0).empty());
  EXPECT_EQ(list.size(), 0u);
  // Ids are not given twice.
  EXPECT_EQ(list.OnConfirmed(Confirmed(3, Kind::kObject, 0.0, 0.0, 3.0, 3.0)).id, 3u);
}

TEST(ObstacleList, TheListIsLastSeenFirstAndSince) {
  ObstacleList list;
  list.OnConfirmed(Confirmed(1, Kind::kObject, 0.0, 0.0, 100.0, 300.0));
  list.OnConfirmed(Confirmed(2, Kind::kObject, 5.0, 0.0, 200.0, 200.0));
  const Json all = list.ToJson(0.0);
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0]["id"], 1);
  EXPECT_EQ(all[0]["kind"], "OBJECT");
  EXPECT_EQ(all[0]["class"], "ball");
  EXPECT_TRUE(all[0]["image"].is_null());
  EXPECT_EQ(all[0]["action"], "seen");
  EXPECT_EQ(list.ToJson(250.0).size(), 1u);
}

TEST(ObstacleList, TheTextKeepsEverything) {
  ObstacleList list;
  list.OnConfirmed(Confirmed(1, Kind::kPerson, 1.0, 2.0, 100.0, 130.0));
  list.SetPhoto(1, PhotoOf(1, 1234));
  list.OnConfirmed(Confirmed(2, Kind::kObject, 5.0, 0.0, 200.0, 200.0));
  list.Remove(2);
  ObstacleList read;
  ASSERT_TRUE(read.Load(list.Text()));
  EXPECT_EQ(read.ToJson(0.0), list.ToJson(0.0));
  EXPECT_EQ(read.OnConfirmed(Confirmed(9, Kind::kObject, 50.0, 0.0, 300.0, 300.0)).id, 3u);
  EXPECT_FALSE(read.Load("[]"));
  EXPECT_FALSE(read.Load("no json"));
  EXPECT_TRUE(read.Load(R"({"next_id": 4, "entries": [{"id": 1}, 7]})"));
  EXPECT_EQ(read.size(), 0u);
}

// A file changed by hand: a next_id of another type is no crash, a photo of another name is not taken.
TEST(ObstacleList, ATextOfOtherValuesLoadsWhatFits) {
  ObstacleList list;
  list.OnConfirmed(Confirmed(1, Kind::kObject, 1.0, 2.0, 100.0, 130.0));
  list.SetPhoto(1, PhotoOf(1, 1234));
  Json text = Json::parse(list.Text());
  text["next_id"] = "seven";
  text["entries"][0]["image"]["name"] = "../setup.json";
  ObstacleList read;
  ASSERT_TRUE(read.Load(text.dump()));
  ASSERT_NE(read.Get(1), nullptr);
  EXPECT_FALSE(read.Get(1)->photo.has_value());
  EXPECT_EQ(read.OnConfirmed(Confirmed(9, Kind::kObject, 50.0, 0.0, 300.0, 300.0)).id, 2u);
}

}  // namespace
}  // namespace mowbite_obstacles
