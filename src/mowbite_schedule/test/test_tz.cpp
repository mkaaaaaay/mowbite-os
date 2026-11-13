#include <gtest/gtest.h>

#include <string>

#include "mowbite_schedule/tz.h"

namespace mowbite_schedule {
namespace {

constexpr char kBerlin[] = "CET-1CEST,M3.5.0,M10.5.0/3";

int64_t Utc(int year, int month, int day, int hour, int minute) {
  return DaysFromDate(Date{year, month, day}) * 86400 + hour * 3600 + minute * 60;
}

TimeZone Zone(const std::string& text) {
  std::string reason;
  const std::optional<TimeZone> zone = TimeZone::Parse(text, &reason);
  EXPECT_TRUE(zone) << text << ": " << reason;
  return zone ? *zone : TimeZone::Utc();
}

std::string Refused(const std::string& text) {
  std::string reason;
  return TimeZone::Parse(text, &reason) ? "taken" : reason;
}

TEST(Calendar, DaysAndDatesBothWays) {
  EXPECT_EQ(DaysFromDate(Date{1970, 1, 1}), 0);
  EXPECT_EQ(DaysFromDate(Date{2000, 3, 1}), 11017);
  EXPECT_EQ(DaysFromDate(Date{1969, 12, 31}), -1);
  for (int64_t days = -800000; days < 800000; days += 997) {
    EXPECT_EQ(DaysFromDate(DateFromDays(days)), days);
  }
  EXPECT_EQ(DateFromDays(DaysFromDate(Date{2024, 2, 29})), (Date{2024, 2, 29}));
}

TEST(Calendar, WeekdayCountsFromMonday) {
  EXPECT_EQ(Weekday(Date{1970, 1, 1}), 4);
  EXPECT_EQ(Weekday(Date{2026, 11, 13}), 5);
  EXPECT_EQ(Weekday(Date{2026, 11, 15}), 7);
  EXPECT_EQ(Weekday(Date{2026, 11, 16}), 1);
  EXPECT_EQ(Weekday(Date{1969, 12, 31}), 3);
}

TEST(Calendar, DatesAndTimesAreStrict) {
  EXPECT_TRUE(ParseDate("2024-02-29"));
  EXPECT_FALSE(ParseDate("2026-02-29"));
  EXPECT_FALSE(ParseDate("2026-13-01"));
  EXPECT_FALSE(ParseDate("2026-1-01"));
  EXPECT_FALSE(ParseDate("2026-01-01 "));
  EXPECT_EQ(FormatDate(Date{2026, 3, 9}), "2026-03-09");
  EXPECT_EQ(ParseClockTime("00:00"), 0);
  EXPECT_EQ(ParseClockTime("23:59"), 23 * 60 + 59);
  EXPECT_FALSE(ParseClockTime("24:00"));
  EXPECT_FALSE(ParseClockTime("9:00"));
  EXPECT_FALSE(ParseClockTime("09:60"));
  EXPECT_FALSE(ParseClockTime("09-00"));
  EXPECT_EQ(FormatClockTime(9 * 60 + 5), "09:05");
}

TEST(TimeZoneForms, Taken) {
  EXPECT_EQ(Zone("UTC0").OffsetAt(0), 0);
  EXPECT_EQ(Zone("<+01>-1").OffsetAt(0), 3600);
  EXPECT_EQ(Zone("<-0330>3:30").OffsetAt(0), -12600);
  EXPECT_EQ(Zone("EST5EDT,M3.2.0,M11.1.0").OffsetAt(Utc(2026, 1, 15, 12, 0)), -5 * 3600);
  EXPECT_EQ(Zone("EST5EDT,M3.2.0,M11.1.0").OffsetAt(Utc(2026, 7, 15, 12, 0)), -4 * 3600);
  // Greenland: changes at -1 h and 0 h local.
  EXPECT_EQ(Zone("<-02>2<-01>,M3.5.0/-1,M10.5.0/0").OffsetAt(Utc(2026, 7, 1, 0, 0)), -3600);
  // Times up to 167 h.
  EXPECT_EQ(Refused("<+02>-2<+03>,M3.4.4/50,M10.4.4/50"), "taken");
  EXPECT_EQ(Refused("<+02>-2<+03>,M3.4.4/-167,M10.4.4/167"), "taken");
}

TEST(TimeZoneForms, Refused) {
  EXPECT_EQ(Refused(""), "syntax");
  EXPECT_EQ(Refused(":Europe/Berlin"), "syntax");
  EXPECT_EQ(Refused("Europe/Berlin"), "syntax");
  EXPECT_EQ(Refused("AB0"), "syntax");
  EXPECT_EQ(Refused("CET"), "syntax");
  EXPECT_EQ(Refused("CET-1CEST,M3.5.0"), "syntax");
  EXPECT_EQ(Refused("CET-1CEST,M3.5.0,M10.5.0x"), "syntax");
  EXPECT_EQ(Refused("EST5EDT"), "missing_rule");
  EXPECT_EQ(Refused("EST5EDT4"), "missing_rule");
  EXPECT_EQ(Refused("EST5EDT,J60,J300"), "unsupported_rule");
  EXPECT_EQ(Refused("EST5EDT,60,300"), "unsupported_rule");
  EXPECT_EQ(Refused("CET-25"), "range");
  EXPECT_EQ(Refused("CET-1:60"), "range");
  EXPECT_EQ(Refused("CET-1CEST,M13.5.0,M10.5.0"), "range");
  EXPECT_EQ(Refused("CET-1CEST,M3.6.0,M10.5.0"), "range");
  EXPECT_EQ(Refused("CET-1CEST,M3.5.7,M10.5.0"), "range");
  EXPECT_EQ(Refused("CET-1CEST,M3.5.0/168,M10.5.0"), "range");
  EXPECT_EQ(Refused("<+0100000000000000000000000000000000000000000000000000000000000>-1"), "range");
}

TEST(TimeZoneOffsets, BerlinChangesOnTheLastSundays) {
  const TimeZone berlin = Zone(kBerlin);
  // Summer time 2026 from 29 March 01:00 UTC to 25 October 01:00 UTC.
  EXPECT_EQ(berlin.OffsetAt(Utc(2026, 3, 29, 0, 59) + 59), 3600);
  EXPECT_EQ(berlin.OffsetAt(Utc(2026, 3, 29, 1, 0)), 7200);
  EXPECT_EQ(berlin.OffsetAt(Utc(2026, 10, 25, 0, 59) + 59), 7200);
  EXPECT_EQ(berlin.OffsetAt(Utc(2026, 10, 25, 1, 0)), 3600);
  const LocalTime local = berlin.ToLocal(Utc(2026, 6, 15, 8, 0));
  EXPECT_EQ(local.date, (Date{2026, 6, 15}));
  EXPECT_EQ(local.seconds, 10 * 3600);
  EXPECT_EQ(local.weekday, 1);
}

TEST(TimeZoneOffsets, SouthernHemisphere) {
  const TimeZone sydney = Zone("AEST-10AEDT,M10.1.0,M4.1.0/3");
  EXPECT_EQ(sydney.OffsetAt(Utc(2026, 1, 15, 0, 0)), 11 * 3600);
  EXPECT_EQ(sydney.OffsetAt(Utc(2026, 7, 15, 0, 0)), 10 * 3600);
  // Back to standard time on 5 April 2026 at 03:00 local summer time.
  EXPECT_EQ(sydney.OffsetAt(Utc(2026, 4, 4, 15, 59)), 11 * 3600);
  EXPECT_EQ(sydney.OffsetAt(Utc(2026, 4, 4, 16, 0)), 10 * 3600);
  // New year inside summer time.
  EXPECT_EQ(sydney.ToLocal(Utc(2026, 12, 31, 13, 30)).date, (Date{2027, 1, 1}));
}

TEST(TimeZoneOffsets, DublinWithNegativeSummerTime) {
  const TimeZone dublin = Zone("IST-1GMT0,M10.5.0,M3.5.0/1");
  EXPECT_EQ(dublin.OffsetAt(Utc(2026, 1, 15, 12, 0)), 0);
  EXPECT_EQ(dublin.OffsetAt(Utc(2026, 7, 15, 12, 0)), 3600);
  EXPECT_EQ(dublin.OffsetAt(Utc(2026, 3, 29, 0, 59)), 0);
  EXPECT_EQ(dublin.OffsetAt(Utc(2026, 3, 29, 1, 0)), 3600);
}

TEST(TimeZoneOffsets, ChathamWithQuarterHours) {
  const TimeZone chatham = Zone("<+1245>-12:45<+1345>,M9.5.0/2:45,M4.1.0/3:45");
  EXPECT_EQ(chatham.OffsetAt(Utc(2026, 1, 15, 0, 0)), 13 * 3600 + 45 * 60);
  EXPECT_EQ(chatham.OffsetAt(Utc(2026, 7, 15, 0, 0)), 12 * 3600 + 45 * 60);
}

TEST(TimeZoneOffsets, LocalTimeToUnixInTheGapAndTheRepeatedHour) {
  const TimeZone berlin = Zone(kBerlin);
  EXPECT_EQ(berlin.ToUnix(Date{2026, 6, 15}, 10 * 60), Utc(2026, 6, 15, 8, 0));
  EXPECT_EQ(berlin.ToUnix(Date{2026, 1, 15}, 10 * 60), Utc(2026, 1, 15, 9, 0));
  // 02:30 does not exist on 29 March: the end of the gap, 03:00 summer time.
  EXPECT_EQ(berlin.ToUnix(Date{2026, 3, 29}, 2 * 60 + 30), Utc(2026, 3, 29, 1, 0));
  // 02:30 comes twice on 25 October: the first time, in summer time.
  EXPECT_EQ(berlin.ToUnix(Date{2026, 10, 25}, 2 * 60 + 30), Utc(2026, 10, 25, 0, 30));
  const TimeZone dublin = Zone("IST-1GMT0,M10.5.0,M3.5.0/1");
  EXPECT_EQ(dublin.ToUnix(Date{2026, 3, 29}, 1 * 60 + 30), Utc(2026, 3, 29, 1, 0));
  EXPECT_EQ(dublin.ToUnix(Date{2026, 10, 25}, 1 * 60 + 30), Utc(2026, 10, 25, 0, 30));
}

}  // namespace
}  // namespace mowbite_schedule
