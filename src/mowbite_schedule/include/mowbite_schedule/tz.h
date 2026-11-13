#ifndef MOWBITE_SCHEDULE_TZ_H
#define MOWBITE_SCHEDULE_TZ_H

#include <cstdint>
#include <optional>
#include <string>

namespace mowbite_schedule {

// A day of the calendar.
struct Date {
  int year = 1970;
  int month = 1;
  int day = 1;
  bool operator==(const Date& other) const { return year == other.year && month == other.month && day == other.day; }
  bool operator!=(const Date& other) const { return !(*this == other); }
  bool operator<(const Date& other) const;
};

// Days since 1970-01-01 and back.
int64_t DaysFromDate(const Date& date);
Date DateFromDays(int64_t days);
// 1 Monday to 7 Sunday, as date +%u.
int Weekday(const Date& date);
Date AddDays(const Date& date, int64_t days);
// "YYYY-MM-DD", strict; nothing for another form or a day the month does not have.
std::optional<Date> ParseDate(const std::string& text);
std::string FormatDate(const Date& date);
// "HH:MM" 00:00 to 23:59, strict; minutes of the day.
std::optional<int> ParseClockTime(const std::string& text);
std::string FormatClockTime(int minutes);

// Local date and time of a moment.
struct LocalTime {
  Date date;
  int seconds = 0;  // since local midnight
  int weekday = 4;
};

// A POSIX TZ string, read without setenv or localtime: std offset [dst [offset] [,rule]] with rules of the form
// Mm.w.d[/time] only. The offsets in the text count west of Greenwich, those kept here east.
class TimeZone {
 public:
  // The text, or the reason it is not taken: syntax, missing_rule (summer time named without a rule),
  // unsupported_rule (Jn and n) or range.
  static std::optional<TimeZone> Parse(const std::string& text, std::string* reason);
  static TimeZone Utc();

  // Seconds east of UTC at this moment.
  int64_t OffsetAt(int64_t unix_s) const;
  LocalTime ToLocal(int64_t unix_s) const;
  // The moment of a local date and minute of the day. A time in the hour skipped in spring is taken as the end of
  // the gap; one in the hour that repeats in autumn as its first occurrence.
  int64_t ToUnix(const Date& date, int minutes) const;

 private:
  struct Rule {
    int month = 0;
    int week = 0;
    int weekday = 0;  // 0 Sunday to 6 Saturday
    int64_t time = 7200;
  };

  // The change into summer time and back in that year, in UTC.
  int64_t StartOf(int year) const;
  int64_t EndOf(int year) const;

  int64_t std_east_ = 0;
  bool has_dst_ = false;
  int64_t dst_east_ = 0;
  Rule start_;
  Rule end_;
};

}  // namespace mowbite_schedule

#endif  // MOWBITE_SCHEDULE_TZ_H
