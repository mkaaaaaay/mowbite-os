#include "mowbite_schedule/tz.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <utility>
#include <vector>

namespace mowbite_schedule {

namespace {

constexpr int64_t kDay = 86400;
constexpr size_t kMaxTzLength = 64;

int64_t FloorDiv(int64_t a, int64_t b) {
  return a / b - ((a % b != 0) && ((a < 0) != (b < 0)) ? 1 : 0);
}

bool LeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

int DaysInMonth(int year, int month) {
  static const int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return month == 2 && LeapYear(year) ? 29 : kDays[month - 1];
}

// Reads a POSIX TZ string from left to right; the first problem stays in reason.
class TzReader {
 public:
  explicit TzReader(const std::string& text) : text_(text) {}

  bool Done() const { return pos_ == text_.size(); }
  char Peek() const { return Done() ? '\0' : text_[pos_]; }
  bool Take(char c) {
    if (Peek() != c) return false;
    ++pos_;
    return true;
  }
  bool Fail(const char* reason) {
    if (reason_.empty()) reason_ = reason;
    return false;
  }
  const std::string& reason() const { return reason_; }

  // At least three letters, or letters, digits, + and - in angle brackets.
  bool Name() {
    size_t length = 0;
    if (Take('<')) {
      while (!Done() && Peek() != '>') {
        const char c = Peek();
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '+' && c != '-') return Fail("syntax");
        ++pos_;
        ++length;
      }
      if (!Take('>')) return Fail("syntax");
    } else {
      while (std::isalpha(static_cast<unsigned char>(Peek()))) {
        ++pos_;
        ++length;
      }
    }
    return length >= 3 || Fail("syntax");
  }

  bool StartsNumber() const {
    const char c = Peek();
    return c == '+' || c == '-' || std::isdigit(static_cast<unsigned char>(c));
  }

  // [+|-]hh[:mm[:ss]] with hours up to max_hours, in seconds.
  bool Duration(int max_hours, int64_t* seconds) {
    int sign = 1;
    if (Take('-')) sign = -1;
    else Take('+');
    int64_t hours = 0;
    if (!Number(3, &hours)) return false;
    int64_t minutes = 0;
    int64_t secs = 0;
    if (Take(':')) {
      if (!Number(2, &minutes)) return false;
      if (Take(':') && !Number(2, &secs)) return false;
    }
    if (hours > max_hours || minutes > 59 || secs > 59) return Fail("range");
    *seconds = sign * (hours * 3600 + minutes * 60 + secs);
    return true;
  }

  // ,Mm.w.d[/time]
  bool Rule(int64_t* month, int64_t* week, int64_t* weekday, int64_t* time) {
    if (!Take(',')) return Fail("syntax");
    if (Peek() == 'J' || std::isdigit(static_cast<unsigned char>(Peek()))) return Fail("unsupported_rule");
    if (!Take('M')) return Fail("syntax");
    if (!Number(2, month) || !Take('.') || !Number(1, week) || !Take('.') || !Number(1, weekday)) {
      return Fail("syntax");
    }
    if (*month < 1 || *month > 12 || *week < 1 || *week > 5 || *weekday > 6) return Fail("range");
    *time = 7200;
    return !Take('/') || Duration(167, time);
  }

 private:
  // One to max_digits digits.
  bool Number(int max_digits, int64_t* value) {
    int digits = 0;
    *value = 0;
    while (digits < max_digits && std::isdigit(static_cast<unsigned char>(Peek()))) {
      *value = *value * 10 + (Peek() - '0');
      ++pos_;
      ++digits;
    }
    return digits > 0 || Fail("syntax");
  }

  const std::string& text_;
  size_t pos_ = 0;
  std::string reason_;
};

}  // namespace

bool Date::operator<(const Date& other) const {
  if (year != other.year) return year < other.year;
  if (month != other.month) return month < other.month;
  return day < other.day;
}

int64_t DaysFromDate(const Date& date) {
  // Years counted from March, so the leap day ends the year.
  const int64_t year = date.year - (date.month <= 2 ? 1 : 0);
  const int64_t era = FloorDiv(year, 400);
  const int64_t year_of_era = year - era * 400;
  const int64_t month_from_march = (date.month + 9) % 12;
  const int64_t day_of_year = (153 * month_from_march + 2) / 5 + date.day - 1;
  const int64_t day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
  return era * 146097 + day_of_era - 719468;
}

Date DateFromDays(int64_t days) {
  const int64_t shifted = days + 719468;
  const int64_t era = FloorDiv(shifted, 146097);
  const int64_t day_of_era = shifted - era * 146097;
  const int64_t year_of_era =
      (day_of_era - day_of_era / 1460 + day_of_era / 36524 - day_of_era / 146096) / 365;
  const int64_t day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
  const int64_t month_from_march = (5 * day_of_year + 2) / 153;
  Date date;
  date.day = static_cast<int>(day_of_year - (153 * month_from_march + 2) / 5 + 1);
  date.month = static_cast<int>(month_from_march < 10 ? month_from_march + 3 : month_from_march - 9);
  date.year = static_cast<int>(year_of_era + era * 400 + (date.month <= 2 ? 1 : 0));
  return date;
}

int Weekday(const Date& date) {
  // 1970-01-01 was a Thursday.
  return static_cast<int>((DaysFromDate(date) % 7 + 7 + 3) % 7) + 1;
}

Date AddDays(const Date& date, int64_t days) {
  return DateFromDays(DaysFromDate(date) + days);
}

std::optional<Date> ParseDate(const std::string& text) {
  if (text.size() != 10 || text[4] != '-' || text[7] != '-') return std::nullopt;
  for (size_t i = 0; i < text.size(); ++i) {
    if (i != 4 && i != 7 && !std::isdigit(static_cast<unsigned char>(text[i]))) return std::nullopt;
  }
  Date date;
  date.year = std::stoi(text.substr(0, 4));
  date.month = std::stoi(text.substr(5, 2));
  date.day = std::stoi(text.substr(8, 2));
  if (date.month < 1 || date.month > 12 || date.day < 1 || date.day > DaysInMonth(date.year, date.month)) {
    return std::nullopt;
  }
  return date;
}

std::string FormatDate(const Date& date) {
  char text[16];
  std::snprintf(text, sizeof(text), "%04d-%02d-%02d", date.year, date.month, date.day);
  return text;
}

std::optional<int> ParseClockTime(const std::string& text) {
  if (text.size() != 5 || text[2] != ':') return std::nullopt;
  for (size_t i : {0, 1, 3, 4}) {
    if (!std::isdigit(static_cast<unsigned char>(text[i]))) return std::nullopt;
  }
  const int hours = (text[0] - '0') * 10 + (text[1] - '0');
  const int minutes = (text[3] - '0') * 10 + (text[4] - '0');
  if (hours > 23 || minutes > 59) return std::nullopt;
  return hours * 60 + minutes;
}

std::string FormatClockTime(int minutes) {
  char text[8];
  std::snprintf(text, sizeof(text), "%02d:%02d", minutes / 60, minutes % 60);
  return text;
}

std::optional<TimeZone> TimeZone::Parse(const std::string& text, std::string* reason) {
  if (text.size() > kMaxTzLength) {
    *reason = "range";
    return std::nullopt;
  }
  TzReader in(text);
  TimeZone zone;
  int64_t std_west = 0;
  bool ok = in.Name() && in.Duration(24, &std_west);
  zone.std_east_ = -std_west;
  zone.dst_east_ = zone.std_east_;
  if (ok && !in.Done()) {
    zone.has_dst_ = true;
    ok = in.Name();
    if (ok && in.StartsNumber()) {
      int64_t dst_west = 0;
      ok = in.Duration(24, &dst_west);
      zone.dst_east_ = -dst_west;
    } else {
      zone.dst_east_ = zone.std_east_ + 3600;
    }
    if (ok && in.Done()) ok = in.Fail("missing_rule");
    int64_t values[8] = {};
    ok = ok && in.Rule(&values[0], &values[1], &values[2], &values[3]) &&
         in.Rule(&values[4], &values[5], &values[6], &values[7]);
    if (ok && !in.Done()) ok = in.Fail("syntax");
    zone.start_ = {static_cast<int>(values[0]), static_cast<int>(values[1]), static_cast<int>(values[2]), values[3]};
    zone.end_ = {static_cast<int>(values[4]), static_cast<int>(values[5]), static_cast<int>(values[6]), values[7]};
  }
  if (!ok) {
    *reason = in.reason().empty() ? "syntax" : in.reason();
    return std::nullopt;
  }
  return zone;
}

TimeZone TimeZone::Utc() {
  return TimeZone();
}

namespace {

// The day of a rule Mm.w.d in a year: weekday d of week w (5 = the last) of month m.
int64_t RuleDay(int year, int month, int week, int weekday) {
  const Date first{year, month, 1};
  const int first_weekday = Weekday(first) % 7;  // 0 Sunday
  int day = 1 + (weekday - first_weekday + 7) % 7 + (week - 1) * 7;
  while (day > DaysInMonth(year, month)) day -= 7;
  return DaysFromDate(Date{year, month, day});
}

}  // namespace

int64_t TimeZone::StartOf(int year) const {
  return RuleDay(year, start_.month, start_.week, start_.weekday) * kDay + start_.time - std_east_;
}

int64_t TimeZone::EndOf(int year) const {
  return RuleDay(year, end_.month, end_.week, end_.weekday) * kDay + end_.time - dst_east_;
}

int64_t TimeZone::OffsetAt(int64_t unix_s) const {
  if (!has_dst_) return std_east_;
  // The changes of the years around, in order; the last one before the moment decides.
  const int year = DateFromDays(FloorDiv(unix_s + std_east_, kDay)).year;
  std::vector<std::pair<int64_t, bool>> changes;
  for (int y = year - 1; y <= year + 1; ++y) {
    changes.emplace_back(StartOf(y), true);
    changes.emplace_back(EndOf(y), false);
  }
  std::stable_sort(changes.begin(), changes.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });
  bool dst = false;
  for (const auto& change : changes) {
    if (change.first > unix_s) break;
    dst = change.second;
  }
  return dst ? dst_east_ : std_east_;
}

LocalTime TimeZone::ToLocal(int64_t unix_s) const {
  const int64_t local = unix_s + OffsetAt(unix_s);
  const int64_t days = FloorDiv(local, kDay);
  LocalTime out;
  out.date = DateFromDays(days);
  out.seconds = static_cast<int>(local - days * kDay);
  out.weekday = Weekday(out.date);
  return out;
}

int64_t TimeZone::ToUnix(const Date& date, int minutes) const {
  const int64_t local = DaysFromDate(date) * kDay + minutes * 60;
  const int64_t as_std = local - std_east_;
  const int64_t as_dst = local - dst_east_;
  const bool std_fits = OffsetAt(as_std) == std_east_;
  const bool dst_fits = OffsetAt(as_dst) == dst_east_;
  if (std_fits && dst_fits) return std::min(as_std, as_dst);
  if (std_fits) return as_std;
  if (dst_fits) return as_dst;
  // In the gap: the moment the clock jumps, where the offset changes between the two readings.
  int64_t low = std::min(as_std, as_dst);
  int64_t high = std::max(as_std, as_dst);
  const int64_t offset_low = OffsetAt(low);
  while (high - low > 1) {
    const int64_t mid = low + (high - low) / 2;
    if (OffsetAt(mid) == offset_low) low = mid;
    else high = mid;
  }
  return high;
}

}  // namespace mowbite_schedule
