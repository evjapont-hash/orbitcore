#pragma once
#include <cmath>

namespace orbitcore {

// Two-part Julian date (JD = day + frac) to keep sub-millisecond precision.
// `day` is conventionally a half-integer (JD days start at noon).
struct JulianDate {
  double day{0.0};
  double frac{0.0};
  double total() const { return day + frac; }
};

struct CalendarDate {
  int year, month, day, hour, minute;
  double second;
};

// Gregorian calendar -> Julian date. Valid 1900-2100 (Vallado `jday`).
JulianDate to_julian(const CalendarDate& date);

// TLE-style epoch: day-of-year where 1.0 == Jan 1 00:00:00.
JulianDate from_year_and_day_of_year(int year, double day_of_year);

JulianDate add_minutes(JulianDate jd, double minutes);

// Minutes from `from` to `to` (positive if `to` is later).
double minutes_between(JulianDate from, JulianDate to);

// Greenwich Mean Sidereal Time (IAU-82), radians in [0, 2*pi).
// Argument is treated as UT1. NOTE: passing UTC introduces |UT1-UTC| < 0.9 s,
// i.e. up to ~0.4 km of equatorial position error in ECEF. Document or add
// EOP (UT1-UTC) support when you need better.
double gmst_rad(JulianDate ut1);

}  // namespace orbitcore
