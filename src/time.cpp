#include "orbitcore/time.hpp"

#include "orbitcore/units.hpp"

namespace orbitcore {

namespace {
JulianDate normalize(JulianDate jd) {
  const double whole = std::floor(jd.frac);
  jd.day += whole;
  jd.frac -= whole;
  return jd;
}
}  // namespace

JulianDate to_julian(const CalendarDate& d) {
  const double jd = 367.0 * d.year -
                    std::floor((7.0 * (d.year + std::floor((d.month + 9) / 12.0))) * 0.25) +
                    std::floor(275.0 * d.month / 9.0) + d.day + 1721013.5;
  const double frac = (d.second + d.minute * 60.0 + d.hour * 3600.0) / 86400.0;
  return normalize({jd, frac});
}

JulianDate from_year_and_day_of_year(int year, double day_of_year) {
  const JulianDate jan1 = to_julian({year, 1, 1, 0, 0, 0.0});
  return normalize({jan1.day, day_of_year - 1.0});
}

JulianDate add_minutes(JulianDate jd, double minutes) {
  jd.frac += minutes / 1440.0;
  return normalize(jd);
}

double minutes_between(JulianDate from, JulianDate to) {
  return ((to.day - from.day) + (to.frac - from.frac)) * 1440.0;
}

double gmst_rad(JulianDate ut1) {
  const double t = ((ut1.day - 2451545.0) + ut1.frac) / 36525.0;
  const double seconds = -6.2e-6 * t * t * t + 0.093104 * t * t +
                         (876600.0 * 3600.0 + 8640184.812866) * t + 67310.54841;
  double theta = std::fmod(seconds * kDegToRad / 240.0, kTwoPi);
  if (theta < 0.0) theta += kTwoPi;
  return theta;
}

CalendarDate to_calendar(JulianDate jd) {
  // Days since 1970-01-01 00:00 UTC (JD 2440587.5).
  const double days_total = (jd.day - 2440587.5) + jd.frac;
  double whole = std::floor(days_total);
  double secs = std::round((days_total - whole) * 86400.0 * 1000.0) / 1000.0;
  if (secs >= 86400.0) {  // rounding pushed us to the next day
    secs -= 86400.0;
    whole += 1.0;
  }

  // Civil-from-days algorithm (H. Hinnant): days since 1970-01-01 -> year/month/day.
  const long z = static_cast<long>(whole) + 719468;
  const long era = (z >= 0 ? z : z - 146096) / 146097;
  const long doe = z - era * 146097;                                      // [0, 146096]
  const long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;  // [0, 399]
  const long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);                // [0, 365]
  const long mp = (5 * doy + 2) / 153;                                     // [0, 11]
  const int day = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
  const int month = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
  const int year = static_cast<int>(yoe + era * 400 + (month <= 2 ? 1 : 0));

  const int hour = static_cast<int>(secs / 3600.0);
  secs -= hour * 3600.0;
  const int minute = static_cast<int>(secs / 60.0);
  secs -= minute * 60.0;
  return {year, month, day, hour, minute, secs};
}

}  // namespace orbitcore
