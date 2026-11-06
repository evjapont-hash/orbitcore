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

}  // namespace orbitcore
