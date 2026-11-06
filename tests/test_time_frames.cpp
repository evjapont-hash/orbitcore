#include "check.hpp"
#include "orbitcore/frames.hpp"
#include "orbitcore/time.hpp"

using namespace orbitcore;

TEST_CASE(julian_date_j2000) {
  const auto jd = to_julian({2000, 1, 1, 12, 0, 0.0});
  CHECK_NEAR(jd.total(), 2451545.0, 1e-9);
}

TEST_CASE(gmst_at_j2000) {
  // GMST at J2000.0 is 18h 41m 50.54841s = 280.46061837 deg.
  const double deg = gmst_rad({2451545.0, 0.0}) * kRadToDeg;
  CHECK_NEAR(deg, 280.46061837, 1e-6);
}

TEST_CASE(geodetic_origin_and_roundtrip) {
  const auto p0 = geodetic_to_ecef({Radians{0}, Radians{0}, 0.0});
  CHECK_NEAR(p0.x, kWgs84SemiMajorKm, 1e-9);
  CHECK_NEAR(p0.y, 0.0, 1e-9);
  CHECK_NEAR(p0.z, 0.0, 1e-9);

  const double lats[] = {-90.0, -45.0, -0.5, 0.0, 33.3, 39.1, 89.9, 90.0};
  const double lons[] = {-179.0, -84.5, 0.0, 120.0, 180.0};
  const double alts[] = {-0.1, 0.0, 0.5, 400.0, 2000.0, 35786.0};
  for (double la : lats)
    for (double lo : lons)
      for (double h : alts) {
        const Geodetic g{to_radians({la}), to_radians({lo}), h};
        const auto back = ecef_to_geodetic(geodetic_to_ecef(g));
        CHECK_NEAR(back.latitude.value, g.latitude.value, 1e-9);
        CHECK_NEAR(back.altitude_km, h, 1e-6);
        if (std::fabs(la) < 89.999)  // longitude undefined at the poles
          CHECK_NEAR(std::cos(back.longitude.value - g.longitude.value), 1.0, 1e-12);
      }
}

TEST_CASE(enu_overhead_and_ned) {
  const Geodetic stn{to_radians({39.13}), to_radians({-84.51}), 0.15};
  const Geodetic above{stn.latitude, stn.longitude, stn.altitude_km + 100.0};
  const auto enu = ecef_to_enu(geodetic_to_ecef(above), stn);
  CHECK_NEAR(enu.x, 0.0, 1e-9);
  CHECK_NEAR(enu.y, 0.0, 1e-9);
  CHECK_NEAR(enu.z, 100.0, 1e-9);
  const auto la = look_angles(enu);
  CHECK_NEAR(la.elevation.value, kPi / 2.0, 1e-12);
  CHECK_NEAR(la.range_km, 100.0, 1e-9);

  const auto ned = enu_to_ned(Position<Frame::ENU>{1.0, 2.0, 3.0});
  CHECK_NEAR(ned.x, 2.0, 0.0);  // north
  CHECK_NEAR(ned.y, 1.0, 0.0);  // east
  CHECK_NEAR(ned.z, -3.0, 0.0); // down

  // Due east target should have azimuth 90 deg.
  CHECK_NEAR(look_angles(Position<Frame::ENU>{10.0, 0.0, 0.0}).azimuth.value, kPi / 2.0, 1e-12);
}

TEST_CASE(teme_to_ecef_rotation_and_velocity) {
  const JulianDate jd = to_julian({2026, 11, 5, 12, 0, 0.0});
  const Position<Frame::TEME> r{6000.0, 3000.0, 2000.0};
  const auto r_ecef = teme_to_ecef(r, jd);
  CHECK_NEAR(r_ecef.norm(), r.norm(), 1e-9);
  CHECK_NEAR(r_ecef.z, r.z, 0.0);

  // A point fixed to the rotating Earth moves at omega x r in the inertial frame,
  // so its Earth-relative velocity must come out zero.
  const double w = kEarthRotationRadPerSec;
  const Velocity<Frame::TEME> v{-w * r.y, w * r.x, 0.0};
  const auto s = teme_to_ecef(r, v, jd);
  CHECK_NEAR(s.v.x, 0.0, 1e-12);
  CHECK_NEAR(s.v.y, 0.0, 1e-12);
  CHECK_NEAR(s.v.z, 0.0, 1e-12);
}

TEST_CASE(rtn_basis_circular_orbit) {
  const Position<Frame::TEME> r{7000.0, 0.0, 0.0};
  const Velocity<Frame::TEME> v{0.0, 7.5, 0.0};
  const auto rtn = to_rtn({0.5, 1.0, -2.0}, r, v);  // dx, dy (in-track), dz (cross-track)
  CHECK_NEAR(rtn.x, 0.5, 1e-12);   // radial
  CHECK_NEAR(rtn.y, 1.0, 1e-12);   // transverse
  CHECK_NEAR(rtn.z, -2.0, 1e-12);  // normal
}

TEST_CASE(calendar_roundtrip) {
  const CalendarDate cases[] = {
      {2026, 11, 5, 12, 34, 56.789}, {2000, 2, 29, 0, 0, 0.0},   {2024, 12, 31, 23, 59, 59.5},
      {1970, 1, 1, 0, 0, 0.0},       {2008, 9, 20, 12, 25, 40.0}, {2099, 3, 1, 6, 7, 8.25}};
  for (const auto& c : cases) {
    const auto back = to_calendar(to_julian(c));
    CHECK(back.year == c.year);
    CHECK(back.month == c.month);
    CHECK(back.day == c.day);
    CHECK(back.hour == c.hour);
    CHECK(back.minute == c.minute);
    CHECK_NEAR(back.second, c.second, 1e-3);
  }
}