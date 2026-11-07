#include <cmath>
#include <cstdint>
#include <vector>

#include "check.hpp"
#include "orbitcore/time.hpp"
#include "orbitcore/visibility.hpp"

using namespace orbitcore;

// A station on the equator at longitude 0 (ECEF position (a, 0, 0)).
static StationFrame equator_station(double min_el_deg) {
  return make_station_frame({Radians{0.0}, Radians{0.0}, 0.0}, min_el_deg * kDegToRad);
}

TEST_CASE(visibility_overhead_and_axes) {
  const StationFrame st = equator_station(10.0);
  const double a = kWgs84SemiMajorKm;
  // A satellite 500 km straight above the station.
  const LookAngles up = look_from_station(st, {a + 500.0, 0.0, 0.0}, 0.0);
  CHECK_NEAR(up.elevation.value, kPi / 2.0, 1e-9);
  CHECK_NEAR(up.range_km, 500.0, 1e-9);
  // East (+y at longitude 0) is azimuth 90 degrees; north (+z at the equator) is azimuth 0.
  const double r = a + 500.0, g = 5.0 * kDegToRad;
  CHECK_NEAR(look_from_station(st, {r * std::cos(g), r * std::sin(g), 0.0}, 0.0).azimuth.value, kPi / 2.0, 1e-9);
  CHECK_NEAR(look_from_station(st, {r * std::cos(g), 0.0, r * std::sin(g)}, 0.0).azimuth.value, 0.0, 1e-9);
}

TEST_CASE(visibility_matches_closed_form_elevation) {
  const StationFrame st = equator_station(10.0);
  const double a = kWgs84SemiMajorKm, r = a + 500.0;
  // Satellites in the equatorial plane at central angle g east of the station:
  // elevation = atan2(r cos g - a, r sin g) (an independent closed form).
  std::vector<double> xyz;
  std::vector<bool> expected;
  for (int deg = 0; deg <= 40; ++deg) {
    const double g = deg * kDegToRad;
    xyz.insert(xyz.end(), {r * std::cos(g), r * std::sin(g), 0.0});
    const double el = std::atan2(r * std::cos(g) - a, r * std::sin(g));
    CHECK_NEAR(look_from_station(st, {r * std::cos(g), r * std::sin(g), 0.0}, 0.0).elevation.value, el, 1e-9);
    expected.push_back(el >= 10.0 * kDegToRad);
  }
  std::vector<std::uint8_t> flags(xyz.size() / 3, 0);
  const std::size_t visible = mark_visible(st, xyz.data(), flags.size(), 0.0, 1, flags.data());
  std::size_t expected_count = 0;
  for (std::size_t i = 0; i < flags.size(); ++i) {
    CHECK((flags[i] == 1) == expected[i]);
    if (expected[i]) ++expected_count;
  }
  CHECK(visible == expected_count);
  CHECK(expected_count > 5 && expected_count < 35);  // the sweep crosses the 10 degree mask
}

TEST_CASE(visibility_bits_and_bad_positions) {
  const StationFrame overhead = equator_station(10.0);
  const StationFrame far_side = make_station_frame({Radians{0.0}, Radians{kPi}, 0.0}, 10.0 * kDegToRad);
  const double r = kWgs84SemiMajorKm + 500.0;
  // object 0: above the first station; object 1: NaN; object 2: above the second station.
  const double xyz[9] = {r, 0.0, 0.0, std::nan(""), 0.0, 0.0, -r, 0.0, 0.0};
  std::uint8_t flags[3] = {0, 0, 0};
  CHECK(mark_visible(overhead, xyz, 3, 0.0, 1, flags) == 1);
  CHECK(mark_visible(far_side, xyz, 3, 0.0, 2, flags) == 1);
  CHECK(flags[0] == 1);  // bit 0 only
  CHECK(flags[1] == 0);  // NaN is never visible
  CHECK(flags[2] == 2);  // bit 1 only
}

TEST_CASE(visibility_agrees_with_library_frame_chain) {
  // Same answer as teme_to_ecef -> ecef_to_enu -> look_angles at a non-trivial time.
  const Geodetic site{to_radians({39.1}), to_radians({-84.5}), 0.15};
  const StationFrame st = make_station_frame(site, 0.0);
  const JulianDate jd = to_julian({2026, 11, 6, 18, 30, 0.0});
  const Position<Frame::TEME> r{4000.0, -5200.0, 2500.0};
  const LookAngles a = look_from_station(st, r, gmst_rad(jd));
  const LookAngles b = look_angles(ecef_to_enu(teme_to_ecef(r, jd), site));
  CHECK_NEAR(a.elevation.value, b.elevation.value, 1e-9);
  CHECK_NEAR(a.azimuth.value, b.azimuth.value, 1e-9);
  CHECK_NEAR(a.range_km, b.range_km, 1e-6);
}