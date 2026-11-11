#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "check.hpp"
#include "orbitcore/conjunction.hpp"

using namespace orbitcore;

namespace {

const char* kIss1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927";
const char* kIss2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";

// A test catalog: the ISS (index 0) and a family of objects on a crossing orbit, phased so that
// each passes the ISS's position at about the same moment (T* = 30 minutes after the TLE epoch),
// with a spread of along-track offsets so that the miss distances range from a few km to hundreds.
struct Scenario {
  Catalog catalog;
  JulianDate start;          // the ISS TLE epoch
  double span_minutes = 120.0;
  std::size_t family_begin = 1;
  std::size_t decoys_begin = 0;
};

Scenario make_scenario() {
  Scenario sc;
  const ElementSet iss = parse_tle(kIss1, kIss2).elements;
  sc.catalog.add_element_set("ISS", iss);
  sc.start = iss.epoch;

  // Where is the ISS at T*? (declination and right ascension in the inertial frame)
  const double t_star_min = 30.0;
  const PropagationResult at_star = Sgp4(iss).propagate(t_star_min);
  const auto& p = at_star.state.r;
  const double declination = std::asin(p.z / p.norm());
  const double right_ascension = std::atan2(p.y, p.x);

  // A circular orbit with inclination i2 passes through that point if its argument of latitude
  // is u = asin(sin(dec) / sin(i2)) and its RAAN is alpha - atan2(cos(i2) sin(u), cos(u)).
  const double i2 = 98.0 * kDegToRad;
  const double u = std::asin(std::sin(declination) / std::sin(i2));
  const double raan = right_ascension - std::atan2(std::cos(i2) * std::sin(u), std::cos(u));

  for (int k = -60; k <= 60; ++k) {
    ElementSet e = iss;
    e.satnum = 90000 + (k + 60);
    e.ecco = 0.0001;
    e.inclo = i2;
    e.nodeo = std::fmod(raan + kTwoPi, kTwoPi);
    e.argpo = 0.0;
    e.mo = std::fmod(u - iss.no_kozai * t_star_min + (0.01 * k) * kDegToRad + 4.0 * kTwoPi, kTwoPi);
    e.bstar = 0.0;
    sc.catalog.add_element_set("CROSSING " + std::to_string(k), e);
  }
  sc.decoys_begin = sc.catalog.size();
  // Decoys: similar orbits about 800 km higher, which can never come near the ISS.
  for (int k = 0; k < 6; ++k) {
    ElementSet e = iss;
    e.satnum = 91000 + k;
    e.no_kozai = iss.no_kozai * 0.85;   // lower mean motion = larger orbit
    e.inclo = i2;
    e.mo = 0.5 * k;
    e.bstar = 0.0;
    sc.catalog.add_element_set("DECOY " + std::to_string(k), e);
  }
  return sc;
}

double seconds_between(JulianDate a, JulianDate b) { return minutes_between(a, b) * 60.0; }

}  // namespace

TEST_CASE(safe_advance_analytic_cases) {
  const double thr = 10.0;
  {  // head-on, straight-line motion: closing at 10 km/s from 1000 km
    const double r[3] = {1000.0, 0.0, 0.0}, v[3] = {-10.0, 0.0, 0.0};
    CHECK_NEAR(safe_advance_seconds(r, v, thr, 0.0, 600.0), (1000.0 - thr) / 10.0, 1e-9);
  }
  {  // receding: the distance can only grow, so the full step is safe
    const double r[3] = {100.0, 0.0, 0.0}, v[3] = {5.0, 0.0, 0.0};
    CHECK_NEAR(safe_advance_seconds(r, v, thr, 0.0, 600.0), 600.0, 0.0);
  }
  {  // already inside the threshold
    const double r[3] = {5.0, 0.0, 0.0}, v[3] = {1.0, 0.0, 0.0};
    CHECK_NEAR(safe_advance_seconds(r, v, thr, 0.02, 600.0), 0.0, 0.0);
  }
  {  // stationary pair with an acceleration bound: 1000 - a dt^2 / 2 = threshold
    const double r[3] = {1000.0, 0.0, 0.0}, v[3] = {0.0, 0.0, 0.0};
    CHECK_NEAR(safe_advance_seconds(r, v, thr, 0.02, 1000.0), std::sqrt(2.0 * (1000.0 - thr) / 0.02), 1e-6);
  }
  {  // passing with an impact parameter of 50 km: never gets within 10 km, so any step is safe
    const double r[3] = {1000.0, 50.0, 0.0}, v[3] = {-10.0, 0.0, 0.0};
    CHECK_NEAR(safe_advance_seconds(r, v, thr, 0.0, 600.0), 600.0, 0.0);
  }
  {  // the same pass with an impact parameter of 5 km: stop short of entering the 10 km sphere
    const double r[3] = {1000.0, 5.0, 0.0}, v[3] = {-10.0, 0.0, 0.0};
    const double dt = safe_advance_seconds(r, v, thr, 0.0, 600.0);
    CHECK(dt > 0.0 && dt < 100.0);
    CHECK_NEAR(std::hypot(1000.0 - 10.0 * dt, 5.0), thr, 1e-6);
  }
}

TEST_CASE(safe_advance_is_conservative_on_propagated_pairs) {
  Scenario sc = make_scenario();
  const Sgp4& iss = sc.catalog.propagator(0);
  const double thr = 10.0;
  int checked = 0, nontrivial = 0;
  for (std::size_t k = 1; k <= 121; k += 4) {
    const Sgp4& other = sc.catalog.propagator(k);
    for (double t = 0.0; t < 7200.0; t += 90.0) {
      const PropagationResult a = iss.propagate(t / 60.0), b = other.propagate(t / 60.0);
      const double r[3] = {b.state.r.x - a.state.r.x, b.state.r.y - a.state.r.y, b.state.r.z - a.state.r.z};
      const double v[3] = {b.state.v.x - a.state.v.x, b.state.v.y - a.state.v.y, b.state.v.z - a.state.v.z};
      const double dt = std::min(safe_advance_seconds(r, v, thr, 0.02, 600.0), 120.0);
      if (dt <= 0.0) continue;
      ++checked;
      if (dt > 20.0) ++nontrivial;
      for (double s = 0.5; s <= dt; s += 0.5) {  // the true distance must stay above the threshold
        const PropagationResult a2 = iss.propagate((t + s) / 60.0), b2 = other.propagate((t + s) / 60.0);
        const double d = std::sqrt(std::pow(b2.state.r.x - a2.state.r.x, 2) + std::pow(b2.state.r.y - a2.state.r.y, 2) +
                                   std::pow(b2.state.r.z - a2.state.r.z, 2));
        CHECK(d > thr);
        if (!(d > thr)) return;
      }
    }
  }
  CHECK(checked > 500);
  CHECK(nontrivial > 100);  // the test is not vacuous: many steps were long
}

TEST_CASE(screening_matches_brute_force) {
  Scenario sc = make_scenario();
  ScreeningOptions options;
  options.threshold_km = 10.0;
  const ScreeningResult fast = screen_primary(sc.catalog, 0, sc.start, sc.span_minutes, options);
  const ScreeningResult slow = screen_primary_brute_force(sc.catalog, 0, sc.start, sc.span_minutes, 10.0, 2.0);

  CHECK(slow.encounters.size() >= 3);                 // the scenario really contains close approaches
  CHECK(fast.encounters.size() == slow.encounters.size());
  const std::size_t n = std::min(fast.encounters.size(), slow.encounters.size());
  for (std::size_t i = 0; i < n; ++i) {
    CHECK(fast.encounters[i].secondary == slow.encounters[i].secondary);
    CHECK_NEAR(seconds_between(fast.encounters[i].tca, slow.encounters[i].tca), 0.0, 0.01);
    CHECK_NEAR(fast.encounters[i].miss_distance_km, slow.encounters[i].miss_distance_km, 1e-3);
  }
  for (const Encounter& e : fast.encounters) {
    CHECK(e.miss_distance_km <= 10.0);
    CHECK(e.relative_speed_km_s > 1.0 && e.relative_speed_km_s < 16.0);
    CHECK_NEAR(e.miss_rtn.norm(), e.miss_distance_km, 1e-9);   // RTN is only a rotation of the miss vector
  }
  for (std::size_t i = 1; i < fast.encounters.size(); ++i) {
    CHECK(minutes_between(fast.encounters[i - 1].tca, fast.encounters[i].tca) >= 0.0);  // sorted by time
  }
  // The point of the method: far fewer propagations than stepping every pair.
  CHECK(fast.stats.propagations * 5 < slow.stats.propagations);
  CHECK(fast.stats.objects_screened < fast.stats.objects_total - 1);   // the radial-band filter removed the decoys
}

TEST_CASE(screening_without_radial_filter_finds_the_same_events) {
  Scenario sc = make_scenario();
  ScreeningOptions with_filter, without_filter;
  without_filter.radial_margin_km = -1.0;
  const ScreeningResult a = screen_primary(sc.catalog, 0, sc.start, sc.span_minutes, with_filter);
  const ScreeningResult b = screen_primary(sc.catalog, 0, sc.start, sc.span_minutes, without_filter);
  CHECK(a.encounters.size() == b.encounters.size());
  CHECK(b.stats.objects_screened == b.stats.objects_total - 1);
  CHECK(a.stats.objects_screened < b.stats.objects_screened);
}

TEST_CASE(closest_approach_is_accurate) {
  Scenario sc = make_scenario();
  const ScreeningResult result = screen_primary(sc.catalog, 0, sc.start, sc.span_minutes);
  CHECK(!result.encounters.empty());
  if (result.encounters.empty()) return;
  const Encounter& e = result.encounters.front();
  // Independent check: scan +-0.5 s around the reported time in 1 ms steps with plain SGP4.
  const Sgp4& iss = sc.catalog.propagator(0);
  const Sgp4& other = sc.catalog.propagator(e.secondary);
  const double t0 = seconds_between(sc.start, e.tca);
  double best = 1e9, best_t = t0;
  for (int k = -500; k <= 500; ++k) {
    const double t = t0 + k * 1e-3;
    const PropagationResult a = iss.propagate(t / 60.0), b = other.propagate(t / 60.0);
    const double d = std::sqrt(std::pow(b.state.r.x - a.state.r.x, 2) + std::pow(b.state.r.y - a.state.r.y, 2) +
                               std::pow(b.state.r.z - a.state.r.z, 2));
    if (d < best) { best = d; best_t = t; }
  }
  CHECK_NEAR(e.miss_distance_km, best, 1e-5);   // within 1 cm
  CHECK_NEAR(t0, best_t, 2e-3);                 // within 2 ms
}

TEST_CASE(screening_edge_cases) {
  Scenario sc = make_scenario();
  CHECK(screen_primary(sc.catalog, 9999, sc.start, 60.0).encounters.empty());      // no such primary
  CHECK(screen_primary(sc.catalog, 0, sc.start, 0.0).encounters.empty());          // empty window
  ScreeningOptions tiny;
  tiny.threshold_km = 0.0;
  CHECK(screen_primary(sc.catalog, 0, sc.start, sc.span_minutes, tiny).encounters.empty());
  ElementSet deep = parse_tle(kIss1, kIss2).elements;
  deep.no_kozai = 0.0044;                                                          // ~24 h period
  Catalog c;
  CHECK(!c.add_element_set("DEEP", deep));                                         // refused, not added
  CHECK(c.size() == 0);
}