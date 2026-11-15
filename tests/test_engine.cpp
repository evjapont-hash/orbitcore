#include <cmath>
#include <string>

#include "check.hpp"
#include "engine.hpp"  // from wasm/ (header-only, no Emscripten dependency)
#include "orbitcore/conjunction.hpp"
#include "orbitcore/risk.hpp"
#include "orbitcore/units.hpp"

using namespace orbitcore;

static const char* kIssText =
    "ISS (ZARYA)\n"
    "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927\n"
    "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537\n";

// Epoch 2008 day 264.51782528 = JD 2454466.5 + 263.51782528.
static const double kEpochDay = 2454466.5;
static const double kEpochFrac = 263.51782528;

TEST_CASE(engine_describe_matches_native_tool) {
  Engine engine;
  CHECK(engine.load_tle(kIssText) == 1);
  // Reference: orbit_cli examples/iss_2008.tle at t = 10 min.
  CHECK(engine.describe(0, kEpochDay, kEpochFrac + 10.0 / 1440.0));
  const double* info = engine.info();
  CHECK_NEAR(info[0], 1.0, 0.0);
  CHECK_NEAR(info[1], 4550.075230, 1e-5);
  CHECK_NEAR(info[2], 3253.056114, 1e-5);
  CHECK_NEAR(info[3], 3728.338224, 1e-5);
  CHECK_NEAR(info[7], 7.705863, 1e-5);
  CHECK_NEAR(info[8], 33.854631, 1e-4);     // latitude, deg
  CHECK_NEAR(info[9], -153.126294, 1e-4);   // longitude, deg
  CHECK_NEAR(info[10], 350.520963, 1e-3);   // altitude, km
  CHECK_NEAR(info[16], 10.0 / 1440.0, 1e-9);  // element-set age, days
}

TEST_CASE(engine_describe_orbit_shape) {
  Engine engine;
  engine.load_tle(kIssText);
  engine.describe(0, kEpochDay, kEpochFrac);
  const double* info = engine.info();
  CHECK_NEAR(info[11], 1440.0 / 15.72125391, 1e-6);  // period from the TLE mean motion, min
  CHECK_NEAR(info[12], 51.6416, 1e-9);                // inclination, deg
  CHECK_NEAR(info[13], 0.0006703, 1e-12);             // eccentricity
  CHECK(info[14] > 340.0 && info[14] < 370.0);         // perigee altitude, km
  CHECK(info[15] > info[14] && info[15] < 380.0);      // apogee above perigee
  CHECK_NEAR(info[15] - info[14], 2.0 * 6738.0 * 0.0006703, 1.0);
}

TEST_CASE(engine_describe_out_of_range) {
  Engine engine;
  engine.load_tle(kIssText);
  CHECK(!engine.describe(5, kEpochDay, kEpochFrac));
  CHECK_NEAR(engine.info()[0], 0.0, 0.0);
  CHECK(!engine.describe(-1, kEpochDay, kEpochFrac));
}

TEST_CASE(engine_track) {
  Engine engine;
  engine.load_tle(kIssText);
  const double t = kEpochFrac + 5.0 / 1440.0;
  CHECK(engine.track(0, kEpochDay, t, 92.0, 100) == 100);
  CHECK(engine.track_size() == 300);
  const float* p = engine.track_data();

  engine.describe(0, kEpochDay, t);
  const double* info = engine.info();
  // The first track point is the object's own position at the start instant.
  CHECK_NEAR(p[0], info[1], 0.01);
  CHECK_NEAR(p[1], info[2], 0.01);
  CHECK_NEAR(p[2], info[3], 0.01);
  // Every point is a plausible LEO radius.
  for (int j = 0; j < 100; ++j) {
    const double r = std::sqrt(static_cast<double>(p[3 * j]) * p[3 * j] +
                               static_cast<double>(p[3 * j + 1]) * p[3 * j + 1] +
                               static_cast<double>(p[3 * j + 2]) * p[3 * j + 2]);
    CHECK(r > 6700.0 && r < 6800.0);
  }
  // Out of range object: nothing valid, buffer still well-formed.
  CHECK(engine.track(9, kEpochDay, t, 92.0, 10) == 0);
  CHECK(engine.track_size() == 30);
}

TEST_CASE(engine_station_visibility) {
  Engine engine;
  engine.load_tle(kIssText);
  const double t = kEpochFrac + 10.0 / 1440.0;
  engine.propagate(kEpochDay, t);
  engine.describe(0, kEpochDay, t);
  const double lat = engine.info()[8], lon = engine.info()[9];  // sub-satellite point

  CHECK(engine.add_station(lat, lon, 0.0, 10.0) == 0);           // directly under the ISS
  CHECK(engine.add_station(-lat, lon + 180.0, 0.0, 10.0) == 1);  // opposite side of the Earth
  CHECK(engine.station_count() == 2);
  engine.update_visibility(kEpochDay, t);
  CHECK(engine.visible_count(0) == 1);
  CHECK(engine.visible_count(1) == 0);
  CHECK(engine.visibility_size() == 1);
  CHECK(engine.visibility()[0] == 1);  // bit 0 only

  CHECK(engine.look(0, 0, kEpochDay, t));
  CHECK_NEAR(engine.look_info()[1], 90.0, 0.5);    // elevation: the ISS is overhead
  CHECK_NEAR(engine.look_info()[2], 350.52, 0.5);  // range is about its altitude, km
  CHECK(engine.look(1, 0, kEpochDay, t));
  CHECK(engine.look_info()[1] < 0.0);              // below the horizon from the far side
  CHECK(!engine.look(5, 0, kEpochDay, t));         // no such station
  CHECK(!engine.look(0, 3, kEpochDay, t));         // no such object

  engine.clear_stations();
  CHECK(engine.station_count() == 0);
  engine.update_visibility(kEpochDay, t);
  CHECK(engine.visibility()[0] == 0);
}

TEST_CASE(engine_station_limit) {
  Engine engine;
  engine.load_tle(kIssText);
  for (int i = 0; i < 8; ++i) CHECK(engine.add_station(10.0 * i, 0.0, 0.0, 5.0) == i);
  CHECK(engine.add_station(0.0, 0.0, 0.0, 5.0) == -1);  // only 8 stations (one bit each)
  CHECK(engine.station_count() == 8);
}

// An engine with the ISS (index 0) plus a few objects on other orbits, so that screening finds
// encounters (the threshold used below is large, so the exact geometry does not matter).
static Engine make_screening_engine() {
  Engine engine;
  engine.load_tle(kIssText);
  const ElementSet iss = parse_tle("1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927",
                                   "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537").elements;
  for (int k = 1; k <= 6; ++k) {
    ElementSet e = iss;
    e.satnum = 80000 + k;
    e.inclo = (40.0 + 10.0 * k) * kDegToRad;
    e.nodeo = std::fmod(iss.nodeo + 0.9 * k, kTwoPi);
    e.mo = std::fmod(iss.mo + 0.7 * k, kTwoPi);
    e.bstar = 0.0;
    engine.add_element_set("OBJECT " + std::to_string(k), e);
  }
  return engine;
}

TEST_CASE(engine_screening_matches_library_and_chunking_does_not_matter) {
  const double kThreshold = 6000.0;   // large, so every pair has several "encounters" in the window
  Engine reference = make_screening_engine();
  CHECK(reference.screen_begin(0, kEpochDay, kEpochFrac, 180.0, kThreshold, 50.0));
  while (reference.screen_step(1000) < 1.0) {}
  const int n = reference.screen_event_count();
  CHECK(n > 5);
  CHECK(reference.screen_results_size() == static_cast<std::size_t>(n) * Engine::kEncounterStride);

  // Chunk size must not change the answer, bit for bit.
  for (int chunk : {1, 2, 3, 100}) {
    Engine engine = make_screening_engine();
    CHECK(engine.screen_begin(0, kEpochDay, kEpochFrac, 180.0, kThreshold, 50.0));
    double progress = 0.0, previous = -1.0;
    int steps = 0;
    while (progress < 1.0 && ++steps < 1000) {
      progress = engine.screen_step(chunk);
      CHECK(progress > previous);        // always moves forward
      previous = progress;
    }
    CHECK(engine.screen_event_count() == n);
    CHECK(engine.screen_results_size() == reference.screen_results_size());
    for (std::size_t i = 0; i < reference.screen_results_size(); ++i) {
      CHECK(engine.screen_results()[i] == reference.screen_results()[i]);
    }
  }

  // Layout and consistency of the packed rows.
  const double* rows = reference.screen_results();
  const std::size_t stride = Engine::kEncounterStride;
  double previous_tca = -1e300;
  for (int i = 0; i < n; ++i) {
    const double* r = rows + static_cast<std::size_t>(i) * stride;
    const double tca = (r[1] - kEpochDay) + (r[2] - kEpochFrac);
    CHECK(tca >= previous_tca);                                      // sorted by time
    previous_tca = tca;
    CHECK(r[0] >= 1.0 && r[0] <= 6.0);                               // a secondary, never the primary
    CHECK(r[3] > 0.0 && r[3] <= kThreshold);                         // miss distance within the threshold
    CHECK_NEAR(std::sqrt(r[5] * r[5] + r[6] * r[6] + r[7] * r[7]), r[3], 1e-6);   // R, T, N is a rotation of the miss
    const MaxProbability worst = maximum_collision_probability(r[3], 0.05);
    CHECK(r[8] == worst.probability);
    CHECK(r[9] == worst.sigma_km);
    CHECK(r[10] == collision_probability_ceiling(r[3], 0.05));
    CHECK(r[8] <= r[10]);                                            // the isotropic maximum is below the ceiling
    const double dx = r[17] - r[11], dy = r[18] - r[12], dz = r[19] - r[13];
    CHECK_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), r[3], 1e-6);  // positions are consistent with the miss distance
  }
  CHECK(reference.screen_stats()[0] == 7.0);                         // seven objects in the catalog
  CHECK(reference.screen_stats()[2] > 0.0);
}

TEST_CASE(engine_screening_rejects_bad_requests) {
  Engine engine = make_screening_engine();
  CHECK(!engine.screen_begin(99, kEpochDay, kEpochFrac, 60.0, 10.0, 10.0));    // no such primary
  CHECK(!engine.screen_begin(-1, kEpochDay, kEpochFrac, 60.0, 10.0, 10.0));
  CHECK(!engine.screen_begin(0, kEpochDay, kEpochFrac, 0.0, 10.0, 10.0));      // empty window
  CHECK(!engine.screen_begin(0, kEpochDay, kEpochFrac, 60.0, 0.0, 10.0));      // no threshold
  CHECK(!engine.screen_begin(0, kEpochDay, kEpochFrac, 60.0, 10.0, 0.0));      // no hard body
  CHECK(engine.screen_step(10) == 1.0);                                          // no session: nothing to do
  CHECK(engine.screen_event_count() == 0);
  CHECK(engine.screen_begin(0, kEpochDay, kEpochFrac, 60.0, 10.0, 10.0));
  engine.load_tle(kIssText);                                                     // loading data cancels the session
  CHECK(engine.screen_step(10) == 1.0);
}