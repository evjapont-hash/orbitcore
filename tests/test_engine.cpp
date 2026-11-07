#include <cmath>
#include <string>

#include "check.hpp"
#include "engine.hpp"  // from wasm/ (header-only, no Emscripten dependency)

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