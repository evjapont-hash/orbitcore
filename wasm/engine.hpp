#pragma once
// Engine: a thin stateful wrapper around Catalog for the browser build.
// It owns the buffers that JavaScript reads directly from WebAssembly memory.
// Kept free of any Emscripten headers so it can be compiled and tested natively.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "orbitcore/catalog.hpp"
#include "orbitcore/frames.hpp"
#include "orbitcore/visibility.hpp"

namespace orbitcore {

class Engine {
 public:
  // Layout of the array returned by info() after describe():
  //   0 ok (1 = state valid, 0 = propagation failed)   1-3 position, km (TEME)
  //   4-6 velocity, km/s (TEME)                         7 speed, km/s
  //   8 latitude, deg   9 longitude, deg   10 altitude, km (WGS-84)
  //   11 period, min    12 inclination, deg   13 eccentricity
  //   14 perigee altitude, km   15 apogee altitude, km   16 element-set age, days
  // Entries 11-15 are the TLE's mean elements (Kozai mean motion), not osculating values.
  static constexpr std::size_t kInfoSize = 17;

  // Adds the satellites found in `text` (2- or 3-line element sets). Returns the total
  // number of objects now loaded.
  int load_tle(const std::string& text) {
    stats_ = catalog_.load_tle_text(text);
    scratch_.assign(3 * catalog_.size(), 0.0);
    positions_.assign(3 * catalog_.size(), 0.0f);
    return count();
  }

  int count() const { return static_cast<int>(catalog_.size()); }
  int skipped_deep_space() const { return static_cast<int>(stats_.skipped_deep_space); }
  int skipped_unparseable() const { return static_cast<int>(stats_.skipped_parse); }

  int satnum(int i) const {
    return in_range(i) ? catalog_.satnum(static_cast<std::size_t>(i)) : -1;
  }
  std::string name(int i) const {
    return in_range(i) ? catalog_.name(static_cast<std::size_t>(i)) : std::string();
  }

  // Greenwich Mean Sidereal Time (radians) at the given UTC instant, for rotating the Earth.
  double gmst(double jd_day, double jd_frac) const { return gmst_rad({jd_day, jd_frac}); }

  // Propagates every object to the UTC instant (jd_day + jd_frac). Computation is in double
  // precision; results are stored as float32 TEME positions in km (x, y, z per object),
  // which is what the GPU wants. Returns the number of objects that failed (stored as NaN).
  int propagate(double jd_day, double jd_frac) {
    const std::size_t failures = catalog_.propagate_all({jd_day, jd_frac}, scratch_.data());
    for (std::size_t i = 0; i < scratch_.size(); ++i) {
      positions_[i] = static_cast<float>(scratch_[i]);
    }
    return static_cast<int>(failures);
  }

  const float* positions() const { return positions_.data(); }
  std::size_t position_count() const { return positions_.size(); }  // number of floats

  // Fills the info() array for one object at the given instant. Returns true if the object
  // propagated successfully (the orbit-shape entries are filled either way).
  bool describe(int i, double jd_day, double jd_frac) {
    info_.fill(0.0);
    if (!in_range(i)) return false;
    const std::size_t k = static_cast<std::size_t>(i);
    const JulianDate t{jd_day, jd_frac};
    const ElementSet& e = catalog_.elements(k);
    const GravityModel g = wgs72();

    const double a_km = std::pow(g.xke / e.no_kozai, 2.0 / 3.0) * g.radius_earth_km;
    info_[11] = kTwoPi / e.no_kozai;
    info_[12] = e.inclo * kRadToDeg;
    info_[13] = e.ecco;
    info_[14] = a_km * (1.0 - e.ecco) - g.radius_earth_km;
    info_[15] = a_km * (1.0 + e.ecco) - g.radius_earth_km;
    info_[16] = minutes_between(e.epoch, t) / 1440.0;

    const PropagationResult result = catalog_.propagate_one(k, t);
    if (!result.ok()) return false;
    const auto& r = result.state.r;
    const auto& v = result.state.v;
    const Geodetic geo = ecef_to_geodetic(teme_to_ecef(r, t));  // UTC used as UT1
    info_[0] = 1.0;
    info_[1] = r.x; info_[2] = r.y; info_[3] = r.z;
    info_[4] = v.x; info_[5] = v.y; info_[6] = v.z;
    info_[7] = v.norm();
    info_[8] = geo.latitude.value * kRadToDeg;
    info_[9] = geo.longitude.value * kRadToDeg;
    info_[10] = geo.altitude_km;
    return true;
  }

  const double* info() const { return info_.data(); }
  std::size_t info_count() const { return info_.size(); }

  // Computes `n` TEME positions (float32 km, x y z each) of one object, evenly spaced from
  // the given instant to `span_min` minutes later. Returns how many propagated successfully;
  // a failed point repeats the previous valid one so the line stays drawable.
  int track(int i, double jd_day, double jd_frac, double span_min, int n) {
    if (n < 2) n = 2;
    if (n > 2000) n = 2000;
    track_.assign(3 * static_cast<std::size_t>(n), 0.0f);
    if (!in_range(i)) return 0;
    const std::size_t k = static_cast<std::size_t>(i);
    const JulianDate start{jd_day, jd_frac};
    int valid = 0;
    float last[3] = {0.0f, 0.0f, 0.0f};
    for (int j = 0; j < n; ++j) {
      const JulianDate t = add_minutes(start, span_min * j / (n - 1));
      const PropagationResult result = catalog_.propagate_one(k, t);
      if (result.ok()) {
        last[0] = static_cast<float>(result.state.r.x);
        last[1] = static_cast<float>(result.state.r.y);
        last[2] = static_cast<float>(result.state.r.z);
        ++valid;
      }
      const std::size_t base = 3 * static_cast<std::size_t>(j);
      track_[base] = last[0];
      track_[base + 1] = last[1];
      track_[base + 2] = last[2];
    }
    return valid;
  }

  const float* track_data() const { return track_.data(); }
  std::size_t track_size() const { return track_.size(); }  // number of floats

  // ---- Ground stations -----------------------------------------------------------------
  static constexpr std::size_t kMaxStations = 8;  // one bit of visibility()[i] per station

  void clear_stations() {
    stations_.clear();
    station_visible_.fill(0);
    visibility_.assign(catalog_.size(), 0);
  }

  // Adds a station; returns its index, or -1 if kMaxStations are already defined.
  int add_station(double lat_deg, double lon_deg, double alt_km, double min_elevation_deg) {
    if (stations_.size() >= kMaxStations) return -1;
    const Geodetic location{Radians{lat_deg * kDegToRad}, Radians{lon_deg * kDegToRad}, alt_km};
    stations_.push_back(make_station_frame(location, min_elevation_deg * kDegToRad));
    return static_cast<int>(stations_.size()) - 1;
  }

  int station_count() const { return static_cast<int>(stations_.size()); }

  // For the instant of the most recent propagate() call, marks which objects each station
  // can see: bit s of visibility()[i] is set when object i is at or above station s's
  // minimum elevation. Call propagate() with the same instant first.
  void update_visibility(double jd_day, double jd_frac) {
    visibility_.assign(catalog_.size(), 0);
    station_visible_.fill(0);
    const double gmst = gmst_rad({jd_day, jd_frac});
    for (std::size_t s = 0; s < stations_.size(); ++s) {
      station_visible_[s] = mark_visible(stations_[s], scratch_.data(), catalog_.size(), gmst,
                                         static_cast<std::uint8_t>(1u << s), visibility_.data());
    }
  }

  int visible_count(int station) const {
    return (station >= 0 && static_cast<std::size_t>(station) < stations_.size())
               ? static_cast<int>(station_visible_[static_cast<std::size_t>(station)])
               : 0;
  }
  const std::uint8_t* visibility() const { return visibility_.data(); }
  std::size_t visibility_size() const { return visibility_.size(); }

  // Azimuth (deg), elevation (deg) and range (km) of one object as seen from one station,
  // available from look_info() after a successful call. Elevation may be negative (below the
  // horizon); the object is "visible" only at or above the station's minimum elevation.
  bool look(int station, int object, double jd_day, double jd_frac) {
    look_.fill(0.0);
    if (station < 0 || static_cast<std::size_t>(station) >= stations_.size() || !in_range(object)) {
      return false;
    }
    const JulianDate t{jd_day, jd_frac};
    const PropagationResult result = catalog_.propagate_one(static_cast<std::size_t>(object), t);
    if (!result.ok()) return false;
    const LookAngles angles = look_from_station(stations_[static_cast<std::size_t>(station)],
                                                result.state.r, gmst_rad(t));
    look_[0] = angles.azimuth.value * kRadToDeg;
    look_[1] = angles.elevation.value * kRadToDeg;
    look_[2] = angles.range_km;
    return true;
  }
  const double* look_info() const { return look_.data(); }
  std::size_t look_info_count() const { return look_.size(); }

 private:
  bool in_range(int i) const { return i >= 0 && static_cast<std::size_t>(i) < catalog_.size(); }

  Catalog catalog_;
  CatalogLoadStats stats_;
  std::vector<double> scratch_;
  std::vector<float> positions_;
  std::array<double, kInfoSize> info_{};
  std::vector<float> track_;
  std::vector<StationFrame> stations_;
  std::array<std::size_t, kMaxStations> station_visible_{};
  std::vector<std::uint8_t> visibility_;
  std::array<double, 3> look_{};
};

}  // namespace orbitcore