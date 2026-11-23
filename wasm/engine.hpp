#pragma once
// Engine: a thin stateful wrapper around Catalog for the browser build.
// It owns the buffers that JavaScript reads directly from WebAssembly memory.
// Kept free of any Emscripten headers so it can be compiled and tested natively.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "orbitcore/catalog.hpp"
#include "orbitcore/conjunction.hpp"
#include "orbitcore/frames.hpp"
#include "orbitcore/maneuver.hpp"
#include "orbitcore/risk.hpp"
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
    screen_active_ = false;
    stats_ = catalog_.load_tle_text(text);
    scratch_.assign(3 * catalog_.size(), 0.0);
    positions_.assign(3 * catalog_.size(), 0.0f);
    return count();
  }

  // Adds one object from an element set (C++ only; used by tests and what-if scenarios).
  bool add_element_set(const std::string& name, const ElementSet& elements) {
    screen_active_ = false;
    if (!catalog_.add_element_set(name, elements)) return false;
    scratch_.assign(3 * catalog_.size(), 0.0);
    positions_.assign(3 * catalog_.size(), 0.0f);
    visibility_.assign(catalog_.size(), 0);
    return true;
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

  // ---- Conjunction screening -------------------------------------------------------------
  // A screening session finds the close approaches of one object (the "primary") with the rest
  // of the catalog over a time window. It runs in small steps so that a UI can stay responsive:
  //   screen_begin(...);  while (screen_step(n) < 1.0) { ...let the page draw a frame... }
  // then read screen_results(). Layout of one encounter (kEncounterStride doubles), sorted by time:
  //   0 secondary index   1-2 time of closest approach as a Julian date (day, fraction)
  //   3 miss distance, km   4 relative speed, km/s   5-7 secondary minus primary in the primary's
  //   radial / in-track / cross-track axes, km   8 Pc_max   9 sigma* (km)   10 Pc_ceil
  //   11-13 primary position, km (TEME)   14-16 primary velocity, km/s
  //   17-19 secondary position, km   20-22 secondary velocity, km/s
  // hbr_m is the combined hard-body radius in metres used for the probabilities.
  static constexpr std::size_t kEncounterStride = 23;

  bool screen_begin(int primary, double jd_day, double jd_frac, double span_minutes, double threshold_km,
                    double hbr_m) {
    screening_ = ScreeningResult{};
    screen_packed_.clear();
    plan_packed_.clear();
    screen_stats_.fill(0.0);
    screen_active_ = false;
    if (!in_range(primary) || !(span_minutes > 0.0) || !(threshold_km > 0.0) || !(hbr_m > 0.0)) return false;
    screen_primary_ = static_cast<std::size_t>(primary);
    screen_start_ = JulianDate{jd_day, jd_frac};
    screen_span_minutes_ = span_minutes;
    screen_options_ = ScreeningOptions{};
    screen_options_.threshold_km = threshold_km;
    screen_hbr_km_ = hbr_m / 1000.0;
    screen_cursor_ = 0;
    screen_active_ = true;
    return true;
  }

  // Screens up to `count` more objects. Returns the progress in [0, 1]; 1 means finished (also
  // returned when no session is active). Results are available once it reaches 1.
  double screen_step(int count) {
    if (!screen_active_) return 1.0;
    const std::size_t total = catalog_.size();
    const std::size_t last = std::min(total, screen_cursor_ + static_cast<std::size_t>(std::max(count, 1)));
    screen_primary_range(catalog_, screen_primary_, screen_start_, screen_span_minutes_, screen_options_,
                         screen_cursor_, last, screening_);
    screen_cursor_ = last;
    if (screen_cursor_ < total) return static_cast<double>(screen_cursor_) / static_cast<double>(total);
    finish_screening();
    return 1.0;
  }

  // ---- Avoidance-burn planning -------------------------------------------------------------
  // For one encounter of the last finished screening session (index into screen_results(), sorted by
  // time), finds the smallest impulsive burn of the primary, at each of several lead times before the
  // closest approach, that raises the miss distance to target_miss_km. Method and limits: maneuver.hpp.
  // Returns the number of options (0 if there is no finished session, no such encounter, or a bad
  // target). Layout of one option (kBurnStride doubles), in order of increasing lead time:
  //   0 lead time before the closest approach, s
  //   1 status: 0 ok, 1 no burn needed (already far enough), 2 unreachable at this lead time,
  //     3 the burn would have to happen before the screening started
  //   2-4 burn delta-v in the primary's radial / in-track / cross-track axes at the burn time, m/s
  //   5 delta-v magnitude, m/s   6 miss distance before, km   7 miss distance after, km
  //   8 Pc_max after the burn (same hard-body size as the screening)
  //   9-10 burn time as a Julian date (day, fraction)   11-13 primary position at the burn, km (TEME)
  //   14-16 burn direction, unit vector (TEME)
  static constexpr std::size_t kBurnStride = 17;

  int plan_avoidance(int event_index, double target_miss_km) {
    plan_packed_.clear();
    if (screen_active_ || event_index < 0 || !(target_miss_km > 0.0) ||
        static_cast<std::size_t>(event_index) >= screening_.encounters.size()) {
      return 0;
    }
    const Encounter& e = screening_.encounters[static_cast<std::size_t>(event_index)];
    const double n_rad_s = catalog_.elements(screen_primary_).no_kozai / 60.0;
    if (!(n_rad_s > 0.0)) return 0;

    // The encounter geometry in the primary's axes at the closest approach.
    const Vec3d primary_r{e.primary_position.x, e.primary_position.y, e.primary_position.z};
    const Vec3d primary_v{e.primary_velocity.x, e.primary_velocity.y, e.primary_velocity.z};
    const Vec3d relative_v{e.secondary_velocity.x - e.primary_velocity.x,
                           e.secondary_velocity.y - e.primary_velocity.y,
                           e.secondary_velocity.z - e.primary_velocity.z};
    BurnRequest request;
    request.miss_rtn_km = {e.miss_rtn.x, e.miss_rtn.y, e.miss_rtn.z};
    request.relative_velocity_rtn_km_s = project_on_rtn(rtn_axes(primary_r, primary_v), relative_v);
    request.mean_motion_rad_s = n_rad_s;
    request.target_miss_km = target_miss_km;

    const double period_s = kTwoPi / n_rad_s;
    static constexpr double kLeadOrbits[] = {0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0};
    for (const double orbits : kLeadOrbits) {
      const double lead_s = orbits * period_s;
      const BurnOption option = plan_burn(request, lead_s);
      const JulianDate burn_time = add_minutes(e.tca, -lead_s / 60.0);
      int status = static_cast<int>(option.status);

      // Where the primary is at the burn, and which way the burn points in the inertial frame.
      Vec3d burn_position{0.0, 0.0, 0.0}, direction{0.0, 0.0, 0.0};
      const PropagationResult at_burn = catalog_.propagate_one(screen_primary_, burn_time);
      if (at_burn.ok()) {
        const Vec3d r{at_burn.state.r.x, at_burn.state.r.y, at_burn.state.r.z};
        const Vec3d v{at_burn.state.v.x, at_burn.state.v.y, at_burn.state.v.z};
        burn_position = r;
        if (option.delta_v_m_s > 0.0) {
          const Vec3d dv = rtn_to_inertial(rtn_axes(r, v), option.delta_v_rtn_m_s);
          for (std::size_t k = 0; k < 3; ++k) direction[k] = dv[k] / option.delta_v_m_s;
        }
      } else if (status == static_cast<int>(BurnStatus::Ok)) {
        status = static_cast<int>(BurnStatus::Unreachable);
      }
      if (status == static_cast<int>(BurnStatus::Ok) && minutes_between(screen_start_, burn_time) < 0.0) {
        status = static_cast<int>(BurnStatus::InThePast);
      }

      const double miss_after = option.status == BurnStatus::Unreachable ? option.miss_before_km : option.miss_after_km;
      const double pc_after = miss_after > 0.0 ? maximum_collision_probability(miss_after, screen_hbr_km_).probability : 0.0;
      const double row[kBurnStride] = {
          lead_s, static_cast<double>(status),
          option.delta_v_rtn_m_s[0], option.delta_v_rtn_m_s[1], option.delta_v_rtn_m_s[2], option.delta_v_m_s,
          option.miss_before_km, miss_after, pc_after,
          burn_time.day, burn_time.frac,
          burn_position[0], burn_position[1], burn_position[2],
          direction[0], direction[1], direction[2]};
      plan_packed_.insert(plan_packed_.end(), row, row + kBurnStride);
    }
    return static_cast<int>(plan_packed_.size() / kBurnStride);
  }

  const double* plan_results() const { return plan_packed_.data(); }
  std::size_t plan_results_size() const { return plan_packed_.size(); }  // number of doubles

  int screen_event_count() const { return static_cast<int>(screening_.encounters.size()); }  // so far
  const double* screen_results() const { return screen_packed_.data(); }
  std::size_t screen_results_size() const { return screen_packed_.size(); }  // number of doubles
  // objects in the catalog, objects screened after the radial-band filter, SGP4 evaluations
  const double* screen_stats() const { return screen_stats_.data(); }
  std::size_t screen_stats_size() const { return screen_stats_.size(); }

 private:
  void finish_screening() {
    sort_encounters(screening_, screen_start_);
    screen_packed_.clear();
    screen_packed_.reserve(screening_.encounters.size() * kEncounterStride);
    for (const Encounter& e : screening_.encounters) {
      const MaxProbability worst = maximum_collision_probability(e.miss_distance_km, screen_hbr_km_);
      const double ceiling = collision_probability_ceiling(e.miss_distance_km, screen_hbr_km_);
      const double row[kEncounterStride] = {
          static_cast<double>(e.secondary), e.tca.day, e.tca.frac, e.miss_distance_km, e.relative_speed_km_s,
          e.miss_rtn.x, e.miss_rtn.y, e.miss_rtn.z, worst.probability, worst.sigma_km, ceiling,
          e.primary_position.x, e.primary_position.y, e.primary_position.z,
          e.primary_velocity.x, e.primary_velocity.y, e.primary_velocity.z,
          e.secondary_position.x, e.secondary_position.y, e.secondary_position.z,
          e.secondary_velocity.x, e.secondary_velocity.y, e.secondary_velocity.z};
      screen_packed_.insert(screen_packed_.end(), row, row + kEncounterStride);
    }
    screen_stats_ = {static_cast<double>(screening_.stats.objects_total),
                     static_cast<double>(screening_.stats.objects_screened),
                     static_cast<double>(screening_.stats.propagations)};
    screen_active_ = false;
  }

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

  bool screen_active_{false};
  ScreeningResult screening_;
  ScreeningOptions screen_options_;
  std::size_t screen_primary_{0};
  std::size_t screen_cursor_{0};
  JulianDate screen_start_{};
  double screen_span_minutes_{0.0};
  double screen_hbr_km_{0.01};
  std::vector<double> screen_packed_;
  std::array<double, 3> screen_stats_{};
  std::vector<double> plan_packed_;
};

}  // namespace orbitcore