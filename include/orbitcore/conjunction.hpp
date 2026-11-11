#pragma once
// Conjunction screening: find the close approaches between one "primary" object and the rest
// of a catalog over a time window.
//
// Method ("conservative advancement"): relative motion cannot change faster than the relative
// velocity, and its curvature is bounded by the relative acceleration. From the current
// relative state we can therefore compute a time step over which the two objects PROVABLY
// cannot come within the screening threshold, and skip ahead by it. Close to the threshold we
// fall back to fine steps and look for the moment the range rate changes sign (the closest
// approach), then refine that moment by bisection. Nothing is skipped that could contain an
// event, so no close approach is missed (given the acceleration bound below).
#include <cstddef>
#include <vector>

#include "orbitcore/catalog.hpp"
#include "orbitcore/frames.hpp"

namespace orbitcore {

struct ScreeningOptions {
  double threshold_km = 10.0;  // report encounters whose miss distance is at most this
  // Upper bound on the relative acceleration of two objects, km/s^2. Two bodies above 150 km
  // altitude feel at most 2 * 0.0094 km/s^2 from gravity; 0.02 leaves room for perturbations.
  double max_relative_acceleration_km_s2 = 0.02;
  double max_step_s = 600.0;         // longest skip
  double near_step_s = 1.0;          // fine step used close to the threshold
  double refine_tolerance_s = 1e-4;  // time resolution of the closest-approach search
  // Objects whose orbit never comes within threshold + margin of the primary's radial band
  // (perigee..apogee radius) are skipped without propagation. Negative disables the filter.
  double radial_margin_km = 100.0;
};

struct Encounter {
  std::size_t secondary{0};          // index into the catalog
  JulianDate tca{};                  // time of closest approach (UTC)
  double miss_distance_km{0.0};
  double relative_speed_km_s{0.0};
  Position<Frame::TEME> primary_position{};
  Position<Frame::TEME> secondary_position{};
  Velocity<Frame::TEME> primary_velocity{};
  Velocity<Frame::TEME> secondary_velocity{};
  Position<Frame::RTN> miss_rtn{};   // secondary minus primary, in the primary's radial/in-track/cross-track axes
};

struct ScreeningStats {
  std::size_t objects_total{0};
  std::size_t objects_screened{0};   // left after the radial-band filter
  std::size_t propagations{0};       // SGP4 evaluations used
};

struct ScreeningResult {
  std::vector<Encounter> encounters; // sorted by time of closest approach
  ScreeningStats stats;
};

// How far ahead (seconds, at most max_step_s) two objects with relative position r and
// relative velocity v (km, km/s) are guaranteed to stay farther apart than threshold_km,
// assuming their relative acceleration never exceeds max_accel_km_s2. Returns 0 if they are
// already within the threshold.
double safe_advance_seconds(const double r[3], const double v[3], double threshold_km,
                            double max_accel_km_s2, double max_step_s);

// Screens `catalog[primary]` against every other object for encounters whose closest approach
// falls within [start, start + span_minutes].
ScreeningResult screen_primary(const Catalog& catalog, std::size_t primary, JulianDate start,
                               double span_minutes, const ScreeningOptions& options = {});

// Reference implementation for validation: steps every pair at a fixed `step_s` with no
// skipping and no radial-band filter, and refines each range-rate sign change the same way.
// Slow, but its search strategy is independent of screen_primary.
ScreeningResult screen_primary_brute_force(const Catalog& catalog, std::size_t primary,
                                           JulianDate start, double span_minutes,
                                           double threshold_km, double step_s,
                                           double refine_tolerance_s = 1e-4);

}  // namespace orbitcore