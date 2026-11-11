#include "orbitcore/conjunction.hpp"

#include <algorithm>
#include <cmath>

namespace orbitcore {

namespace {

double dot3(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double norm3(const double a[3]) { return std::sqrt(dot3(a, a)); }

// Evaluates one (primary, secondary) pair at times measured in seconds from the screening start.
struct PairEvaluator {
  const Sgp4* primary;
  const Sgp4* secondary;
  double primary_minutes0;    // minutes from the primary's epoch to the screening start
  double secondary_minutes0;
  std::size_t* propagations;

  bool states(double t_s, PropagationResult& a, PropagationResult& b) const {
    a = primary->propagate(primary_minutes0 + t_s / 60.0);
    b = secondary->propagate(secondary_minutes0 + t_s / 60.0);
    *propagations += 2;
    return a.ok() && b.ok();
  }

  // Relative position and velocity, secondary minus primary.
  bool relative(double t_s, double r[3], double v[3]) const {
    PropagationResult a, b;
    if (!states(t_s, a, b)) return false;
    r[0] = b.state.r.x - a.state.r.x; r[1] = b.state.r.y - a.state.r.y; r[2] = b.state.r.z - a.state.r.z;
    v[0] = b.state.v.x - a.state.v.x; v[1] = b.state.v.y - a.state.v.y; v[2] = b.state.v.z - a.state.v.z;
    return true;
  }
};

// The closest approach lies where the range rate (r . v) changes from negative to non-negative.
// Given f(t_lo) < 0 <= f(t_hi), bisect to the requested time resolution.
double refine_closest_approach(const PairEvaluator& pair, double t_lo, double t_hi, double tolerance_s) {
  double r[3], v[3];
  while (t_hi - t_lo > tolerance_s) {
    const double mid = 0.5 * (t_lo + t_hi);
    if (!pair.relative(mid, r, v)) break;
    if (dot3(r, v) < 0.0) t_lo = mid; else t_hi = mid;
  }
  return 0.5 * (t_lo + t_hi);
}

bool make_encounter(const PairEvaluator& pair, double t_s, JulianDate start, std::size_t secondary,
                    Encounter& out) {
  PropagationResult a, b;
  if (!pair.states(t_s, a, b)) return false;
  const Position<Frame::TEME> rel{b.state.r.x - a.state.r.x, b.state.r.y - a.state.r.y,
                                  b.state.r.z - a.state.r.z};
  const Velocity<Frame::TEME> rel_v{b.state.v.x - a.state.v.x, b.state.v.y - a.state.v.y,
                                    b.state.v.z - a.state.v.z};
  out.secondary = secondary;
  out.tca = add_minutes(start, t_s / 60.0);
  out.miss_distance_km = rel.norm();
  out.relative_speed_km_s = rel_v.norm();
  out.primary_position = a.state.r;
  out.secondary_position = b.state.r;
  out.primary_velocity = a.state.v;
  out.secondary_velocity = b.state.v;
  out.miss_rtn = to_rtn(rel, a.state.r, a.state.v);
  return true;
}

struct RadialBand { double perigee_km; double apogee_km; };

RadialBand radial_band(const ElementSet& e) {
  const GravityModel g = wgs72();
  const double a_km = std::pow(g.xke / e.no_kozai, 2.0 / 3.0) * g.radius_earth_km;
  return {a_km * (1.0 - e.ecco), a_km * (1.0 + e.ecco)};
}

void sort_by_time(std::vector<Encounter>& encounters, JulianDate start) {
  std::sort(encounters.begin(), encounters.end(), [&](const Encounter& x, const Encounter& y) {
    return minutes_between(start, x.tca) < minutes_between(start, y.tca);
  });
}

}  // namespace

double safe_advance_seconds(const double r[3], const double v[3], double threshold_km,
                            double max_accel_km_s2, double max_step_s) {
  const double vv = dot3(v, v);
  const double rv = dot3(r, v);
  // Over [0, dt] the relative position is r + v*tau + e(tau) with |e(tau)| <= a * tau^2 / 2.
  // The distance can therefore never drop below
  //   (closest distance of the straight-line path over [0, dt]) - a * dt^2 / 2.
  auto lower_bound = [&](double dt) {
    double tau = vv > 0.0 ? -rv / vv : 0.0;
    tau = std::clamp(tau, 0.0, dt);
    const double x[3] = {r[0] + v[0] * tau, r[1] + v[1] * tau, r[2] + v[2] * tau};
    return norm3(x) - 0.5 * max_accel_km_s2 * dt * dt;
  };
  if (norm3(r) <= threshold_km) return 0.0;
  if (lower_bound(max_step_s) > threshold_km) return max_step_s;
  double lo = 0.0, hi = max_step_s;  // lower_bound only decreases as dt grows, so bisect
  for (int i = 0; i < 40; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (lower_bound(mid) > threshold_km) lo = mid; else hi = mid;
  }
  return lo;
}

ScreeningResult screen_primary(const Catalog& catalog, std::size_t primary, JulianDate start,
                               double span_minutes, const ScreeningOptions& options) {
  ScreeningResult result;
  result.stats.objects_total = catalog.size();
  if (primary >= catalog.size()) return result;

  const Sgp4& p = catalog.propagator(primary);
  const double span_s = span_minutes * 60.0;
  const double threshold = options.threshold_km;
  const RadialBand pband = radial_band(p.elements());
  const double primary_minutes0 = minutes_between(p.elements().epoch, start);

  for (std::size_t i = 0; i < catalog.size(); ++i) {
    if (i == primary) continue;
    const Sgp4& s = catalog.propagator(i);
    if (options.radial_margin_km >= 0.0) {
      const RadialBand sband = radial_band(s.elements());
      const double reach = threshold + options.radial_margin_km;
      if (sband.perigee_km > pband.apogee_km + reach || sband.apogee_km < pband.perigee_km - reach) continue;
    }
    ++result.stats.objects_screened;

    const PairEvaluator pair{&p, &s, primary_minutes0, minutes_between(s.elements().epoch, start),
                             &result.stats.propagations};
    double t = 0.0;
    double r[3], v[3];
    if (!pair.relative(t, r, v)) continue;
    while (t < span_s) {
      if (norm3(r) > threshold) {
        const double dt = safe_advance_seconds(r, v, threshold, options.max_relative_acceleration_km_s2,
                                               options.max_step_s);
        if (dt >= options.near_step_s) {                 // provably no encounter before t + dt
          t += dt;
          if (t >= span_s) break;
          if (!pair.relative(t, r, v)) break;
          continue;
        }
      }
      // Close to the threshold: fine steps, watching the range rate for a sign change.
      const double f0 = dot3(r, v);
      const double t1 = std::min(t + options.near_step_s, span_s);
      double r1[3], v1[3];
      if (!pair.relative(t1, r1, v1)) break;
      if (f0 < 0.0 && dot3(r1, v1) >= 0.0) {
        const double tca = refine_closest_approach(pair, t, t1, options.refine_tolerance_s);
        Encounter e;
        if (make_encounter(pair, tca, start, i, e) && e.miss_distance_km <= threshold) {
          result.encounters.push_back(e);
        }
      }
      t = t1;
      for (int k = 0; k < 3; ++k) { r[k] = r1[k]; v[k] = v1[k]; }
    }
  }
  sort_by_time(result.encounters, start);
  return result;
}

ScreeningResult screen_primary_brute_force(const Catalog& catalog, std::size_t primary,
                                           JulianDate start, double span_minutes,
                                           double threshold_km, double step_s,
                                           double refine_tolerance_s) {
  ScreeningResult result;
  result.stats.objects_total = catalog.size();
  if (primary >= catalog.size()) return result;
  const Sgp4& p = catalog.propagator(primary);
  const double span_s = span_minutes * 60.0;
  const double primary_minutes0 = minutes_between(p.elements().epoch, start);

  for (std::size_t i = 0; i < catalog.size(); ++i) {
    if (i == primary) continue;
    ++result.stats.objects_screened;
    const Sgp4& s = catalog.propagator(i);
    const PairEvaluator pair{&p, &s, primary_minutes0, minutes_between(s.elements().epoch, start),
                             &result.stats.propagations};
    double r[3], v[3];
    if (!pair.relative(0.0, r, v)) continue;
    double f_prev = dot3(r, v);
    for (double t_prev = 0.0; t_prev < span_s; t_prev += step_s) {
      const double t_now = std::min(t_prev + step_s, span_s);
      if (!pair.relative(t_now, r, v)) break;
      const double f_now = dot3(r, v);
      if (f_prev < 0.0 && f_now >= 0.0) {
        const double tca = refine_closest_approach(pair, t_prev, t_now, refine_tolerance_s);
        Encounter e;
        if (make_encounter(pair, tca, start, i, e) && e.miss_distance_km <= threshold_km) {
          result.encounters.push_back(e);
        }
      }
      f_prev = f_now;
    }
  }
  sort_by_time(result.encounters, start);
  return result;
}

}  // namespace orbitcore