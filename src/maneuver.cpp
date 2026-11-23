#include "orbitcore/maneuver.hpp"

#include <algorithm>
#include <cmath>

#include "orbitcore/units.hpp"

namespace orbitcore {

namespace {

double dot(const Vec3d& a, const Vec3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double norm(const Vec3d& a) { return std::sqrt(dot(a, a)); }
Vec3d cross(const Vec3d& a, const Vec3d& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3d scaled(const Vec3d& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
Vec3d unit(const Vec3d& a) { return scaled(a, 1.0 / norm(a)); }

// The response matrix of Hill's equations: displacement (km) per unit burn (km/s), 3x3.
struct Response { double m[3][3]; };

Response hill_response(double n, double tau) {
  const double s = std::sin(n * tau), c = std::cos(n * tau);
  Response r{};
  r.m[0][0] = s / n;              r.m[0][1] = 2.0 * (1.0 - c) / n;            r.m[0][2] = 0.0;
  r.m[1][0] = 2.0 * (c - 1.0) / n; r.m[1][1] = (4.0 * s - 3.0 * n * tau) / n;  r.m[1][2] = 0.0;
  r.m[2][0] = 0.0;                r.m[2][1] = 0.0;                            r.m[2][2] = s / n;
  return r;
}

Vec3d respond(const Response& r, const Vec3d& v) {
  return {r.m[0][0] * v[0] + r.m[0][1] * v[1] + r.m[0][2] * v[2],
          r.m[1][0] * v[0] + r.m[1][1] * v[1] + r.m[1][2] * v[2],
          r.m[2][0] * v[0] + r.m[2][1] * v[1] + r.m[2][2] * v[2]};
}

// Row i of (plane basis e_i)^T * Response: how a unit burn along each axis moves the primary along e_i.
Vec3d row_of(const Vec3d& e, const Response& r) {
  return {e[0] * r.m[0][0] + e[1] * r.m[1][0] + e[2] * r.m[2][0],
          e[0] * r.m[0][1] + e[1] * r.m[1][1] + e[2] * r.m[2][1],
          e[0] * r.m[0][2] + e[1] * r.m[1][2] + e[2] * r.m[2][2]};
}

}  // namespace

RtnAxes rtn_axes(const Vec3d& position, const Vec3d& velocity) {
  RtnAxes axes;
  axes.radial = unit(position);
  axes.normal = unit(cross(position, velocity));
  axes.in_track = cross(axes.normal, axes.radial);
  return axes;
}

Vec3d project_on_rtn(const RtnAxes& axes, const Vec3d& v) {
  return {dot(v, axes.radial), dot(v, axes.in_track), dot(v, axes.normal)};
}

Vec3d rtn_to_inertial(const RtnAxes& axes, const Vec3d& rtn) {
  return {rtn[0] * axes.radial[0] + rtn[1] * axes.in_track[0] + rtn[2] * axes.normal[0],
          rtn[0] * axes.radial[1] + rtn[1] * axes.in_track[1] + rtn[2] * axes.normal[1],
          rtn[0] * axes.radial[2] + rtn[1] * axes.in_track[2] + rtn[2] * axes.normal[2]};
}

Vec3d cw_displacement_km(double n_rad_s, double tau_s, const Vec3d& dv_rtn_m_s) {
  return respond(hill_response(n_rad_s, tau_s), scaled(dv_rtn_m_s, 1e-3));
}

BurnOption plan_burn(const BurnRequest& req, double lead_s) {
  BurnOption out;
  out.lead_s = lead_s;
  const double n = req.mean_motion_rad_s;
  const double target = req.target_miss_km;
  const double vnorm = norm(req.relative_velocity_rtn_km_s);
  if (!(n > 0.0) || !(lead_s > 0.0) || !(target > 0.0) || !(vnorm > 0.0)) return out;  // Unreachable

  // An orthonormal basis (e1, e2) of the plane perpendicular to the relative velocity.
  const Vec3d vhat = scaled(req.relative_velocity_rtn_km_s, 1.0 / vnorm);
  int smallest = 0;
  for (int i = 1; i < 3; ++i) if (std::fabs(vhat[static_cast<std::size_t>(i)]) < std::fabs(vhat[static_cast<std::size_t>(smallest)])) smallest = i;
  Vec3d axis{0.0, 0.0, 0.0};
  axis[static_cast<std::size_t>(smallest)] = 1.0;
  const Vec3d e1 = unit(cross(vhat, axis));
  const Vec3d e2 = cross(vhat, e1);

  // The miss vector in that plane.
  const double m1 = dot(e1, req.miss_rtn_km), m2 = dot(e2, req.miss_rtn_km);
  const double d0 = std::hypot(m1, m2);
  out.miss_before_km = d0;
  if (d0 >= target) {
    out.status = BurnStatus::NoBurnNeeded;
    out.miss_after_km = d0;
    return out;
  }

  // How a burn moves the primary within the plane: displacement_in_plane = A * dv, A is 2x3 (km per km/s).
  // The miss vector is "secondary minus primary", so moving the primary by D changes it by -D.
  const Response resp = hill_response(n, lead_s);
  const Vec3d row1 = row_of(e1, resp), row2 = row_of(e2, resp);
  // M = A A^T (2x2, symmetric positive semi-definite). A displacement d costs dv_min^2 = d^T M^-1 d.
  const double M11 = dot(row1, row1), M12 = dot(row1, row2), M22 = dot(row2, row2);
  const double half_trace = 0.5 * (M11 + M22);
  const double radius = std::sqrt(0.25 * (M11 - M22) * (M11 - M22) + M12 * M12);
  const double l1 = half_trace + radius;   // eigenvalues, l1 >= l2 >= 0
  const double l2 = std::max(0.0, half_trace - radius);
  if (!(l1 > 1e-12)) return out;           // the burn cannot move the primary within the plane at all

  double u1x, u1y;                          // eigenvector of l1
  if (std::fabs(M12) > 1e-300) { u1x = l1 - M22; u1y = M12; }
  else if (M11 >= M22) { u1x = 1.0; u1y = 0.0; }
  else { u1x = 0.0; u1y = 1.0; }
  const double u1n = std::hypot(u1x, u1y);
  u1x /= u1n; u1y /= u1n;
  const double u2x = -u1y, u2y = u1x;

  Vec3d dv_kms{0.0, 0.0, 0.0};
  if (l2 < 1e-9 * l1) {
    // Rank one (for example, a whole number of orbits): displacement is only possible along u1.
    // Choose s so that |m + s u1| = target; the smaller root costs less.
    const double b = m1 * u1x + m2 * u1y;
    const double disc = b * b - (d0 * d0 - target * target);   // positive, since d0 < target
    const double root = std::sqrt(std::max(0.0, disc));
    const double s_a = -b + root, s_b = -b - root;
    const double s = std::fabs(s_a) <= std::fabs(s_b) ? s_a : s_b;
    const double w1 = s / l1 * u1x, w2 = s / l1 * u1y;          // M^+ delta, as coordinates along e1, e2
    for (std::size_t k = 0; k < 3; ++k) dv_kms[k] = -(w1 * row1[k] + w2 * row2[k]);   // the miss changes by -A dv
  } else {
    // Full rank: the new miss vector lies on the circle |m + d| = target. Minimise d^T M^-1 d over it.
    auto delta = [&](double theta, double& dx, double& dy) {
      dx = target * std::cos(theta) - m1;
      dy = target * std::sin(theta) - m2;
    };
    auto cost = [&](double theta) {
      double dx, dy;
      delta(theta, dx, dy);
      const double a = dx * u1x + dy * u1y, b = dx * u2x + dy * u2y;
      return a * a / l1 + b * b / l2;
    };
    const int samples = 3600;
    int best = 0;
    double best_cost = cost(0.0);
    for (int k = 1; k < samples; ++k) {
      const double c = cost(2.0 * kPi * k / samples);
      if (c < best_cost) { best_cost = c; best = k; }
    }
    double lo = 2.0 * kPi * (best - 1) / samples, hi = 2.0 * kPi * (best + 1) / samples;
    const double phi = 0.5 * (std::sqrt(5.0) - 1.0);
    double c1 = hi - phi * (hi - lo), c2 = lo + phi * (hi - lo);
    double f1 = cost(c1), f2 = cost(c2);
    for (int it = 0; it < 80; ++it) {
      if (f1 < f2) { hi = c2; c2 = c1; f2 = f1; c1 = hi - phi * (hi - lo); f1 = cost(c1); }
      else { lo = c1; c1 = c2; f1 = f2; c2 = lo + phi * (hi - lo); f2 = cost(c2); }
    }
    double dx, dy;
    delta(0.5 * (lo + hi), dx, dy);
    const double a = dx * u1x + dy * u1y, b = dx * u2x + dy * u2y;
    const double w1 = (a / l1) * u1x + (b / l2) * u2x;           // M^-1 delta along e1
    const double w2 = (a / l1) * u1y + (b / l2) * u2y;           // ... and along e2
    for (std::size_t k = 0; k < 3; ++k) dv_kms[k] = -(w1 * row1[k] + w2 * row2[k]);   // the miss changes by -A dv
  }

  out.delta_v_rtn_m_s = scaled(dv_kms, 1e3);
  out.delta_v_m_s = norm(out.delta_v_rtn_m_s);
  out.displacement_rtn_km = respond(resp, dv_kms);
  const double after1 = m1 - dot(e1, out.displacement_rtn_km);
  const double after2 = m2 - dot(e2, out.displacement_rtn_km);
  out.miss_after_km = std::hypot(after1, after2);
  if (!std::isfinite(out.delta_v_m_s) || !std::isfinite(out.miss_after_km)) {
    out = BurnOption{};
    out.lead_s = lead_s;
    out.miss_before_km = d0;
    return out;
  }
  out.status = BurnStatus::Ok;
  return out;
}

std::vector<BurnOption> plan_burns(const BurnRequest& request, const std::vector<double>& lead_times_s) {
  std::vector<BurnOption> options;
  options.reserve(lead_times_s.size());
  for (double lead : lead_times_s) options.push_back(plan_burn(request, lead));
  return options;
}

}  // namespace orbitcore