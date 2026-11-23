#include <algorithm>
#include <cmath>
#include <vector>

#include "check.hpp"
#include "orbitcore/maneuver.hpp"
#include "orbitcore/units.hpp"

using namespace orbitcore;

namespace {

constexpr double kMu = 398600.8;  // km^3/s^2

double dot3(const Vec3d& a, const Vec3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double norm3(const Vec3d& a) { return std::sqrt(dot3(a, a)); }
Vec3d add3(const Vec3d& a, const Vec3d& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec3d sub3(const Vec3d& a, const Vec3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
Vec3d mul3(const Vec3d& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }

// A reference that shares nothing with the planner: two-body motion integrated with RK4.
struct State { Vec3d r, v; };

Vec3d gravity(const Vec3d& r) { return mul3(r, -kMu / std::pow(norm3(r), 3.0)); }

State rk4_step(const State& s, double h) {
  const Vec3d k1r = s.v, k1v = gravity(s.r);
  const Vec3d k2r = add3(s.v, mul3(k1v, h / 2)), k2v = gravity(add3(s.r, mul3(k1r, h / 2)));
  const Vec3d k3r = add3(s.v, mul3(k2v, h / 2)), k3v = gravity(add3(s.r, mul3(k2r, h / 2)));
  const Vec3d k4r = add3(s.v, mul3(k3v, h)), k4v = gravity(add3(s.r, mul3(k3r, h)));
  State out;
  for (std::size_t i = 0; i < 3; ++i) {
    out.r[i] = s.r[i] + h / 6 * (k1r[i] + 2 * k2r[i] + 2 * k3r[i] + k4r[i]);
    out.v[i] = s.v[i] + h / 6 * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i]);
  }
  return out;
}

// Advance by `duration` seconds (negative runs backwards) in steps of at most `step`.
State propagate(State s, double duration, double step = 1.0) {
  const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(duration) / step)));
  const double h = duration / n;
  for (int i = 0; i < n; ++i) s = rk4_step(s, h);
  return s;
}

// A circular orbit of radius r0 inclined by `incl` rad, at the node: position along x.
State circular_state(double r0, double incl) {
  const double v0 = std::sqrt(kMu / r0);
  return {{r0, 0.0, 0.0}, {0.0, v0 * std::cos(incl), v0 * std::sin(incl)}};
}

}  // namespace

TEST_CASE(rtn_axes_are_orthonormal_and_invertible) {
  const State s = circular_state(6900.0, 51.6 * kDegToRad);
  const RtnAxes ax = rtn_axes(s.r, s.v);
  CHECK_NEAR(norm3(ax.radial), 1.0, 1e-14);
  CHECK_NEAR(norm3(ax.in_track), 1.0, 1e-14);
  CHECK_NEAR(norm3(ax.normal), 1.0, 1e-14);
  CHECK_NEAR(dot3(ax.radial, ax.in_track), 0.0, 1e-14);
  CHECK_NEAR(dot3(ax.radial, ax.normal), 0.0, 1e-14);
  CHECK_NEAR(dot3(ax.in_track, ax.normal), 0.0, 1e-14);
  // right-handed: R x T = N, and T points along the velocity for a circular orbit
  const Vec3d rxt = {ax.radial[1] * ax.in_track[2] - ax.radial[2] * ax.in_track[1],
                     ax.radial[2] * ax.in_track[0] - ax.radial[0] * ax.in_track[2],
                     ax.radial[0] * ax.in_track[1] - ax.radial[1] * ax.in_track[0]};
  CHECK_NEAR(norm3(sub3(rxt, ax.normal)), 0.0, 1e-14);
  CHECK_NEAR(dot3(ax.in_track, mul3(s.v, 1.0 / norm3(s.v))), 1.0, 1e-14);
  const Vec3d v{0.3, -1.2, 2.5};
  CHECK_NEAR(norm3(sub3(rtn_to_inertial(ax, project_on_rtn(ax, v)), v)), 0.0, 1e-13);
  // equatorial orbit: R = x, T = y, N = z
  const RtnAxes eq = rtn_axes({7000.0, 0.0, 0.0}, {0.0, 7.5, 0.0});
  CHECK_NEAR(eq.radial[0], 1.0, 1e-15);
  CHECK_NEAR(eq.in_track[1], 1.0, 1e-15);
  CHECK_NEAR(eq.normal[2], 1.0, 1e-15);
}

TEST_CASE(hill_response_special_cases) {
  const double n = std::sqrt(kMu / std::pow(6900.0, 3.0));
  const double period = kTwoPi / n;
  // After a whole number of orbits only an in-track burn moves the object, by -3 tau dv.
  const Vec3d one = cw_displacement_km(n, period, {0.0, 0.3, 0.0});
  CHECK_NEAR(one[0], 0.0, 1e-9);
  CHECK_NEAR(one[1], -3.0 * period * 0.3e-3, 1e-9);
  CHECK_NEAR(one[2], 0.0, 1e-9);
  const Vec3d two = cw_displacement_km(n, 2.0 * period, {0.0, 0.3, 0.0});
  CHECK_NEAR(two[1], -3.0 * (2.0 * period) * 0.3e-3, 1e-8);   // twice as early, twice the effect
  CHECK_NEAR(norm3(cw_displacement_km(n, period, {0.4, 0.0, 0.0})), 0.0, 1e-9);   // a radial burn comes back
  // Half an orbit after a radial burn: -4 dv / n along track; after an in-track burn: 4 dv / n radially.
  const Vec3d half_r = cw_displacement_km(n, 0.5 * period, {0.4, 0.0, 0.0});
  CHECK_NEAR(half_r[1], -4.0 * 0.4e-3 / n, 1e-9);
  const Vec3d half_t = cw_displacement_km(n, 0.5 * period, {0.0, 0.4, 0.0});
  CHECK_NEAR(half_t[0], 4.0 * 0.4e-3 / n, 1e-9);
  // Cross-track motion is a simple oscillation: sin(n tau) / n.
  const Vec3d z = cw_displacement_km(n, 0.25 * period, {0.0, 0.0, 0.5});
  CHECK_NEAR(z[2], 0.5e-3 / n, 1e-12);
  // Linear in the burn.
  const Vec3d a = cw_displacement_km(n, 1234.5, {0.1, 0.2, 0.3});
  const Vec3d b = cw_displacement_km(n, 1234.5, {0.2, 0.4, 0.6});
  CHECK_NEAR(norm3(sub3(b, mul3(a, 2.0))), 0.0, 1e-12);
}

TEST_CASE(hill_response_matches_two_body_integration) {
  // The linear model against the real (nonlinear) two-body motion. A 0.3 m/s burn is small compared
  // with 7.6 km/s, so the two must agree to a fraction of a percent.
  const double r0 = 6900.0;
  const State burn = circular_state(r0, 51.6 * kDegToRad);
  const double n = std::sqrt(kMu / (r0 * r0 * r0));
  const double period = kTwoPi / n;
  const RtnAxes axes0 = rtn_axes(burn.r, burn.v);
  const Vec3d burns[] = {{0.0, 0.3, 0.0}, {0.2, 0.0, 0.0}, {0.0, 0.0, 0.25}, {0.1, -0.2, 0.15}};
  for (const Vec3d& dv_m_s : burns) {
    for (double orbits : {0.5, 1.0, 1.7, 2.3}) {
      const double tau = orbits * period;
      State burned = burn;
      burned.v = add3(burn.v, rtn_to_inertial(axes0, mul3(dv_m_s, 1e-3)));
      const State nominal_end = propagate(burn, tau);
      const State burned_end = propagate(burned, tau);
      const RtnAxes axes_end = rtn_axes(nominal_end.r, nominal_end.v);
      const Vec3d truth = project_on_rtn(axes_end, sub3(burned_end.r, nominal_end.r));
      const Vec3d linear = cw_displacement_km(n, tau, dv_m_s);
      CHECK(norm3(sub3(truth, linear)) <= 0.01 * norm3(linear) + 1e-4);
    }
  }
}

TEST_CASE(plan_burn_special_cases) {
  const double n = std::sqrt(kMu / std::pow(6900.0, 3.0));
  const double period = kTwoPi / n;
  BurnRequest req;
  req.mean_motion_rad_s = n;
  req.miss_rtn_km = {0.0, 0.0, 2.0};
  req.relative_velocity_rtn_km_s = {7.0, 0.0, 0.0};   // closing along the radial direction: the plane is (in-track, cross-track)
  req.target_miss_km = 5.0;

  // Whole number of orbits: only in-track burns act, and the in-track axis lies in the plane.
  // The new miss vector is (s, 0, 2) with |.| = 5, so the object must move s = sqrt(21) km.
  const BurnOption one = plan_burn(req, period);
  CHECK(one.status == BurnStatus::Ok);
  CHECK_NEAR(one.miss_before_km, 2.0, 1e-12);
  CHECK_NEAR(one.miss_after_km, 5.0, 1e-9);
  CHECK_NEAR(std::fabs(one.displacement_rtn_km[1]), std::sqrt(21.0), 1e-6);
  CHECK_NEAR(one.delta_v_m_s, 1e3 * std::sqrt(21.0) / (3.0 * period), 1e-6);
  CHECK_NEAR(one.delta_v_rtn_m_s[0], 0.0, 1e-9);
  CHECK_NEAR(one.delta_v_rtn_m_s[2], 0.0, 1e-9);
  // Twice as early costs half as much (the in-track effect grows linearly with lead time).
  const BurnOption two = plan_burn(req, 2.0 * period);
  CHECK(two.status == BurnStatus::Ok);
  CHECK_NEAR(two.delta_v_m_s / one.delta_v_m_s, 0.5, 1e-9);
  CHECK(plan_burn(req, 3.0 * period).delta_v_m_s < two.delta_v_m_s);

  // Already far enough apart: nothing to do.
  BurnRequest far = req;
  far.miss_rtn_km = {0.0, 0.0, 8.0};
  const BurnOption none = plan_burn(far, period);
  CHECK(none.status == BurnStatus::NoBurnNeeded);
  CHECK_NEAR(none.delta_v_m_s, 0.0, 0.0);
  CHECK_NEAR(none.miss_after_km, 8.0, 1e-12);

  // Closing along the in-track direction: moving the primary in-track cannot change the miss, and
  // after a whole number of orbits nothing else moves it, so no burn works.
  BurnRequest head_on = req;
  head_on.relative_velocity_rtn_km_s = {0.0, -7.0, 0.0};
  head_on.miss_rtn_km = {2.0, 0.0, 0.0};
  CHECK(plan_burn(head_on, period).status == BurnStatus::Unreachable);
  // ...but at a lead time where radial and cross-track motion exist, it works.
  const BurnOption half = plan_burn(head_on, 0.37 * period);
  CHECK(half.status == BurnStatus::Ok);
  CHECK_NEAR(half.miss_after_km, 5.0, 1e-9);

  // A direct hit (zero miss) still has a finite answer.
  BurnRequest direct = req;
  direct.miss_rtn_km = {0.0, 0.0, 0.0};
  const BurnOption hit = plan_burn(direct, 1.5 * period);
  CHECK(hit.status == BurnStatus::Ok);
  CHECK_NEAR(hit.miss_after_km, 5.0, 1e-9);
  CHECK(std::isfinite(hit.delta_v_m_s) && hit.delta_v_m_s > 0.0);

  // Invalid inputs.
  CHECK(plan_burn(req, 0.0).status == BurnStatus::Unreachable);
  CHECK(plan_burn(req, -10.0).status == BurnStatus::Unreachable);
  BurnRequest no_motion = req; no_motion.mean_motion_rad_s = 0.0;
  CHECK(plan_burn(no_motion, period).status == BurnStatus::Unreachable);
  BurnRequest no_speed = req; no_speed.relative_velocity_rtn_km_s = {0.0, 0.0, 0.0};
  CHECK(plan_burn(no_speed, period).status == BurnStatus::Unreachable);
  BurnRequest no_target = req; no_target.target_miss_km = 0.0;
  CHECK(plan_burn(no_target, period).status == BurnStatus::Unreachable);
  CHECK(plan_burns(req, {period, 2.0 * period, 3.0 * period}).size() == 3);
}

namespace {

// Everything needed to replay an encounter with two-body motion, independently of the planner:
// two objects 2 km apart radially at the closest approach (t = 0), crossing at angle `alpha`.
struct Crossing {
  State primary, secondary;       // at t = 0, the closest approach
  BurnRequest request;
  double period;
};

Crossing make_crossing(double alpha, double radial_miss_km, double target_km) {
  const double r0 = 6900.0;
  Crossing c;
  c.primary = circular_state(r0, 51.6 * kDegToRad);
  const RtnAxes axes = rtn_axes(c.primary.r, c.primary.v);
  const double v0 = norm3(c.primary.v);
  c.secondary.r = add3(c.primary.r, mul3(axes.radial, radial_miss_km));
  c.secondary.v = add3(mul3(axes.in_track, v0 * std::cos(alpha)), mul3(axes.normal, v0 * std::sin(alpha)));
  c.request.miss_rtn_km = {radial_miss_km, 0.0, 0.0};
  c.request.relative_velocity_rtn_km_s = {0.0, v0 * (std::cos(alpha) - 1.0), v0 * std::sin(alpha)};
  c.request.mean_motion_rad_s = std::sqrt(kMu / (r0 * r0 * r0));
  c.request.target_miss_km = target_km;
  c.period = kTwoPi / c.request.mean_motion_rad_s;
  return c;
}

// Closest distance between the burned primary and the secondary around the closest approach,
// found by stepping both objects with two-body motion.
double miss_after_burn(const Crossing& c, const BurnOption& option) {
  const State at_burn = propagate(c.primary, -option.lead_s);            // primary, `lead` before the closest approach
  const RtnAxes axes = rtn_axes(at_burn.r, at_burn.v);
  State burned = at_burn;
  burned.v = add3(at_burn.v, rtn_to_inertial(axes, mul3(option.delta_v_rtn_m_s, 1e-3)));
  const double window = 12.0, step = 0.01;
  State p = propagate(burned, option.lead_s - window);                   // up to `window` seconds before t = 0
  State s = propagate(c.secondary, -window);
  double best = 1e300;
  for (int i = 0; i <= static_cast<int>(2.0 * window / step); ++i) {
    best = std::min(best, norm3(sub3(p.r, s.r)));
    p = rk4_step(p, step);
    s = rk4_step(s, step);
  }
  return best;
}

}  // namespace

TEST_CASE(planned_burn_achieves_the_target_miss_in_two_body_motion) {
  // The whole chain, checked against brute-force dynamics: plan a burn from the encounter geometry,
  // then actually apply it to the primary, run both objects through the encounter, and measure.
  for (double alpha : {60.0 * kDegToRad, 90.0 * kDegToRad, 140.0 * kDegToRad}) {
    const Crossing c = make_crossing(alpha, 2.0, 6.0);
    const double before = miss_after_burn(c, BurnOption{});             // no burn: should be the 2 km we started with
    CHECK_NEAR(before, 2.0, 0.01);
    for (double orbits : {0.5, 1.0, 1.5, 2.0, 3.0}) {
      const BurnOption option = plan_burn(c.request, orbits * c.period);
      CHECK(option.status == BurnStatus::Ok);
      if (option.status != BurnStatus::Ok) continue;
      CHECK_NEAR(option.miss_after_km, 6.0, 1e-9);                      // what the planner promises
      const double measured = miss_after_burn(c, option);               // what two-body motion delivers
      CHECK_NEAR(measured, 6.0, 0.01);                                  // within 0.2%: the linearization error
    }
  }
}

TEST_CASE(planned_burn_is_the_smallest_that_works) {
  // Search every burn direction (a dense grid on the sphere) for one that reaches the target with a
  // smaller magnitude than the plan. Nothing may.
  const Crossing c = make_crossing(60.0 * kDegToRad, 2.0, 6.0);
  const double lead = 1.5 * c.period;
  const BurnOption option = plan_burn(c.request, lead);
  CHECK(option.status == BurnStatus::Ok);
  const double n = c.request.mean_motion_rad_s;
  const Vec3d vhat = mul3(c.request.relative_velocity_rtn_km_s, 1.0 / norm3(c.request.relative_velocity_rtn_km_s));
  // The miss distance after a burn: the miss vector minus the primary's displacement, perpendicular to v_rel.
  auto resulting_miss = [&](const Vec3d& dv_m_s) {
    const Vec3d moved = sub3(c.request.miss_rtn_km, cw_displacement_km(n, lead, dv_m_s));
    return norm3(sub3(moved, mul3(vhat, dot3(moved, vhat))));
  };
  CHECK_NEAR(resulting_miss(option.delta_v_rtn_m_s), 6.0, 1e-9);        // the plan does reach the target
  auto best_reach = [&](double magnitude) {
    double best = 0.0;
    for (int i = 0; i <= 360; ++i) {
      const double theta = kPi * i / 360.0;
      for (int j = 0; j < 720; ++j) {
        const double phi = kTwoPi * j / 720.0;
        const Vec3d dir{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
        best = std::max(best, resulting_miss(mul3(dir, magnitude)));
      }
    }
    return best;
  };
  CHECK(best_reach(0.99 * option.delta_v_m_s) < 6.0);                   // a 1% smaller burn falls short in every direction
  CHECK(best_reach(1.01 * option.delta_v_m_s) > 6.0);                   // a 1% larger one has room to spare
}

TEST_CASE(plan_burns_lists_one_option_per_lead_time) {
  const Crossing c = make_crossing(60.0 * kDegToRad, 2.0, 6.0);
  const std::vector<double> leads = {0.25 * c.period, 0.5 * c.period, c.period, 2.0 * c.period};
  const std::vector<BurnOption> options = plan_burns(c.request, leads);
  CHECK(options.size() == leads.size());
  for (std::size_t i = 0; i < options.size(); ++i) {
    CHECK_NEAR(options[i].lead_s, leads[i], 0.0);
    CHECK(options[i].status == BurnStatus::Ok);
    CHECK_NEAR(options[i].delta_v_m_s, norm3(options[i].delta_v_rtn_m_s), 1e-12);
  }
  CHECK(options[3].delta_v_m_s < options[2].delta_v_m_s);               // earlier is cheaper
  CHECK(options[2].delta_v_m_s < options[1].delta_v_m_s);
}