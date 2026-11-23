#pragma once
// Avoidance-burn planning with the linearized relative-motion model (Clohessy-Wiltshire / Hill).
//
// Question answered: "if the primary fires a single impulsive burn `lead` seconds before the closest
// approach, how small can the burn be and still push the miss distance up to a target?"
//
// Model. Work in the primary's radial / in-track / cross-track (RTN) axes. A small impulsive burn
// dv moves the primary away from where it would have been; for a circular reference orbit with mean
// motion n, the displacement `tau` seconds later is exactly linear in dv (Hill's equations):
//
//     [ x ]   [ sin(n tau)/n          2(1 - cos(n tau))/n        0            ] [ dvx ]
//     [ y ] = [ 2(cos(n tau) - 1)/n   (4 sin(n tau) - 3 n tau)/n 0            ] [ dvy ]
//     [ z ]   [ 0                     0                          sin(n tau)/n ] [ dvz ]
//
// (x radial, y in-track, z cross-track). The in-track term grows like 3 tau dvy: a small prograde or
// retrograde burn made early moves the object a long way along its orbit, which is why early burns
// are cheap.
//
// The encounter is fast (about 10 km/s), so only the part of the displacement PERPENDICULAR to the
// relative velocity changes the miss distance; the rest merely shifts the time of closest approach.
// The planner therefore works in the plane perpendicular to the relative velocity: it finds the
// burn of least magnitude whose displacement puts the new miss vector at the target distance. That
// is a small exact optimization (a one-dimensional search over the direction of the new miss
// vector), not a heuristic.
//
// Limits (stated in the UI too): near-circular orbits, short lead times (a few orbits), perturbations
// (J2, drag) ignored in the burn response, impulsive burns, no covariance. It is a planning aid,
// not a flight-dynamics product.
#include <array>
#include <vector>

namespace orbitcore {

using Vec3d = std::array<double, 3>;

// Orthonormal axes of an orbiting object, as unit vectors in the inertial frame the position and
// velocity were given in: radial = r / |r|, normal = r x v / |r x v|, in_track = normal x radial
// (along the velocity for a circular orbit).
struct RtnAxes {
  Vec3d radial{};
  Vec3d in_track{};
  Vec3d normal{};
};

RtnAxes rtn_axes(const Vec3d& position, const Vec3d& velocity);
Vec3d project_on_rtn(const RtnAxes& axes, const Vec3d& inertial);  // components along R, T, N
Vec3d rtn_to_inertial(const RtnAxes& axes, const Vec3d& rtn);      // inverse of project_on_rtn

// Displacement (km) of a burned object relative to where it would have been, `tau_s` seconds after
// an impulsive burn dv_rtn_m_s (m/s, in the radial / in-track / cross-track axes at the burn time),
// for a circular reference orbit with mean motion n_rad_s.
Vec3d cw_displacement_km(double n_rad_s, double tau_s, const Vec3d& dv_rtn_m_s);

enum class BurnStatus {
  Ok = 0,
  NoBurnNeeded = 1,   // the miss distance is already at least the target
  Unreachable = 2,    // no burn at this lead time can reach the target (the geometry is degenerate)
  InThePast = 3,      // set by the caller: the burn time would be before the screening start
};

struct BurnRequest {
  Vec3d miss_rtn_km{};                  // secondary minus primary at the closest approach, primary's RTN axes
  Vec3d relative_velocity_rtn_km_s{};   // secondary minus primary, same axes
  double mean_motion_rad_s{0.0};        // the primary's mean motion
  double target_miss_km{0.0};           // wanted miss distance (in the plane perpendicular to the relative velocity)
};

struct BurnOption {
  double lead_s{0.0};                   // the burn happens this long before the closest approach
  BurnStatus status{BurnStatus::Unreachable};
  Vec3d delta_v_rtn_m_s{};              // radial, in-track, cross-track, at the burn time
  double delta_v_m_s{0.0};              // magnitude
  double miss_before_km{0.0};           // in the plane perpendicular to the relative velocity
  double miss_after_km{0.0};            // same plane; equals the target when status is Ok
  Vec3d displacement_rtn_km{};          // where the burn moves the primary at the closest approach
};

// The smallest burn made `lead_s` seconds before the closest approach that reaches the target miss.
BurnOption plan_burn(const BurnRequest& request, double lead_s);

// One plan_burn per lead time.
std::vector<BurnOption> plan_burns(const BurnRequest& request, const std::vector<double>& lead_times_s);

}  // namespace orbitcore