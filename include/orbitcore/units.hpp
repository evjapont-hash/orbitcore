#pragma once
// Constants, angle types, and gravity models.
// Convention across orbitcore: lengths in km, velocities in km/s, time in
// minutes (SGP4 internals) or Julian days, angles in radians unless the type says otherwise.
#include <cmath>
#include <numbers>

namespace orbitcore {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kRadToDeg = 180.0 / kPi;

// Earth rotation rate (rad/s), as used in Vallado's reference code.
inline constexpr double kEarthRotationRadPerSec = 7.29211514670698e-5;

// WGS-84 ellipsoid: used for geodetic coordinates (lat/lon/alt).
inline constexpr double kWgs84SemiMajorKm = 6378.137;
inline constexpr double kWgs84Flattening = 1.0 / 298.257223563;

struct Radians { double value{0.0}; };
struct Degrees { double value{0.0}; };

constexpr Radians to_radians(Degrees d) { return {d.value * kDegToRad}; }
constexpr Degrees to_degrees(Radians r) { return {r.value * kRadToDeg}; }

// Gravity model constants. SGP4 is defined against WGS-72, NOT WGS-84:
// feeding it different constants breaks agreement with published test vectors
// and with how element sets were fit. Geodetic conversion uses WGS-84 above.
struct GravityModel {
  double mu;               // km^3 / s^2
  double radius_earth_km;  // km
  double xke;              // sqrt(GM) in earth radii^1.5 / min
  double tumin;            // 1 / xke (minutes per time unit)
  double j2, j3, j4, j3oj2;
};

inline GravityModel wgs72() {
  GravityModel g{};
  g.mu = 398600.8;
  g.radius_earth_km = 6378.135;
  g.xke = 60.0 / std::sqrt(g.radius_earth_km * g.radius_earth_km * g.radius_earth_km / g.mu);
  g.tumin = 1.0 / g.xke;
  g.j2 = 0.001082616;
  g.j3 = -0.00000253881;
  g.j4 = -0.00000165597;
  g.j3oj2 = g.j3 / g.j2;
  return g;
}

}  // namespace orbitcore
