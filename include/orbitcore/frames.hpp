#pragma once
// Frame- and unit-tagged vectors. Mixing frames or units is a compile error.
//
//   TEME : True Equator Mean Equinox. Native output frame of SGP4. (Not J2000!)
//   ECEF : Earth-fixed (rotates with Earth). Polar motion neglected.
//   ENU  : local East-North-Up at a ground station.
//   NED  : local North-East-Down at a ground station.
//   RTN  : Radial / Transverse (in-track) / Normal, centered on a reference object.
#include <utility>

#include "orbitcore/time.hpp"
#include "orbitcore/units.hpp"

namespace orbitcore {

enum class Frame { TEME, ECEF, ENU, NED, RTN };
enum class Unit { Km, KmPerS };

template <Frame F, Unit U>
struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};
  constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

template <Frame F, Unit U>
constexpr double dot(const Vec3<F, U>& a, const Vec3<F, U>& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
template <Frame F, Unit U>
constexpr Vec3<F, U> cross(const Vec3<F, U>& a, const Vec3<F, U>& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

template <Frame F> using Position = Vec3<F, Unit::Km>;
template <Frame F> using Velocity = Vec3<F, Unit::KmPerS>;

struct Geodetic {  // WGS-84
  Radians latitude;
  Radians longitude;
  double altitude_km;
};

struct EcefState {
  Position<Frame::ECEF> r;
  Velocity<Frame::ECEF> v;
};

struct LookAngles {
  Radians azimuth;    // from north, clockwise, [0, 2*pi)
  Radians elevation;  // above local horizon
  double range_km;
};

// TEME -> ECEF (rotation by GMST; polar motion and nutation corrections omitted).
// Velocity includes the -omega x r term so it is the velocity relative to the rotating Earth.
Position<Frame::ECEF> teme_to_ecef(const Position<Frame::TEME>& r, JulianDate ut1);
EcefState teme_to_ecef(const Position<Frame::TEME>& r, const Velocity<Frame::TEME>& v, JulianDate ut1);

Geodetic ecef_to_geodetic(const Position<Frame::ECEF>& r);
Position<Frame::ECEF> geodetic_to_ecef(const Geodetic& g);

// Target position relative to a ground station, expressed in the station's local frame.
Position<Frame::ENU> ecef_to_enu(const Position<Frame::ECEF>& target, const Geodetic& station);
Position<Frame::NED> enu_to_ned(const Position<Frame::ENU>& enu);
LookAngles look_angles(const Position<Frame::ENU>& enu);

// Express a relative position (e.g. secondary - primary, both TEME) in the primary's RTN frame.
Position<Frame::RTN> to_rtn(const Position<Frame::TEME>& relative,
                            const Position<Frame::TEME>& ref_r,
                            const Velocity<Frame::TEME>& ref_v);

}  // namespace orbitcore
