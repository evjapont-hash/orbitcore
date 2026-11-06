#include "orbitcore/frames.hpp"

#include <cmath>

namespace orbitcore {

namespace {
// Passive rotation about z by theta: ECI-like -> Earth-fixed.
template <Frame From, Frame To, Unit U>
Vec3<To, U> rot3(const Vec3<From, U>& v, double theta) {
  const double c = std::cos(theta), s = std::sin(theta);
  return {c * v.x + s * v.y, -s * v.x + c * v.y, v.z};
}
}  // namespace

Position<Frame::ECEF> teme_to_ecef(const Position<Frame::TEME>& r, JulianDate ut1) {
  return rot3<Frame::TEME, Frame::ECEF>(r, gmst_rad(ut1));
}

EcefState teme_to_ecef(const Position<Frame::TEME>& r, const Velocity<Frame::TEME>& v,
                       JulianDate ut1) {
  const double theta = gmst_rad(ut1);
  EcefState out;
  out.r = rot3<Frame::TEME, Frame::ECEF>(r, theta);
  const auto v_rot = rot3<Frame::TEME, Frame::ECEF>(v, theta);
  // v_ecef = R v_teme - omega x r_ecef, with omega = (0, 0, w)
  const double w = kEarthRotationRadPerSec;
  out.v = {v_rot.x + w * out.r.y, v_rot.y - w * out.r.x, v_rot.z};
  return out;
}

Position<Frame::ECEF> geodetic_to_ecef(const Geodetic& g) {
  const double a = kWgs84SemiMajorKm;
  const double e2 = kWgs84Flattening * (2.0 - kWgs84Flattening);
  const double sl = std::sin(g.latitude.value), cl = std::cos(g.latitude.value);
  const double n = a / std::sqrt(1.0 - e2 * sl * sl);
  return {(n + g.altitude_km) * cl * std::cos(g.longitude.value),
          (n + g.altitude_km) * cl * std::sin(g.longitude.value),
          (n * (1.0 - e2) + g.altitude_km) * sl};
}

Geodetic ecef_to_geodetic(const Position<Frame::ECEF>& r) {
  const double a = kWgs84SemiMajorKm;
  const double e2 = kWgs84Flattening * (2.0 - kWgs84Flattening);
  const double p = std::sqrt(r.x * r.x + r.y * r.y);
  const double lon = std::atan2(r.y, r.x);

  double lat = std::atan2(r.z, p * (1.0 - e2));
  double n = a;
  for (int i = 0; i < 20; ++i) {
    const double sl = std::sin(lat);
    n = a / std::sqrt(1.0 - e2 * sl * sl);
    const double next = std::atan2(r.z + e2 * n * sl, p);
    const bool done = std::fabs(next - lat) < 1e-13;
    lat = next;
    if (done) break;
  }
  const double sl = std::sin(lat), cl = std::cos(lat);
  n = a / std::sqrt(1.0 - e2 * sl * sl);
  // Use the better-conditioned expression away from the equator.
  const double h = (std::fabs(lat) < kPi / 4.0) ? p / cl - n : r.z / sl - n * (1.0 - e2);
  return {Radians{lat}, Radians{lon}, h};
}

Position<Frame::ENU> ecef_to_enu(const Position<Frame::ECEF>& target, const Geodetic& station) {
  const auto d = target - geodetic_to_ecef(station);
  const double sp = std::sin(station.latitude.value), cp = std::cos(station.latitude.value);
  const double sl = std::sin(station.longitude.value), cl = std::cos(station.longitude.value);
  return {-sl * d.x + cl * d.y,
          -sp * cl * d.x - sp * sl * d.y + cp * d.z,
          cp * cl * d.x + cp * sl * d.y + sp * d.z};
}

Position<Frame::NED> enu_to_ned(const Position<Frame::ENU>& enu) {
  return {enu.y, enu.x, -enu.z};
}

LookAngles look_angles(const Position<Frame::ENU>& enu) {
  const double range = enu.norm();
  double az = std::atan2(enu.x, enu.y);  // atan2(east, north)
  if (az < 0.0) az += kTwoPi;
  const double el = range > 0.0 ? std::asin(enu.z / range) : 0.0;
  return {Radians{az}, Radians{el}, range};
}

Position<Frame::RTN> to_rtn(const Position<Frame::TEME>& relative,
                            const Position<Frame::TEME>& ref_r,
                            const Velocity<Frame::TEME>& ref_v) {
  const double rn = ref_r.norm();
  const Position<Frame::TEME> rhat = ref_r * (1.0 / rn);
  const Position<Frame::TEME> h = {ref_r.y * ref_v.z - ref_r.z * ref_v.y,
                                   ref_r.z * ref_v.x - ref_r.x * ref_v.z,
                                   ref_r.x * ref_v.y - ref_r.y * ref_v.x};
  const Position<Frame::TEME> nhat = h * (1.0 / h.norm());
  const Position<Frame::TEME> that = cross(nhat, rhat);
  return {dot(relative, rhat), dot(relative, that), dot(relative, nhat)};
}

}  // namespace orbitcore
