#pragma once
// Ground-station visibility: which objects are above a station's minimum elevation.
//
// A StationFrame holds a station's Earth-fixed position and its local East/North/Up axes
// (expressed in ECEF). Objects are given in TEME (the SGP4 output frame) and rotated into
// ECEF using the Greenwich Mean Sidereal Time of the instant of interest.
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "orbitcore/frames.hpp"

namespace orbitcore {

struct StationFrame {
  Position<Frame::ECEF> position;  // km
  double east[3];                  // local axes as unit vectors in ECEF
  double north[3];
  double up[3];
  double min_elevation_rad;
};

inline StationFrame make_station_frame(const Geodetic& location, double min_elevation_rad) {
  const double sp = std::sin(location.latitude.value), cp = std::cos(location.latitude.value);
  const double sl = std::sin(location.longitude.value), cl = std::cos(location.longitude.value);
  StationFrame s{};
  s.position = geodetic_to_ecef(location);
  s.east[0] = -sl;        s.east[1] = cl;         s.east[2] = 0.0;
  s.north[0] = -sp * cl;  s.north[1] = -sp * sl;  s.north[2] = cp;
  s.up[0] = cp * cl;      s.up[1] = cp * sl;      s.up[2] = sp;
  s.min_elevation_rad = min_elevation_rad;
  return s;
}

// Azimuth, elevation and range of a TEME position as seen from the station.
inline LookAngles look_from_station(const StationFrame& s, const Position<Frame::TEME>& r,
                                    double gmst_rad) {
  const double c = std::cos(gmst_rad), sn = std::sin(gmst_rad);
  const double dx = (c * r.x + sn * r.y) - s.position.x;   // TEME -> ECEF, minus the station
  const double dy = (-sn * r.x + c * r.y) - s.position.y;
  const double dz = r.z - s.position.z;
  const Position<Frame::ENU> enu{dx * s.east[0] + dy * s.east[1] + dz * s.east[2],
                                 dx * s.north[0] + dy * s.north[1] + dz * s.north[2],
                                 dx * s.up[0] + dy * s.up[1] + dz * s.up[2]};
  return look_angles(enu);
}

// For `count` objects (x, y, z in km, TEME, tightly packed), sets `bit` in flags[i] when the
// object is at or above the station's minimum elevation. Objects with non-finite positions
// are treated as not visible. flags[i] is never cleared, so several stations can share the
// array using different bits. Returns the number of objects visible from this station.
inline std::size_t mark_visible(const StationFrame& s, const double* xyz_teme, std::size_t count,
                                double gmst_rad, std::uint8_t bit, std::uint8_t* flags) {
  const double c = std::cos(gmst_rad), sn = std::sin(gmst_rad);
  const double sin_min = std::sin(s.min_elevation_rad);
  std::size_t visible = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const double x = xyz_teme[3 * i], y = xyz_teme[3 * i + 1], z = xyz_teme[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const double dx = (c * x + sn * y) - s.position.x;
    const double dy = (-sn * x + c * y) - s.position.y;
    const double dz = z - s.position.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double up = dx * s.up[0] + dy * s.up[1] + dz * s.up[2];
    if (up >= sin_min * range) {  // elevation >= minimum, without computing asin
      flags[i] = static_cast<std::uint8_t>(flags[i] | bit);
      ++visible;
    }
  }
  return visible;
}

}  // namespace orbitcore