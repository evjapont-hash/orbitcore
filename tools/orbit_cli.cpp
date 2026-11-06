// orbit_cli: read a TLE file, propagate with SGP4, and print a table of states.
//
//   orbit_cli <tle-file> [--duration MIN] [--step MIN] [--csv]
//
// Columns: time since epoch, UTC, TEME position (km), geodetic latitude/longitude/altitude
// (WGS-84), and speed relative to the TEME frame (km/s).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "orbitcore/frames.hpp"
#include "orbitcore/sgp4.hpp"
#include "orbitcore/tle.hpp"

using namespace orbitcore;

namespace {

void print_usage() {
  std::fprintf(stderr,
               "usage: orbit_cli <tle-file> [--duration MIN] [--step MIN] [--csv]\n"
               "  --duration MIN   minutes after the TLE epoch to cover (default 90)\n"
               "  --step MIN       minutes between rows (default 10)\n"
               "  --csv            machine-readable output\n");
}

bool starts_with(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

std::string trim_right(std::string s) {
  while (!s.empty() && (s.back() == '\r' || s.back() == ' ' || s.back() == '\n')) s.pop_back();
  return s;
}

bool parse_positive(const char* text, double& out) {
  char* end = nullptr;
  out = std::strtod(text, &end);
  return end != text && *end == '\0' && out > 0.0;
}

const char* parse_error_text(ParseError e) {
  switch (e) {
    case ParseError::BadLength: return "a TLE line is shorter than 69 characters";
    case ParseError::BadLineNumber: return "TLE lines must start with '1 ' and '2 '";
    case ParseError::BadChecksum: return "TLE checksum mismatch (corrupted line?)";
    case ParseError::SatnumMismatch: return "line 1 and line 2 are for different satellites";
    case ParseError::BadField: return "a TLE field could not be read";
    case ParseError::None: break;
  }
  return "unknown error";
}

const char* sgp4_error_text(Sgp4Error e) {
  switch (e) {
    case Sgp4Error::DeepSpaceUnsupported:
      return "deep-space orbit (period >= 225 min); orbitcore implements near-Earth SGP4 only";
    case Sgp4Error::Decayed: return "satellite has decayed at this time";
    case Sgp4Error::None: return "none";
    default: return "propagation failed";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string path;
  double duration_min = 90.0, step_min = 10.0;
  bool csv = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { print_usage(); return 0; }
    if (arg == "--csv") { csv = true; continue; }
    if (arg == "--duration" || arg == "--step") {
      if (i + 1 >= argc) { std::fprintf(stderr, "error: %s needs a value\n", arg.c_str()); return 2; }
      double& target = (arg == "--duration") ? duration_min : step_min;
      if (!parse_positive(argv[++i], target)) {
        std::fprintf(stderr, "error: %s must be a positive number\n", arg.c_str());
        return 2;
      }
      continue;
    }
    if (starts_with(arg, "--") || !path.empty()) { print_usage(); return 2; }
    path = arg;
  }
  if (path.empty()) { print_usage(); return 2; }

  std::ifstream in(path);
  if (!in) { std::fprintf(stderr, "error: cannot open '%s'\n", path.c_str()); return 1; }
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(trim_right(line));

  // Find the first "1 ..." line directly followed by a "2 ..." line; an optional name line precedes it.
  std::size_t first = lines.size();
  for (std::size_t i = 0; i + 1 < lines.size(); ++i) {
    if (starts_with(lines[i], "1 ") && starts_with(lines[i + 1], "2 ")) { first = i; break; }
  }
  if (first == lines.size()) {
    std::fprintf(stderr, "error: no two-line element set found in '%s'\n", path.c_str());
    return 1;
  }
  std::string name = "(unnamed)";
  if (first > 0 && !lines[first - 1].empty()) name = lines[first - 1];

  const ParseResult parsed = parse_tle(lines[first], lines[first + 1]);
  if (!parsed.ok()) {
    std::fprintf(stderr, "error: %s\n", parse_error_text(parsed.error));
    return 1;
  }
  const Sgp4 sgp4(parsed.elements);
  if (sgp4.init_status() != Sgp4Error::None) {
    std::fprintf(stderr, "error: %s\n", sgp4_error_text(sgp4.init_status()));
    return 1;
  }

  const int rows = static_cast<int>(std::floor(duration_min / step_min + 1e-9)) + 1;
  if (rows > 100000) {
    std::fprintf(stderr, "error: that would print %d rows; use a larger --step\n", rows);
    return 2;
  }

  if (csv) {
    std::printf("t_min,utc,x_km,y_km,z_km,lat_deg,lon_deg,alt_km,speed_km_s\n");
  } else {
    const CalendarDate e = to_calendar(parsed.elements.epoch);
    std::printf("Satellite: %s  (NORAD %05d)\n", name.c_str(), parsed.elements.satnum);
    std::printf("Epoch (UTC): %04d-%02d-%02d %02d:%02d:%06.3f   Model: SGP4 (WGS-72), frame: TEME\n",
                e.year, e.month, e.day, e.hour, e.minute, e.second);
    std::printf("Geodetic: WGS-84. UTC is used as UT1 (error < 0.9 s). Speed is in the TEME frame.\n\n");
    std::printf("%9s  %-23s %12s %12s %12s  %9s %10s %9s  %11s\n", "t[min]", "UTC", "x[km]", "y[km]",
                "z[km]", "lat[deg]", "lon[deg]", "alt[km]", "speed[km/s]");
  }

  for (int k = 0; k < rows; ++k) {
    const double t = k * step_min;
    const PropagationResult result = sgp4.propagate(t);
    if (!result.ok()) {
      std::fprintf(stderr, "error at t=%.3f min: %s\n", t, sgp4_error_text(result.error));
      return 1;
    }
    const JulianDate jd = add_minutes(parsed.elements.epoch, t);
    const CalendarDate cal = to_calendar(jd);
    const Geodetic g = ecef_to_geodetic(teme_to_ecef(result.state.r, jd));
    const double lat = g.latitude.value * kRadToDeg;
    const double lon = g.longitude.value * kRadToDeg;
    const double speed = result.state.v.norm();

    char utc[40];
    std::snprintf(utc, sizeof utc, "%04d-%02d-%02d %02d:%02d:%06.3f", cal.year, cal.month, cal.day,
                  cal.hour, cal.minute, cal.second);
    const auto& r = result.state.r;
    if (csv) {
      std::printf("%.3f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", t, utc, r.x, r.y, r.z, lat, lon,
                  g.altitude_km, speed);
    } else {
      std::printf("%9.2f  %-23s %12.3f %12.3f %12.3f  %9.4f %10.4f %9.3f  %11.5f\n", t, utc, r.x, r.y,
                  r.z, lat, lon, g.altitude_km, speed);
    }
  }
  return 0;
}