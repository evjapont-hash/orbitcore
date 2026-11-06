#pragma once
#include <string_view>

#include "orbitcore/time.hpp"

namespace orbitcore {

// Mean elements as published in a TLE, converted to SGP4 working units.
struct ElementSet {
  int satnum{0};
  JulianDate epoch{};
  double bstar{0.0};      // 1 / earth radii
  double ndot{0.0};       // rev/day^2 (as published; not used by SGP4 itself)
  double nddot{0.0};      // rev/day^3 (as published; not used by SGP4 itself)
  double inclo{0.0};      // rad
  double nodeo{0.0};      // rad (RAAN)
  double ecco{0.0};       // unitless
  double argpo{0.0};      // rad
  double mo{0.0};         // rad (mean anomaly)
  double no_kozai{0.0};   // rad/min (Kozai mean motion, as published in TLE)
};

enum class ParseError { None, BadLength, BadLineNumber, BadChecksum, SatnumMismatch, BadField };

struct ParseResult {
  ParseError error{ParseError::None};
  ElementSet elements{};
  bool ok() const { return error == ParseError::None; }
};

// Fixed-column TLE parser. Only the first 69 columns of each line are used, so
// the extended lines in Vallado's SGP4-VER.TLE parse fine. Set strict_checksum
// to false for test files whose checksums are not valid.
ParseResult parse_tle(std::string_view line1, std::string_view line2, bool strict_checksum = true);

}  // namespace orbitcore
