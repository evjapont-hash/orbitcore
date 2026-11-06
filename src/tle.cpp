#include "orbitcore/tle.hpp"

#include <cmath>
#include <cstdlib>
#include <string>

#include "orbitcore/units.hpp"

namespace orbitcore {

namespace {

std::string trim(std::string_view sv) {
  const auto b = sv.find_first_not_of(' ');
  if (b == std::string_view::npos) return {};
  const auto e = sv.find_last_not_of(' ');
  return std::string(sv.substr(b, e - b + 1));
}

bool parse_double(std::string_view sv, double& out) {
  const std::string s = trim(sv);
  if (s.empty()) return false;
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return end == s.c_str() + s.size();
}

bool parse_int(std::string_view sv, int& out) {
  const std::string s = trim(sv);
  if (s.empty()) return false;
  char* end = nullptr;
  out = static_cast<int>(std::strtol(s.c_str(), &end, 10));
  return end == s.c_str() + s.size();
}

// TLE "implied decimal + exponent" field, e.g. "-11606-4" => -0.11606e-4. Width 8.
bool parse_implied_exp(std::string_view sv, double& out) {
  if (sv.size() < 8) return false;
  double sign = 1.0;
  if (sv[0] == '-') sign = -1.0;
  else if (sv[0] != '+' && sv[0] != ' ') return false;
  int mantissa = 0, exponent = 0;
  if (!parse_int(sv.substr(1, 5), mantissa)) return false;
  if (!parse_int(sv.substr(7, 1), exponent)) return false;
  if (sv[6] == '-') exponent = -exponent;
  else if (sv[6] != '+' && sv[6] != ' ') return false;
  out = sign * (mantissa / 1.0e5) * std::pow(10.0, exponent);
  return true;
}

int checksum(std::string_view line) {
  int sum = 0;
  for (std::size_t i = 0; i < 68; ++i) {
    const char c = line[i];
    if (c >= '0' && c <= '9') sum += c - '0';
    else if (c == '-') sum += 1;
  }
  return sum % 10;
}

bool checksum_ok(std::string_view line) {
  const char c = line[68];
  return c >= '0' && c <= '9' && (c - '0') == checksum(line);
}

}  // namespace

ParseResult parse_tle(std::string_view l1, std::string_view l2, bool strict_checksum) {
  ParseResult res;
  auto fail = [&](ParseError e) { res.error = e; return res; };

  if (l1.size() < 69 || l2.size() < 69) return fail(ParseError::BadLength);
  if (l1[0] != '1' || l2[0] != '2') return fail(ParseError::BadLineNumber);
  if (strict_checksum && (!checksum_ok(l1) || !checksum_ok(l2))) return fail(ParseError::BadChecksum);

  int sat1 = 0, sat2 = 0;
  if (!parse_int(l1.substr(2, 5), sat1) || !parse_int(l2.substr(2, 5), sat2))
    return fail(ParseError::BadField);
  if (sat1 != sat2) return fail(ParseError::SatnumMismatch);

  ElementSet& e = res.elements;
  e.satnum = sat1;

  int yr2 = 0;
  double doy = 0.0, ecc_digits = 0.0;
  double incl_deg = 0, raan_deg = 0, argp_deg = 0, ma_deg = 0, rev_per_day = 0;
  if (!parse_int(l1.substr(18, 2), yr2) || !parse_double(l1.substr(20, 12), doy) ||
      !parse_double(l1.substr(33, 10), e.ndot) || !parse_implied_exp(l1.substr(44, 8), e.nddot) ||
      !parse_implied_exp(l1.substr(53, 8), e.bstar) || !parse_double(l2.substr(8, 8), incl_deg) ||
      !parse_double(l2.substr(17, 8), raan_deg) ||
      !parse_double("0." + std::string(l2.substr(26, 7)), ecc_digits) ||
      !parse_double(l2.substr(34, 8), argp_deg) || !parse_double(l2.substr(43, 8), ma_deg) ||
      !parse_double(l2.substr(52, 11), rev_per_day)) {
    return fail(ParseError::BadField);
  }

  const int year = yr2 < 57 ? 2000 + yr2 : 1900 + yr2;
  e.epoch = from_year_and_day_of_year(year, doy);
  e.ecco = ecc_digits;
  e.inclo = incl_deg * kDegToRad;
  e.nodeo = raan_deg * kDegToRad;
  e.argpo = argp_deg * kDegToRad;
  e.mo = ma_deg * kDegToRad;
  e.no_kozai = rev_per_day * kTwoPi / 1440.0;
  return res;
}

}  // namespace orbitcore
