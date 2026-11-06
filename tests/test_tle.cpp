#include <string>

#include "check.hpp"
#include "orbitcore/tle.hpp"
#include "orbitcore/units.hpp"

using namespace orbitcore;

static const char* kL1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927";
static const char* kL2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";

TEST_CASE(tle_parses_iss_example) {
  const auto r = parse_tle(kL1, kL2);
  CHECK(r.ok());
  const auto& e = r.elements;
  CHECK(e.satnum == 25544);
  CHECK_NEAR(e.epoch.total(), 2454466.5 + 263.51782528, 1e-9);  // 2008-09-20 12:25:40 UTC
  CHECK_NEAR(e.bstar, -1.1606e-5, 1e-12);
  CHECK_NEAR(e.ndot, -2.182e-5, 1e-12);
  CHECK_NEAR(e.nddot, 0.0, 1e-15);
  CHECK_NEAR(e.inclo, 51.6416 * kDegToRad, 1e-12);
  CHECK_NEAR(e.nodeo, 247.4627 * kDegToRad, 1e-12);
  CHECK_NEAR(e.ecco, 0.0006703, 1e-12);
  CHECK_NEAR(e.argpo, 130.5360 * kDegToRad, 1e-12);
  CHECK_NEAR(e.mo, 325.0288 * kDegToRad, 1e-12);
  CHECK_NEAR(e.no_kozai, 15.72125391 * kTwoPi / 1440.0, 1e-12);
}

TEST_CASE(tle_checksum_and_format_errors) {
  std::string bad = kL1;
  bad[68] = '8';
  CHECK(parse_tle(bad, kL2).error == ParseError::BadChecksum);
  CHECK(parse_tle(bad, kL2, /*strict_checksum=*/false).ok());
  CHECK(parse_tle("1 short", kL2).error == ParseError::BadLength);
  CHECK(parse_tle(kL2, kL1).error == ParseError::BadLineNumber);

  std::string other = kL2;
  other[6] = '9';
  CHECK(parse_tle(kL1, other, false).error == ParseError::SatnumMismatch);
}
