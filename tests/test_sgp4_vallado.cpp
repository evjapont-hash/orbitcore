// Validation against Vallado et al., "Revisiting Spacetrack Report #3" (AIAA 2006-6753).
// Download the verification files from CelesTrak and drop them in tests/data/ -- see
// tests/data/README.md. Until then the Vallado test SKIPs.
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "check.hpp"
#include "orbitcore/sgp4.hpp"

using namespace orbitcore;

#ifndef ORBITCORE_TEST_DATA_DIR
#define ORBITCORE_TEST_DATA_DIR "tests/data"
#endif

namespace {

struct Row { double tsince, x, y, z, vx, vy, vz; };

// Parses the expected-output file: a header line "<satnum> xx" starts each satellite,
// followed by rows "tsince x y z xdot ydot zdot" (min, km, km/s).
// If your copy of the file differs in layout, adjust here.
std::map<int, std::vector<Row>> read_expected(const std::string& path) {
  std::map<int, std::vector<Row>> out;
  std::ifstream in(path);
  std::string line;
  int current = -1;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string first, second;
    if (!(ss >> first)) continue;
    std::istringstream ss2(line);
    ss2 >> first >> second;
    if (second == "xx") {
      current = std::atoi(first.c_str());
      out[current];
      continue;
    }
    Row r{};
    std::istringstream rs(line);
    if (current >= 0 && (rs >> r.tsince >> r.x >> r.y >> r.z >> r.vx >> r.vy >> r.vz))
      out[current].push_back(r);
  }
  return out;
}

std::map<int, ElementSet> read_elements(const std::string& path) {
  std::map<int, ElementSet> out;
  std::ifstream in(path);
  std::string line, l1;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.rfind("1 ", 0) == 0) {
      l1 = line;
    } else if (line.rfind("2 ", 0) == 0 && !l1.empty()) {
      const auto r = parse_tle(l1, line, /*strict_checksum=*/false);
      if (r.ok() && !out.count(r.elements.satnum)) out[r.elements.satnum] = r.elements;
      l1.clear();
    }
  }
  return out;
}

}  // namespace

// Tolerances: start tight. Loosen only with a documented reason in docs/VALIDATION.md.
constexpr double kPosTolKm = 1e-6;
constexpr double kVelTolKmS = 1e-8;

TEST_CASE(sgp4_iss_sanity) {
  // After implementing SGP4: the 2008 ISS TLE should be a ~350 km LEO at epoch.
  const auto r = parse_tle("1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927",
                           "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537");
  const Sgp4 sgp4(r.elements);
  const auto s = sgp4.propagate(0.0);
  CHECK(s.ok());
  if (!s.ok()) return;
  const double radius = s.state.r.norm();
  const double speed = s.state.v.norm();
  CHECK(radius > 6600.0 && radius < 6900.0);
  CHECK(speed > 7.5 && speed < 7.8);
}

TEST_CASE(sgp4_rejects_deep_space) {
  // Same elements but a ~24 h period: must be reported as deep space, not propagated.
  auto r = parse_tle("1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927",
                     "2 25544  51.6416 247.4627 0006703 130.5360 325.0288  1.00270000563537",
                     /*strict_checksum=*/false);
  CHECK(r.ok());
  CHECK(Sgp4(r.elements).init_status() == Sgp4Error::DeepSpaceUnsupported);
}

TEST_CASE(sgp4_vallado_vectors) {
  const std::string dir = ORBITCORE_TEST_DATA_DIR;
  const auto elements = read_elements(dir + "/SGP4-VER.TLE");
  const auto expected = read_expected(dir + "/tmatverDec2015.out");
  if (elements.empty() || expected.empty())
    SKIP("tests/data/SGP4-VER.TLE and tmatverDec2015.out not found; see tests/data/README.md");

  int compared = 0, deep_space = 0, mismatched = 0;
  for (const auto& [satnum, rows] : expected) {
    const auto it = elements.find(satnum);
    if (it == elements.end()) continue;
    const Sgp4 sgp4(it->second);
    if (sgp4.init_status() == Sgp4Error::DeepSpaceUnsupported) { ++deep_space; continue; }
    if (sgp4.init_status() == Sgp4Error::NotImplemented) {
      CHECK(false && "SGP4 not implemented yet (src/sgp4.cpp)");
      return;
    }
    ++compared;
    for (const auto& row : rows) {
      const auto s = sgp4.propagate(row.tsince);
      const bool ok = s.ok() && std::fabs(s.state.r.x - row.x) <= kPosTolKm &&
                      std::fabs(s.state.r.y - row.y) <= kPosTolKm &&
                      std::fabs(s.state.r.z - row.z) <= kPosTolKm &&
                      std::fabs(s.state.v.x - row.vx) <= kVelTolKmS &&
                      std::fabs(s.state.v.y - row.vy) <= kVelTolKmS &&
                      std::fabs(s.state.v.z - row.vz) <= kVelTolKmS;
      if (!ok) {
        ++mismatched;
        CHECK(ok);
        std::fprintf(stderr, "      sat %05d t=%.4f min: got (%.8f %.8f %.8f), want (%.8f %.8f %.8f)\n",
                     satnum, row.tsince, s.state.r.x, s.state.r.y, s.state.r.z, row.x, row.y, row.z);
        break;  // one report per satellite
      }
    }
  }
  std::fprintf(stderr, "    compared %d satellites, skipped %d deep-space, %d mismatched\n", compared,
               deep_space, mismatched);
  CHECK(compared > 0);
}
