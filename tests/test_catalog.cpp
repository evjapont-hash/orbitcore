#include <cmath>
#include <string>
#include <vector>

#include "check.hpp"
#include "orbitcore/catalog.hpp"

using namespace orbitcore;

static const char* kIssLine1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927";
static const char* kIssLine2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";

static const char* kCatalogText =
    "ISS (ZARYA)\n"
    "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927\n"
    "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537\n"
    "1 25545U 98067B   08264.51782528 -.00002182  00000-0 -11606-4 0  2928\n"
    "2 25545  51.6416 100.0000 0006703 130.5360 200.0000 15.72125391563531\n"
    "DEEP SPACE EXAMPLE\n"
    "1 25546U 98067C   08264.51782528 -.00002182  00000-0 -11606-4 0  2929\n"
    "2 25546  55.0000 100.0000 0100000 130.5360 200.0000  2.00560000563531\n"
    "BROKEN CHECKSUM\n"
    "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2928\n"
    "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537\n";

TEST_CASE(catalog_loads_and_counts) {
  Catalog catalog;
  const CatalogLoadStats stats = catalog.load_tle_text(kCatalogText);
  CHECK(stats.loaded == 2);              // the ISS (named) and the unnamed second object
  CHECK(stats.skipped_deep_space == 1);  // ~2 rev/day
  CHECK(stats.skipped_parse == 1);       // bad checksum
  CHECK(catalog.size() == 2);
  CHECK(catalog.name(0) == "ISS (ZARYA)");
  CHECK(catalog.name(1).empty());
  CHECK(catalog.satnum(0) == 25544);
  CHECK(catalog.satnum(1) == 25545);
}

TEST_CASE(catalog_propagate_all_matches_single) {
  Catalog catalog;
  catalog.load_tle_text(kCatalogText);
  const auto first = parse_tle(kIssLine1, kIssLine2);
  const JulianDate t = add_minutes(first.elements.epoch, 37.5);

  std::vector<double> xyz(3 * catalog.size());
  CHECK(catalog.propagate_all(t, xyz.data()) == 0);

  const auto single = Sgp4(first.elements).propagate_to(t);
  CHECK(single.ok());
  CHECK_NEAR(xyz[0], single.state.r.x, 1e-9);
  CHECK_NEAR(xyz[1], single.state.r.y, 1e-9);
  CHECK_NEAR(xyz[2], single.state.r.z, 1e-9);
  // Second object has a different RAAN and mean anomaly, so it must be somewhere else.
  const double dx = xyz[3] - xyz[0], dy = xyz[4] - xyz[1], dz = xyz[5] - xyz[2];
  CHECK(std::sqrt(dx * dx + dy * dy + dz * dz) > 100.0);
}

TEST_CASE(catalog_replicate) {
  Catalog catalog;
  catalog.load_tle_text(kCatalogText);
  catalog.replicate(5);
  CHECK(catalog.size() == 10);
  CHECK(catalog.satnum(9) == 25545);
}