// catalog_bench: load a TLE catalog and measure how fast all objects can be propagated.
//
//   catalog_bench <tle-file> [--scale N] [--iterations K]
//
// --scale N      repeat the catalog N times (synthetic stress test, clearly not real objects)
// --iterations K number of timed full-catalog propagations (default 20)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "orbitcore/catalog.hpp"

using namespace orbitcore;

namespace {

void print_usage() {
  std::fprintf(stderr, "usage: catalog_bench <tle-file> [--scale N] [--iterations K]\n");
}

bool parse_count(const char* text, long& out) {
  char* end = nullptr;
  out = std::strtol(text, &end, 10);
  return end != text && *end == '\0' && out >= 1;
}

// Current UTC time as a Julian date (Unix epoch is JD 2440587.5).
JulianDate now_julian() {
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  const double seconds = std::chrono::duration<double>(since_epoch).count();
  return {2440587.5, seconds / 86400.0};
}

}  // namespace

int main(int argc, char** argv) {
  std::string path;
  long scale = 1, iterations = 20;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { print_usage(); return 0; }
    if (arg == "--scale" || arg == "--iterations") {
      if (i + 1 >= argc || !parse_count(argv[i + 1], arg == "--scale" ? scale : iterations)) {
        std::fprintf(stderr, "error: %s needs a whole number >= 1\n", arg.c_str());
        return 2;
      }
      ++i;
      continue;
    }
    if (!path.empty() || arg.rfind("--", 0) == 0) { print_usage(); return 2; }
    path = arg;
  }
  if (path.empty()) { print_usage(); return 2; }

  std::ifstream in(path);
  if (!in) { std::fprintf(stderr, "error: cannot open '%s'\n", path.c_str()); return 1; }
  std::stringstream buffer;
  buffer << in.rdbuf();

  Catalog catalog;
  const auto load_start = std::chrono::steady_clock::now();
  const CatalogLoadStats stats = catalog.load_tle_text(buffer.str());
  const double load_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - load_start).count();

  std::printf("Loaded %zu objects in %.1f ms\n", stats.loaded, load_ms);
  std::printf("Skipped: %zu deep-space, %zu unparseable, %zu other\n", stats.skipped_deep_space,
              stats.skipped_parse, stats.skipped_other);
  if (catalog.size() == 0) { std::fprintf(stderr, "error: nothing to propagate\n"); return 1; }

  catalog.replicate(static_cast<std::size_t>(scale));
  if (scale > 1) std::printf("Replicated x%ld (synthetic): %zu objects\n", scale, catalog.size());

  std::vector<double> xyz(3 * catalog.size());
  const JulianDate t0 = now_julian();

  // One untimed warm-up pass, then timed passes at slightly different instants.
  std::size_t failures = catalog.propagate_all(t0, xyz.data());
  std::vector<double> times_ms;
  for (long k = 0; k < iterations; ++k) {
    const JulianDate t = add_minutes(t0, static_cast<double>(k));
    const auto start = std::chrono::steady_clock::now();
    failures = catalog.propagate_all(t, xyz.data());
    times_ms.push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  std::sort(times_ms.begin(), times_ms.end());
  const double median = times_ms[times_ms.size() / 2];
  const double best = times_ms.front();

  // Sanity check on the output: mean orbital radius of the objects that propagated.
  double radius_sum = 0.0;
  std::size_t finite = 0;
  for (std::size_t i = 0; i < catalog.size(); ++i) {
    const double r = std::sqrt(xyz[3 * i] * xyz[3 * i] + xyz[3 * i + 1] * xyz[3 * i + 1] +
                               xyz[3 * i + 2] * xyz[3 * i + 2]);
    if (std::isfinite(r)) { radius_sum += r; ++finite; }
  }

  std::printf("\nFull-catalog propagation (%zu objects, %ld timed passes):\n", catalog.size(), iterations);
  std::printf("  median %.3f ms   best %.3f ms\n", median, best);
  std::printf("  %.0f ns per object (median)   %.2f million objects/s\n",
              median * 1e6 / static_cast<double>(catalog.size()),
              static_cast<double>(catalog.size()) / (median * 1e3));
  std::printf("  failures at last instant: %zu of %zu\n", failures, catalog.size());
  if (finite > 0)
    std::printf("  mean orbital radius of propagated objects: %.1f km\n",
                radius_sum / static_cast<double>(finite));
  return 0;
}