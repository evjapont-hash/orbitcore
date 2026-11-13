// conjunction_cli: screen one satellite against a TLE catalog for close approaches.
//
//   conjunction_cli <tle-file> --primary NORAD_ID [--hours H | --days D]
//                   [--threshold-km X] [--hbr-m R] [--max-rows N] [--check-brute-force STEP_S]
//
// The window starts now (UTC). --hbr-m is the combined hard-body radius of the two objects, in
// metres (default 10), used for the covariance-free collision probabilities. --check-brute-force
// also runs a slow reference search that steps every pair at STEP_S seconds and reports whether
// it finds exactly the same encounters.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "orbitcore/catalog.hpp"
#include "orbitcore/conjunction.hpp"
#include "orbitcore/risk.hpp"

using namespace orbitcore;

namespace {

void print_usage() {
  std::fprintf(stderr,
               "usage: conjunction_cli <tle-file> --primary NORAD_ID [--hours H | --days D]\n"
               "                       [--threshold-km X] [--hbr-m R] [--max-rows N] [--check-brute-force STEP_S]\n");
}

bool parse_number(const char* text, double& out) {
  char* end = nullptr;
  out = std::strtod(text, &end);
  return end != text && *end == '\0' && std::isfinite(out);
}

JulianDate now_julian() {
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  return {2440587.5, std::chrono::duration<double>(since_epoch).count() / 86400.0};
}

std::string utc_text(JulianDate jd) {
  const CalendarDate c = to_calendar(jd);
  char buffer[40];
  std::snprintf(buffer, sizeof buffer, "%04d-%02d-%02d %02d:%02d:%06.3f", c.year, c.month, c.day, c.hour,
                c.minute, c.second);
  return buffer;
}

double elapsed_ms(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since).count();
}

void print_table(const Catalog& catalog, const ScreeningResult& result, std::size_t max_rows, double hbr_km) {
  std::printf("%4s  %-23s %9s %10s %9s %9s %9s %9s %9s %9s   %s\n", "#", "TCA (UTC)", "miss[km]", "v_rel[km/s]",
              "R[km]", "T[km]", "N[km]", "Pc_max", "sigma*[km]", "Pc_ceil", "secondary");
  std::size_t row = 0;
  for (const Encounter& e : result.encounters) {
    if (row >= max_rows) { std::printf("  ... %zu more\n", result.encounters.size() - row); break; }
    const MaxProbability worst = maximum_collision_probability(e.miss_distance_km, hbr_km);
    const double ceiling = collision_probability_ceiling(e.miss_distance_km, hbr_km);
    std::printf("%4zu  %-23s %9.3f %10.3f %9.3f %9.3f %9.3f %9.1e %9.2f %9.1e   %s (NORAD %05d)\n", ++row,
                utc_text(e.tca).c_str(), e.miss_distance_km, e.relative_speed_km_s, e.miss_rtn.x, e.miss_rtn.y,
                e.miss_rtn.z, worst.probability, worst.sigma_km, ceiling, catalog.name(e.secondary).c_str(),
                catalog.satnum(e.secondary));
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string path;
  long primary_id = -1;
  double hours = 24.0, threshold_km = 10.0, brute_step_s = 0.0, max_rows = 40.0, hbr_m = 10.0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next_number = [&](double& target) {
      if (i + 1 >= argc || !parse_number(argv[i + 1], target)) {
        std::fprintf(stderr, "error: %s needs a number\n", arg.c_str());
        return false;
      }
      ++i;
      return true;
    };
    double value = 0.0;
    if (arg == "--help" || arg == "-h") { print_usage(); return 0; }
    else if (arg == "--primary") { if (!next_number(value)) return 2; primary_id = static_cast<long>(value); }
    else if (arg == "--hours") { if (!next_number(hours)) return 2; }
    else if (arg == "--days") { if (!next_number(value)) return 2; hours = value * 24.0; }
    else if (arg == "--threshold-km") { if (!next_number(threshold_km)) return 2; }
    else if (arg == "--hbr-m") { if (!next_number(hbr_m)) return 2; }
    else if (arg == "--max-rows") { if (!next_number(max_rows)) return 2; }
    else if (arg == "--check-brute-force") { if (!next_number(brute_step_s)) return 2; }
    else if (arg.rfind("--", 0) == 0 || !path.empty()) { print_usage(); return 2; }
    else path = arg;
  }
  if (path.empty() || primary_id < 0 || hours <= 0.0 || threshold_km <= 0.0 || hbr_m <= 0.0) { print_usage(); return 2; }
  const double hbr_km = hbr_m / 1000.0;

  std::ifstream in(path);
  if (!in) { std::fprintf(stderr, "error: cannot open '%s'\n", path.c_str()); return 1; }
  std::stringstream buffer;
  buffer << in.rdbuf();
  Catalog catalog;
  const CatalogLoadStats load = catalog.load_tle_text(buffer.str());

  std::size_t primary = catalog.size();
  for (std::size_t i = 0; i < catalog.size(); ++i) {
    if (catalog.satnum(i) == primary_id) { primary = i; break; }
  }
  if (primary == catalog.size()) {
    std::fprintf(stderr, "error: NORAD %ld not found among the %zu near-Earth objects loaded from '%s'\n",
                 primary_id, catalog.size(), path.c_str());
    return 1;
  }

  const JulianDate start = now_julian();
  const JulianDate end = add_minutes(start, hours * 60.0);
  const Sgp4& p = catalog.propagator(primary);
  std::printf("Primary:   %s (NORAD %05d), element set %.2f days old at the window start\n",
              catalog.name(primary).c_str(), catalog.satnum(primary), minutes_between(p.elements().epoch, start) / 1440.0);
  std::printf("Window:    %s .. %s UTC (%.1f h)\n", utc_text(start).c_str(), utc_text(end).c_str(), hours);
  std::printf("Catalog:   %zu near-Earth objects loaded (%zu deep-space skipped)\n", catalog.size(), load.skipped_deep_space);

  ScreeningOptions options;
  options.threshold_km = threshold_km;
  const auto t0 = std::chrono::steady_clock::now();
  const ScreeningResult result = screen_primary(catalog, primary, start, hours * 60.0, options);
  const double screen_ms = elapsed_ms(t0);
  std::printf("Screening: %zu of %zu objects passed the radial-band filter; %zu encounters within %.1f km\n",
              result.stats.objects_screened, result.stats.objects_total - 1, result.encounters.size(), threshold_km);
  std::printf("           %.1f ms, %.2f million SGP4 evaluations\n\n", screen_ms,
              static_cast<double>(result.stats.propagations) / 1e6);
  print_table(catalog, result, static_cast<std::size_t>(max_rows), hbr_km);

  // Summary of the covariance-free risk numbers (illustrative thresholds, not operational criteria).
  double largest = 0.0;
  std::size_t largest_index = 0, over_1e5 = 0, over_1e4 = 0;
  for (std::size_t i = 0; i < result.encounters.size(); ++i) {
    const double pc = maximum_collision_probability(result.encounters[i].miss_distance_km, hbr_km).probability;
    if (pc > largest) { largest = pc; largest_index = i + 1; }
    if (pc >= 1e-5) ++over_1e5;
    if (pc >= 1e-4) ++over_1e4;
  }
  if (!result.encounters.empty()) {
    std::printf("\nHard-body radius %.1f m. Largest Pc_max: %.1e (event #%zu). Events with Pc_max >= 1e-5: %zu; >= 1e-4: %zu.\n",
                hbr_m, largest, largest_index, over_1e5, over_1e4);
  }
  std::printf("\nR, T, N = the secondary's position relative to the primary, in the primary's radial, in-track and\n"
              "cross-track axes at the time of closest approach.\n"
              "Pc_max = worst-case collision probability over the SIZE of the position uncertainty (equal in both\n"
              "directions of the encounter plane); sigma* is the uncertainty at which that worst case occurs.\n"
              "Pc_ceil = upper bound over ANY uncertainty size and shape (Gaussian). TLEs carry no covariance, so\n"
              "these are bounds, not predictions. Positions are good to ~1 km at epoch and degrade with element\n"
              "age: treat all of this as a demonstration of the method, not operational screening.\n");

  if (brute_step_s > 0.0) {
    std::printf("\nBrute-force check (every pair, %.2f s steps, no skipping, no radial filter)...\n", brute_step_s);
    const auto t1 = std::chrono::steady_clock::now();
    const ScreeningResult slow = screen_primary_brute_force(catalog, primary, start, hours * 60.0, threshold_km, brute_step_s);
    const double slow_ms = elapsed_ms(t1);
    bool same = slow.encounters.size() == result.encounters.size();
    double worst_dt = 0.0, worst_dd = 0.0;
    for (std::size_t i = 0; same && i < result.encounters.size(); ++i) {
      same = result.encounters[i].secondary == slow.encounters[i].secondary;
      worst_dt = std::fmax(worst_dt, std::fabs(minutes_between(result.encounters[i].tca, slow.encounters[i].tca)) * 60.0);
      worst_dd = std::fmax(worst_dd, std::fabs(result.encounters[i].miss_distance_km - slow.encounters[i].miss_distance_km));
    }
    std::printf("  brute force: %zu encounters, %.1f ms, %.1f million evaluations (%.1fx more than the screening above)\n",
                slow.encounters.size(), slow_ms, static_cast<double>(slow.stats.propagations) / 1e6,
                static_cast<double>(slow.stats.propagations) / static_cast<double>(result.stats.propagations));
    if (same) {
      std::printf("  MATCH: same encounters; worst difference %.4f s in TCA and %.6f km in miss distance\n", worst_dt, worst_dd);
    } else {
      std::printf("  DIFFER: the searches disagree. Brute force found %zu, screening found %zu.\n", slow.encounters.size(), result.encounters.size());
      return 3;
    }
  }
  return 0;
}