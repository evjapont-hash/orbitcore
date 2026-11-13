#include "orbitcore/risk.hpp"

#include <algorithm>
#include <cmath>

namespace orbitcore {

namespace {

constexpr double kEps = 1e-15;
constexpr double kTiny = 1e-300;

// Largest of f(x) over x in [lo, hi] (log-spaced coarse scan, then golden-section refinement).
// Assumes f is unimodal on the scale of one scan cell. Returns the maximising x through `x_best`.
template <typename F>
double maximise_log_scale(F f, double lo, double hi, double* x_best) {
  const int n = 80;
  const double ratio = std::log(hi / lo);
  int best = 0;
  double best_value = -1.0;
  for (int i = 0; i < n; ++i) {
    const double x = lo * std::exp(ratio * i / (n - 1));
    const double value = f(x);
    if (value > best_value) { best_value = value; best = i; }
  }
  double a = std::log(lo) + ratio * std::max(best - 1, 0) / (n - 1);
  double b = std::log(lo) + ratio * std::min(best + 1, n - 1) / (n - 1);
  const double phi = 0.5 * (std::sqrt(5.0) - 1.0);
  double c = b - phi * (b - a), d = a + phi * (b - a);
  double fc = f(std::exp(c)), fd = f(std::exp(d));
  for (int i = 0; i < 100 && (b - a) > 1e-12; ++i) {
    if (fc > fd) { b = d; d = c; fd = fc; c = b - phi * (b - a); fc = f(std::exp(c)); }
    else { a = c; c = d; fc = fd; d = a + phi * (b - a); fd = f(std::exp(d)); }
  }
  const double x_refined = std::exp(0.5 * (a + b));
  const double f_refined = f(x_refined);
  if (f_refined >= best_value) { *x_best = x_refined; return f_refined; }
  *x_best = lo * std::exp(ratio * best / (n - 1));
  return best_value;
}

}  // namespace

double regularized_gamma_p(double a, double x) {
  if (x <= 0.0) return 0.0;
  const double log_prefactor = -x + a * std::log(x) - std::lgamma(a);
  if (x < a + 1.0) {  // power series
    double ap = a, term = 1.0 / a, sum = term;
    for (int n = 0; n < 100000; ++n) {
      ap += 1.0;
      term *= x / ap;
      sum += term;
      if (std::fabs(term) < std::fabs(sum) * kEps) break;
    }
    return std::min(1.0, sum * std::exp(log_prefactor));
  }
  // continued fraction for the complement (modified Lentz)
  double b = x + 1.0 - a, c = 1.0 / kTiny, d = 1.0 / b, h = d;
  for (int i = 1; i < 100000; ++i) {
    const double an = -i * (i - a);
    b += 2.0;
    d = an * d + b;
    if (std::fabs(d) < kTiny) d = kTiny;
    c = b + an / c;
    if (std::fabs(c) < kTiny) c = kTiny;
    d = 1.0 / d;
    const double delta = d * c;
    h *= delta;
    if (std::fabs(delta - 1.0) < kEps) break;
  }
  return std::max(0.0, 1.0 - std::exp(log_prefactor) * h);
}

double collision_probability_isotropic(double miss_km, double hbr_km, double sigma_km) {
  if (!(hbr_km > 0.0)) return 0.0;
  if (!(sigma_km > 0.0)) return miss_km <= hbr_km ? 1.0 : 0.0;
  if (miss_km + 37.0 * sigma_km <= hbr_km) return 1.0;      // the whole distribution is inside
  if (miss_km - 37.0 * sigma_km >= hbr_km) return 0.0;      // the disk is 37 sigma out in the tail
  const double u = miss_km * miss_km / (2.0 * sigma_km * sigma_km);
  const double v = hbr_km * hbr_km / (2.0 * sigma_km * sigma_km);
  if (u > 1e6) {
    // The disk edge is almost straight on the scale of sigma: use the half-plane Gaussian tail.
    return 0.5 * std::erfc((miss_km - hbr_km) / (sigma_km * std::sqrt(2.0)));
  }
  // Pc = sum_k Poisson(k; u) * P(Poisson(v) > k): a Poisson mixture that is exact for an isotropic
  // Gaussian (the non-central chi-square form of the Marcum Q function), stable in every regime.
  const double spread = 12.0 * std::sqrt(u) + 12.0;
  const double k_lo = std::max(0.0, std::floor(u - spread));
  const double k_hi = std::ceil(u + spread);
  double sum = 0.0;
  for (double k = k_lo; k <= k_hi; k += 1.0) {
    const double log_weight = u > 0.0 ? (-u + k * std::log(u) - std::lgamma(k + 1.0)) : (k == 0.0 ? 0.0 : -1e300);
    const double weight = std::exp(log_weight);
    if (weight < 1e-300) continue;
    sum += weight * regularized_gamma_p(k + 1.0, v);
  }
  return std::min(1.0, std::max(0.0, sum));
}

MaxProbability maximum_collision_probability(double miss_km, double hbr_km) {
  if (!(hbr_km > 0.0)) return {0.0, 0.0};
  if (miss_km <= hbr_km) return {1.0, 0.0};
  double sigma = 0.0;
  const double lo = (miss_km - hbr_km) / 8.0, hi = 20.0 * miss_km;
  const double p = maximise_log_scale([&](double s) { return collision_probability_isotropic(miss_km, hbr_km, s); },
                                      lo, hi, &sigma);
  return {p, sigma};
}

double collision_probability_ceiling(double miss_km, double hbr_km) {
  if (!(hbr_km > 0.0)) return 0.0;
  if (miss_km <= hbr_km) return 1.0;
  // Probability that a 1-D Gaussian (mean miss_km, standard deviation s) lies within +-hbr_km of zero,
  // maximised over s: the disk is contained in that strip.
  auto strip = [&](double s) {
    const double root2 = std::sqrt(2.0);
    return 0.5 * (std::erfc((miss_km - hbr_km) / (s * root2)) - std::erfc((miss_km + hbr_km) / (s * root2)));
  };
  double s_best = 0.0;
  return maximise_log_scale(strip, (miss_km - hbr_km) / 8.0, 20.0 * miss_km, &s_best);
}

}  // namespace orbitcore