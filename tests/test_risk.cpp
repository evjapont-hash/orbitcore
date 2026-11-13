#include <algorithm>
#include <cmath>
#include <vector>

#include "check.hpp"
#include "orbitcore/risk.hpp"
#include "orbitcore/units.hpp"

using namespace orbitcore;

namespace {

// Independent reference: the probability that a 2-D Gaussian with mean (miss, 0) and covariance with
// principal standard deviations s1, s2 (principal axis rotated by `theta`) lies in the disk of radius R
// at the origin, by midpoint-rule integration on a polar grid. Uses no Bessel or gamma functions.
double integrate_disk(double miss, double R, double s1, double s2, double theta, int nr = 600, int nt = 1440) {
  const double c = std::cos(theta), s = std::sin(theta);
  const double dr = R / nr, dphi = 2.0 * kPi / nt;
  double sum = 0.0;
  for (int ir = 0; ir < nr; ++ir) {
    const double r = (ir + 0.5) * dr;
    for (int it = 0; it < nt; ++it) {
      const double phi = (it + 0.5) * dphi;
      const double dx = r * std::cos(phi) - miss, dy = r * std::sin(phi);
      const double a = c * dx + s * dy, b = -s * dx + c * dy;
      sum += std::exp(-0.5 * (a * a / (s1 * s1) + b * b / (s2 * s2))) / (2.0 * kPi * s1 * s2) * r * dr * dphi;
    }
  }
  return sum;
}

double relative_error(double actual, double expected) { return std::fabs(actual - expected) / std::fabs(expected); }

}  // namespace

TEST_CASE(gamma_p_known_values) {
  CHECK_NEAR(regularized_gamma_p(1.0, 0.7), 1.0 - std::exp(-0.7), 1e-14);
  CHECK_NEAR(regularized_gamma_p(2.0, 2.0), 1.0 - 3.0 * std::exp(-2.0), 1e-14);
  CHECK_NEAR(regularized_gamma_p(0.5, 1.3), std::erf(std::sqrt(1.3)), 1e-13);
  CHECK_NEAR(regularized_gamma_p(10.0, 10.0), 0.5420702855281478, 1e-12);  // Poisson(10) tail above 9
  CHECK_NEAR(regularized_gamma_p(3.0, 40.0), 1.0, 1e-12);                   // continued-fraction branch
  CHECK(regularized_gamma_p(5.0, 0.0) == 0.0);
}

TEST_CASE(isotropic_probability_zero_miss_closed_form) {
  // For a centred distribution Pc = 1 - exp(-R^2 / (2 sigma^2)).
  for (double ratio : {0.001, 0.1, 0.5, 1.0, 2.0, 5.0}) {
    const double expected = -std::expm1(-ratio * ratio / 2.0);
    CHECK(relative_error(collision_probability_isotropic(0.0, ratio, 1.0), expected) < 1e-12);
  }
}

TEST_CASE(isotropic_probability_matches_numerical_integration) {
  struct Case { double miss, hbr, sigma; };
  const Case cases[] = {{1.0, 1.0, 1.0}, {3.0, 1.0, 2.0}, {10.0, 1.0, 5.0}, {5.0, 0.3, 1.2},
                        {2.0, 1.0, 0.5}, {20.0, 1.0, 8.0}, {0.5, 1.0, 0.7}, {4.0, 2.0, 0.6}};
  for (const Case& c : cases) {
    const double exact = integrate_disk(c.miss, c.hbr, c.sigma, c.sigma, 0.0);
    const double series = collision_probability_isotropic(c.miss, c.hbr, c.sigma);
    // The reference grid itself is only second-order accurate (its error is ~1e-5 for the sharpest case,
    // and shrinks 16x per 4x refinement), so the tolerance reflects the reference, not the series.
    CHECK(relative_error(series, exact) < 5e-5);
  }
}

TEST_CASE(isotropic_probability_small_disk_and_limits) {
  // R << sigma: Pc -> (R^2 / (2 sigma^2)) exp(-d^2 / (2 sigma^2)).
  const double R = 1e-3, d = 1.0, sigma = 0.7;
  const double approx = R * R / (2.0 * sigma * sigma) * std::exp(-d * d / (2.0 * sigma * sigma));
  CHECK(relative_error(collision_probability_isotropic(d, R, sigma), approx) < 1e-5);
  // Vanishing uncertainty: a hit if the centres are inside the hard body, a miss otherwise.
  CHECK_NEAR(collision_probability_isotropic(0.5, 1.0, 1e-4), 1.0, 1e-12);
  CHECK(collision_probability_isotropic(2.0, 1.0, 1e-3) < 1e-300);
  CHECK(collision_probability_isotropic(2.0, 1.0, 0.0) == 0.0);
  // The half-plane regime (disk edge nearly straight on the scale of sigma) stays accurate.
  const double edge = collision_probability_isotropic(1.0005, 1.0, 1e-3);   // edge 0.5 sigma away
  CHECK_NEAR(edge, 0.5 * std::erfc(0.5 / std::sqrt(2.0)), 5e-3);
  // Always a probability.
  for (double sigma_k : {0.01, 0.1, 1.0, 10.0, 1000.0}) {
    const double p = collision_probability_isotropic(7.0, 0.05, sigma_k);
    CHECK(p >= 0.0 && p <= 1.0);
  }
}

TEST_CASE(maximum_probability_matches_closed_form_for_small_disks) {
  const double d = 10.0, R = 0.01;
  const MaxProbability m = maximum_collision_probability(d, R);
  CHECK(relative_error(m.probability, R * R / (std::exp(1.0) * d * d)) < 2e-3);   // R^2 / (e d^2)
  CHECK(relative_error(m.sigma_km, d / std::sqrt(2.0)) < 5e-3);                   // at sigma = d / sqrt(2)
  // It really is the maximum: no other sigma does better.
  for (double s = 0.2; s < 200.0; s *= 1.15) {
    CHECK(collision_probability_isotropic(d, R, s) <= m.probability * (1.0 + 1e-9));
  }
  // Exact regime (not small-disk): still the maximum of a sweep.
  const MaxProbability big = maximum_collision_probability(3.0, 1.0);
  for (double s = 0.05; s < 100.0; s *= 1.05) {
    CHECK(collision_probability_isotropic(3.0, 1.0, s) <= big.probability * (1.0 + 1e-9));
  }
  CHECK(big.probability > 0.0 && big.probability < 1.0);
}

TEST_CASE(maximum_probability_edge_cases_and_trends) {
  CHECK(maximum_collision_probability(0.5, 1.0).probability == 1.0);    // already inside the hard body
  CHECK(maximum_collision_probability(1.0, 1.0).probability == 1.0);
  CHECK(maximum_collision_probability(5.0, 0.0).probability == 0.0);    // no hard body
  const double near_hit = maximum_collision_probability(1.01, 1.0).probability;
  CHECK(near_hit > 0.1 && near_hit < 0.6);                               // barely outside: close to a coin flip
  double previous = 2.0;
  for (double d = 1.5; d < 50.0; d *= 1.6) {                              // farther misses are less risky
    const double p = maximum_collision_probability(d, 1.0).probability;
    CHECK(p < previous);
    previous = p;
  }
  CHECK(maximum_collision_probability(10.0, 0.02).probability > maximum_collision_probability(10.0, 0.01).probability);
}

TEST_CASE(ceiling_bounds_every_gaussian_covariance) {
  const double d = 4.0, R = 1.0;
  const double ceiling = collision_probability_ceiling(d, R);
  const double iso = maximum_collision_probability(d, R).probability;
  CHECK(ceiling >= iso);
  // Many covariances: sizes, aspect ratios and orientations (including thin needles along the miss vector).
  const double sigmas[] = {0.2, 0.5, 1.0, 2.0, 4.0, 8.0};
  const double thetas[] = {0.0, 0.3, 0.7, 1.2, 1.5707963267948966};
  double best = 0.0;
  for (double s1 : sigmas) {
    for (double s2 : sigmas) {
      for (double theta : thetas) {
        const double p = integrate_disk(d, R, s1, s2, theta, 200, 480);
        CHECK(p <= ceiling * (1.0 + 1e-3));
        best = std::max(best, p);
      }
    }
  }
  CHECK(best > 0.5 * ceiling);   // the bound is not vacuous: needle-like covariances get close to it
  // Small-disk closed form: ceiling -> 2R / (d sqrt(2 pi e)).
  const double d2 = 100.0, R2 = 0.1;
  CHECK(relative_error(collision_probability_ceiling(d2, R2), 2.0 * R2 / (d2 * std::sqrt(2.0 * kPi * std::exp(1.0)))) < 2e-3);
  CHECK(collision_probability_ceiling(0.5, 1.0) == 1.0);
}