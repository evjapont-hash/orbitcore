#pragma once
// Collision probability without a covariance.
//
// Public element sets (TLEs) carry no position uncertainty, so the probability of collision
// (Pc) of a close approach cannot be computed outright. What can be said honestly is a worst
// case over the unknown uncertainty:
//
//  * maximum_collision_probability(): the worst case over the SIZE of the uncertainty, assuming
//    it is the same in both directions of the encounter plane (isotropic Gaussian). For a small
//    hard-body radius R and miss distance d it is about R^2 / (e d^2), at sigma = d / sqrt(2).
//  * collision_probability_ceiling(): the worst case over the size AND SHAPE of the uncertainty.
//    The hard-body disk lies inside a strip of width 2R, so no Gaussian can put more probability
//    in the disk than in the best strip. This is a hard upper bound for any Gaussian error and
//    is much larger than the isotropic maximum (about 2R / (d sqrt(2 pi e)) for small R/d).
//
// Geometry: the encounter is treated as a short, straight-line relative motion, so the miss
// vector at the time of closest approach (already perpendicular to the relative velocity) lies
// in the encounter plane, and the miss distance is the in-plane distance between the centres.
#include <cmath>

namespace orbitcore {

// Regularized lower incomplete gamma function P(a, x) for a > 0, x >= 0.
double regularized_gamma_p(double a, double x);

// Probability that an isotropic 2-D Gaussian with standard deviation sigma_km in each axis,
// centred miss_km from the origin, falls inside the disk of radius hbr_km at the origin.
double collision_probability_isotropic(double miss_km, double hbr_km, double sigma_km);

struct MaxProbability {
  double probability{0.0};
  double sigma_km{0.0};  // the uncertainty (per axis) at which the worst case occurs
};

// Worst case of collision_probability_isotropic() over sigma. Returns probability 1 and
// sigma 0 if the centres are already within the hard-body radius.
MaxProbability maximum_collision_probability(double miss_km, double hbr_km);

// Upper bound on the collision probability for ANY Gaussian covariance (any size, shape and
// orientation) centred miss_km from the origin.
double collision_probability_ceiling(double miss_km, double hbr_km);

}  // namespace orbitcore