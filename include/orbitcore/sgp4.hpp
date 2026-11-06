#pragma once
#include "orbitcore/frames.hpp"
#include "orbitcore/tle.hpp"
#include "orbitcore/units.hpp"

namespace orbitcore {

// Values 1-6 match the error codes in Vallado's reference implementation so you
// can cross-check against it directly.
enum class Sgp4Error {
  None = 0,
  MeanElements = 1,       // eccentricity >= 1 or < -0.001, or semi-major axis < 0.95 er
  MeanMotion = 2,         // mean motion < 0
  PerturbedElements = 3,  // perturbed eccentricity < 0 or > 1
  SemiLatusRectum = 4,    // semi-latus rectum < 0
  SubOrbital = 5,         // epoch elements are sub-orbital
  Decayed = 6,            // satellite has decayed
  // orbitcore-specific:
  NotImplemented = 100,
  DeepSpaceUnsupported = 101,  // period >= 225 min: needs SDP4 deep-space terms (out of scope)
};

struct Sgp4State {
  Position<Frame::TEME> r;  // km
  Velocity<Frame::TEME> v;  // km/s
};

struct PropagationResult {
  Sgp4Error error{Sgp4Error::None};
  Sgp4State state{};
  bool ok() const { return error == Sgp4Error::None; }
};

// Near-Earth SGP4 propagator (LEO/MEO, period < 225 min).
class Sgp4 {
 public:
  explicit Sgp4(const ElementSet& elements, const GravityModel& grav = wgs72());

  Sgp4Error init_status() const noexcept { return init_status_; }
  const ElementSet& elements() const noexcept { return elements_; }

  // State at `minutes_since_epoch` (negative allowed), TEME frame.
  PropagationResult propagate(double minutes_since_epoch) const;
  PropagationResult propagate_to(JulianDate t) const;

 private:
  ElementSet elements_;
  GravityModel grav_;
  Sgp4Error init_status_{Sgp4Error::NotImplemented};

  // Constants computed once in the constructor (Vallado's `sgp4init`) and reused by
  // every call to propagate(). Names follow the reference implementation so you can
  // compare it line by line.
  struct Coeffs {
    double no{0.0};  // un-Kozai'd ("Brouwer") mean motion, rad/min
    double mdot{0.0}, argpdot{0.0}, nodedot{0.0}, nodecf{0.0};
    double cc1{0.0}, cc4{0.0}, cc5{0.0};
    double t2cof{0.0}, t3cof{0.0}, t4cof{0.0}, t5cof{0.0};
    double d2{0.0}, d3{0.0}, d4{0.0};
    double omgcof{0.0}, xmcof{0.0}, eta{0.0}, delmo{0.0}, sinmao{0.0};
    double aycof{0.0}, xlcof{0.0}, con41{0.0}, x1mth2{0.0}, x7thm1{0.0};
    bool isimp{false};  // true => "simple" drag model (perigee below 220 km)
  };
  Coeffs c_{};
};

// Period >= 225 minutes => deep-space (SDP4) branch, which orbitcore does not implement.
bool is_deep_space(const ElementSet& elements);

}  // namespace orbitcore
