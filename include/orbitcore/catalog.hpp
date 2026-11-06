#pragma once
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "orbitcore/sgp4.hpp"

namespace orbitcore {

struct CatalogLoadStats {
  std::size_t loaded{0};              // objects ready to propagate
  std::size_t skipped_parse{0};       // malformed lines or failed checksum
  std::size_t skipped_deep_space{0};  // period >= 225 min (not supported)
  std::size_t skipped_other{0};       // SGP4 initialization failed for another reason
};

// A collection of satellites that can all be propagated to the same instant.
class Catalog {
 public:
  // Parses text containing 2-line or 3-line (name + 2 lines) element sets.
  // Objects that cannot be propagated are counted in the returned stats, not loaded.
  CatalogLoadStats load_tle_text(std::string_view text);

  // Stress testing only: repeat the loaded objects so the catalog is `copies` times larger.
  void replicate(std::size_t copies);

  std::size_t size() const { return propagators_.size(); }
  const std::string& name(std::size_t i) const { return names_[i]; }
  int satnum(std::size_t i) const { return propagators_[i].elements().satnum; }
  const ElementSet& elements(std::size_t i) const { return propagators_[i].elements(); }

  // Propagates a single object to the instant `t` (UTC).
  PropagationResult propagate_one(std::size_t i, JulianDate t) const {
    return propagators_[i].propagate_to(t);
  }

  // Propagates every object to the instant `t` (UTC). Writes TEME positions in km into
  // `out_xyz_km`, which must hold 3 * size() doubles (x0,y0,z0,x1,y1,z1,...).
  // Objects whose propagation fails get NaN. Returns the number of failures.
  std::size_t propagate_all(JulianDate t, double* out_xyz_km) const;

 private:
  std::vector<std::string> names_;
  std::vector<Sgp4> propagators_;
};

}  // namespace orbitcore