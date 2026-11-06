#pragma once
// Engine: a thin stateful wrapper around Catalog for the browser build.
// It owns a float32 position buffer that JavaScript reads directly from WebAssembly memory.
// Kept free of any Emscripten headers so it can be compiled and tested natively.
#include <cstddef>
#include <string>
#include <vector>

#include "orbitcore/catalog.hpp"

namespace orbitcore {

class Engine {
 public:
  // Adds the satellites found in `text` (2- or 3-line element sets). Returns the total
  // number of objects now loaded.
  int load_tle(const std::string& text) {
    stats_ = catalog_.load_tle_text(text);
    scratch_.assign(3 * catalog_.size(), 0.0);
    positions_.assign(3 * catalog_.size(), 0.0f);
    return count();
  }

  int count() const { return static_cast<int>(catalog_.size()); }
  int skipped_deep_space() const { return static_cast<int>(stats_.skipped_deep_space); }
  int skipped_unparseable() const { return static_cast<int>(stats_.skipped_parse); }

  int satnum(int i) const {
    return in_range(i) ? catalog_.satnum(static_cast<std::size_t>(i)) : -1;
  }
  std::string name(int i) const {
    return in_range(i) ? catalog_.name(static_cast<std::size_t>(i)) : std::string();
  }

  // Greenwich Mean Sidereal Time (radians) at the given UTC instant, for rotating the Earth.
  double gmst(double jd_day, double jd_frac) const { return gmst_rad({jd_day, jd_frac}); }

  // Propagates every object to the UTC instant (jd_day + jd_frac). Computation is in double
  // precision; results are stored as float32 TEME positions in km (x, y, z per object),
  // which is what the GPU wants. Returns the number of objects that failed (stored as NaN).
  int propagate(double jd_day, double jd_frac) {
    const std::size_t failures = catalog_.propagate_all({jd_day, jd_frac}, scratch_.data());
    for (std::size_t i = 0; i < scratch_.size(); ++i) {
      positions_[i] = static_cast<float>(scratch_[i]);
    }
    return static_cast<int>(failures);
  }

  const float* positions() const { return positions_.data(); }
  std::size_t position_count() const { return positions_.size(); }  // number of floats

 private:
  bool in_range(int i) const { return i >= 0 && static_cast<std::size_t>(i) < catalog_.size(); }

  Catalog catalog_;
  CatalogLoadStats stats_;
  std::vector<double> scratch_;
  std::vector<float> positions_;
};

}  // namespace orbitcore