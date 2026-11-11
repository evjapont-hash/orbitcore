#include "orbitcore/catalog.hpp"

#include <limits>

namespace orbitcore {

namespace {
bool starts_with(std::string_view s, std::string_view prefix) {
  return s.size() >= prefix.size() && s.substr(0, prefix.size()) == prefix;
}
}  // namespace

CatalogLoadStats Catalog::load_tle_text(std::string_view text) {
  CatalogLoadStats stats;

  // Split into lines, dropping trailing spaces and carriage returns.
  std::vector<std::string_view> lines;
  for (std::size_t pos = 0; pos < text.size();) {
    std::size_t end = text.find('\n', pos);
    if (end == std::string_view::npos) end = text.size();
    std::string_view line = text.substr(pos, end - pos);
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.remove_suffix(1);
    lines.push_back(line);
    pos = end + 1;
  }

  for (std::size_t i = 0; i + 1 < lines.size(); ++i) {
    if (!starts_with(lines[i], "1 ") || !starts_with(lines[i + 1], "2 ")) continue;

    std::string name;
    if (i > 0 && !lines[i - 1].empty() && !starts_with(lines[i - 1], "1 ") &&
        !starts_with(lines[i - 1], "2 ")) {
      name = std::string(lines[i - 1]);
    }

    const ParseResult parsed = parse_tle(lines[i], lines[i + 1]);
    ++i;  // line 2 is consumed whether or not it parsed
    if (!parsed.ok()) {
      ++stats.skipped_parse;
      continue;
    }
    Sgp4 propagator(parsed.elements);
    if (propagator.init_status() == Sgp4Error::DeepSpaceUnsupported) {
      ++stats.skipped_deep_space;
    } else if (propagator.init_status() != Sgp4Error::None) {
      ++stats.skipped_other;
    } else {
      names_.push_back(std::move(name));
      propagators_.push_back(std::move(propagator));
      ++stats.loaded;
    }
  }
  return stats;
}

bool Catalog::add_element_set(std::string name, const ElementSet& elements) {
  Sgp4 propagator(elements);
  if (propagator.init_status() != Sgp4Error::None) return false;
  names_.push_back(std::move(name));
  propagators_.push_back(std::move(propagator));
  return true;
}

void Catalog::replicate(std::size_t copies) {
  const std::size_t n = propagators_.size();
  if (copies <= 1 || n == 0) return;
  names_.reserve(n * copies);
  propagators_.reserve(n * copies);
  for (std::size_t c = 1; c < copies; ++c) {
    for (std::size_t i = 0; i < n; ++i) {
      names_.push_back(names_[i]);
      propagators_.push_back(propagators_[i]);
    }
  }
}

std::size_t Catalog::propagate_all(JulianDate t, double* out_xyz_km) const {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::size_t failures = 0;
  for (std::size_t i = 0; i < propagators_.size(); ++i) {
    const PropagationResult r = propagators_[i].propagate_to(t);
    if (r.ok()) {
      out_xyz_km[3 * i] = r.state.r.x;
      out_xyz_km[3 * i + 1] = r.state.r.y;
      out_xyz_km[3 * i + 2] = r.state.r.z;
    } else {
      out_xyz_km[3 * i] = out_xyz_km[3 * i + 1] = out_xyz_km[3 * i + 2] = nan;
      ++failures;
    }
  }
  return failures;
}

}  // namespace orbitcore