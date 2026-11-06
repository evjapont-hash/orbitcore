# Validation

## SGP4 against the Vallado reference set

**Reference:** Vallado, Crawford, Hujsak, Kelso, "Revisiting Spacetrack Report #3",
AIAA 2006-6753 (https://celestrak.org/publications/AIAA/2006-6753/).
Element sets: `tests/data/SGP4-VER.TLE`. Expected states: `tests/data/tmatverDec2015.out`
(from the reference MATLAB implementation).

**Method:** for each satellite with a matching element set, propagate to every time listed in
the reference file and compare TEME position and velocity component by component.

**Tolerances:** position 1e-6 km (1 mm) and velocity 1e-8 km/s, chosen a little looser than
the reference file's printed precision (1e-8 km and 1e-9 km/s).

**Result (enforced in CI):** 9 near-Earth satellites compared at every reference time,
0 mismatches. 23 deep-space satellites in the set are skipped (see Limitations).

**Gravity constants:** WGS-72, as SGP4 requires. Geodetic conversions use WGS-84.

## Other tests

- Julian date and GMST checked against J2000 values.
- Geodetic <-> ECEF round trips, including poles and negative altitudes.
- ENU / NED / look-angle and RTN conversions on hand-checkable geometry.
- Rotation and velocity transforms checked by identities (for example, a point fixed to the
  Earth must have zero Earth-relative velocity).
- TLE parsing of a published example, plus checksum and format error handling.

## Limitations (not implemented or not yet validated)

- Deep-space objects (period >= 225 min) are rejected with an error, not propagated.
- TEME -> ECEF uses GMST only, computed from UTC treated as UT1. Polar motion and nutation
  are neglected. These transforms are tested by identities, not yet against an external
  reference.
- Public TLE accuracy is on the order of 1 km at epoch and degrades by kilometers per day.
  SGP4 reproduces the model, not the true orbit.

## Reproduce

    cmake -S . -B build
    cmake --build build -j
    ./build/orbitcore_tests