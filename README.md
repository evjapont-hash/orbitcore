# orbitcore

C++20 orbital mechanics core for a real-time LEO situational-awareness display:
TLE ingest, near-Earth SGP4, frame transforms (TEME/ECEF/ENU/NED/RTN), and (planned)
conjunction assessment. Uses only unclassified public element data.

## Build and test

    cmake -S . -B build -DORBITCORE_SANITIZE=ON
    cmake --build build -j
    ctest --test-dir build --output-on-failure

## Status

| Component | State |
|-----------|-------|
| Time (JD, GMST) | done, tested |
| Frame transforms (TEME, ECEF, geodetic, ENU, NED, RTN) | done, tested |
| TLE parser | done, tested (OMM JSON: planned) |
| Near-Earth SGP4 | done, matches the Vallado reference set (9 satellites, 0 mismatches) |
| Validation | runs in CI, see [docs/VALIDATION.md](docs/VALIDATION.md) |
| Conjunction assessment | planned |

Limitations: near-Earth only (period < 225 min); public TLE accuracy is about 1 km at epoch.

## References

SGP4 follows the reference implementation in Vallado, Crawford, Hujsak, and Kelso,
"Revisiting Spacetrack Report #3", AIAA 2006-6753
(https://celestrak.org/publications/AIAA/2006-6753/).

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the system design.