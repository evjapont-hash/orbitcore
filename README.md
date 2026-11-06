# orbitcore

C++20 orbital mechanics core for a real-time LEO situational-awareness display:
TLE/OMM ingest, near-Earth SGP4, frame transforms (TEME/ECEF/ENU/NED/RTN), and (later)
conjunction assessment. Uses only unclassified public element data.

## Build & test

```sh
cmake -S . -B build -DORBITCORE_SANITIZE=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Status

| Component | State |
|-----------|-------|
| Time (JD, GMST) | done, tested |
| Frame transforms (TEME, ECEF, geodetic, ENU, NED, RTN) | done, tested |
| TLE parser | done, tested (OMM JSON: planned) |
| Near-Earth SGP4 | **TODO** (`src/sgp4.cpp`) |
| Vallado validation | harness ready; needs data files (`tests/data/README.md`) |
| Conjunction assessment | planned (phase 4) |

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the system design.
