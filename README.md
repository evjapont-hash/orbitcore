# orbitcore

**Live demo: https://evjapont-hash.github.io/orbitcore/** (rebuilt every 6 hours with fresh public element sets)

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
| WebAssembly build (embind) | done, matches the native build to float32 precision |
| Web viewer (WebGL2) | done, deployed to GitHub Pages with scheduled data refresh |
| Conjunction assessment | planned |

Limitations: near-Earth only (period < 225 min); public TLE accuracy is about 1 km at epoch.

## Performance

Bulk propagation of the public catalog (CelesTrak "active" group plus four debris groups,
downloaded in early November 2026): 17,766 near-Earth objects propagated to a common
instant. 792 deep-space objects in the files are skipped.

| Objects | Median time for all objects | Per object |
|---------|-----------------------------|------------|
| 17,766 (real catalog) | 3.41 ms | 192 ns |
| 88,830 (real catalog repeated 5x, synthetic) | 17.69 ms | 199 ns |

Single thread, SGP4 in double precision, Release build (`-DCMAKE_BUILD_TYPE=Release`),
GCC 15.2.0, CPU: Intel(R) Core(TM) i9-14900HX
Reproduce with `./build-release/catalog_bench <tle-file>`.

## References

SGP4 follows the reference implementation in Vallado, Crawford, Hujsak, and Kelso,
"Revisiting Spacetrack Report #3", AIAA 2006-6753
(https://celestrak.org/publications/AIAA/2006-6753/).

Coastlines: Natural Earth 1:110m (public domain), https://www.naturalearthdata.com/.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the system design.