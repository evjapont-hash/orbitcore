# Validation

What is checked, against what, and the result. Everything here is reproducible with the commands in the right-hand column. "Not validated yet" lists what is not covered.

## Propagation and frames

| Check | Against | Tolerance | Result | Where |
|-------|---------|-----------|--------|-------|
| SGP4 position and velocity | Vallado's published verification vectors (`tests/data/SGP4-VER.TLE`, `tmatverDec2015.out`) | 1e-6 km, 1e-8 km/s | 9 near-Earth satellites compared, **0 mismatches**; 23 deep-space cases skipped (not implemented) | `sgp4_vallado_vectors` |
| Time: Julian date, GMST | published values at J2000; calendar round trip | 1e-9 | pass | `test_time_frames.cpp` |
| Frames: geodetic round trip, ENU / NED, TEME to ECEF, RTN of a circular orbit | closed-form geometry | tight analytic tolerances | pass | `test_time_frames.cpp` |
| Station visibility (elevation, azimuth, range) | closed-form elevation; the library's own frame chain | 1e-9 | pass | `test_visibility.cpp` |
| WebAssembly vs. native | the same positions computed natively | float32 rendering precision | max difference 8.9e-5 km; describe-values match | `wasm/test_node.cjs` |

## Screening

| Check | Against | Result | Where |
|-------|---------|--------|-------|
| Same encounters as a brute-force search | a search stepping every object pair at a fixed step and bisecting range-rate sign changes | identical encounter lists in unit tests; on the real catalog (ISS, 2 h, 100 km, 2 s steps): **160 encounters identical**, worst difference 0.0001 s in TCA and 0.000000 km in miss | `screening_matches_brute_force`; `conjunction_cli --check-brute-force 2` |
| Cost | the same brute-force search | 90.5 ms vs. 21,030 ms; 0.44 M vs. 129.3 M evaluations: **294x fewer** | same command |
| The safe-advance bound is conservative | propagated object pairs | never violated | `safe_advance_is_conservative_on_propagated_pairs` |
| The radial prefilter loses nothing | the same screening with the prefilter off | same events | `screening_without_radial_filter_finds_the_same_events` |
| Closest-approach accuracy | constructed crossing orbits with a known answer | pass | `closest_approach_is_accurate` |
| Chunked sessions are exact | one-shot screening | identical, bit for bit, for chunk sizes from 1 to the whole catalog | `screening_in_chunks_equals_one_shot`; `engine_screening_...` |

## Collision-probability bounds

| Check | Against | Result | Where |
|-------|---------|--------|-------|
| The probability series | an independent polar-grid numerical integration of the Gaussian over the disk (uses no special functions) | agreement within 5e-5 relative, which is the reference grid's own error (it shrinks 16x per 4x refinement) | `isotropic_probability_matches_numerical_integration` |
| Exact limits | closed forms: zero miss, `R << sigma` | within 1e-12 and 1e-5 | `test_risk.cpp` |
| Pc_max | the closed form `R^2 / (e d^2)` at `sigma = d / sqrt(2)`; sweeps over sigma | matches; no sigma does better | `maximum_probability_...` |
| Pc_ceiling | integrals over 180 covariances (6 sizes x 6 sizes x 5 orientations, including thin needles along the miss vector) | **never exceeded**, and reached more than half of the bound (so it is not vacuous) | `ceiling_bounds_every_gaussian_covariance` |

## Burn planning

| Check | Against | Result | Where |
|-------|---------|--------|-------|
| The linear response | a brute-force two-body integration (RK4) of a burned and an unburned orbit, 4 burns x 4 lead times | within 1% (plus 0.1 m) | `hill_response_matches_two_body_integration` |
| **The whole chain** | plan a burn from the encounter geometry, apply it to the primary, integrate both objects through the encounter, measure the miss: 3 crossing angles x 5 lead times | planned for a 6.0 km miss, measured **5.999 to 6.002 km** (the test requires within 0.01 km) | `planned_burn_achieves_the_target_miss_in_two_body_motion` |
| It is the smallest burn that works | an exhaustive search of burn directions on a 360 x 720 grid on the sphere | a burn 1% smaller falls short in **every** direction; one 1% larger has room to spare | `planned_burn_is_the_smallest_that_works` |
| Special cases | analytic answers (whole-orbit leads, in-track-only, direct hit, head-on in-track, invalid input) | pass | `plan_burn_special_cases` |
| The browser's copy of the burn model | the C++ numbers: three reference cases to 12 digits, and for a real encounter on every option | agrees (`agrees with C++ for 4 options -> PASS` in the run on a real catalog) | `web/maneuver.test.mjs`, `wasm/test_node.cjs` |

**A development note.** The first draft of the planner had the sign of the miss change reversed (the miss vector is secondary minus primary, so moving the primary changes it by the negative of the displacement). The whole-chain test caught it at once: burns planned for 6 km produced 3.1 to 5.4 km. It was fixed, and that test now guards it. This is the reason the planner is validated against independent dynamics rather than against itself.

## Software quality

- 49 C++ test cases (a small custom harness), run in CI with g++ and clang++ under AddressSanitizer and UBSan.
- 6 JavaScript test suites (`mat4`, `picking`, `geometry`, `coastline`, `screening`, `maneuver`).
- The engine wrapper is header-only and tested natively; the page is also exercised in development against a simulated browser.
- The page's data refresh falls back to the last published catalog and labels it when CelesTrak is unreachable.

## Not validated yet

- **The burn plans against J2 propagation.** The linear model ignores J2 and drag; the plan to check it numerically is not done.
- **Against an independent conjunction service** (for example a public screening report) over the same window.
- **Real-sky accuracy.** Whether TLE-derived misses match true misses cannot be tested without precise ephemerides.
- **Deep-space objects** (not implemented).
- **Re-screening after a burn** for newly created conjunctions.