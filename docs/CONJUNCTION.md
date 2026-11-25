# Conjunction assessment

What this does, how, a worked example from a real run, and where it stops being trustworthy.

**Pipeline:** screen one satellite against the catalog, then for each close approach compute its geometry, bound the collision probability, and plan a burn that increases the miss distance.

## 1. Screening

**Question:** which objects come within `threshold` km of the primary during the next `T` days, and exactly when?

Checking every pair every second would cost hundreds of millions of propagations. Instead:

1. **Radial prefilter.** An object whose perigee-to-apogee shell can never come within the threshold (plus a 100 km margin) of the primary's shell is skipped.
2. **Conservative advancement.** For each remaining object, take the relative position `r` and velocity `v`. If `|r|` is larger than the threshold, the distance cannot drop below the threshold before the time `dt` at which `(closest distance of the straight-line path over [0, dt]) - 0.5 * a_max * dt^2` reaches it, where `a_max` (0.02 km/s^2) bounds how fast the relative velocity can change. The search jumps ahead by that `dt` (at most 600 s), which is provably safe, so a fast encounter cannot be stepped over.
3. **Near the threshold,** it steps 1 s at a time and watches the range rate `r . v`. A change from negative to non-negative marks a closest approach; bisection refines the time to 0.0001 s.

Each encounter records the time of closest approach (TCA), the miss distance, the relative speed, both objects' TEME states at that moment, and the miss vector in the primary's radial / in-track / cross-track (RTN) axes.

**Performance:** the whole catalog for a 3-day window at 25 km takes about 2.7 s natively (about 14 million SGP4 evaluations); in a run compared against brute force it needed 294x fewer evaluations and found identical encounters.

## 2. Collision probability without a covariance

A collision-probability number needs the position uncertainty (a covariance). Public element sets do not provide one. Rather than invent one, the tool reports **bounds**, using a combined hard-body radius `R` (you choose it; default 10 m) and the miss distance `d` in the plane perpendicular to the relative velocity:

- **Pc_max:** the largest probability over the *size* of the uncertainty, assuming it is equal in both axes of the encounter plane (an isotropic Gaussian). It occurs at an uncertainty of about `d / sqrt(2)` (shown as sigma*), and for small `R` is approximately `R^2 / (e d^2)`. The probability for a given uncertainty is computed exactly (a Poisson mixture of regularized incomplete gamma functions).
- **Pc_ceiling:** an upper bound for **any** Gaussian uncertainty, whatever its size, shape, or orientation: the disk lies inside a strip of width `2R`, so no Gaussian can put more probability in the disk than in the best strip. For small `R` it is approximately `2R / (d sqrt(2 pi e))`.

How to read them: with fresh elements the real uncertainty (about 1 km) is far smaller than sigma* for typical misses, so the true probability is much lower than Pc_max; a strongly elongated uncertainty (real along-track errors are) can exceed Pc_max but never Pc_ceiling. **Neither is a prediction.** Operational action thresholds (on the order of 1e-4 to 1e-5) apply to real covariances from tracking data, so the summary counts in the command-line tool are illustrative only.

## 3. Avoidance-burn planning

**Question:** if the primary fires one impulsive burn some time before the closest approach, what is the smallest burn that raises the miss distance to a target?

**Model.** In the primary's RTN axes, a small burn `dv` displaces the primary, `tau` seconds later, by a quantity linear in `dv` (Hill's / Clohessy-Wiltshire equations) for a circular reference orbit of mean motion `n`:

```
 x (radial)      = ( sin(n tau)/n )         dvx + ( 2(1 - cos(n tau))/n )         dvy
 y (in-track)    = ( 2(cos(n tau) - 1)/n )  dvx + ( (4 sin(n tau) - 3 n tau)/n )  dvy
 z (cross-track) = ( sin(n tau)/n )         dvz
```

The in-track term grows like `3 tau dvy`: a small prograde or retrograde burn made early moves the object a long way along its orbit, which is why early burns are cheap. At a whole number of orbits only in-track burns do anything.

**What matters for the miss.** The encounter is fast (about 10 km/s), so only the part of the displacement *perpendicular* to the relative velocity changes the miss distance; the rest just shifts the time of closest approach. The planner works in that plane. The miss vector is *secondary minus primary*, so displacing the primary by `D` changes it by `-D`.

**Optimization.** For a given lead time, it finds the burn of least magnitude whose displacement puts the new miss vector at distance `target`. The new miss vector lies on a circle of radius `target`; the burn cost for each point on the circle is a quadratic form, so the minimum is found by a one-dimensional search over the angle (with a closed-form branch for the degenerate whole-orbit case). It is an exact minimization, not a heuristic.

**Output.** For lead times of 1/4, 1/2, 3/4, 1, 1.5, 2, and 3 orbits: the burn in RTN components, its size, a plain-words direction, where and when it happens, and the new miss and Pc_max. Options that cannot work are labelled: *no burn needed* (already far enough), *cannot reach the target at this lead time* (degenerate geometry), or *would have to happen before the screening started*.

**In the viewer:** choosing an option jumps the clock to the burn and draws a green arrow along the burn direction; **At closest approach** shows a cyan ghost of the burned satellite and its new, larger separation from the other object next to the red line of the original miss.

## 4. Worked example (a real run)

ISS (NORAD 25544), 10 m hard body. The element set in this run was 3.6 days old, so treat the numbers as an illustration of the mechanics.

Event 21 of 91 in the window:

| | |
|---|---|
| Secondary | STARLINK-2063 (NORAD 48553) |
| Closest approach | 2026-11-25 05:28:21 UTC |
| Miss distance | 17.106 km |
| Relative speed | 10.95 km/s |
| Miss in R / T / N | 12.34 / -8.29 / -8.46 km |
| Pc_max | 1.3e-7 (worst case, at an uncertainty of 12.10 km) |
| Pc_ceiling | 2.8e-4 |

Planning a burn to a 23 km miss (new Pc_max 7.0e-8):

| Lead | Size | Direction |
|------|------|-----------|
| 23 min | 2.636 m/s | mostly retrograde |
| 46 min | 0.852 m/s | mostly retrograde |
| 70 min | 0.605 m/s | retrograde |
| 93 min | 0.646 m/s | retrograde |
| 139 min | 0.375 m/s | retrograde |
| 3.1 h | 0.323 m/s | retrograde |
| 4.6 h | 0.215 m/s | retrograde |

Short notice is expensive and early is cheap. The sizes are not strictly decreasing because the encounter geometry matters: the 93-minute option (about one orbit) can only use an in-track burn, and happens to be slightly more expensive than the 70-minute one. (This event was not dangerous to begin with; the point is the mechanics.)

## 5. How to reproduce

```sh
./build-release/conjunction_cli ~/catalog.tle --primary 25544 --days 3 --threshold-km 25 --hbr-m 50
./build-release/conjunction_cli ~/catalog.tle --primary 25544 --hours 2 --threshold-km 100 --check-brute-force 2
./build/orbitcore_tests            # includes the risk and maneuver tests
node wasm/test_node.cjs examples/iss_2008.tle ~/catalog.tle    # screening and planning in WebAssembly
```

## Limits

1. **Element-set accuracy.** TLE positions are about 1 km at epoch and worse with age; misses of a few kilometres are inside the noise. The viewer shows the primary's element age and flags more than 3 days.
2. **No covariance,** so only bounds (section 2).
3. **Straight-line relative motion** is assumed near the closest approach. That holds for the fast encounters typical of LEO; very slow relative speeds would break it.
4. **The burn planner** assumes a near-circular reference orbit, a single impulsive burn, and short lead times (a few orbits); the burn's effect ignores J2 and drag (a numerical J2 check is planned). It does not re-screen the maneuvered orbit, so a burn can create a *new* conjunction. It ignores operational constraints (thrust windows, attitude, ground contact, fuel).
5. **Near-Earth objects only.** Deep-space objects (period of 225 minutes or more) are skipped: 789 of about 18,500 in the data used here.
6. **One primary at a time,** and the secondary is assumed not to maneuver.
7. **The hard-body radius is an input,** not derived from the objects.