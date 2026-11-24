// Checks the WebAssembly build from Node.
//
//   node wasm/test_node.cjs examples/iss_2008.tle [catalog.tle]
//
// 1) Correctness: ISS 2008 at epoch + 10 min must match the native orbit_cli to float32 precision,
//    including the describe(), track() and ground-station visibility results the viewer uses.
// 2) Speed (if a catalog file is given): time a full-catalog propagation in WebAssembly.
const fs = require('fs');
const path = require('path');

const createOrbitcore = require(path.join(__dirname, '..', 'build-wasm', 'orbitcore_wasm.js'));

async function main() {
  const [issPath, catalogPath] = process.argv.slice(2);
  if (!issPath) {
    console.error('usage: node wasm/test_node.cjs <iss_2008.tle> [catalog.tle]');
    process.exit(2);
  }
  const oc = await createOrbitcore();
  let ok = true;
  const check = (label, pass) => {
    console.log(`  ${label} -> ${pass ? 'PASS' : 'FAIL'}`);
    if (!pass) ok = false;
  };

  // --- 1) correctness against the native tool -------------------------------------------
  const iss = new oc.Engine();
  iss.loadTle(fs.readFileSync(issPath, 'utf8'));
  // TLE epoch is 2008 day 264.51782528 = JD 2454466.5 + 263.51782528; look 10 minutes later.
  const jdDay = 2454466.5;
  const jdFrac = 263.51782528 + 10 / 1440;
  const failures = iss.propagate(jdDay, jdFrac);
  const p = iss.positions();
  const expected = [4550.07523, 3253.056114, 3728.338224]; // native orbit_cli, t = 10 min (km)
  let maxErr = 0;
  for (let k = 0; k < 3; k++) maxErr = Math.max(maxErr, Math.abs(p[k] - expected[k]));
  console.log(`ISS ${iss.name(0)} (NORAD ${iss.satnum(0)}), failures: ${failures}`);
  console.log(`  wasm:   ${Array.from(p).map((v) => v.toFixed(3)).join(', ')} km`);
  console.log(`  native: ${expected.map((v) => v.toFixed(3)).join(', ')} km`);
  check(`positions: max difference ${maxErr.toExponential(2)} km`, maxErr < 0.01);

  // describe(): geodetic position, speed, and orbit shape (layout is documented in engine.hpp)
  const described = iss.describe(0, jdDay, jdFrac);
  const info = iss.info();
  console.log(`  describe: lat ${info[8].toFixed(4)} deg, lon ${info[9].toFixed(4)} deg, alt ${info[10].toFixed(3)} km, ` +
              `speed ${info[7].toFixed(5)} km/s, period ${info[11].toFixed(2)} min, ` +
              `perigee/apogee ${info[14].toFixed(1)}/${info[15].toFixed(1)} km, age ${info[16].toFixed(5)} d`);
  check('describe: lat/lon/alt/speed match native',
        described && Math.abs(info[8] - 33.854631) < 1e-3 && Math.abs(info[9] + 153.126294) < 1e-3 &&
        Math.abs(info[10] - 350.520963) < 0.01 && Math.abs(info[7] - 7.705863) < 1e-4);
  check('describe: element-set age is 10 minutes', Math.abs(info[16] - 10 / 1440) < 1e-9);

  // track(): one orbit of positions, all finite and at LEO radius
  const valid = iss.track(0, jdDay, jdFrac, 92, 50);
  const tp = iss.trackPoints();
  let radiusOk = tp.length === 150;
  for (let j = 0; j < 50 && radiusOk; j++) {
    const r = Math.hypot(tp[3 * j], tp[3 * j + 1], tp[3 * j + 2]);
    radiusOk = r > 6700 && r < 6800;
  }
  check(`track: ${valid} valid points, ${tp.length} floats, LEO radii`, valid === 50 && radiusOk);

  // stations: one directly under the ISS (must see it) and one on the far side (must not)
  const subLat = info[8], subLon = info[9];
  const s0 = iss.addStation(subLat, subLon, 0, 10);
  const s1 = iss.addStation(-subLat, subLon + 180, 0, 10);
  iss.propagate(jdDay, jdFrac);
  iss.updateVisibility(jdDay, jdFrac);
  const flags = iss.visibility();
  const seen = iss.look(0, 0, jdDay, jdFrac);
  const look = iss.lookInfo();
  console.log(`  stations: overhead el ${look[1].toFixed(2)} deg, range ${look[2].toFixed(1)} km; ` +
              `visible counts ${iss.visibleCount(0)} / ${iss.visibleCount(1)}; flags ${flags[0]}`);
  check('stations: overhead station sees the ISS, far-side station does not',
        s0 === 0 && s1 === 1 && seen && flags.length === 1 && flags[0] === 1 &&
        iss.visibleCount(0) === 1 && iss.visibleCount(1) === 0 && look[1] > 89.5);
  iss.clearStations();
  iss.delete();

  // --- 2) speed on a real catalog --------------------------------------------------------
  if (catalogPath) {
    const engine = new oc.Engine();
    const text = fs.readFileSync(catalogPath, 'utf8');
    let t = performance.now();
    const loaded = engine.loadTle(text);
    const loadMs = performance.now() - t;
    console.log(`\nLoaded ${loaded} objects in ${loadMs.toFixed(1)} ms (skipped ${engine.skippedDeepSpace()} deep-space)`);

    const nowFrac = Date.now() / 86400000; // JD = 2440587.5 + unix days
    engine.propagate(2440587.5, nowFrac); // warm-up
    const times = [];
    let lastFailures = 0;
    for (let k = 0; k < 20; k++) {
      t = performance.now();
      lastFailures = engine.propagate(2440587.5, nowFrac + k / 1440);
      times.push(performance.now() - t);
    }
    times.sort((a, b) => a - b);
    const median = times[Math.floor(times.length / 2)];
    console.log(`Full-catalog propagation (WebAssembly, ${loaded} objects, 20 passes):`);
    console.log(`  median ${median.toFixed(3)} ms   best ${times[0].toFixed(3)} ms`);
    console.log(`  ${(median * 1e6 / loaded).toFixed(0)} ns per object (median)   failures: ${lastFailures}`);
    
    // Conjunction screening session: the ISS against the catalog over 6 hours, in chunks.
    let primary = -1;
    for (let i = 0; i < loaded; i++) if (engine.satnum(i) === 25544) { primary = i; break; }
    if (primary >= 0) {
      const STRIDE = 23;
      const screen = (chunk) => {
        if (!engine.screenBegin(primary, 2440587.5, nowFrac, 6 * 60, 25, 50)) throw new Error('screenBegin failed');
        let progress = 0, steps = 0;
        const t0 = performance.now();
        while (progress < 1) { progress = engine.screenStep(chunk); steps++; }
        return { rows: Array.from(engine.screenResults()), steps, ms: performance.now() - t0,
                 stats: Array.from(engine.screenStats()) };
      };
      const small = screen(37);
      const large = screen(100000);
      const n = small.rows.length / STRIDE;
      console.log(`\nScreening ${engine.name(primary)} over 6 h (25 km, 50 m): ${n} encounters in ${small.ms.toFixed(0)} ms ` +
                  `(${small.steps} chunks of 37 objects), ${(small.stats[2] / 1e6).toFixed(2)} M evaluations`);
      check('screening: chunk size does not change the results',
            small.rows.length === large.rows.length && small.rows.every((v, i) => v === large.rows[i]));
      let sorted = true, bounded = true, plausible = true;
      for (let i = 0; i < n; i++) {
        const r = small.rows.slice(i * STRIDE, (i + 1) * STRIDE);
        if (i > 0 && (r[1] - small.rows[(i - 1) * STRIDE + 1]) + (r[2] - small.rows[(i - 1) * STRIDE + 2]) < 0) sorted = false;
        if (!(r[3] > 0 && r[3] <= 25)) bounded = false;
        if (!(r[8] <= r[10] && r[8] >= 0 && r[10] <= 1 && r[4] > 0.1 && r[4] < 17)) plausible = false;
      }
      check(`screening: ${n} encounters sorted by time, within the threshold, Pc_max <= Pc_ceil`, sorted && bounded && plausible);

      if (n > 0) {
        // Avoidance burns for the first encounter: raise its miss distance by 5 km.
        const BURN_STRIDE = 17;
        const target = small.rows[3] + 5;
        const options = engine.planAvoidance(0, target);
        const plan = Array.from(engine.planResults());
        console.log(`\nAvoidance burns for the first encounter (miss ${small.rows[3].toFixed(2)} km -> ${target.toFixed(2)} km):`);
        let consistent = options === 7 && plan.length === 7 * BURN_STRIDE;
        for (let k = 0; k < options; k++) {
          const r = plan.slice(k * BURN_STRIDE, (k + 1) * BURN_STRIDE);
          console.log(`  lead ${(r[0] / 60).toFixed(0).padStart(4)} min  status ${r[1]}  dv ${r[5].toFixed(4)} m/s ` +
                      `(R ${r[2].toFixed(4)}, T ${r[3].toFixed(4)}, N ${r[4].toFixed(4)})  miss after ${r[7].toFixed(3)} km  Pc_max ${r[8].toExponential(1)}`);
          if (r[1] === 0 && (Math.abs(r[7] - target) > 1e-6 || !(r[5] > 0 && r[5] < 100) ||
                             Math.abs(Math.hypot(r[14], r[15], r[16]) - 1) > 1e-9)) consistent = false;
        }
        check('planning: 7 lead times; every feasible burn reaches the target with a unit direction', consistent);

        // The page re-derives the displacement with its own copy of the linear model (web/maneuver.js).
        // For the same burns it must agree with the C++ result.
        try {
          const { cwDisplacementKm, inPlaneMiss, projectOnRtn, rtnAxes } = await import('../web/maneuver.js');
          engine.describe(primary, 2440587.5, nowFrac);
          const n = 2 * Math.PI / (engine.info()[11] * 60);
          const e = small.rows.slice(0, STRIDE);
          const axes = rtnAxes(e.slice(11, 14), e.slice(14, 17));
          const relativeVelocity = projectOnRtn(axes, [e[20] - e[14], e[21] - e[15], e[22] - e[16]]);
          let agree = true, compared = 0;
          for (let k = 0; k < options; k++) {
            const r = plan.slice(k * BURN_STRIDE, (k + 1) * BURN_STRIDE);
            if (r[1] !== 0) continue;
            const miss = inPlaneMiss(e.slice(5, 8), relativeVelocity, cwDisplacementKm(n, r[0], r.slice(2, 5)));
            compared++;
            if (Math.abs(miss - r[7]) > 1e-6) agree = false;
          }
          check(`planning: the page's copy of the burn model agrees with C++ for ${compared} options`, agree && compared > 0);
        } catch (err) {
          console.log(`  (skipped the page-model cross-check: ${err.message})`);
        }
      }
    }
    engine.delete();
  }
  process.exit(ok ? 0 : 1);
}

main();