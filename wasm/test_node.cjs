// Checks the WebAssembly build from Node.
//
//   node wasm/test_node.cjs examples/iss_2008.tle [catalog.tle]
//
// 1) Correctness: ISS 2008 at epoch + 10 min must match the native orbit_cli to float32 precision.
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

  // --- 1) correctness against the native tool -------------------------------------------
  const iss = new oc.Engine();
  iss.loadTle(fs.readFileSync(issPath, 'utf8'));
  // TLE epoch is 2008 day 264.51782528 = JD 2454466.5 + 263.51782528; look 10 minutes later.
  const failures = iss.propagate(2454466.5, 263.51782528 + 10 / 1440);
  const p = iss.positions();
  const expected = [4550.07523, 3253.056114, 3728.338224]; // native orbit_cli, t = 10 min (km)
  let maxErr = 0;
  for (let k = 0; k < 3; k++) maxErr = Math.max(maxErr, Math.abs(p[k] - expected[k]));
  console.log(`ISS ${iss.name(0)} (NORAD ${iss.satnum(0)}), failures: ${failures}`);
  console.log(`  wasm:   ${Array.from(p).map((v) => v.toFixed(3)).join(', ')} km`);
  console.log(`  native: ${expected.map((v) => v.toFixed(3)).join(', ')} km`);
  console.log(`  max difference ${maxErr.toExponential(2)} km -> ${maxErr < 0.01 ? 'PASS' : 'FAIL'}`);
  if (!(maxErr < 0.01)) ok = false;
  iss.delete();

  // --- 2) speed on a real catalog --------------------------------------------------------
  if (catalogPath) {
    const engine = new oc.Engine();
    const text = fs.readFileSync(catalogPath, 'utf8');
    let t = performance.now();
    const loaded = engine.loadTle(text);
    const loadMs = performance.now() - t;
    console.log(`\nLoaded ${loaded} objects in ${loadMs.toFixed(1)} ms (skipped ${engine.skippedDeepSpace()} deep-space)`);

    const jdFrac = Date.now() / 86400000; // JD = 2440587.5 + unix days
    engine.propagate(2440587.5, jdFrac); // warm-up
    const times = [];
    let lastFailures = 0;
    for (let k = 0; k < 20; k++) {
      t = performance.now();
      lastFailures = engine.propagate(2440587.5, jdFrac + k / 1440);
      times.push(performance.now() - t);
    }
    times.sort((a, b) => a - b);
    const median = times[Math.floor(times.length / 2)];
    console.log(`Full-catalog propagation (WebAssembly, ${loaded} objects, 20 passes):`);
    console.log(`  median ${median.toFixed(3)} ms   best ${times[0].toFixed(3)} ms`);
    console.log(`  ${(median * 1e6 / loaded).toFixed(0)} ns per object (median)   failures: ${lastFailures}`);
    engine.delete();
  }
  process.exit(ok ? 0 : 1);
}

main();