// Run with: node web/maneuver.test.mjs
import assert from 'node:assert/strict';
import {
  BURN_STATUS, BURN_STRIDE, burnDirectionLabel, cwDisplacementKm, formatLead, fromRtn, inPlaneMiss, parseBurnOptions,
  projectOnRtn, rtnAxes, statusNote,
} from './maneuver.js';

const near = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b}`);
const nearVec = (a, b, tol, what) => a.forEach((v, k) => near(v, b[k], tol, `${what}[${k}]`));

// --- frame axes ---
{
  const eq = rtnAxes([7000, 0, 0], [0, 7.5, 0]);
  nearVec(eq.radial, [1, 0, 0], 1e-15, 'R'); nearVec(eq.inTrack, [0, 1, 0], 1e-15, 'T'); nearVec(eq.normal, [0, 0, 1], 1e-15, 'N');
  // the same numbers as the C++ rtn_axes gives for this tilted state
  const a = rtnAxes([4000, 5000, 2500], [-3, 2, 5]);
  nearVec(a.radial, [0.581914373962646, 0.727392967453308, 0.363696483726654], 1e-14, 'R vs C++');
  nearVec(a.inTrack, [-0.651169834216939, 0.148838819249586, 0.74419409624793], 1e-14, 'T vs C++');
  nearVec(a.normal, [0.487189396827913, -0.66988542063838, 0.5602678063521], 1e-14, 'N vs C++');
  const v = [0.3, -1.2, 2.5];
  nearVec(fromRtn(a, projectOnRtn(a, v)), v, 1e-13, 'round trip');
}

// --- the burn response: the same numbers as the C++ cw_displacement_km ---
{
  const n = 0.0011;
  nearVec(cwDisplacementKm(n, 1234.5, [0.1, 0.2, 0.3]), [0.375678383608582, -0.173249585055756, 0.266572802381836], 1e-12, 'cw 1');
  nearVec(cwDisplacementKm(n, 5000, [-0.4, 0.25, 0]), [0.388982948256843, -4.17952376818491, 0], 1e-12, 'cw 2');
  nearVec(cwDisplacementKm(n, 9000, [0, -0.3, 0.2]), [-1.03046790143201, 8.59913006593671, -0.0831883443227857], 1e-12, 'cw 3');
  // whole orbit: only in-track moves, by -3 tau dv
  const period = 2 * Math.PI / n;
  nearVec(cwDisplacementKm(n, period, [0, 0.3, 0]), [0, -3 * period * 0.3e-3, 0], 1e-9, 'whole orbit');
  nearVec(cwDisplacementKm(n, period, [0.4, 0, 0]), [0, 0, 0], 1e-9, 'radial comes back');
  // linear in the burn
  const a = cwDisplacementKm(n, 777, [0.1, 0.2, 0.3]), b = cwDisplacementKm(n, 777, [0.2, 0.4, 0.6]);
  nearVec(b, a.map((v) => 2 * v), 1e-12, 'linear');
}

// --- miss distance after a displacement ---
{
  // closing along +R: the plane is (T, N); a displacement along R changes nothing, one along T does
  assert.equal(inPlaneMiss([0, 0, 2], [7, 0, 0], [5, 0, 0]), 2);
  near(inPlaneMiss([0, 0, 2], [7, 0, 0], [0, -Math.sqrt(21), 0]), 5, 1e-12, 'in-track displacement');
  assert.equal(inPlaneMiss([0, 0, 2], [7, 0, 0], [0, 0, 0]), 2);
}

// --- decoding ---
{
  const row = (status) => [5400, status, -0.1, -0.32, 0.01, 0.335, 24.87, 29.87, 1e-6, 2440587.5, 0.25, 7000, 100, 200, -0.3, -0.95, 0.03];
  assert.equal(row(0).length, BURN_STRIDE);
  const options = parseBurnOptions(Float64Array.from([...row(0), ...row(3)]));
  assert.equal(options.length, 2);
  assert.equal(options[0].leadS, 5400);
  assert.equal(options[0].status, BURN_STATUS.OK);
  assert.deepEqual(options[0].dvRtnMS, [-0.1, -0.32, 0.01]);
  assert.equal(options[0].dvMS, 0.335);
  assert.equal(options[0].missAfterKm, 29.87);
  assert.equal(options[0].pcAfter, 1e-6);
  assert.equal(options[0].burnTimeMs, 0.25 * 86400000);
  assert.deepEqual(options[0].burnPositionKm, [7000, 100, 200]);
  assert.deepEqual(options[0].directionTeme, [-0.3, -0.95, 0.03]);
  assert.equal(options[1].status, BURN_STATUS.IN_THE_PAST);
  assert.equal(parseBurnOptions(new Float64Array(0)).length, 0);
  assert.equal(parseBurnOptions(new Float64Array(BURN_STRIDE - 1)).length, 0);
}

// --- words ---
assert.equal(burnDirectionLabel([0, -0.32, 0]), 'retrograde (slows down)');
assert.equal(burnDirectionLabel([0, 0.32, 0]), 'prograde (speeds up)');
assert.equal(burnDirectionLabel([-0.5, 0, 0]), 'radially inward');
assert.equal(burnDirectionLabel([0.5, 0, 0]), 'radially outward');
assert.equal(burnDirectionLabel([0, 0, -1]), 'out of plane (-N)');
assert.equal(burnDirectionLabel([-3.3, -0.68, -0.24]), 'mostly radially inward');
assert.equal(burnDirectionLabel([0, 0, 0]), 'none');
assert.ok(statusNote(BURN_STATUS.IN_THE_PAST).includes('before the screening started'));
assert.ok(statusNote(BURN_STATUS.UNREACHABLE).includes('cannot reach'));
assert.ok(statusNote(BURN_STATUS.NO_BURN_NEEDED).includes('no burn needed'));
assert.equal(statusNote(BURN_STATUS.OK), '');
assert.equal(formatLead(5580), '93 min');
assert.equal(formatLead(11160), '3.1 h');
assert.equal(formatLead(600), '10 min');

console.log('maneuver tests passed');