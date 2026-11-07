// Run with: node web/picking.test.mjs
import assert from 'node:assert/strict';
import { lookAt, multiply, perspective } from './mat4.js';
import { occludedByUnitSphere, pickNearest } from './picking.js';

const WIDTH = 900, HEIGHT = 600;
const eye = [3, 0, 0]; // camera on +x looking at the Earth's centre, z up
const viewProj = multiply(perspective(0.8, WIDTH / HEIGHT, 0.05, 200), lookAt(eye, [0, 0, 0], [0, 0, 1]));

// Independent screen projection used to decide where to click (does not use pickNearest).
function project(p) {
  const clip = [0, 1, 2, 3].map((r) => viewProj[r] * p[0] + viewProj[4 + r] * p[1] + viewProj[8 + r] * p[2] + viewProj[12 + r]);
  return [(clip[0] / clip[3] * 0.5 + 0.5) * WIDTH, (1 - (clip[1] / clip[3] * 0.5 + 0.5)) * HEIGHT];
}
function pick(points, x, y, radius = 12) {
  return pickNearest({ positions: Float32Array.from(points.flat()), count: points.length, scale: 1,
                       viewProj, eye, width: WIDTH, height: HEIGHT, x, y, radius });
}

// --- occlusion by the Earth ---
assert.equal(occludedByUnitSphere(eye, [1.1, 0, 0]), false);   // near side of the Earth: visible
assert.equal(occludedByUnitSphere(eye, [-1.1, 0, 0]), true);   // far side: hidden by the Earth
assert.equal(occludedByUnitSphere(eye, [0, 2.5, 0]), false);   // off to the side, clear of the Earth

// --- picking ---
const front = [1.1, 0, 0], back = [-1.1, 0, 0], side = [1.1, 0.3, 0.2];
const [cx, cy] = project(front);
assert.ok(Math.abs(cx - WIDTH / 2) < 1 && Math.abs(cy - HEIGHT / 2) < 1, 'front object is at screen centre');

assert.equal(pick([front, back], cx, cy), 0);                  // front and back overlap on screen: front wins
assert.equal(pick([back], cx, cy), -1);                        // only the hidden one: nothing to pick
const [sx, sy] = project(side);
assert.equal(pick([front, side], sx + 3, sy - 2), 1);          // nearest to the click wins
assert.equal(pick([front, side], sx + 3, sy - 2, 2), -1);      // outside a tight radius: nothing
assert.equal(pick([front], 50, 50), -1);                       // click far from everything
assert.equal(pick([[NaN, 0, 0], front], cx, cy), 1);           // failed (NaN) objects are skipped
assert.equal(pick([[5, 0, 0]], WIDTH / 2, HEIGHT / 2), -1);    // behind the camera: skipped

console.log('picking tests passed');