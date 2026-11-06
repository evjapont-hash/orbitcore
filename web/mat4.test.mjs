// Run with: node web/mat4.test.mjs
import assert from 'node:assert/strict';
import { identity, lookAt, multiply, perspective, rotateZ } from './mat4.js';

function transform(m, v) {
  const [x, y, z] = v;
  const out = [
    m[0] * x + m[4] * y + m[8] * z + m[12],
    m[1] * x + m[5] * y + m[9] * z + m[13],
    m[2] * x + m[6] * y + m[10] * z + m[14],
    m[3] * x + m[7] * y + m[11] * z + m[15],
  ];
  return out;
}
function near(actual, expected, tol = 1e-5) {
  assert.equal(actual.length, expected.length);
  actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) <= tol, `${actual} != ${expected}`));
}

// rotateZ(90 deg) sends +x to +y and +y to -x.
near(transform(rotateZ(Math.PI / 2), [1, 0, 0]).slice(0, 3), [0, 1, 0]);
near(transform(rotateZ(Math.PI / 2), [0, 1, 0]).slice(0, 3), [-1, 0, 0]);

// A camera at (0,0,5) looking at the origin sees the origin 5 units straight ahead (-z).
near(transform(lookAt([0, 0, 5], [0, 0, 0], [0, 1, 0]), [0, 0, 0]).slice(0, 3), [0, 0, -5]);

// Camera on +x looking at the origin with z up: a point above the origin appears "up" on screen.
near(transform(lookAt([5, 0, 0], [0, 0, 0], [0, 0, 1]), [0, 0, 1]).slice(0, 3), [0, 1, -5]);
// ...and +y (to the camera's right when looking down -x with z up) appears on screen-right.
near(transform(lookAt([5, 0, 0], [0, 0, 0], [0, 0, 1]), [0, 1, 0]).slice(0, 3), [1, 0, -5]);

// Perspective: near plane maps to NDC z = -1 and far plane to +1 (after dividing by w).
const p = perspective(Math.PI / 2, 1, 0.5, 50);
const nearPt = transform(p, [0, 0, -0.5]);
const farPt = transform(p, [0, 0, -50]);
near([nearPt[2] / nearPt[3]], [-1]);
near([farPt[2] / farPt[3]], [1], 1e-4);

// multiply(a, b) applies b first.
const a = rotateZ(Math.PI / 2);
const b = lookAt([0, 0, 5], [0, 0, 0], [0, 1, 0]);
near(transform(multiply(b, a), [1, 0, 0]).slice(0, 3), transform(b, transform(a, [1, 0, 0])).slice(0, 3));
near(transform(multiply(identity(), a), [1, 2, 3]).slice(0, 3), transform(a, [1, 2, 3]).slice(0, 3));

console.log('mat4 tests passed');