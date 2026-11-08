// Run with: node web/geometry.test.mjs
import assert from 'node:assert/strict';
import { lookAt, multiply, perspective } from './mat4.js';
import {
  WGS84_F, coneLines, geodeticToScene, localAxes, rayEllipsoid, rotateZVec,
  sceneToGeodeticSurface, screenRay,
} from './geometry.js';

function near(actual, expected, tol = 1e-9) {
  assert.equal(actual.length, expected.length);
  actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) <= tol, `${actual} != ${expected}`));
}
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// --- WGS-84 conversions ---
near(geodeticToScene(0, 0), [1, 0, 0]);
near(geodeticToScene(0, 90), [0, 1, 0]);
near(geodeticToScene(90, 0), [0, 0, 1 - WGS84_F]);   // polar radius = b / a
for (const lat of [-89, -45, -0.5, 0, 33.3, 39.1, 89]) {
  for (const lon of [-179, -84.5, 0, 120, 180]) {
    const back = sceneToGeodeticSurface(geodeticToScene(lat, lon, 0));
    assert.ok(Math.abs(back.latDeg - lat) < 1e-9, `lat ${lat}`);
    assert.ok(Math.abs(Math.cos((back.lonDeg - lon) * Math.PI / 180) - 1) < 1e-12, `lon ${lon}`);
  }
}

// --- local axes ---
let ax = localAxes(0, 0);
near(ax.east, [0, 1, 0]); near(ax.north, [0, 0, 1]); near(ax.up, [1, 0, 0]);
ax = localAxes(39.1, -84.5);
assert.ok(Math.abs(norm(ax.east) - 1) < 1e-12 && Math.abs(norm(ax.north) - 1) < 1e-12 && Math.abs(norm(ax.up) - 1) < 1e-12);
assert.ok(Math.abs(dot(ax.east, ax.north)) < 1e-12 && Math.abs(dot(ax.east, ax.up)) < 1e-12 && Math.abs(dot(ax.north, ax.up)) < 1e-12);
near(cross(ax.east, ax.north), ax.up, 1e-12);                       // right-handed E, N, U

// --- rotation ---
near(rotateZVec([1, 0, 0], Math.PI / 2), [0, 1, 0], 1e-12);
near(rotateZVec(rotateZVec([1, 2, 3], 0.7), -0.7), [1, 2, 3], 1e-12);

// --- camera ray: must agree with the projection used for drawing ---
const eye = [3, 0, 0], fovy = 0.8, aspect = 1.5;
near(screenRay(eye, fovy, aspect, 0, 0), [-1, 0, 0], 1e-12);        // centre of the screen looks at the Earth
const viewProj = multiply(perspective(fovy, aspect, 0.05, 200), lookAt(eye, [0, 0, 0], [0, 0, 1]));
for (const p of [[1.1, 0, 0], [1.1, 0.3, 0.2], [0.5, -0.9, 0.4], [-0.2, 0.7, -0.6]]) {
  const clip = [0, 1, 2, 3].map((r) => viewProj[r] * p[0] + viewProj[4 + r] * p[1] + viewProj[8 + r] * p[2] + viewProj[12 + r]);
  const ray = screenRay(eye, fovy, aspect, clip[0] / clip[3], clip[1] / clip[3]);
  const toPoint = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
  near(cross(ray, toPoint).map((v) => v / norm(toPoint)), [0, 0, 0], 1e-6);   // ray passes through p
  assert.ok(dot(ray, toPoint) > 0);
}
// a camera that is not on an axis
const eye2 = [1.5, -2.0, 1.0];
const vp2 = multiply(perspective(fovy, aspect, 0.05, 200), lookAt(eye2, [0, 0, 0], [0, 0, 1]));
{
  const p = [0.3, 0.2, 0.1];
  const clip = [0, 1, 2, 3].map((r) => vp2[r] * p[0] + vp2[4 + r] * p[1] + vp2[8 + r] * p[2] + vp2[12 + r]);
  const ray = screenRay(eye2, fovy, aspect, clip[0] / clip[3], clip[1] / clip[3]);
  const toPoint = [p[0] - eye2[0], p[1] - eye2[1], p[2] - eye2[2]];
  near(cross(ray, toPoint).map((v) => v / norm(toPoint)), [0, 0, 0], 1e-6);
}

// --- ray vs the WGS-84 surface ---
near(rayEllipsoid([3, 0, 0], [-1, 0, 0]), [1, 0, 0], 1e-12);
near(rayEllipsoid([0, 0, 3], [0, 0, -1]), [0, 0, 1 - WGS84_F], 1e-12);
assert.equal(rayEllipsoid([3, 0, 0], [-0.9, 0.5, 0]), null);       // passes beside the Earth
assert.equal(rayEllipsoid([3, 0, 0], [1, 0, 0]), null);            // pointing away
{
  const hit = rayEllipsoid([3, 0.4, 1.2], screenRay([3, 0.4, 1.2], fovy, aspect, 0.1, -0.05));
  assert.ok(hit !== null);
  const g = sceneToGeodeticSurface(hit);
  near(geodeticToScene(g.latDeg, g.lonDeg, 0), hit, 1e-9);          // the hit really is on the surface
}

// --- visibility cone ---
{
  const lat = 39.1, lon = -84.5, h = 0.0, minEl = 10, shell = 1 + 1000 / 6378.137, spokes = 24;
  const lines = coneLines(lat, lon, h, minEl, shell, spokes);
  assert.equal(lines.length, 2 * spokes * 2 * 3);
  const s = geodeticToScene(lat, lon, h);
  const { up } = localAxes(lat, lon);
  for (let k = 0; k < spokes; k++) {
    near([lines[6 * k], lines[6 * k + 1], lines[6 * k + 2]], s, 1e-6);                  // every spoke starts at the station
    const rim = [lines[6 * k + 3], lines[6 * k + 4], lines[6 * k + 5]];
    assert.ok(Math.abs(norm(rim) - shell) < 1e-6, 'rim lies on the shell');
    const d = [rim[0] - s[0], rim[1] - s[1], rim[2] - s[2]];
    const elevation = Math.asin(dot(d, up) / norm(d)) * 180 / Math.PI;
    assert.ok(Math.abs(elevation - minEl) < 1e-4, `rim elevation ${elevation}`);      // rays are at the minimum elevation
  }
  // the ring closes: last ring segment ends at the first rim point
  const n = lines.length;
  near([lines[n - 3], lines[n - 2], lines[n - 1]], [lines[3], lines[4], lines[5]], 1e-6);
  // a lower mask opens the cone wider: rim points move farther from the station's zenith
  const wide = coneLines(lat, lon, h, 0, shell, spokes);
  const rimDist = (arr) => norm([arr[3] - s[0], arr[4] - s[1], arr[5] - s[2]]);
  assert.ok(rimDist(wide) > rimDist(lines));
}

console.log('geometry tests passed');