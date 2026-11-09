// Run with: node web/coastline.test.mjs
import assert from 'node:assert/strict';
import { geodeticToScene, geoJsonToLineSegments } from './geometry.js';

function near(actual, expected, tol = 1e-6) {
  assert.equal(actual.length, expected.length);
  actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) <= tol, `${actual} != ${expected}`));
}
const vertex = (arr, k) => Array.from(arr.slice(3 * k, 3 * k + 3));

// A LineString of 3 points is 2 segments = 4 vertices; GeoJSON order is [longitude, latitude].
{
  const seg = geoJsonToLineSegments({ type: 'LineString', coordinates: [[10, 50], [20, 50], [20, 60]] });
  assert.equal(seg.length, 4 * 3);
  near(vertex(seg, 0), geodeticToScene(50, 10));       // lat = 50, lon = 10 (not the other way round)
  near(vertex(seg, 1), geodeticToScene(50, 20));
  near(vertex(seg, 2), geodeticToScene(50, 20));       // the second segment starts where the first ended
  near(vertex(seg, 3), geodeticToScene(60, 20));
}

// MultiLineString, Polygon (all rings), MultiPolygon, and the wrappers.
assert.equal(geoJsonToLineSegments({ type: 'MultiLineString', coordinates: [[[0, 0], [1, 1]], [[5, 5], [6, 6], [7, 7]]] }).length, 3 * 2 * 3);
assert.equal(geoJsonToLineSegments({ type: 'Polygon', coordinates: [[[0, 0], [1, 0], [1, 1], [0, 0]], [[0.2, 0.1], [0.4, 0.1], [0.2, 0.3], [0.2, 0.1]]] }).length, 6 * 2 * 3);
assert.equal(geoJsonToLineSegments({ type: 'MultiPolygon', coordinates: [[[[0, 0], [1, 0], [1, 1]]], [[[5, 5], [6, 5], [6, 6]]]] }).length, 4 * 2 * 3);
{
  const collection = {
    type: 'FeatureCollection',
    features: [
      { type: 'Feature', geometry: { type: 'LineString', coordinates: [[0, 0], [1, 0]] } },
      { type: 'Feature', geometry: { type: 'Point', coordinates: [3, 3] } },     // ignored
      { type: 'Feature', geometry: null },                                         // ignored
      { type: 'Feature', geometry: { type: 'GeometryCollection', geometries: [{ type: 'LineString', coordinates: [[2, 2], [3, 3]] }] } },
    ],
  };
  assert.equal(geoJsonToLineSegments(collection).length, 2 * 2 * 3);
}

// Bad coordinates are skipped, not turned into NaN vertices.
{
  const seg = geoJsonToLineSegments({ type: 'LineString', coordinates: [[0, 0], [NaN, 5], [2, 2], [3, 3]] });
  assert.equal(seg.length, 1 * 2 * 3);                                              // only the (2,2)-(3,3) segment survives
  assert.ok(Array.from(seg).every(Number.isFinite));
}

// A height offset lifts the lines off the surface.
{
  const flat = geoJsonToLineSegments({ type: 'LineString', coordinates: [[10, 50], [20, 50]] }, 0);
  const raised = geoJsonToLineSegments({ type: 'LineString', coordinates: [[10, 50], [20, 50]] }, 0.01);
  const len = (v) => Math.hypot(v[0], v[1], v[2]);
  assert.ok(len(vertex(raised, 0)) > len(vertex(flat, 0)) + 0.009);
}

// Empty / unknown input gives an empty array.
assert.equal(geoJsonToLineSegments({ type: 'FeatureCollection', features: [] }).length, 0);
assert.equal(geoJsonToLineSegments(null).length, 0);

console.log('coastline tests passed');