// Run with: node web/screening.test.mjs
import assert from 'node:assert/strict';
import {
  ENCOUNTER_STRIDE, distanceKm, formatOffset, formatPc, formatUtc, jdToUnixMs, nextChunkSize, parseEncounters,
} from './screening.js';

// --- time conversion ---
assert.equal(jdToUnixMs(2440587.5, 0), 0);                                    // the Unix epoch
assert.equal(jdToUnixMs(2440588.5, 0), 86400000);                             // one day later
assert.equal(jdToUnixMs(2440587.5, 1.5), 1.5 * 86400000);                     // the fraction may exceed 1
assert.equal(formatUtc(0), '1970-01-01 00:00:00');
assert.equal(formatUtc(jdToUnixMs(2461000.5, 0.5)), '2025-11-21 12:00:00');   // JD 2461000.5 = 2025-11-21 00:00 UTC
assert.equal(formatUtc(1731542187000), '2024-11-13 23:56:27');

// --- decoding the packed results ---
{
  const row = (secondary, tcaDay, tcaFrac) => [
    secondary, tcaDay, tcaFrac, 12.5, 13.25,        // miss, relative speed
    1, 2, 3,                                          // R, T, N
    4e-7, 7.5, 2e-3,                                  // Pc_max, sigma*, Pc_ceil
    7000, 100, 200, 1, 2, 3,                          // primary position, velocity
    7005, 110, 210, 4, 5, 6,                          // secondary position, velocity
  ];
  assert.equal(row(1, 0, 0).length, ENCOUNTER_STRIDE);
  const packed = Float64Array.from([...row(17, 2440587.5, 0.25), ...row(42, 2440588.5, 0.5)]);
  const events = parseEncounters(packed);
  assert.equal(events.length, 2);
  assert.equal(events[0].secondary, 17);
  assert.equal(events[0].tcaMs, 0.25 * 86400000);
  assert.equal(events[1].tcaMs, 1.5 * 86400000);
  assert.equal(events[0].missKm, 12.5);
  assert.equal(events[0].relativeSpeedKmS, 13.25);
  assert.deepEqual(events[0].rtnKm, [1, 2, 3]);
  assert.equal(events[0].pcMax, 4e-7);
  assert.equal(events[0].sigmaKm, 7.5);
  assert.equal(events[0].pcCeiling, 2e-3);
  assert.deepEqual(events[0].primaryPositionKm, [7000, 100, 200]);
  assert.deepEqual(events[0].primaryVelocityKmS, [1, 2, 3]);
  assert.deepEqual(events[0].secondaryPositionKm, [7005, 110, 210]);
  assert.deepEqual(events[0].secondaryVelocityKmS, [4, 5, 6]);
  assert.equal(parseEncounters(new Float64Array(0)).length, 0);
  assert.equal(parseEncounters(new Float64Array(ENCOUNTER_STRIDE - 1)).length, 0);   // an incomplete row is ignored
}

// --- chunk sizing ---
assert.equal(nextChunkSize(100, 16), 100);          // exactly on target: unchanged
assert.equal(nextChunkSize(100, 8), 200);           // twice as fast as needed: doubles
assert.equal(nextChunkSize(100, 4), 200);           // ...but never more than doubles
assert.equal(nextChunkSize(100, 32), 50);           // too slow: halves
assert.equal(nextChunkSize(100, 320), 50);          // ...but never more than halves
assert.equal(nextChunkSize(100, 0), 200);           // unmeasurably fast: doubles
assert.equal(nextChunkSize(6, 100), 5);             // never below the minimum
assert.equal(nextChunkSize(4000, 1), 5000);         // never above the maximum

// --- formatting ---
assert.equal(formatOffset(-12), 'T-00:12');
assert.equal(formatOffset(65), 'T+01:05');
assert.equal(formatOffset(0), 'T+00:00');
assert.equal(formatOffset(-3725), 'T-1:02:05');
assert.equal(formatOffset(-12.4), 'T-00:12');
assert.equal(formatPc(7.34e-7), '7.3e-7');
assert.equal(formatPc(0.25), '0.25');
assert.equal(formatPc(1), '1.00');
assert.equal(formatPc(0), '0');
assert.equal(formatPc(NaN), '0');
assert.equal(distanceKm([0, 0, 0], [3, 4, 12]), 13);

console.log('screening tests passed');