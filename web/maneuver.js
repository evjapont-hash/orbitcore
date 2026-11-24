// Avoidance-burn helpers for the viewer. The C++ engine (src/maneuver.cpp) finds the smallest burn
// for each lead time; these functions decode its results, describe them in words, and re-derive the
// displacement with the same linear model (Hill's equations) so the page can show where the
// satellite would be at the closest approach if it had burned.
import { jdToUnixMs } from './screening.js';

export const BURN_STRIDE = 17; // numbers per option in Engine.planResults()
export const BURN_STATUS = { OK: 0, NO_BURN_NEEDED: 1, UNREACHABLE: 2, IN_THE_PAST: 3 };

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];

// Radial / in-track / cross-track unit vectors of an orbiting object (as in the C++ rtn_axes).
export function rtnAxes(position, velocity) {
  const radial = scale(position, 1 / Math.hypot(...position));
  const h = cross(position, velocity);
  const normal = scale(h, 1 / Math.hypot(...h));
  return { radial, inTrack: cross(normal, radial), normal };
}

export function projectOnRtn(axes, v) {
  return [dot(v, axes.radial), dot(v, axes.inTrack), dot(v, axes.normal)];
}

export function fromRtn(axes, rtn) {
  return [0, 1, 2].map((k) => rtn[0] * axes.radial[k] + rtn[1] * axes.inTrack[k] + rtn[2] * axes.normal[k]);
}

// Displacement (km, radial / in-track / cross-track) of an object tauS seconds after a burn dvMS
// (m/s, same axes at the burn time), relative to where it would have been; circular reference
// orbit with mean motion n (rad/s). Hill's equations.
export function cwDisplacementKm(n, tauS, dvMS) {
  const s = Math.sin(n * tauS), c = Math.cos(n * tauS);
  const [dx, dy, dz] = dvMS.map((v) => v * 1e-3); // km/s
  return [
    (s / n) * dx + (2 * (1 - c) / n) * dy,
    (2 * (c - 1) / n) * dx + ((4 * s - 3 * n * tauS) / n) * dy,
    (s / n) * dz,
  ];
}

// Miss distance after the primary is displaced: the miss vector (secondary minus primary) changes by
// minus the displacement, and only the part perpendicular to the relative velocity counts.
export function inPlaneMiss(missRtnKm, relativeVelocityRtn, displacementRtnKm) {
  const moved = [0, 1, 2].map((k) => missRtnKm[k] - displacementRtnKm[k]);
  const vhat = scale(relativeVelocityRtn, 1 / Math.hypot(...relativeVelocityRtn));
  const along = dot(moved, vhat);
  return Math.hypot(...[0, 1, 2].map((k) => moved[k] - along * vhat[k]));
}

// Decodes Engine.planResults() (BURN_STRIDE numbers per option, see engine.hpp).
export function parseBurnOptions(packed) {
  const count = Math.floor(packed.length / BURN_STRIDE);
  const options = [];
  for (let i = 0; i < count; i++) {
    const o = i * BURN_STRIDE;
    const at = (from, n) => Array.from({ length: n }, (_, k) => packed[o + from + k]);
    options.push({
      leadS: packed[o],
      status: packed[o + 1],
      dvRtnMS: at(2, 3),
      dvMS: packed[o + 5],
      missBeforeKm: packed[o + 6],
      missAfterKm: packed[o + 7],
      pcAfter: packed[o + 8],
      burnTimeMs: jdToUnixMs(packed[o + 9], packed[o + 10]),
      burnPositionKm: at(11, 3),
      directionTeme: at(14, 3),
    });
  }
  return options;
}

// The burn in words: which way it pushes the satellite.
export function burnDirectionLabel(dvRtn) {
  const total = Math.hypot(...dvRtn);
  if (!(total > 0)) return 'none';
  const parts = Array.from(dvRtn, Math.abs);
  const main = parts.indexOf(Math.max(...parts));
  const names = [['radially outward', 'radially inward'], ['prograde (speeds up)', 'retrograde (slows down)'],
                 ['out of plane (+N)', 'out of plane (-N)']];
  const label = names[main][dvRtn[main] >= 0 ? 0 : 1];
  return parts[main] / total > 0.99 ? label : `mostly ${label}`;
}

export function statusNote(status) {
  switch (status) {
    case BURN_STATUS.NO_BURN_NEEDED: return 'no burn needed: the miss is already at least the target';
    case BURN_STATUS.UNREACHABLE: return 'this lead time cannot reach the target';
    case BURN_STATUS.IN_THE_PAST: return 'would have to happen before the screening started';
    default: return '';
  }
}

// "93 min" or "3.1 h" for a lead time given in seconds.
export function formatLead(leadS) {
  const minutes = leadS / 60;
  return minutes < 180 ? `${Math.round(minutes)} min` : `${(minutes / 60).toFixed(1)} h`;
}