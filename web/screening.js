// Helpers for the conjunction screening panel: decoding the engine's packed results and formatting.

export const ENCOUNTER_STRIDE = 23; // numbers per encounter in Engine.screenResults()
const JD_UNIX_EPOCH = 2440587.5;
const MS_PER_DAY = 86400000;

// Julian date given as day + fraction -> milliseconds since the Unix epoch (UTC).
export function jdToUnixMs(day, frac) {
  return ((day - JD_UNIX_EPOCH) + frac) * MS_PER_DAY;
}

// Decodes Engine.screenResults() into objects. Layout (see engine.hpp): secondary index, time of
// closest approach (Julian date day, fraction), miss km, relative speed km/s, miss in the primary's
// radial / in-track / cross-track axes, Pc_max, sigma*, Pc_ceiling, then both objects' TEME
// position (km) and velocity (km/s) at that moment.
export function parseEncounters(packed) {
  const count = Math.floor(packed.length / ENCOUNTER_STRIDE);
  const events = [];
  for (let i = 0; i < count; i++) {
    const o = i * ENCOUNTER_STRIDE;
    const at = (from, n) => Array.from({ length: n }, (_, k) => packed[o + from + k]);
    events.push({
      secondary: packed[o],
      tcaMs: jdToUnixMs(packed[o + 1], packed[o + 2]),
      missKm: packed[o + 3],
      relativeSpeedKmS: packed[o + 4],
      rtnKm: at(5, 3),
      pcMax: packed[o + 8],
      sigmaKm: packed[o + 9],
      pcCeiling: packed[o + 10],
      primaryPositionKm: at(11, 3),
      primaryVelocityKmS: at(14, 3),
      secondaryPositionKm: at(17, 3),
      secondaryVelocityKmS: at(20, 3),
    });
  }
  return events;
}

// How many objects to screen in the next chunk so that a chunk takes about targetMs: grow or shrink
// toward the target, by at most a factor of two per step, within [min, max].
export function nextChunkSize(current, elapsedMs, targetMs = 16, min = 5, max = 5000) {
  const wanted = elapsedMs > 0 ? (current * targetMs) / elapsedMs : current * 2;
  const limited = Math.max(current / 2, Math.min(current * 2, wanted));
  return Math.round(Math.max(min, Math.min(max, limited)));
}

// "T-00:12" before the closest approach, "T+01:05" after it, "T-1:02:05" for an hour or more.
export function formatOffset(seconds) {
  const sign = seconds < 0 ? '-' : '+';
  const whole = Math.round(Math.abs(seconds));
  const h = Math.floor(whole / 3600), m = Math.floor((whole % 3600) / 60), s = whole % 60;
  const mm = String(m).padStart(2, '0'), ss = String(s).padStart(2, '0');
  return h > 0 ? `T${sign}${h}:${mm}:${ss}` : `T${sign}${mm}:${ss}`;
}

// A probability for a table: "0.25", "4.3e-6", or "0".
export function formatPc(p) {
  if (!(p > 0)) return '0';
  if (p >= 0.1) return p.toFixed(2);
  return p.toExponential(1);
}

// "2026-11-13 23:56:27" (UTC) from milliseconds since the Unix epoch.
export function formatUtc(ms) {
  return new Date(ms).toISOString().slice(0, 19).replace('T', ' ');
}

export function distanceKm(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}