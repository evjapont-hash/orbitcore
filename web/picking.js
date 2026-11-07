// CPU picking: find the object whose projected position is nearest to a click, ignoring
// objects hidden behind the Earth (a unit sphere in scene units).

// True if the straight line from `eye` to `p` crosses the unit sphere before reaching `p`.
export function occludedByUnitSphere(eye, p) {
  const dx = p[0] - eye[0], dy = p[1] - eye[1], dz = p[2] - eye[2];
  const a = dx * dx + dy * dy + dz * dz;
  const b = 2 * (eye[0] * dx + eye[1] * dy + eye[2] * dz);
  const c = eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2] - 1;
  const disc = b * b - 4 * a * c;
  if (disc < 0) return false;
  const t = (-b - Math.sqrt(disc)) / (2 * a);
  return t > 0 && t < 1;
}

// Returns the index of the nearest visible object within `radius` pixels of (x, y), or -1.
//   positions: x,y,z per object (any unit); multiplied by `scale` to get scene units (Earth radii)
//   viewProj : column-major 4x4 view-projection matrix
//   eye      : camera position in scene units
//   width, height, x, y, radius: in CSS pixels, with (x, y) measured from the canvas's top-left
export function pickNearest({ positions, count, scale, viewProj, eye, width, height, x, y, radius }) {
  const m = viewProj;
  let best = -1;
  let bestDist = radius;
  for (let i = 0; i < count; i++) {
    const px = positions[3 * i] * scale;
    const py = positions[3 * i + 1] * scale;
    const pz = positions[3 * i + 2] * scale;
    if (!Number.isFinite(px) || !Number.isFinite(py) || !Number.isFinite(pz)) continue;
    const w = m[3] * px + m[7] * py + m[11] * pz + m[15];
    if (w <= 0) continue; // behind the camera
    const sx = (((m[0] * px + m[4] * py + m[8] * pz + m[12]) / w) * 0.5 + 0.5) * width;
    const sy = (1 - (((m[1] * px + m[5] * py + m[9] * pz + m[13]) / w) * 0.5 + 0.5)) * height;
    const d = Math.hypot(sx - x, sy - y);
    if (d < bestDist && !occludedByUnitSphere(eye, [px, py, pz])) {
      best = i;
      bestDist = d;
    }
  }
  return best;
}