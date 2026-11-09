// Earth-surface geometry for the viewer: WGS-84 conversions, the camera ray used to place
// stations by clicking, and the visibility cone drawn above a ground station.
// Scene units: the WGS-84 equatorial radius is 1.

export const WGS84_F = 1 / 298.257223563;
export const WGS84_E2 = WGS84_F * (2 - WGS84_F);
const DEG = Math.PI / 180;

// Geodetic latitude/longitude (degrees) and height above the ellipsoid (in Earth radii)
// -> Earth-fixed position in scene units.
export function geodeticToScene(latDeg, lonDeg, h = 0) {
  const lat = latDeg * DEG, lon = lonDeg * DEG;
  const s = Math.sin(lat), c = Math.cos(lat);
  const n = 1 / Math.sqrt(1 - WGS84_E2 * s * s);
  return [(n + h) * c * Math.cos(lon), (n + h) * c * Math.sin(lon), (n * (1 - WGS84_E2) + h) * s];
}

// A point ON the ellipsoid surface (scene units, Earth-fixed) -> geodetic latitude/longitude, degrees.
export function sceneToGeodeticSurface(p) {
  const lon = Math.atan2(p[1], p[0]);
  const lat = Math.atan2(p[2], Math.hypot(p[0], p[1]) * (1 - WGS84_E2));
  return { latDeg: lat / DEG, lonDeg: lon / DEG };
}

// Local East, North, Up unit vectors at a geodetic latitude/longitude, expressed in Earth-fixed axes.
export function localAxes(latDeg, lonDeg) {
  const lat = latDeg * DEG, lon = lonDeg * DEG;
  const sp = Math.sin(lat), cp = Math.cos(lat), sl = Math.sin(lon), cl = Math.cos(lon);
  return {
    east: [-sl, cl, 0],
    north: [-sp * cl, -sp * sl, cp],
    up: [cp * cl, cp * sl, sp],
  };
}

// Direction (unit vector) of the camera ray through normalized device coordinates (ndcX, ndcY)
// for a camera at `eye` looking at the origin with +z up. fovy in radians, aspect = width / height.
export function screenRay(eye, fovy, aspect, ndcX, ndcY) {
  const len = Math.hypot(eye[0], eye[1], eye[2]);
  const f = [-eye[0] / len, -eye[1] / len, -eye[2] / len];            // forward
  const rl = Math.hypot(f[1], f[0]);
  const r = [f[1] / rl, -f[0] / rl, 0];                                // right = forward x up(0,0,1)
  const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]; // right x forward
  const t = Math.tan(fovy / 2);
  const d = [0, 1, 2].map((k) => f[k] + ndcX * aspect * t * r[k] + ndcY * t * u[k]);
  const dl = Math.hypot(d[0], d[1], d[2]);
  return [d[0] / dl, d[1] / dl, d[2] / dl];
}

// First intersection of the ray eye + t * dir (t > 0) with the WGS-84 surface (scene units), or null.
export function rayEllipsoid(eye, dir) {
  const k = 1 / (1 - WGS84_F);                  // stretch z so the ellipsoid becomes the unit sphere
  const e = [eye[0], eye[1], eye[2] * k], d = [dir[0], dir[1], dir[2] * k];
  const a = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  const b = 2 * (e[0] * d[0] + e[1] * d[1] + e[2] * d[2]);
  const c = e[0] * e[0] + e[1] * e[1] + e[2] * e[2] - 1;
  const disc = b * b - 4 * a * c;
  if (disc < 0) return null;
  const t = (-b - Math.sqrt(disc)) / (2 * a);
  if (t <= 0) return null;
  return [eye[0] + t * dir[0], eye[1] + t * dir[1], eye[2] + t * dir[2]];
}

// Rotation of a vector about +z by `angle` radians (counter-clockwise looking down from +z).
// angle = +GMST turns Earth-fixed vectors into inertial (TEME) ones; -GMST does the reverse.
export function rotateZVec(v, angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
}

// The visibility cone above a ground station as line segments (pairs of vertices), Earth-fixed
// scene units: `spokes` lines from the station to the rim, then the rim ring. The rim is where
// rays at the minimum elevation, in every azimuth, reach a sphere of radius `shellRadius`
// centred on the Earth (e.g. 1 + 1000 km / Earth radius). Anything above the horizon mask and
// below that shell lies inside the cone. Returns a Float32Array of 2 * spokes * 2 * 3 numbers.
export function coneLines(latDeg, lonDeg, h, minElDeg, shellRadius, spokes = 24) {
  const s = geodeticToScene(latDeg, lonDeg, h);
  const { east, north, up } = localAxes(latDeg, lonDeg);
  const el = minElDeg * DEG;
  const rim = [];
  for (let k = 0; k < spokes; k++) {
    const az = (2 * Math.PI * k) / spokes;
    const horizontal = [0, 1, 2].map((i) => Math.sin(az) * east[i] + Math.cos(az) * north[i]);
    const d = [0, 1, 2].map((i) => Math.cos(el) * horizontal[i] + Math.sin(el) * up[i]);
    const sd = s[0] * d[0] + s[1] * d[1] + s[2] * d[2];
    const ss = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
    const t = -sd + Math.sqrt(sd * sd - ss + shellRadius * shellRadius);
    rim.push([s[0] + t * d[0], s[1] + t * d[1], s[2] + t * d[2]]);
  }
  const out = [];
  for (let k = 0; k < spokes; k++) out.push(...s, ...rim[k]);
  for (let k = 0; k < spokes; k++) out.push(...rim[k], ...rim[(k + 1) % spokes]);
  return Float32Array.from(out);
}

// Converts GeoJSON lines and polygon outlines ([longitude, latitude] in degrees) into line
// segments (pairs of vertices) on the WGS-84 surface raised by `h` Earth radii, in Earth-fixed
// scene units. Accepts LineString, MultiLineString, Polygon, MultiPolygon, GeometryCollection,
// Feature and FeatureCollection; anything else (points, null geometry) is ignored, as are
// coordinates that are not finite numbers.
export function geoJsonToLineSegments(geojson, h = 0) {
  const out = [];
  const addLine = (coords) => {
    for (let i = 0; i + 1 < coords.length; i++) {
      const a = coords[i], b = coords[i + 1];
      if (!Number.isFinite(a[0]) || !Number.isFinite(a[1]) || !Number.isFinite(b[0]) || !Number.isFinite(b[1])) continue;
      out.push(...geodeticToScene(a[1], a[0], h), ...geodeticToScene(b[1], b[0], h));
    }
  };
  const visit = (g) => {
    if (!g) return;
    switch (g.type) {
      case 'FeatureCollection': g.features.forEach(visit); break;
      case 'Feature': visit(g.geometry); break;
      case 'GeometryCollection': g.geometries.forEach(visit); break;
      case 'LineString': addLine(g.coordinates); break;
      case 'MultiLineString': g.coordinates.forEach(addLine); break;
      case 'Polygon': g.coordinates.forEach(addLine); break;
      case 'MultiPolygon': g.coordinates.forEach((polygon) => polygon.forEach(addLine)); break;
      default: break;
    }
  };
  visit(geojson);
  return Float32Array.from(out);
}