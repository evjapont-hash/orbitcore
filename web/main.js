// LEO situational awareness viewer.
//
// orbitcore (C++ compiled to WebAssembly) propagates every object with SGP4 in double precision
// and hands back float32 TEME positions. This file uploads them to the GPU each frame and draws
// every object as a point sprite in a single draw call. Scene units are Earth radii. The frame
// is TEME (inertial), so the Earth is rotated by GMST while satellites are drawn as propagated.
import { lookAt, multiply, perspective, rotateZ } from './mat4.js';
import { pickNearest } from './picking.js';
import { coneLines, geoJsonToLineSegments, geodeticToScene, rayEllipsoid, rotateZVec, sceneToGeodeticSurface, screenRay } from './geometry.js';

const EARTH_RADIUS_KM = 6378.137; // WGS-84 equatorial radius; only used to scale the scene
const JD_UNIX_EPOCH = 2440587.5; // Julian date of 1970-01-01 00:00 UTC
const MS_PER_DAY = 86400000;

const $ = (id) => document.getElementById(id);
const canvas = $('gl');
const gl = canvas.getContext('webgl2', { antialias: true });

function showMessage(text) {
  const el = $('message');
  el.textContent = text;
  el.style.display = 'grid';
}
function hideMessage() {
  $('message').style.display = 'none';
}

// ---------------------------------------------------------------------------------------------
// Shaders
// ---------------------------------------------------------------------------------------------
const POINTS_VERT = `#version 300 es
layout(location = 0) in vec3 aPos;    // TEME position, km
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aFlag;  // non-zero: in view of a ground station
uniform mat4 uViewProj;
uniform float uScale;                 // km -> Earth radii
uniform float uPointSize;
out vec4 vColor;
void main() {
  if (isnan(aPos.x) || isnan(aPos.y) || isnan(aPos.z)) {  // object failed to propagate: hide it
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    gl_PointSize = 0.0;
    vColor = vec4(0.0);
    return;
  }
  bool inView = aFlag > 0.5;
  gl_Position = uViewProj * vec4(aPos * uScale, 1.0);
  gl_PointSize = inView ? uPointSize * 1.8 : uPointSize;
  vColor = inView ? vec4(0.45, 1.0, 0.55, 1.0) : aColor;
}`;

const POINTS_FRAG = `#version 300 es
precision mediump float;
in vec4 vColor;
uniform float uRing;               // 1.0 draws a hollow ring (selection marker)
out vec4 outColor;
void main() {
  vec2 c = gl_PointCoord - vec2(0.5);
  float r2 = dot(c, c);
  if (r2 > 0.25) discard;          // round points
  if (uRing > 0.5 && r2 < 0.16) discard;
  outColor = vColor;
}`;

const MESH_VERT = `#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMvp;
void main() { gl_Position = uMvp * vec4(aPos, 1.0); }`;

const MESH_FRAG = `#version 300 es
precision mediump float;
uniform vec4 uColor;
out vec4 outColor;
void main() { outColor = uColor; }`;

function compile(type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error('shader compile failed: ' + gl.getShaderInfoLog(shader));
  }
  return shader;
}

function makeProgram(vertSource, fragSource) {
  const program = gl.createProgram();
  gl.attachShader(program, compile(gl.VERTEX_SHADER, vertSource));
  gl.attachShader(program, compile(gl.FRAGMENT_SHADER, fragSource));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error('program link failed: ' + gl.getProgramInfoLog(program));
  }
  return program;
}

// ---------------------------------------------------------------------------------------------
// Geometry (Earth-fixed, in Earth radii)
// ---------------------------------------------------------------------------------------------
// Latitude/longitude mesh of the WGS-84 ellipsoid, offset by `h` Earth radii (negative = below the surface).
function ellipsoidMesh(latSegs, lonSegs, h) {
  const verts = [];
  for (let i = 0; i <= latSegs; i++) {
    const latDeg = -90 + (180 * i) / latSegs;
    for (let j = 0; j <= lonSegs; j++) {
      verts.push(...geodeticToScene(latDeg, (360 * j) / lonSegs, h));
    }
  }
  const indices = [];
  const stride = lonSegs + 1;
  for (let i = 0; i < latSegs; i++) {
    for (let j = 0; j < lonSegs; j++) {
      const a = i * stride + j, b = a + 1, c = a + stride, d = c + 1;
      indices.push(a, c, b, b, c, d);
    }
  }
  return { vertices: new Float32Array(verts), indices: new Uint16Array(indices) };
}

// Geodetic graticule (latitude circles and meridians) on the ellipsoid, as line segments.
function graticule(latsDeg, lonsDeg) {
  const v = [];
  const STEP = 5; // degrees per segment
  for (const lat of latsDeg) {
    for (let lon = -180; lon < 180; lon += STEP) {
      v.push(...geodeticToScene(lat, lon), ...geodeticToScene(lat, lon + STEP));
    }
  }
  for (const lon of lonsDeg) {
    for (let lat = -90; lat < 90; lat += STEP) {
      v.push(...geodeticToScene(lat, lon), ...geodeticToScene(lat + STEP, lon));
    }
  }
  return new Float32Array(v);
}

function staticBuffer(target, data) {
  const buffer = gl.createBuffer();
  gl.bindBuffer(target, buffer);
  gl.bufferData(target, data, gl.STATIC_DRAW);
  return buffer;
}

// A vertex array holding tightly packed xyz positions (and optional indices).
function meshObject(vertices, indices) {
  const vao = gl.createVertexArray();
  gl.bindVertexArray(vao);
  staticBuffer(gl.ARRAY_BUFFER, vertices);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  if (indices) staticBuffer(gl.ELEMENT_ARRAY_BUFFER, indices);
  gl.bindVertexArray(null);
  return { vao, count: indices ? indices.length : vertices.length / 3 };
}

// ---------------------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------------------
async function main() {
  if (!gl) {
    showMessage('This viewer needs WebGL2, which this browser does not provide.');
    return;
  }

  showMessage('Loading orbitcore (WebAssembly)...');
  const oc = await createOrbitcore();
  const engine = new oc.Engine();

  showMessage('Loading catalog...');
  const response = await fetch('catalog.tle');
  if (!response.ok) throw new Error(`could not load catalog.tle (HTTP ${response.status})`);
  const fileDate = response.headers.get('last-modified');

  // Provenance written by the deploy workflow (absent when running locally).
  let buildInfo = null;
  try {
    const info = await fetch('build-info.json');
    if (info.ok) buildInfo = await info.json();
  } catch (_) { /* running locally */ }

    // Coastlines (Natural Earth, public domain). Optional: the viewer works without them.
  let coastline = null;
  try {
    const res = await fetch('data/ne_110m_coastline.geojson');
    if (res.ok) coastline = geoJsonToLineSegments(await res.json(), 0.0008);
  } catch (_) { /* no coastline data */ }

  const text = await response.text();
  const count = engine.loadTle(text);
  const skippedDeepSpace = engine.skippedDeepSpace();
  if (count === 0) throw new Error('catalog.tle contained no usable objects');

  // Classify by name (CelesTrak marks debris "DEB" and rocket bodies "R/B") and pick colors.
  const colors = new Uint8Array(count * 4);
  const kind = new Uint8Array(count); // 0 active, 1 debris, 2 rocket body
  const KIND_LABELS = ['Active satellite', 'Debris', 'Rocket body'];
  const tally = { active: 0, debris: 0, rocket: 0 };
  const COLOR_ACTIVE = [80, 200, 255, 255];
  const COLOR_DEBRIS = [255, 150, 60, 255];
  const COLOR_ROCKET = [170, 170, 185, 255];
  for (let i = 0; i < count; i++) {
    const name = engine.name(i);
    let color = COLOR_ACTIVE;
    if (name.endsWith(' DEB')) { color = COLOR_DEBRIS; kind[i] = 1; tally.debris++; }
    else if (name.endsWith(' R/B')) { color = COLOR_ROCKET; kind[i] = 2; tally.rocket++; }
    else { tally.active++; }
    colors.set(color, i * 4);
  }

  // GPU objects
  const meshProgram = makeProgram(MESH_VERT, MESH_FRAG);
  const meshMvp = gl.getUniformLocation(meshProgram, 'uMvp');
  const meshColor = gl.getUniformLocation(meshProgram, 'uColor');
  const pointsProgram = makeProgram(POINTS_VERT, POINTS_FRAG);
  const pointsViewProj = gl.getUniformLocation(pointsProgram, 'uViewProj');
  const pointsScale = gl.getUniformLocation(pointsProgram, 'uScale');
  const pointsSize = gl.getUniformLocation(pointsProgram, 'uPointSize');
  const pointsRing = gl.getUniformLocation(pointsProgram, 'uRing');

  const sphere = ellipsoidMesh(36, 72, -0.003);
  const earth = meshObject(sphere.vertices, sphere.indices);
  const minorLats = [-75, -60, -45, -30, -15, 15, 30, 45, 60, 75];
  const minorLons = [];
  for (let lon = -165; lon <= 165; lon += 15) if (lon !== 0) minorLons.push(lon);
  const minorGrid = meshObject(graticule(minorLats, minorLons));
  const majorGrid = meshObject(graticule([0], [0, 180])); // equator + prime/anti meridian
  const coastGrid = coastline ? meshObject(coastline) : null;

  const positionBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, count * 3 * 4, gl.DYNAMIC_DRAW);
  const pointsVao = gl.createVertexArray();
  gl.bindVertexArray(pointsVao);
  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  staticBuffer(gl.ARRAY_BUFFER, colors);
  gl.enableVertexAttribArray(1);
  gl.vertexAttribPointer(1, 4, gl.UNSIGNED_BYTE, true, 0, 0);
  const flagsBuffer = gl.createBuffer(); // per-object "in view of a ground station" flags
  gl.bindBuffer(gl.ARRAY_BUFFER, flagsBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, count, gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(2);
  gl.vertexAttribPointer(2, 1, gl.UNSIGNED_BYTE, false, 0, 0);
  gl.bindVertexArray(null);

  // Selection marker: one point (a hollow ring) drawn over the selected object
  const markerPosBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, markerPosBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, 3 * 4, gl.DYNAMIC_DRAW);
  const markerVao = gl.createVertexArray();
  gl.bindVertexArray(markerVao);
  gl.bindBuffer(gl.ARRAY_BUFFER, markerPosBuffer);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  staticBuffer(gl.ARRAY_BUFFER, new Uint8Array([255, 255, 255, 255]));
  gl.enableVertexAttribArray(1);
  gl.vertexAttribPointer(1, 4, gl.UNSIGNED_BYTE, true, 0, 0);
  gl.bindVertexArray(null);

  // Orbit track of the selected object: a line strip, rebuilt every frame (inertial TEME frame)
  const TRACK_POINTS = 180;
  const trackScaled = new Float32Array(TRACK_POINTS * 3);
  const trackBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, trackBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, trackScaled.byteLength, gl.DYNAMIC_DRAW);
  const trackVao = gl.createVertexArray();
  gl.bindVertexArray(trackVao);
  gl.bindBuffer(gl.ARRAY_BUFFER, trackBuffer);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  gl.bindVertexArray(null);

  // Ground stations: markers (points, TEME km) and visibility cones (lines, Earth-fixed)
  const MAX_STATIONS = 8; // the engine keeps one visibility bit per station
  const CONE_SPOKES = 24;
  const VERTS_PER_CONE = CONE_SPOKES * 4; // spokes + rim ring, two vertices per segment
  const coneData = new Float32Array(MAX_STATIONS * VERTS_PER_CONE * 3);
  const coneBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, coneBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, coneData.byteLength, gl.DYNAMIC_DRAW);
  const coneVao = gl.createVertexArray();
  gl.bindVertexArray(coneVao);
  gl.bindBuffer(gl.ARRAY_BUFFER, coneBuffer);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  gl.bindVertexArray(null);

  const stationPositions = new Float32Array(MAX_STATIONS * 3); // rewritten every frame
  const stationPosBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, stationPosBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, stationPositions.byteLength, gl.DYNAMIC_DRAW);
  const stationVao = gl.createVertexArray();
  gl.bindVertexArray(stationVao);
  gl.bindBuffer(gl.ARRAY_BUFFER, stationPosBuffer);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  const stationColors = new Uint8Array(MAX_STATIONS * 4);
  for (let k = 0; k < MAX_STATIONS; k++) stationColors.set([255, 235, 90, 255], 4 * k);
  staticBuffer(gl.ARRAY_BUFFER, stationColors);
  gl.enableVertexAttribArray(1);
  gl.vertexAttribPointer(1, 4, gl.UNSIGNED_BYTE, true, 0, 0);
  gl.bindVertexArray(null);

  // Legend
  const legendRow = (rgba, label, n) =>
    `<div><span class="dot" style="background:rgb(${rgba[0]},${rgba[1]},${rgba[2]})"></span>` +
    `${label} <span style="color:var(--dim)">${n.toLocaleString()}</span></div>`;
  $('legend').innerHTML =
    legendRow(COLOR_ACTIVE, 'Active satellites', tally.active) +
    legendRow(COLOR_DEBRIS, 'Debris', tally.debris) +
    legendRow(COLOR_ROCKET, 'Rocket bodies', tally.rocket) +
    '<div><span class="dot" style="background:rgb(115,255,140)"></span>In view of a station</div>' +
    '<div><span class="dot" style="background:rgb(255,235,90)"></span>Ground station</div>';

  // Camera (orbits the Earth's centre; z is up)
  const cam = { yaw: 0.6, pitch: 0.35, dist: 3.2 }; // dist in Earth radii
  let dragging = false, moved = false, lastX = 0, lastY = 0, downX = 0, downY = 0;
  canvas.addEventListener('pointerdown', (e) => {
    dragging = true; moved = false;
    lastX = downX = e.clientX; lastY = downY = e.clientY;
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    if (Math.hypot(e.clientX - downX, e.clientY - downY) > 4) moved = true;
    cam.yaw -= (e.clientX - lastX) * 0.006;
    cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + (e.clientY - lastY) * 0.006));
    lastX = e.clientX; lastY = e.clientY;
  });
  canvas.addEventListener('pointerup', (e) => {
    if (dragging && !moved) handleClick(e.clientX, e.clientY); // a click, not a drag
    dragging = false;
  });
  canvas.addEventListener('pointercancel', () => { dragging = false; });
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    cam.dist = Math.max(1.15, Math.min(30, cam.dist * Math.exp(e.deltaY * 0.001)));
  }, { passive: false });
  window.addEventListener('keydown', (e) => { if (e.key === 'Escape') { setPlacing(false); select(-1); } });

  // Simulation clock
  let simMs = Date.now();
  let scale = 1;
  let paused = false;
  let needsPropagate = true;
  const scaleButtons = [...document.querySelectorAll('button[data-scale]')];
  function refreshButtons() {
    $('btn-pause').textContent = paused ? 'Resume' : 'Pause';
    for (const b of scaleButtons) {
      b.classList.toggle('active', !paused && Number(b.dataset.scale) === scale);
    }
  }
  for (const b of scaleButtons) {
    b.addEventListener('click', () => { scale = Number(b.dataset.scale); paused = false; refreshButtons(); });
  }
  $('btn-pause').addEventListener('click', () => { paused = !paused; refreshButtons(); });
  $('btn-now').addEventListener('click', () => {
    simMs = Date.now(); scale = 1; paused = false; needsPropagate = true; refreshButtons();
  });
  refreshButtons();

  // Selection
  let selected = -1;
  let selectedPeriodMin = 95;
  let lastViewProj = null, lastEye = null; // from the most recent frame, used for picking
  const esc = (text) => text.replace(/[&<>"']/g, (c) =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const fmtLat = (v) => `${Math.abs(v).toFixed(2)}° ${v >= 0 ? 'N' : 'S'}`;
  const fmtLon = (v) => `${Math.abs(v).toFixed(2)}° ${v >= 0 ? 'E' : 'W'}`;

  function handleClick(clientX, clientY) {
    if (!lastViewProj) return;
    if (placing) { placeStationAt(clientX, clientY); return; }
    const rect = canvas.getBoundingClientRect();
    select(pickNearest({
      positions: engine.positions(), count, scale: 1 / EARTH_RADIUS_KM,
      viewProj: lastViewProj, eye: lastEye,
      width: canvas.clientWidth, height: canvas.clientHeight,
      x: clientX - rect.left, y: clientY - rect.top, radius: 12,
    }));
  }

  function select(index) {
    selected = index;
    updateInfoPanel();
  }

  // Details of the selected object. Every number comes from the C++ engine (Engine.describe).
  function updateInfoPanel() {
    const panel = $('info');
    if (selected < 0) { panel.style.display = 'none'; return; }
    engine.describe(selected, JD_UNIX_EPOCH, simMs / MS_PER_DAY);
    const d = engine.info();
    if (d[11] > 0) selectedPeriodMin = d[11];
    const age = d[16];
    const ageText = `${Math.abs(age).toFixed(2)} d ${age >= 0 ? 'old' : 'before epoch'}`;
    const ageClass = Math.abs(age) > 3 ? ' class="warn"' : '';
    const lines = [
      '<span class="head">SELECTED OBJECT</span>',
      '',
      `Name          ${esc(engine.name(selected))}`,
      `NORAD ID      ${engine.satnum(selected)}`,
      `Type          ${KIND_LABELS[kind[selected]]}`,
    ];
    if (d[0] === 1) {
      lines.push(
        `Altitude      ${d[10].toFixed(1)} km`,
        `Speed         ${d[7].toFixed(3)} km/s (TEME)`,
        `Sub-sat point ${fmtLat(d[8])}  ${fmtLon(d[9])}`);
    } else {
      lines.push('<span class="warn">Propagation failed at this time</span>');
    }
    lines.push(
      `Period        ${d[11].toFixed(1)} min`,
      `Inclination   ${d[12].toFixed(2)}°`,
      `Eccentricity  ${d[13].toFixed(5)}`,
      `Perigee/apog. ${d[14].toFixed(0)} / ${d[15].toFixed(0)} km (mean)`,
      `Element age   <span${ageClass}>${ageText}</span>`);
    if (stations.length > 0) {
      lines.push('', '<span class="head">FROM GROUND STATIONS</span>');
      const jdFrac = simMs / MS_PER_DAY;
      stations.forEach((station, i) => {
        if (!engine.look(i, selected, JD_UNIX_EPOCH, jdFrac)) return;
        const l = engine.lookInfo(); // azimuth deg, elevation deg, range km
        const inView = l[1] >= minElevationDeg;
        lines.push(`<span class="${inView ? 'ok' : ''}">${esc(station.name.slice(0, 14).padEnd(14))} ` +
                   `Az ${l[0].toFixed(1).padStart(5)}° El ${l[1].toFixed(1).padStart(5)}° ` +
                   `${l[2].toFixed(0).padStart(5)} km ${inView ? '▲' : '·'}</span>`);
      });
    }
    lines.push('', 'Esc or click empty space to clear');
    panel.innerHTML = lines.join('\n');
    panel.style.display = 'block';
  }

  // Ground stations. The engine (C++) decides what each station can see; this code manages the
  // list, draws the markers and cones, and lets you add stations by clicking on the Earth.
  const FOV_Y = 0.8; // vertical field of view of the camera, radians
  const CONE_ALTITUDE_KM = 1000; // cones are cut off at this altitude above the Earth's surface
  const CONE_SHELL = 1 + CONE_ALTITUDE_KM / EARTH_RADIUS_KM;
  const PRESETS = [ // approximate public coordinates
    { name: 'Vandenberg SFB', lat: 34.74, lon: -120.57 },
    { name: 'Wallops Island', lat: 37.94, lon: -75.47 },
    { name: 'Svalbard', lat: 78.23, lon: 15.40 },
    { name: 'Cape Canaveral', lat: 28.49, lon: -80.58 },
    { name: 'Kwajalein', lat: 9.39, lon: 167.47 },
    { name: 'Pituffik', lat: 76.53, lon: -68.70 },
  ];
  let stations = PRESETS.slice(0, 3).map((p) => ({ ...p, alt: 0 })); // alt: km above the ellipsoid
  let minElevationDeg = Number($('mask-select').value) || 10;
  let placing = false;
  let coneCount = 0;
  let lastGmst = 0;
  let lastStationHtml = '';
  const fmtPlace = (st) =>
    `${Math.abs(st.lat).toFixed(2)}°${st.lat >= 0 ? 'N' : 'S'} ${Math.abs(st.lon).toFixed(2)}°${st.lon >= 0 ? 'E' : 'W'}`;

  function setNote(text) { $('station-note').textContent = text; }

  function setPlacing(on) {
    placing = on;
    canvas.style.cursor = on ? 'crosshair' : '';
    $('btn-add-station').classList.toggle('active', on);
    setNote(on ? 'Click the globe to place a station (Esc cancels)' : '');
  }

  function renderStationList() {
    const html = stations.length === 0
      ? '<div class="dim">none: use Add by click or Add presets</div>'
      : stations.map((st, i) =>
          `<div class="srow"><span class="name">${esc(st.name)}</span>` +
          `<span class="dim">${fmtPlace(st)}</span>` +
          `<span class="ok">${engine.visibleCount(i).toLocaleString()} in view</span>` +
          `<button data-remove="${i}">×</button></div>`).join('');
    if (html !== lastStationHtml) {
      $('station-list').innerHTML = html;
      lastStationHtml = html;
    }
  }

  // Sends the station list to the engine and rebuilds the cones.
  function syncStations() {
    engine.clearStations();
    stations.forEach((st, k) => {
      engine.addStation(st.lat, st.lon, st.alt, minElevationDeg);
      coneData.set(
        coneLines(st.lat, st.lon, st.alt / EARTH_RADIUS_KM, minElevationDeg, CONE_SHELL, CONE_SPOKES),
        k * VERTS_PER_CONE * 3);
    });
    coneCount = stations.length;
    gl.bindBuffer(gl.ARRAY_BUFFER, coneBuffer);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, coneData);
    needsPropagate = true; // recompute what is in view at the next frame
    renderStationList();
    if (selected >= 0) updateInfoPanel();
  }

  function placeStationAt(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const dir = screenRay(lastEye, FOV_Y, w / h, ((clientX - rect.left) / w) * 2 - 1, 1 - ((clientY - rect.top) / h) * 2);
    const hit = rayEllipsoid(lastEye, dir);
    if (!hit) { setNote('That click missed the Earth. Try again.'); return; }
    if (stations.length >= MAX_STATIONS) { setPlacing(false); setNote(`Maximum of ${MAX_STATIONS} stations`); return; }
    const { latDeg, lonDeg } = sceneToGeodeticSurface(rotateZVec(hit, -lastGmst)); // inertial -> Earth-fixed
    stations.push({ name: `Site ${stations.length + 1}`, lat: latDeg, lon: lonDeg, alt: 0 });
    setPlacing(false);
    syncStations();
  }

  // pointerdown (not click): the list is redrawn as counts change, which could swallow a click
  $('station-list').addEventListener('pointerdown', (e) => {
    const target = e.target;
    if (target && target.dataset && target.dataset.remove !== undefined) {
      stations.splice(Number(target.dataset.remove), 1);
      syncStations();
    }
  });
  $('btn-add-station').addEventListener('click', () => setPlacing(!placing));
  $('btn-presets').addEventListener('click', () => {
    for (const preset of PRESETS) {
      if (stations.some((st) => st.name === preset.name)) continue;
      if (stations.length >= MAX_STATIONS) { setNote(`Maximum of ${MAX_STATIONS} stations`); break; }
      stations.push({ ...preset, alt: 0 });
    }
    syncStations();
  });
  $('btn-clear-stations').addEventListener('click', () => { stations = []; syncStations(); });
  $('mask-select').addEventListener('change', () => {
    minElevationDeg = Number($('mask-select').value);
    syncStations();
  });
  syncStations();

  // HUD
  let propagateMs = 0, fps = 0;
  function hudText() {
    const utc = new Date(simMs).toISOString().slice(0, 19).replace('T', ' ') + 'Z';
    const rate = paused ? 'PAUSED' : `x${scale}`;
    const flags = engine.visibility();
    let inView = 0;
    for (let i = 0; i < flags.length; i++) if (flags[i] !== 0) inView++;
    return [
      'LEO SITUATIONAL AWARENESS',
      '',
      `UTC           ${utc}`,
      `Sim rate      ${rate}`,
      `Ref frame     TEME (inertial), km`,
      `Objects       ${count.toLocaleString()} propagated (SGP4, WGS-72)`,
      `Skipped       ${skippedDeepSpace.toLocaleString()} deep-space`,
      `Stations      ${stations.length} (min elevation ${minElevationDeg}°)`,
      `In view       ${inView.toLocaleString()} objects`,
      `Data          CelesTrak GP, ${buildInfo ? 'fetched ' + buildInfo.catalogFetchedUtc : 'file dated ' + (fileDate || 'unknown')}`,
      `Build         ${buildInfo ? buildInfo.commit : 'local'}`,
      `Coastlines    ${coastline ? 'Natural Earth 1:110m (public domain)' : 'not loaded'}`,
      '',
      `Propagate     ${propagateMs.toFixed(1)} ms (WebAssembly, f64)`,
      `Render        ${fps.toFixed(0)} fps`,
      `Camera        ${(cam.dist * EARTH_RADIUS_KM).toFixed(0)} km from Earth centre`,
    ].join('\n');
  }

  // Frame loop
  let lastTime = performance.now();
  let lastHud = -1e9;
  function frame(now) {
    const dt = Math.min(now - lastTime, 250);
    lastTime = now;
    if (dt > 0) fps = fps === 0 ? 1000 / dt : 0.95 * fps + 0.05 * (1000 / dt);
    if (!paused) { simMs += dt * scale; needsPropagate = true; }
    const jdFrac = simMs / MS_PER_DAY;

    if (needsPropagate) {
      const t0 = performance.now();
      engine.propagate(JD_UNIX_EPOCH, jdFrac);
      gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, 0, engine.positions()); // view into WebAssembly memory
      engine.updateVisibility(JD_UNIX_EPOCH, jdFrac); // which objects each station can see
      gl.bindBuffer(gl.ARRAY_BUFFER, flagsBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, 0, engine.visibility());
      propagateMs = performance.now() - t0;
      needsPropagate = false;
    }
    const gmst = engine.gmst(JD_UNIX_EPOCH, jdFrac);
    lastGmst = gmst;

    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.floor(canvas.clientWidth * dpr));
    const height = Math.max(1, Math.floor(canvas.clientHeight * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    gl.viewport(0, 0, width, height);

    const cp = Math.cos(cam.pitch);
    const eye = [cam.dist * cp * Math.cos(cam.yaw), cam.dist * cp * Math.sin(cam.yaw), cam.dist * Math.sin(cam.pitch)];
    const view = lookAt(eye, [0, 0, 0], [0, 0, 1]);
    const viewProj = multiply(perspective(FOV_Y, width / height, 0.05, 200), view);
    const earthMvp = multiply(viewProj, rotateZ(gmst)); // Earth-fixed -> TEME
    lastViewProj = viewProj;
    lastEye = [eye[0], eye[1], eye[2]];

    gl.clearColor(0.02, 0.03, 0.06, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

    gl.useProgram(meshProgram);
    gl.uniformMatrix4fv(meshMvp, false, earthMvp);
    gl.uniform4f(meshColor, 0.04, 0.10, 0.17, 1.0);
    gl.bindVertexArray(earth.vao);
    gl.drawElements(gl.TRIANGLES, earth.count, gl.UNSIGNED_SHORT, 0);
    gl.uniform4f(meshColor, 0.35, 0.50, 0.70, 0.35);
    gl.bindVertexArray(minorGrid.vao);
    gl.drawArrays(gl.LINES, 0, minorGrid.count);
    gl.uniform4f(meshColor, 0.45, 0.70, 1.0, 0.8);
    gl.bindVertexArray(majorGrid.vao);
    gl.drawArrays(gl.LINES, 0, majorGrid.count);
    if (coastGrid) {
      gl.uniform4f(meshColor, 0.55, 0.78, 0.95, 0.75);
      gl.bindVertexArray(coastGrid.vao);
      gl.drawArrays(gl.LINES, 0, coastGrid.count);
  }

    // Visibility cones (Earth-fixed): dim spokes and a brighter rim
    if (coneCount > 0) {
      gl.useProgram(meshProgram);
      gl.uniformMatrix4fv(meshMvp, false, earthMvp);
      gl.bindVertexArray(coneVao);
      for (let k = 0; k < coneCount; k++) {
        gl.uniform4f(meshColor, 0.35, 1.0, 0.55, 0.22);
        gl.drawArrays(gl.LINES, k * VERTS_PER_CONE, CONE_SPOKES * 2);
        gl.uniform4f(meshColor, 0.45, 1.0, 0.65, 0.75);
        gl.drawArrays(gl.LINES, k * VERTS_PER_CONE + CONE_SPOKES * 2, CONE_SPOKES * 2);
      }
    }

    gl.useProgram(pointsProgram);
    gl.uniformMatrix4fv(pointsViewProj, false, viewProj);
    gl.uniform1f(pointsScale, 1 / EARTH_RADIUS_KM);
    gl.uniform1f(pointsSize, 2 * dpr);
    gl.uniform1f(pointsRing, 0);
    gl.bindVertexArray(pointsVao);
    gl.drawArrays(gl.POINTS, 0, count);

    // Ground station markers (Earth-fixed positions rotated into the inertial frame)
    if (stations.length > 0) {
      stations.forEach((st, k) => {
        const p = rotateZVec(geodeticToScene(st.lat, st.lon, st.alt / EARTH_RADIUS_KM), gmst);
        stationPositions[3 * k] = p[0] * EARTH_RADIUS_KM;
        stationPositions[3 * k + 1] = p[1] * EARTH_RADIUS_KM;
        stationPositions[3 * k + 2] = p[2] * EARTH_RADIUS_KM;
      });
      gl.bindBuffer(gl.ARRAY_BUFFER, stationPosBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, 0, stationPositions);
      gl.uniform1f(pointsSize, 9 * dpr);
      gl.bindVertexArray(stationVao);
      gl.drawArrays(gl.POINTS, 0, stations.length);
    }

    if (selected >= 0) {
      // Orbit track: one orbital period from now, in the inertial frame
      engine.track(selected, JD_UNIX_EPOCH, jdFrac, selectedPeriodMin, TRACK_POINTS);
      const tp = engine.trackPoints();
      for (let k = 0; k < trackScaled.length; k++) trackScaled[k] = tp[k] / EARTH_RADIUS_KM;
      gl.bindBuffer(gl.ARRAY_BUFFER, trackBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, 0, trackScaled);
      gl.useProgram(meshProgram);
      gl.uniformMatrix4fv(meshMvp, false, viewProj);
      gl.uniform4f(meshColor, 1.0, 0.82, 0.3, 0.9);
      gl.bindVertexArray(trackVao);
      gl.drawArrays(gl.LINE_STRIP, 0, TRACK_POINTS);

      // Selection ring on the object itself
      gl.bindBuffer(gl.ARRAY_BUFFER, markerPosBuffer);
      gl.bufferSubData(gl.ARRAY_BUFFER, 0, engine.positions().subarray(3 * selected, 3 * selected + 3));
      gl.useProgram(pointsProgram);
      gl.uniformMatrix4fv(pointsViewProj, false, viewProj);
      gl.uniform1f(pointsScale, 1 / EARTH_RADIUS_KM);
      gl.uniform1f(pointsSize, 14 * dpr);
      gl.uniform1f(pointsRing, 1);
      gl.bindVertexArray(markerVao);
      gl.drawArrays(gl.POINTS, 0, 1);
    }

    if (now - lastHud > 250) {
      $('hud').textContent = hudText();
      renderStationList();
      if (selected >= 0) updateInfoPanel();
      lastHud = now;
    }
    requestAnimationFrame(frame);
  }

  hideMessage();
  requestAnimationFrame(frame);
}

main().catch((err) => {
  console.error(err);
  showMessage('Error: ' + err.message);
});