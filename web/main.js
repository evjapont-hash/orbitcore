// LEO situational awareness viewer.
//
// orbitcore (C++ compiled to WebAssembly) propagates every object with SGP4 in double precision
// and hands back float32 TEME positions. This file uploads them to the GPU each frame and draws
// every object as a point sprite in a single draw call. Scene units are Earth radii. The frame
// is TEME (inertial), so the Earth is rotated by GMST while satellites are drawn as propagated.
import { lookAt, multiply, perspective, rotateZ } from './mat4.js';

const EARTH_RADIUS_KM = 6378.137; // WGS-84 equatorial radius; only used to scale the scene
const JD_UNIX_EPOCH = 2440587.5; // Julian date of 1970-01-01 00:00 UTC
const MS_PER_DAY = 86400000;
const DEG = Math.PI / 180;

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
  gl_Position = uViewProj * vec4(aPos * uScale, 1.0);
  gl_PointSize = uPointSize;
  vColor = aColor;
}`;

const POINTS_FRAG = `#version 300 es
precision mediump float;
in vec4 vColor;
out vec4 outColor;
void main() {
  vec2 c = gl_PointCoord - vec2(0.5);
  if (dot(c, c) > 0.25) discard;   // round points
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
function sphereMesh(latSegs, lonSegs, radius) {
  const verts = [];
  for (let i = 0; i <= latSegs; i++) {
    const lat = -Math.PI / 2 + (Math.PI * i) / latSegs;
    for (let j = 0; j <= lonSegs; j++) {
      const lon = (2 * Math.PI * j) / lonSegs;
      verts.push(
        radius * Math.cos(lat) * Math.cos(lon),
        radius * Math.cos(lat) * Math.sin(lon),
        radius * Math.sin(lat));
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

// Latitude circles and meridians as line segments (pairs of vertices).
function graticule(latsDeg, lonsDeg, radius) {
  const v = [];
  const push = (latDeg, lonDeg) => {
    const lat = latDeg * DEG, lon = lonDeg * DEG;
    v.push(
      radius * Math.cos(lat) * Math.cos(lon),
      radius * Math.cos(lat) * Math.sin(lon),
      radius * Math.sin(lat));
  };
  const STEP = 5; // degrees per segment
  for (const lat of latsDeg) {
    for (let lon = -180; lon < 180; lon += STEP) { push(lat, lon); push(lat, lon + STEP); }
  }
  for (const lon of lonsDeg) {
    for (let lat = -90; lat < 90; lat += STEP) { push(lat, lon); push(lat + STEP, lon); }
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
  const text = await response.text();
  const count = engine.loadTle(text);
  const skippedDeepSpace = engine.skippedDeepSpace();
  if (count === 0) throw new Error('catalog.tle contained no usable objects');

  // Classify by name (CelesTrak marks debris "DEB" and rocket bodies "R/B") and pick colors.
  const colors = new Uint8Array(count * 4);
  const tally = { active: 0, debris: 0, rocket: 0 };
  const COLOR_ACTIVE = [80, 200, 255, 255];
  const COLOR_DEBRIS = [255, 150, 60, 255];
  const COLOR_ROCKET = [170, 170, 185, 255];
  for (let i = 0; i < count; i++) {
    const name = engine.name(i);
    let color = COLOR_ACTIVE;
    if (name.endsWith(' DEB')) { color = COLOR_DEBRIS; tally.debris++; }
    else if (name.endsWith(' R/B')) { color = COLOR_ROCKET; tally.rocket++; }
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

  const sphere = sphereMesh(36, 72, 0.997);
  const earth = meshObject(sphere.vertices, sphere.indices);
  const minorLats = [-75, -60, -45, -30, -15, 15, 30, 45, 60, 75];
  const minorLons = [];
  for (let lon = -165; lon <= 165; lon += 15) if (lon !== 0) minorLons.push(lon);
  const minorGrid = meshObject(graticule(minorLats, minorLons, 1.0));
  const majorGrid = meshObject(graticule([0], [0, 180], 1.0)); // equator + prime/anti meridian

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
  gl.bindVertexArray(null);

  // Legend
  const legendRow = (rgba, label, n) =>
    `<div><span class="dot" style="background:rgb(${rgba[0]},${rgba[1]},${rgba[2]})"></span>` +
    `${label} <span style="color:var(--dim)">${n.toLocaleString()}</span></div>`;
  $('legend').innerHTML =
    legendRow(COLOR_ACTIVE, 'Active satellites', tally.active) +
    legendRow(COLOR_DEBRIS, 'Debris', tally.debris) +
    legendRow(COLOR_ROCKET, 'Rocket bodies', tally.rocket);

  // Camera (orbits the Earth's centre; z is up)
  const cam = { yaw: 0.6, pitch: 0.35, dist: 3.2 }; // dist in Earth radii
  let dragging = false, lastX = 0, lastY = 0;
  canvas.addEventListener('pointerdown', (e) => {
    dragging = true; lastX = e.clientX; lastY = e.clientY;
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    cam.yaw -= (e.clientX - lastX) * 0.006;
    cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + (e.clientY - lastY) * 0.006));
    lastX = e.clientX; lastY = e.clientY;
  });
  const stopDrag = () => { dragging = false; };
  canvas.addEventListener('pointerup', stopDrag);
  canvas.addEventListener('pointercancel', stopDrag);
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    cam.dist = Math.max(1.15, Math.min(30, cam.dist * Math.exp(e.deltaY * 0.001)));
  }, { passive: false });

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

  // HUD
  let propagateMs = 0, fps = 0;
  function hudText() {
    const utc = new Date(simMs).toISOString().slice(0, 19).replace('T', ' ') + 'Z';
    const rate = paused ? 'PAUSED' : `x${scale}`;
    return [
      'LEO SITUATIONAL AWARENESS',
      '',
      `UTC           ${utc}`,
      `Sim rate      ${rate}`,
      `Ref frame     TEME (inertial), km`,
      `Objects       ${count.toLocaleString()} propagated (SGP4, WGS-72)`,
      `Skipped       ${skippedDeepSpace.toLocaleString()} deep-space`,
      `Catalog file  ${fileDate || 'date unknown'}`,
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
      propagateMs = performance.now() - t0;
      needsPropagate = false;
    }
    const gmst = engine.gmst(JD_UNIX_EPOCH, jdFrac);

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
    const viewProj = multiply(perspective(0.8, width / height, 0.05, 200), view);
    const earthMvp = multiply(viewProj, rotateZ(gmst)); // Earth-fixed -> TEME

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

    gl.useProgram(pointsProgram);
    gl.uniformMatrix4fv(pointsViewProj, false, viewProj);
    gl.uniform1f(pointsScale, 1 / EARTH_RADIUS_KM);
    gl.uniform1f(pointsSize, 2 * dpr);
    gl.bindVertexArray(pointsVao);
    gl.drawArrays(gl.POINTS, 0, count);

    if (now - lastHud > 250) {
      $('hud').textContent = hudText();
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