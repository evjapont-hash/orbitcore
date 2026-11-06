# Architecture

Single C++20 core (`orbitcore`) built twice: natively for the server and tests, and to
WebAssembly for in-browser propagation. All orbital math lives in C++; the frontend only renders.

```mermaid
flowchart LR
  A[CelesTrak / Space-Track<br/>OMM JSON] --> B[Ingest Service<br/>C++ · libcurl · cache]
  B --> C[(Catalog Store<br/>latest elements + epoch)]
  C --> D[Conjunction Service<br/>C++ · screening · TCA · Pc]
  C --> E[Catalog API<br/>WebSocket + binary protocol]
  D --> E
  E --> F[Browser: Web Worker<br/>orbitcore WASM · SGP4 f64]
  F --> G[Position buffer<br/>SharedArrayBuffer / transfer]
  G --> H[WebGPU Renderer<br/>instanced draw · GPU coverage pass]
  D -. alerts .-> I[Operator HUD<br/>object table · conjunction list · burn cues]
  H --> I
```

## Frames and units

| Frame | Meaning | Used for |
|-------|---------|----------|
| TEME  | SGP4 output (true equator, mean equinox) | propagation, conjunction math |
| ECEF  | Earth-fixed, WGS-84 | ground tracks, station geometry |
| ENU / NED | local tangent at a station | look angles, coverage cones |
| RTN   | radial / in-track / normal about a reference object | conjunction geometry, burn vectors |

Lengths km, velocities km/s, angles rad internally. SGP4 uses WGS-72 constants; geodetic uses WGS-84.

## Known limitations (state these in the UI too)

- TLE/OMM accuracy is ~1 km at epoch and degrades by km/day.
- Near-Earth only (period < 225 min); deep-space (SDP4) not implemented.
- GMST computed from UTC (up to 0.9 s UT1 error); polar motion and nutation neglected in TEME->ECEF.
