#!/usr/bin/env bash
# Copies the WebAssembly build and a catalog snapshot into web/ and serves it locally.
#   bash scripts/serve_web.sh [catalog.tle]      (default catalog: ~/all_objects.tle)
set -euo pipefail
cd "$(dirname "$0")/.."

CATALOG="${1:-$HOME/all_objects.tle}"
for f in build-wasm/orbitcore_wasm.js build-wasm/orbitcore_wasm.wasm; do
  [ -f "$f" ] || { echo "missing $f: build the WebAssembly target first"; exit 1; }
  cp -p "$f" web/
done
[ -f "$CATALOG" ] || { echo "catalog file not found: $CATALOG"; exit 1; }
cp -p "$CATALOG" web/catalog.tle

echo "Serving web/ at http://localhost:8080  (press Ctrl+C to stop)"
python3 -m http.server 8080 --directory web