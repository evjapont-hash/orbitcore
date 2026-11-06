#!/usr/bin/env bash
# Downloads the public CelesTrak element sets used by the viewer and merges them into one TLE file.
#   bash scripts/fetch_catalog.sh [output-file]      (default: web/catalog.tle)
# CelesTrak updates its data every few hours and asks users not to download it repeatedly,
# so do not run this in a loop. Set FETCH_DELAY (seconds between requests, default 5) to change pacing.
set -euo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-web/catalog.tle}"
DELAY="${FETCH_DELAY:-5}"
BASE="https://celestrak.org/NORAD/elements/gp.php"
DEBRIS_GROUPS=(cosmos-2251-debris iridium-33-debris fengyun-1c-debris cosmos-1408-debris)
MIN_ACTIVE_OBJECTS=8000   # the active group has ~15,000 objects; far fewer means a bad download

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fetch() {  # fetch <group> -> $TMP/<group>.tle, validated
  local group="$1" file="$TMP/$1.tle"
  echo "fetching $group"
  curl --fail --silent --show-error --retry 3 --retry-delay 15 \
       --user-agent "orbitcore-viewer (https://github.com/evjapont-hash/orbitcore)" \
       -o "$file" "$BASE?GROUP=$group&FORMAT=tle"
  local lines ones twos
  lines=$(wc -l < "$file")
  ones=$(grep -c '^1 ' "$file" || true)
  twos=$(grep -c '^2 ' "$file" || true)
  if [ "$ones" -ne "$twos" ] || [ "$lines" -ne $((ones * 3)) ]; then
    echo "error: $group is not a well-formed 3-line TLE file (lines=$lines, line1=$ones, line2=$twos)" >&2
    head -n 3 "$file" >&2
    exit 1
  fi
  echo "  $group: $ones objects"
}

fetch active
if [ "$(grep -c '^1 ' "$TMP/active.tle")" -lt "$MIN_ACTIVE_OBJECTS" ]; then
  echo "error: active group has fewer than $MIN_ACTIVE_OBJECTS objects" >&2
  exit 1
fi
for group in "${DEBRIS_GROUPS[@]}"; do
  sleep "$DELAY"
  fetch "$group"
done

mkdir -p "$(dirname "$OUT")"
cat "$TMP/active.tle" $(printf "$TMP/%s.tle " "${DEBRIS_GROUPS[@]}") > "$OUT"
echo "wrote $OUT ($(grep -c '^1 ' "$OUT") objects)"