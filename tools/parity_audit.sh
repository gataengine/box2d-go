#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM="${1:-$ROOT/../box2d}"

if [[ ! -d "$UPSTREAM/include/box2d" ]]; then
  echo "upstream include dir not found: $UPSTREAM/include/box2d" >&2
  exit 1
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

rg -n '^B2_API' "$UPSTREAM"/include/box2d/*.h \
  | perl -ne 'if(/\b(b2[A-Za-z0-9_]+)\s*\(/){print "$1\n"}' \
  | sort -u > "$tmpdir/c_api.txt"

rg -n '^func ' "$ROOT"/*.go \
  | sed -E 's/^[^:]+:[0-9]+:func (\([^)]*\) )?([A-Za-z0-9_]+).*/\2/' \
  | sort -u > "$tmpdir/go_funcs.txt"

missing=0
while read -r sym; do
  core="${sym#b2}"
  if [[ "$core" == *_* ]]; then
    cand="${core#*_}"
  else
    cand="$core"
  fi

  if ! grep -qx "$cand" "$tmpdir/go_funcs.txt"; then
    echo "$sym -> $cand"
    missing=$((missing+1))
  fi
done < "$tmpdir/c_api.txt"

echo "---"
echo "upstream_symbols=$(wc -l < "$tmpdir/c_api.txt")"
echo "go_exported_funcs=$(wc -l < "$tmpdir/go_funcs.txt")"
echo "missing_symbol_names=$missing"

if [[ $missing -ne 0 ]]; then
  exit 2
fi
