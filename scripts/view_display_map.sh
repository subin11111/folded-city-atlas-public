#!/usr/bin/env bash
# Loopback-only display; frozen map/poses are never overwritten.
set -euo pipefail
display_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$display_workspace"
exec /usr/bin/env PYTHONPATH="$display_workspace/outputs/surface_runtime/packages" \
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 EGL_PLATFORM=surfaceless \
  /usr/bin/python3 -s src/serve_display_map.py "$@"
