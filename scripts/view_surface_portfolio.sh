#!/usr/bin/env bash
set -euo pipefail
surface_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$surface_workspace"
exec /usr/bin/env PYTHONPATH="$surface_workspace/outputs/surface_runtime/packages" \
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 -s src/view_surface_portfolio.py "$@"
