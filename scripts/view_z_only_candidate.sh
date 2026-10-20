#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="$PWD/outputs/surface_runtime/packages${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
candidate_review_dir=$(mktemp -d "$PWD/outputs/z-only-view-XXXXXX")
exec /usr/bin/python3 -s src/review_z_only_candidate.py \
  --poses outputs/student_union_floor2_z_only_v1 \
  --rebuild outputs/student_union_floor2_z_rebuild_v1 \
  --variant Z02 --native --evidence "$candidate_review_dir/evidence" "$@"
