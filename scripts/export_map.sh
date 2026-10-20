#!/usr/bin/env bash
set -euo pipefail

workspace=/home/subin/folded-city-atlas
db="$workspace/data/raw/rtabmap/261006-162929.db"

/usr/bin/python3 "$workspace/src/export_rtabmap_db.py" "$db" \
  --cloud "$workspace/outputs/pointcloud/colored_cloud_5cm.ply" \
  --trajectory "$workspace/outputs/trajectory/raw_trajectory.csv" \
  --summary "$workspace/outputs/export_raw_summary.json" \
  --frame-stride 2 \
  --pixel-stride 1 \
  --voxel 0.05 \
  --min-confidence 50
