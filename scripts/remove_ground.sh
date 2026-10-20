#!/usr/bin/env bash
set -euo pipefail

workspace=/home/subin/folded-city-atlas

/usr/bin/python3 "$workspace/src/remove_ground_ransac.py" \
  "$workspace/outputs/pointcloud/colored_cloud_5cm.ply" \
  --ground "$workspace/outputs/pointcloud/ground_ransac_5cm.ply" \
  --non-ground "$workspace/outputs/pointcloud/colored_cloud_no_ground_5cm.ply" \
  --report "$workspace/outputs/ground_ransac_report.json" \
  --threshold 0.06 \
  --iterations 600 \
  --max-tilt 15
