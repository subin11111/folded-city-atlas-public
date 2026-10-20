#!/usr/bin/env bash
set -euo pipefail

workspace=/home/subin/folded-city-atlas
db="$workspace/data/raw/rtabmap/261006-162929.db"

/usr/bin/python3 "$workspace/src/optimize_pose_graph.py" "$db" \
  --raw-trajectory "$workspace/outputs/trajectory/raw_trajectory.csv" \
  --optimized-trajectory "$workspace/outputs/trajectory/optimized_trajectory.csv" \
  --floor-observations "$workspace/outputs/floor_observations.csv" \
  --loop-report "$workspace/outputs/loop_closure_validation.csv" \
  --local-correction "$workspace/outputs/local_floor_rotation.json" \
  --report "$workspace/outputs/pose_graph_optimization.json" \
  --max-evaluations 250

/usr/bin/python3 "$workspace/src/export_rtabmap_db.py" "$db" \
  --cloud "$workspace/outputs/pointcloud/colored_cloud_optimized_5cm.ply" \
  --trajectory "$workspace/outputs/trajectory/optimized_trajectory.csv" \
  --summary "$workspace/outputs/export_optimized_summary.json" \
  --pose-csv "$workspace/outputs/trajectory/optimized_trajectory.csv" \
  --local-correction-json "$workspace/outputs/local_floor_rotation.json" \
  --frame-stride 2 \
  --pixel-stride 1 \
  --voxel 0.05 \
  --min-confidence 50

/usr/bin/python3 "$workspace/src/remove_ground_ransac.py" \
  "$workspace/outputs/pointcloud/colored_cloud_optimized_5cm.ply" \
  --ground "$workspace/outputs/pointcloud/ground_optimized_ransac_5cm.ply" \
  --non-ground "$workspace/outputs/pointcloud/colored_cloud_optimized_no_ground_5cm.ply" \
  --report "$workspace/outputs/ground_optimized_ransac_report.json" \
  --threshold 0.06 \
  --iterations 600 \
  --max-tilt 15
