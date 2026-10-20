#!/usr/bin/env bash
set -eo pipefail

workspace=/home/subin/folded-city-atlas
source /opt/ros/humble/setup.bash
set -u

raw_cloud="$workspace/outputs/pointcloud/colored_cloud_5cm.ply"
optimized_cloud="$workspace/outputs/pointcloud/colored_cloud_optimized_5cm.ply"
optimized_non_ground="$workspace/outputs/pointcloud/colored_cloud_optimized_no_ground_5cm.ply"
optimized_ground="$workspace/outputs/pointcloud/ground_optimized_ransac_5cm.ply"
raw_trajectory="$workspace/outputs/trajectory/raw_trajectory.csv"
optimized_trajectory="$workspace/outputs/trajectory/optimized_trajectory.csv"
config="$workspace/config/folded_city.rviz"

if [[ ! -f "$raw_cloud" || ! -f "$raw_trajectory" ]]; then
  "$workspace/scripts/export_map.sh"
fi
if [[ ! -f "$optimized_cloud" || ! -f "$optimized_trajectory" || ! -f "$optimized_non_ground" ]]; then
  "$workspace/scripts/optimize_map.sh"
fi

/usr/bin/python3 "$workspace/src/rviz_cloud_publisher.py" \
  --raw-cloud "$raw_cloud" \
  --optimized-cloud "$optimized_cloud" \
  --optimized-non-ground "$optimized_non_ground" \
  --optimized-ground "$optimized_ground" \
  --raw-trajectory "$raw_trajectory" \
  --optimized-trajectory "$optimized_trajectory" \
  >"$workspace/logs/rviz_publisher.log" 2>&1 &
publisher_pid=$!

cleanup() {
  kill "$publisher_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rviz2 -d "$config"
