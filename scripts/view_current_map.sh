#!/usr/bin/env bash
# Open the explicitly selected working-map release; leave historical previews intact.
set -euo pipefail
selected_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$selected_workspace"
# This workspace targets the installed Humble runtime. Its full setup script
# scans every package dependency, which currently blocks on filesystem I/O.
# Scope the equivalent runtime search paths to this viewer process only.
exec /usr/bin/env \
  PATH="/opt/ros/humble/bin:$PATH" \
  PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages \
  LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/opt/rviz_ogre_vendor/lib \
  AMENT_PREFIX_PATH=/opt/ros/humble ROS_DISTRO=humble ROS_VERSION=2 ROS_PYTHON_VERSION=3 \
  QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms \
  /usr/bin/python3 -s src/view_selected_map.py "$@"
