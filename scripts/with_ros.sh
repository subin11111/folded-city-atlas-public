#!/usr/bin/env bash
# Process-scoped Humble environment; never change the user's conda/base shell.
set -euo pipefail
replay_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$replay_workspace"
exec /usr/bin/env \
 PATH="/opt/ros/humble/bin:$PATH" \
 PYTHONPATH="$replay_workspace/src:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}" \
 LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/opt/rviz_ogre_vendor/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
 AMENT_PREFIX_PATH=/opt/ros/humble ROS_DISTRO=humble ROS_VERSION=2 ROS_PYTHON_VERSION=3 \
 QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms \
 "$@"
