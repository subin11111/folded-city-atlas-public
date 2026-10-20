#!/usr/bin/env bash
# Visualization only. No graph acceptance flags or candidate poses are changed.
set -eo pipefail
preview_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "$preview_workspace"
exec /usr/bin/python3 src/view_joint_preview.py "$@"
