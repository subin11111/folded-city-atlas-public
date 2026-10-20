#!/usr/bin/env bash
# Visualization only: no pose estimation or graph-acceptance changes.
set -eo pipefail
preview_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "$preview_workspace"
exec /usr/bin/python3 src/view_all_sessions_preview.py "$@"
