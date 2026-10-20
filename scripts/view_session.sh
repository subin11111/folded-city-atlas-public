#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "$workspace"
exec /usr/bin/python3 src/view_session.py "$@"
