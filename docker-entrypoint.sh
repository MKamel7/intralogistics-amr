#!/usr/bin/env bash
# Source the workspace, then do what was asked.
#
# `pytest` with no arguments is the default because the test suite is the part
# of this project a reader can check in ninety seconds without a simulator, and
# it is the part that carries the measurements: a test here fails when a
# constant loses its provenance, when a protective field falls inside the
# sensor's blind zone, or when a config that must be generated per platform
# stops being.
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws/src/intralogistics-amr

if [ "${1:-}" = "pytest" ]; then
  shift
  exec python3 -m pytest src -q "$@"
fi
exec "$@"
