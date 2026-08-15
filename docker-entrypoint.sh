#!/usr/bin/env bash
# Source the workspace, then do what was asked.
#
# `pytest` with no arguments is the default because the test suite is the part
# of this project a reader can check in ninety seconds without a simulator, and
# it is the part that carries the measurements: a test here fails when a
# constant loses its provenance, when a protective field falls inside the
# sensor's blind zone, or when a config that must be generated per platform
# stops being.
set -eo pipefail

# SET -u IS LIFTED ACROSS THE ROS SETUP SCRIPTS, deliberately.
#
# They read variables they have not set, so with -u in force the very first
# source aborts on "AMENT_TRACE_SETUP_FILES: unbound variable" and the
# container exits before running anything. tools/run_stack.sh documents this
# and does the same thing; writing the entrypoint fresh reproduced the fault
# anyway, which is what a documented trap in one file and not the other buys.
set +u
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u
cd /ws/src/intralogistics-amr

if [ "${1:-}" = "pytest" ]; then
  shift
  exec python3 -m pytest src -q "$@"
fi
exec "$@"
