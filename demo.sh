#!/usr/bin/env bash
# One command, one result table. This is the front door.
#
# WHY THIS IS AT THE TOP LEVEL AND NOT IN tools/
#
# Someone evaluating this repository gives it about five minutes. If they cannot
# get a robot moving in that time they close the tab, and everything else in
# here becomes irrelevant. `tools/run_stack.sh` is the real instrument, with a
# dozen flags and a preflight gate; this is the version with no decisions in it.
#
# It runs the KNOWN GOOD configuration deliberately: the MiR250 on the imported
# warehouse, cameras off. That is the path measured at 5 of 5 transport cycles.
# The test track and the second platform are more interesting and are one flag
# away, but a front door should open.
set -uo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO"

CYCLES="${1:-2}"

echo
echo "  Intralogistics AMR: transport demo"
echo "  =================================="
echo

# FAIL WITH A SENTENCE, not a stack trace. Every check below has been the thing
# that actually went wrong for someone at least once.
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "  ROS 2 Jazzy is not installed at /opt/ros/jazzy."
  echo "  See the Build section of README.md."
  exit 1
fi

if ! command -v gz >/dev/null 2>&1; then
  echo "  Gazebo (gz) is not on PATH. Harmonic is required; see README.md."
  exit 1
fi

if [ ! -f install/setup.bash ]; then
  echo "  The workspace is not built yet. Building now, this takes a few minutes."
  echo
  # shellcheck disable=SC1091
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  colcon build --symlink-install || {
    echo; echo "  Build failed. The output above says why."; exit 1; }
  echo
fi

echo "  Running $CYCLES transport cycle(s) on the MiR250, cameras off."
echo "  Expect about $(( 90 + CYCLES * 90 )) seconds: the stack brings itself up,"
echo "  checks its own health, then fetches and delivers a load while"
echo "  pedestrians move around it."
echo
echo "  A window will open. Nothing needs clicking."
echo

tools/run_stack.sh --cameras off --run mission --cycles "$CYCLES"
STATUS=$?

RUN=$(readlink -f /tmp/amr-logs/latest 2>/dev/null)
echo
echo "  =================================="
if [ -n "$RUN" ] && [ -f "$RUN/mission.log" ]; then
  # The transport task already prints a summary block. Showing its own words
  # rather than reformatting them keeps one source for the numbers.
  sed -n '/====/,/====/p' "$RUN/mission.log" | sed 's/^.*transport_task]: //' \
    | sed 's/^/  /'
  echo
  echo "  Full logs: $RUN"
else
  echo "  The run produced no mission log. Logs, if any: ${RUN:-none}"
fi

echo
echo "  What to look at next:"
echo "    docs/validation.md          what was measured, and what turned out false"
echo "    HANDOVER.md                 current state, honestly"
echo "    tools/run_stack.sh --help   the real instrument"
echo

exit $STATUS
