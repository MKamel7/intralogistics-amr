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
# It runs the GENERATED TEST TRACK, cameras off. That is the world every recent
# measurement covers: derived aisle widths, moving pedestrians on routes and
# crossings, painted delivery bays, and a ground truth oracle for scoring.
#
# It used to run the imported AWS warehouse on the strength of a 5 of 5 result.
# That result predates the protective field change in V-39 and nothing has
# re-measured it since, so pointing the front door at it would be showing a
# number this repository can no longer stand behind. The AWS world is still one
# flag away and is still the honest robustness case, a found building nobody
# sized for this vehicle.
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

echo "  Running $CYCLES transport cycle(s) on the generated track, cameras off."
echo "  Expect about $(( 600 + CYCLES * 120 )) seconds. The vehicle surveys the"
echo "  building first, because a map is not shipped with the repository, then"
echo "  fetches and delivers a load while people walk routes and cross in front"
echo "  of it. The survey is most of that time and it only happens once."
echo
echo "  Add --world warehouse to run the imported AWS building instead."
echo
echo "  A window will open. Nothing needs clicking."
echo

tools/run_stack.sh --test-track --cameras off --run survey_mission --cycles "$CYCLES"
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
