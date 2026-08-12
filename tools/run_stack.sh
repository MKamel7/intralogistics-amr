#!/usr/bin/env bash
# Bring up the stack, optionally run something against it, and keep the logs.
#
# WHY THIS IS IN THE REPO AND NOT A SCRATCH FILE
#
# For most of this project's life the bring-up sequence lived as an ad hoc
# script in /tmp, rewritten whenever something needed changing. Two costs came
# out of that and both were paid repeatedly.
#
# The logs were fixed filenames, overwritten by every run. A run started while
# a previous one was still shutting down would read the old file and report the
# old result. That happened three times in one session, and once a previous
# run's numbers were nearly reported as a new measurement. Every run now gets
# its own timestamped directory and `latest` points at the newest, so a stale
# read is impossible rather than merely unlikely.
#
# The sequence itself was not versioned, so the gate that refuses to measure an
# unhealthy system existed in one copy of the script and not in another. A
# mission duly ran on a stack that had failed its own preflight.
#
# THE ORDER MATTERS. Each stage waits for the previous one to be genuinely
# ready. Starting them together overloads the lifecycle services and the
# collision monitor gets left INACTIVE, which silently breaks the command chain
# to the wheels.
#
# Usage:
#   tools/run_stack.sh                        bring up and hold
#   tools/run_stack.sh --cameras off          fleet tier, cheaper
#   tools/run_stack.sh --run survey           bring up, then survey
#   tools/run_stack.sh --run mission          bring up, then transport task
#   tools/run_stack.sh --run mission --classify   also attribute safety stops
#   tools/run_stack.sh --no-gate              measure anyway if preflight fails
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

CAMERAS=true
RVIZ=true
TASK=none
CLASSIFY=false
TRACK=false
TFTRACK=false
GATE=true
CYCLES=2

while [ $# -gt 0 ]; do
  case "$1" in
    --cameras) [ "${2:-}" = off ] && CAMERAS=false; shift 2 ;;
    --rviz)    [ "${2:-}" = off ] && RVIZ=false; shift 2 ;;
    --run)     TASK="${2:-none}"; shift 2 ;;
    --cycles)  CYCLES="${2:-2}"; shift 2 ;;
    --classify) CLASSIFY=true; shift ;;
    --track)   TRACK=true; shift ;;
    --tf)      TFTRACK=true; shift ;;
    --no-gate) GATE=false; shift ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

# A directory per run, named for when it started. `latest` is a symlink, so
# tooling can follow it without ever reading a file from a different run.
ROOT=/tmp/amr-logs
RUN="$ROOT/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN"
ln -sfn "$RUN" "$ROOT/latest"
echo "logs: $RUN  (also $ROOT/latest)"

say() { echo "$(date +%T) $*" | tee -a "$RUN/stage.log"; }

# ROS's setup scripts read variables they have not set, so `set -u` has to be
# lifted across them. Leaving it on aborts the whole script on
# "AMENT_TRACE_SETUP_FILES: unbound variable" before anything starts.
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

wait_active() {  # node, timeout seconds
  local end=$((SECONDS + ${2:-180}))
  until timeout 8 ros2 lifecycle get "$1" 2>/dev/null | grep -q active; do
    [ $SECONDS -gt $end ] && return 1
    sleep 3
  done
}

say "starting: cameras=$CAMERAS rviz=$RVIZ task=$TASK"
ros2 launch amr_bringup robot.launch.py \
     gui:=true rviz:=$RVIZ cameras:=$CAMERAS > "$RUN/robot.log" 2>&1 &

wait_active /slam_toolbox 200 && say "slam active" || { say "SLAM FAILED"; exit 1; }

# The monitor is safety critical, so it gets an explicit retry rather than a
# silent skip. Its activation has timed out under load before, and a stack
# without it forwards nothing to the wheels.
if ! wait_active /collision_monitor 120; then
  say "collision_monitor not active, retrying activation"
  timeout 20 ros2 lifecycle set /collision_monitor configure >/dev/null 2>&1
  timeout 20 ros2 lifecycle set /collision_monitor activate  >/dev/null 2>&1
  wait_active /collision_monitor 60 || { say "COLLISION MONITOR INACTIVE"; exit 1; }
fi
say "collision_monitor active"

sleep 10
ros2 launch amr_navigation navigation.launch.py > "$RUN/nav.log" 2>&1 &
wait_active /bt_navigator 200 && say "nav2 active" || { say "NAV2 FAILED"; exit 1; }

sleep 10
ros2 launch amr_sim people.launch.py scenario:=walking_people > "$RUN/people.log" 2>&1 &
sleep 15
say "people spawned"

python3 tools/preflight.py > "$RUN/preflight.log" 2>&1
PF=$?
say "preflight exit $PF"
if [ $PF -ne 0 ]; then
  grep '^  \[FAIL\]' "$RUN/preflight.log" | tee -a "$RUN/stage.log"
  if [ "$GATE" = true ]; then
    # THE GATE. Measuring an unhealthy stack produces numbers that describe the
    # fault rather than the system, and they are indistinguishable from real
    # results after the fact. Override with --no-gate only when the failure is
    # understood and irrelevant to what is being measured.
    say "PREFLIGHT FAILED, refusing to measure. Use --no-gate to override."
    exit 1
  fi
  say "preflight failed but --no-gate was given, continuing"
fi

if [ "$TRACK" = true ]; then
  python3 -u tools/track_goal.py --ros-args -p duration_s:=400.0 \
          > "$RUN/goal.log" 2>&1 &
  TRK=$!
fi

if [ "$TFTRACK" = true ]; then
  python3 -u tools/track_map_odom.py --ros-args -p duration_s:=600.0 \
          > "$RUN/map_odom.log" 2>&1 &
  TFT=$!
fi

if [ "$CLASSIFY" = true ]; then
  python3 tools/classify_stops.py --ros-args -p duration_s:=400.0 \
          > "$RUN/stops.log" 2>&1 &
  CLS=$!
fi

case "$TASK" in
  survey)
    ros2 run amr_navigation survey_runner --ros-args -p use_sim_time:=true \
        > "$RUN/survey.log" 2>&1
    say "survey exited $?" ;;
  mission)
    ros2 launch amr_mission transport.launch.py cycles:=$CYCLES \
        > "$RUN/mission.log" 2>&1
    say "mission exited $?" ;;
  none)
    say "stack up, holding. Ctrl-C to stop, or tools/stop_all.sh"
    while true; do sleep 60; done ;;
  *) say "unknown task $TASK"; exit 2 ;;
esac

if [ "$CLASSIFY" = true ]; then
  wait ${CLS:-} 2>/dev/null
  say "classifier done"
fi
if [ "$TRACK" = true ]; then
  wait ${TRK:-} 2>/dev/null
  say "tracker done"
fi
if [ "$TFTRACK" = true ]; then
  # The tf tracker runs to its own duration; do not wait the full ten minutes
  # if the mission finished early, just stop it and take what it has.
  kill -INT ${TFT:-} 2>/dev/null
  wait ${TFT:-} 2>/dev/null
  say "tf tracker done"
fi
say "run complete"
