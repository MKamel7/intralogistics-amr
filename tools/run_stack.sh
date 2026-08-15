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
#   tools/run_stack.sh --run survey_mission   survey first, then transport
#   tools/run_stack.sh --run mission --classify   also attribute safety stops
#   tools/run_stack.sh --run mission --latency    also measure control_latency
#   tools/run_stack.sh --no-gate              measure anyway if preflight fails
#   tools/run_stack.sh --platform mp400_class the second platform
#   tools/run_stack.sh --test-track            the datasheet-sized test track
#   tools/run_stack.sh --world <name>          any world in amr_sim/worlds
#
# TWO WORLDS, DIFFERENT JOBS. The default is the AWS RoboMaker warehouse: a
# found building nobody sized for this vehicle, which is the honest robustness
# case. `--track` selects the generated test track whose aisle widths ARE the
# corridor figures the platform datasheet publishes, so a cycle measures the
# vehicle against its own manufacturer's claims instead of against whatever
# geometry happened to be in the asset pack. Both get run; neither replaces the
# other.
#
# THE PLATFORM IS PASSED TO EVERY LAUNCH THAT NEEDS IT, and that is the point of
# having it here. The robot description, the protective fields and the whole
# Nav2 configuration are each generated per platform, so all three have to be
# told the same name. Passing it to one of them and not the others is exactly
# the failure this argument exists to prevent.
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

CAMERAS=true
RVIZ=true
PLATFORM=mir250_class
WORLD=warehouse
TRACK_WORLD=false
KEEPOUT=keepout_mask
STATIONS=
SCENARIO=walking_people
TRUTH_MAP=warehouse_truth
SPAWN_X=2.0
SPAWN_Y=-1.0
TASK=none
CLASSIFY=false
TRACK=false
TFTRACK=false
LATENCY=false
GATE=true
CYCLES=2

while [ $# -gt 0 ]; do
  case "$1" in
    --cameras) [ "${2:-}" = off ] && CAMERAS=false; shift 2 ;;
    --platform) PLATFORM="${2:?--platform needs a name}"; shift 2 ;;
    --world)   WORLD="${2:?--world needs a name}"; TRACK_WORLD=false; shift 2 ;;
    --test-track) TRACK_WORLD=true; shift ;;
    --rviz)    [ "${2:-}" = off ] && RVIZ=false; shift 2 ;;
    --run)     TASK="${2:-none}"; shift 2 ;;
    --cycles)  CYCLES="${2:-2}"; shift 2 ;;
    --classify) CLASSIFY=true; shift ;;
    --track)   TRACK=true; shift ;;
    --tf)      TFTRACK=true; shift ;;
    --latency) LATENCY=true; shift ;;
    --no-gate) GATE=false; shift ;;
    -h|--help)
      # The usage block is the comment header of this file, so there is one
      # copy of it and it cannot drift from the flags below.
      sed -n '/^# Usage:/,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \?//'
      exit 0 ;;
    *) echo "unknown option: $1"; echo "try --help"; exit 2 ;;
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
  # ANCHORED, and this was a diagnostic that lied. `ros2 lifecycle get` prints
  # "active [3]" or "inactive [2]", and the test here used to be `grep -q
  # active`, which matches INACTIVE. So a node that failed to activate was
  # reported as active, immediately, on the first poll.
  #
  # It mattered most on the node it could least afford to lie about. The
  # collision monitor's retry block below exists precisely to catch an inactive
  # monitor, and it could never fire: the check that guards it was satisfied by
  # the word it was looking for. Measured on an MP-400 bringup: the script said
  # "collision_monitor active" at 18:41:25 and preflight found it inactive
  # sixty seconds later, having never been anything else.
  local end=$((SECONDS + ${2:-180}))
  until timeout 8 ros2 lifecycle get "$1" 2>/dev/null | grep -qE '^active'; do
    [ $SECONDS -gt $end ] && return 1
    sleep 3
  done
}

if [ "$TRACK_WORLD" = true ]; then
  WORLD="test_track.$PLATFORM"
  # The open bay, west of the racking, so the vehicle starts with room and a
  # slow departure is its own fault rather than the building's.
  SPAWN_X=2.5
  SPAWN_Y=6.0
  # PER PLATFORM, like everything else derived from the track geometry. Both
  # of these were single files written by whichever platform was generated
  # last, and both differ between platforms: the truth map because the
  # building depth is derived from the vehicle's turning width, the keepout
  # mask because it is sized to the same building. Sixth and seventh instance
  # of that shape, after controllers.yaml and track_people.yaml.
  KEEPOUT=keepout_mask_test_track.$PLATFORM
  STATIONS="$REPO/src/amr_mission/config/stations.test_track.$PLATFORM.yaml"
  SCENARIO=track_people.$PLATFORM
  TRUTH_MAP=test_track_truth.$PLATFORM
  # THE SPAWN COMES FROM THE GENERATED STATIONS FILE, not from a number typed
  # here. Station coordinates are in the map frame, whose origin SLAM puts at
  # the vehicle's start pose, so spawning anywhere else silently shifts every
  # goal by the difference. One generator owns both, and this reads it back.
  SPAWN_X=$(awk '/^spawn:/{f=1;next} f&&/^  x:/{print $2;exit}' "$STATIONS")
  SPAWN_Y=$(awk '/^spawn:/{f=1;next} f&&/^  y:/{print $2;exit}' "$STATIONS")
  [ -n "$SPAWN_X" ] && [ -n "$SPAWN_Y" ] || {
    echo "could not read the spawn pose from $STATIONS"; exit 2; }
fi

# FAIL BEFORE LAUNCHING, not thirty seconds in. A platform whose generated
# configurations are missing brings the stack up far enough to look like it is
# working and then leaves a lifecycle node stuck in unconfigured, which reports
# only as "failed to change state".
for f in "src/amr_navigation/config/nav2.$PLATFORM.yaml" \
         "src/amr_safety/config/collision_monitor.$PLATFORM.yaml" \
         "src/amr_description/config/platforms/$PLATFORM.yaml" \
         "src/amr_sim/worlds/$WORLD.sdf" \
         ${TRACK_WORLD:+"src/amr_sim/scenarios/$SCENARIO.yaml"}; do
  [ -f "$f" ] || { echo "no $f; is $PLATFORM a platform, and has it been generated?"; exit 2; }
done

# REFUSE TO SHARE THE ROS DOMAIN.
#
# Preflight already checks for a single /clock publisher, and it works: it
# caught this exact fault. But it only runs once the whole stack is up, so by
# then a second simulator has booted and two minutes are gone.
#
# The reason this is worth a pre-launch gate rather than trusting preflight is
# what a second stack does while it is up. Both publish /scan, /tf, /map and
# /cmd_vel, so the two vehicles drive each other, and every measurement taken
# during that window is meaningless in a way that looks entirely ordinary in
# the logs.
#
# Measured: a MiR250 stack from a completed run was still up three hours and
# forty minutes later, because nothing had torn it down. An MP-400 run launched
# beside it inherited two /clock publishers and six nodes it did not own.
# -x, so the WHOLE command line must equal `gz sim server`, which is exactly
# what that process is called. Without -x this matches any process whose
# command line merely contains the words, including the shell that invoked
# this script when the phrase appears anywhere in its command. The first
# version of this guard did that and reported a stale simulator on a clean
# machine, which would have blocked every run in the project.
STALE=$(pgrep -xcf 'gz sim server' || true)
if [ "${STALE:-0}" -gt 0 ]; then
  echo "a simulator is already running ($STALE process(es))."
  echo "Two stacks on one ROS domain corrupt each other silently. Stop it with:"
  echo "    tools/stop_all.sh"
  exit 2
fi

say "starting: platform=$PLATFORM world=$WORLD cameras=$CAMERAS rviz=$RVIZ task=$TASK"
ros2 launch amr_bringup robot.launch.py platform:=$PLATFORM world:=$WORLD \
     x:=$SPAWN_X y:=$SPAWN_Y \
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
ros2 launch amr_navigation navigation.launch.py platform:=$PLATFORM \
    keepout_mask:=$KEEPOUT > "$RUN/nav.log" 2>&1 &

# THE KEEPOUT FILTER GETS THE SAME EXPLICIT RETRY THE MONITOR GETS, and for the
# same reason: when it fails it does so silently. filter_mask_server reads a map
# from disk on configure and has repeatedly missed its manager's service
# timeout under load. When it does, the mask is never published, both costmaps
# run with NO keepout zones, and the vehicle plans through floor that was
# declared forbidden before it was switched on. One five-cycle run completed
# that way with 441 warnings and every preflight check passing. See V-25.
for n in /filter_mask_server /costmap_filter_info_server; do
  if ! wait_active $n 90; then
    say "$n not active, retrying activation"
    timeout 20 ros2 lifecycle set $n configure >/dev/null 2>&1
    timeout 20 ros2 lifecycle set $n activate  >/dev/null 2>&1
    wait_active $n 60 || { say "KEEPOUT FILTER INACTIVE: $n"; exit 1; }
  fi
done
say "keepout filter active"

wait_active /bt_navigator 200 && say "nav2 active" || { say "NAV2 FAILED"; exit 1; }

sleep 10
ros2 launch amr_sim people.launch.py scenario:=$SCENARIO \
    world:=$WORLD truth_map:=$TRUTH_MAP > "$RUN/people.log" 2>&1 &
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
  # ALL THE PROBES OUTLIVE A SURVEY. At 400 s these expired partway through
  # exploration and heard nothing of the mission that followed, which reads as
  # a quiet run rather than as a probe that had stopped listening. The latency
  # probe was caught doing exactly that: 600 s window, mission started at 1400.
  python3 -u tools/track_goal.py --ros-args -p duration_s:=2400.0 \
          > "$RUN/goal.log" 2>&1 &
  TRK=$!
fi

if [ "$TFTRACK" = true ]; then
  python3 -u tools/track_map_odom.py --ros-args -p duration_s:=2400.0 \
          > "$RUN/map_odom.log" 2>&1 &
  TFT=$!
fi

if [ "$CLASSIFY" = true ]; then
  python3 -u tools/classify_stops.py --ros-args -p duration_s:=2400.0 \
          > "$RUN/stops.log" 2>&1 &
  CLS=$!
fi

# THE NUMBER A SAFETY ASSESSOR ASKS ABOUT FIRST. control_latency feeds every
# protective field in the stack and is still an estimate on both platforms.
# This measures it passively from whatever the run does anyway.
if [ "$LATENCY" = true ]; then
  # LONG ENOUGH TO OUTLIVE A SURVEY. At 600 s the probe expired at 03:01 on a
  # run whose mission did not start until 03:04, so all sixteen protective
  # stops happened after it had stopped listening and it correctly reported no
  # samples. The probe was right; the window was wrong.
  python3 -u tools/measure_control_latency.py --ros-args -p duration_s:=2400.0 \
          > "$RUN/latency.log" 2>&1 &
  LAT=$!
fi

case "$TASK" in
  survey)
    # THE SURVEY IS TOLD WHICH STATIONS IT MUST COVER. Without this it can
    # declare a building surveyed while a delivery station is still off the
    # map, and every goal afterwards is rejected as outside bounds. Measured:
    # a survey stopped at 295.1 m2 of a 544 m2 building and the mission scored
    # 0 of 3 with the survey reporting success. See V-40.
    ros2 run amr_navigation survey_runner --ros-args -p use_sim_time:=true \
        ${STATIONS:+-p stations_file:="$STATIONS"} \
        > "$RUN/survey.log" 2>&1
    say "survey exited $?" ;;
  survey_mission)
    # SURVEY, THEN TRANSPORT, in one bring-up.
    #
    # The vehicle plans only on floor it has surveyed, `allow_unknown: false`,
    # which V-16 argued for at length and which is the right decision. The
    # consequence is that a transport goal beyond the opening scans is refused
    # before it is attempted:
    #
    #   Goal Coordinates of(17.000000, 2.375000) was outside bounds
    #
    # The AWS warehouse tolerates a cold mission because its stations sit inside
    # what the vehicle can see from its start pose. The test track is 24 m long
    # and its dispatch station is not. So the track is surveyed first, in the
    # same run, against the same map, rather than loosening the planner.
    # THE SURVEY IS TOLD WHICH STATIONS IT MUST COVER. Without this it can
    # declare a building surveyed while a delivery station is still off the
    # map, and every goal afterwards is rejected as outside bounds. Measured:
    # a survey stopped at 295.1 m2 of a 544 m2 building and the mission scored
    # 0 of 3 with the survey reporting success. See V-40.
    ros2 run amr_navigation survey_runner --ros-args -p use_sim_time:=true \
        ${STATIONS:+-p stations_file:="$STATIONS"} \
        > "$RUN/survey.log" 2>&1
    say "survey exited $?"
    # WAIT FOR THE CONTROLLER TO GO IDLE, do not sleep a guessed interval.
    #
    # The survey's last navigation goal is still unwinding when the survey
    # process exits, and a mission goal issued into that window is refused:
    #
    #   Timed out while waiting for action server to acknowledge goal request
    #   for follow_path
    #
    # Measured on an MiR250 run: that fired 8 seconds into the mission, on the
    # very first goal, and cost cycle 1. It was NOT load. The worst control
    # loop iteration in the whole run was 180 ms against a 1000 ms timeout and
    # nothing exceeded 1000 ms at all, so the controller was not busy, it was
    # mid transition.
    #
    # Idle means no velocity command for three consecutive seconds.
    quiet=0
    for _ in $(seq 1 40); do
      if timeout 2 ros2 topic echo /cmd_vel_nav --once >/dev/null 2>&1; then
        quiet=0
      else
        quiet=$((quiet + 1))
        [ "$quiet" -ge 3 ] && break
      fi
      sleep 1
    done
    say "controller idle after survey (${quiet}s quiet)"
    ros2 launch amr_mission transport.launch.py cycles:=$CYCLES \
        platform:=$PLATFORM stations_file:="$STATIONS" > "$RUN/mission.log" 2>&1
    say "mission exited $?" ;;
  mission)
    ros2 launch amr_mission transport.launch.py cycles:=$CYCLES \
        platform:=$PLATFORM stations_file:="$STATIONS" > "$RUN/mission.log" 2>&1
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
if [ "$LATENCY" = true ]; then
  kill -INT ${LAT:-} 2>/dev/null
  wait ${LAT:-} 2>/dev/null
  say "latency probe done; see $RUN/latency.log"
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
