#!/usr/bin/env bash
# Stop every simulation process, and be sure it actually stopped.
#
# WHY THIS IS A SCRIPT AND NOT A pkill
#
# Two failures kept coming back, and both cost real debugging time.
#
# 1. `pkill -f gz sim` matches the shell running the pkill, because that
#    shell's own command line contains the pattern. The shell dies partway
#    through the list, the rest of the pattern list never runs, and the caller
#    sees a confusing exit 144 while half the stack is still alive.
#
# 2. A surviving ros_gz_bridge from an earlier launch keeps publishing /clock.
#    With two clock publishers, simulated time jumps backwards, every node
#    logs "Detected jump back in time. Clearing TF buffer." several times a
#    second, and navigation cannot hold a transform long enough to plan. This
#    is invisible unless you go looking: each launch appears healthy on its
#    own. Diagnosed by `ros2 topic info /clock` reporting Publisher count: 2.
#
# So: collect the PIDs FIRST, filter this script and its parents out by PID,
# then signal. Verify afterwards, and say plainly what is left.
set -uo pipefail

SELF=$$
WS="$(cd "$(dirname "$0")/.." && pwd)"

# THE PATTERN LIST IS DERIVED, NOT HAND MAINTAINED, and the reason is that the
# hand maintained one was wrong for most of this project's life.
#
# It listed the simulator, the bridges and the Nav2 servers, and it silently
# omitted every node this workspace builds itself: leg_detector, people_tracker,
# scan_merger, battery_model, and the rest. Each restart therefore left those
# behind. They accumulated across dozens of restarts until `ros2 node list`
# showed 19 battery_model, 6 leg_detector and 5 people_tracker nodes alive at
# once, all burning CPU and all publishing onto the same topics. Load average
# reached 47 on 12 cores and the MPPI control loop starved down to 2.5 Hz
# against a required 20 Hz, which presented as a navigation tuning problem.
#
# So: anything executing out of this workspace's install tree is ours and gets
# stopped, whatever it happens to be called. Only the third-party processes
# need naming, and those are a short and stable list.
EXTERNAL='gz sim|ros_gz_bridge|parameter_bridge|ros_gz_sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor|lifecycle_manager|robot_state_publisher|ros2_control_node|spawner|component_container'

collect() {
  {
    # Everything running out of our own install tree. TWO tests are needed,
    # and using only the first left thirty processes alive.
    #
    #   The executable path catches C++ nodes, whose /proc/PID/exe resolves
    #   into the install tree.
    #
    #   The COMMAND LINE catches Python nodes, whose exe is /usr/bin/python3.
    #   Resolving the executable of a Python node tells you about the
    #   interpreter, not about the node, so every Python node in this
    #   workspace survived the previous version of this script: battery_model,
    #   pedestrian_driver, ground_truth_publisher, truth_map_publisher and
    #   survey_runner. They accumulated to 19 battery_model nodes on one graph.
    for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
      exe=$(readlink -f "/proc/$pid/exe" 2>/dev/null)
      case "$exe" in "$WS"/*) echo "$pid"; continue ;; esac
      # 2>/dev/null because processes exit while this loop is walking /proc,
      # and a vanished pid is the normal case here, not an error.
      cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
      [ -z "$cmd" ] && continue
      case "$cmd" in *"$WS/install/"*) echo "$pid" ;; esac
    done
    # Plus the third-party processes, matched on their command lines.
    pgrep -f "$EXTERNAL" 2>/dev/null
  } | sort -u | while read -r pid; do
        [ -z "$pid" ] && continue
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PPID" ] && continue
        [ -r "/proc/$pid/cmdline" ] || continue
        # Never match this script itself, or an editor holding it open.
        tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q 'stop_all' && continue
        echo "$pid"
      done
}

pids=$(collect)
if [ -z "$pids" ]; then
  echo "nothing running"
  exit 0
fi

echo "stopping $(echo "$pids" | wc -l) process(es)"
# shellcheck disable=SC2086
kill -TERM $pids 2>/dev/null

# Give them a chance to shut down cleanly. The pedestrian driver in particular
# publishes a zero twist on the way out, and skipping that leaves the walkers
# coasting at their last commanded speed for as long as the world lives.
for _ in $(seq 1 20); do
  sleep 0.5
  [ -z "$(collect)" ] && break
done

left=$(collect)
if [ -n "$left" ]; then
  echo "forcing $(echo "$left" | wc -l) survivor(s)"
  # shellcheck disable=SC2086
  kill -KILL $left 2>/dev/null
  sleep 1
fi

left=$(collect)
if [ -n "$left" ]; then
  echo "STILL RUNNING:"
  # shellcheck disable=SC2086
  ps -o pid=,args= -p $left | cut -c1-120
  exit 1
fi

echo "all stopped"
exit 0
