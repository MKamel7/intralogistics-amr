# Source the ROS and workspace environments safely. Source this, do not run it.
#
#   . tools/ros_env.sh
#
# WHY THIS EXISTS AS A FILE
#
# ROS's setup scripts read variables they have not set, so under `set -u` the
# very first source aborts with:
#
#   /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
#
# and whatever was about to run does not. This has now bitten three separate
# scripts in this project: tools/run_stack.sh, docker-entrypoint.sh, and a
# planner comparison that silently produced nothing for its first attempt.
#
# Each time it was fixed in place with a comment explaining the trap, and each
# time the next script written from scratch reproduced it. A lesson recorded in
# one file is not available to the next file. So the dance lives here once and
# callers source this instead of writing it again.
_ros_env_had_u=0
case "$-" in *u*) _ros_env_had_u=1 ;; esac
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
if [ -f "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash"
fi
[ "$_ros_env_had_u" = 1 ] && set -u
unset _ros_env_had_u
