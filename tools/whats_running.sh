#!/usr/bin/env bash
# What of this project is running, without counting the question itself.
#
# WHY THIS EXISTS
#
# `pgrep -f run_stack` and `ps | grep -E 'experiment|gz sim'` both match the
# shell asking the question, because that shell's command line contains the
# pattern. This has produced a wrong answer three times in one session:
#
#   the simulator guard counted 1 on a clean machine and would have blocked
#   every run in the project,
#
#   a teardown killed the shell that was about to launch an experiment,
#
#   and a pre-launch check reported an experiment and a run_stack alive when
#   neither was, seconds before a launch that then collided with a real
#   experiment nobody had checked for.
#
# The fix is to compare PIDs against this script's own ancestry rather than to
# hope the pattern is specific enough.
set -uo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

ANC=" "
_p=$$
while [ "$_p" != "1" ] && [ -r "/proc/$_p/stat" ]; do
  ANC="$ANC$_p "
  _p=$(awk '{print $4}' "/proc/$_p/stat" 2>/dev/null) || break
  [ -z "$_p" ] && break
done

count() {  # pattern, label
  local n=0
  for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
    case "$ANC" in *" $pid "*) continue ;; esac
    # The redirect itself fails noisily when a process exits mid walk, and a
    # vanished pid is the normal case here rather than an error, so the whole
    # read is wrapped instead of just the command inside it.
    cmd=$( { tr '\0' ' ' < "/proc/$pid/cmdline"; } 2>/dev/null ) || continue
    [ -z "$cmd" ] && continue
    case "$cmd" in *whats_running*) continue ;; esac
    case "$cmd" in $1) n=$((n+1)) ;; esac
  done
  printf '  %-22s %d\n' "$2" "$n"
}

count '*tools/experiment.py*' 'experiment.py'
count '*run_stack.sh*'        'run_stack.sh'
count 'gz sim server*'        'gz sim server'
count "*$WS/install/*"        'stack nodes'
