"""The runner's own preconditions, checked as text.

`tools/run_stack.sh` cannot be unit tested in the ordinary sense: running it
brings up a simulator. What can be tested is that its guards are present, that
they sit before the launch rather than after it, and that the strings they
match are the strings the tools actually print. Every check below exists
because the corresponding fault happened.
"""

import re
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
RUN_STACK = REPO / 'tools' / 'run_stack.sh'


def text():
    return RUN_STACK.read_text()


def test_refuses_to_share_the_domain_before_launching():
    """Two stacks on one domain drive each other and log nothing unusual.

    Preflight catches it, but only after a second simulator has booted. This
    asserts the cheap check runs first: the guard must appear BEFORE the
    bringup launch, or it is not a pre-launch guard.
    """
    t = text()
    guard = t.find("pgrep -xcf 'gz sim server'")
    launch = t.find('ros2 launch amr_bringup')
    assert guard != -1, 'no running-simulator guard in run_stack.sh'
    assert launch != -1, 'could not find the bringup launch'
    assert guard < launch, (
        'the running-simulator guard is after the launch, so it can only fire '
        'once a second simulator is already up, which is the thing it exists '
        'to prevent')


def test_the_guard_matches_what_gazebo_is_actually_called():
    """A guard that greps for the wrong process name is worse than none.

    `gz sim server` is the long-lived process; `gz sim` alone also matches the
    ruby wrapper and this script's own pgrep, and `gzserver` is the name from
    Gazebo Classic, which this project does not use.
    """
    t = text()
    assert 'gzserver' not in t, (
        'gzserver is Gazebo Classic; this project runs Harmonic, so a guard '
        'on that name would never fire')
    assert re.search(r"pgrep -xcf 'gz sim server'", t), (
        'the guard must match the gz sim server process specifically')
    assert "pgrep -cf 'gz sim server'" not in t, (
        'without -x this matches any command line CONTAINING the phrase, '
        'including the invoking shell. Measured: it counted 1 on a machine '
        'with no simulator running, which would block every run.')


def test_the_guard_names_the_way_out():
    """A refusal that does not say what to do next gets worked around."""
    i = text().find("pgrep -xcf 'gz sim server'")
    assert 'tools/stop_all.sh' in text()[i:i + 800], (
        'the guard should tell the reader how to clear the condition')


def test_stop_all_exists_because_the_guard_recommends_it():
    assert (REPO / 'tools' / 'stop_all.sh').exists(), (
        'run_stack.sh points at tools/stop_all.sh when it refuses to start')


def test_active_checks_are_anchored():
    """`grep -q active` matches `inactive`, which is how a retry block that
    existed to catch an inactive collision monitor could never fire. Recorded
    in docs/findings.md. Any lifecycle check here must be anchored.
    """
    for line in text().splitlines():
        if 'lifecycle get' in line or 'grep' in line and 'active' in line:
            if re.search(r"grep\s+-q\w*\s+active", line):
                raise AssertionError(
                    f'unanchored active check, matches "inactive" too:\n  {line.strip()}')


def test_script_is_syntactically_valid():
    """Editing this file while it was running once produced `LATENCY: unbound
    variable` mid-run, because bash reads a script incrementally. Parsing it
    here does not prevent that, but it does catch the edit that broke it.
    """
    r = subprocess.run(['bash', '-n', str(RUN_STACK)],
                       capture_output=True, text=True)
    assert r.returncode == 0, f'bash -n failed:\n{r.stderr}'


def preflight_text():
    return (REPO / 'tools' / 'preflight.py').read_text()


def test_preflight_checks_the_monitor_passes_commands_not_just_that_it_is_active():
    """`active` is a statement about a transition that happened once.

    The collision monitor returned `active [3]` while publishing neither its
    command output nor its state for 30 seconds, with a healthy scan source
    and its input flowing at 16.7 Hz. The vehicle stood still for eight
    minutes and every check in preflight passed. See V-41.
    """
    t = preflight_text()
    assert 'collision monitor passes commands through' in t


def test_that_check_is_conditional_on_there_being_commands():
    """The monitor processes on an incoming command, so with no goal active
    both its output and its state are legitimately silent. An unconditional
    check would fail every healthy bringup, and a check that cries wolf is one
    people learn to ignore."""
    t = preflight_text()
    assert 'no commands in flight, nothing to pass (not a fault)' in t
    assert "in_rate > 0.0" in t


def demo_text():
    return (REPO / 'demo.sh').read_text()


def test_the_front_door_runs_a_world_the_evidence_covers():
    """demo.sh ran the AWS warehouse on a 5 of 5 result that predates the
    protective field change in V-39, and nothing has re-measured it since.
    Pointing the front door at a number the repository can no longer stand
    behind is worse than showing a smaller one.
    """
    t = demo_text()
    assert '--test-track' in t, 'the demo must run the generated track'
    assert '--run survey_mission' in t, (
        'no map ships with the repository, so the demo has to survey before '
        'it can transport')


def test_the_demo_is_honest_about_how_long_it_takes():
    """The old message promised 90 seconds plus 90 per cycle, which was true
    for a pre-built map and is not true for a survey. A front door that
    under-promises time gets closed before it opens."""
    t = demo_text()
    assert '600 + CYCLES * 120' in t
    assert 'survey is most of that time' in t


def test_the_other_world_is_still_reachable():
    """The AWS warehouse is the honest robustness case, a found building nobody
    sized for this vehicle. Repointing the demo must not hide it."""
    assert '--world warehouse' in demo_text()


def stop_all_text():
    return (REPO / 'tools' / 'stop_all.sh').read_text()


def test_teardown_catches_the_orchestrators_not_just_the_stack():
    """run_stack.sh lives in $WS/tools, not $WS/install, so a teardown that
    matched only the install tree stopped the stack and left the script that
    launches it. It carried on to its next stage and started a fresh stack
    into the one just cleared: consecutive experiment runs collided, came up
    with no /scan and no /odom, and were excluded as vehicle failures.
    """
    t = stop_all_text()
    assert '*"$WS/"*' in t, 'the workspace match must cover tools, not only install'
    assert 'ORCHESTRATORS' in t


def test_teardown_spares_every_ancestor_not_just_the_parent():
    """experiment.py calls this through a subprocess shell, so it is the
    grandparent. A pattern broad enough to catch run_stack.sh is broad enough
    to catch the experiment calling it, which would kill its own caller
    halfway through a five run measurement."""
    t = stop_all_text()
    assert 'ANCESTORS' in t
    assert '/proc/$_p/stat' in t, 'ancestry must be walked, not assumed'


def test_there_is_a_process_check_that_does_not_count_itself():
    """`pgrep -f run_stack` matches the shell asking the question.

    That has produced a wrong answer three times in one session: the simulator
    guard counted 1 on a clean machine, a teardown killed the shell about to
    launch an experiment, and a pre-launch check reported an experiment alive
    when none was, seconds before a launch that collided with a real one.
    """
    t = (REPO / 'tools' / 'whats_running.sh').read_text()
    assert 'ANC' in t, 'it must exclude its own ancestry rather than a pattern'
    assert 'whats_running' in t, 'and exclude itself by name too'


def test_the_survey_to_mission_handoff_waits_for_idle_not_a_guess():
    """The survey's last goal is still unwinding when the survey process
    exits, and a mission goal issued into that window is refused with
    "Timed out while waiting for action server to acknowledge goal request".

    Measured on an MiR250 run: it fired 8 seconds into the mission on the very
    first goal and cost cycle 1. It was not load. The worst control loop
    iteration in that entire run was 180 ms against a 1000 ms timeout, and
    nothing exceeded 1000 ms at all.
    """
    t = text()
    assert 'controller idle after survey' in t
    assert 'cmd_vel_nav' in t, 'idleness must be observed, not assumed'


def test_there_is_one_shared_way_to_source_ros():
    """The unbound-variable trap has bitten three separate scripts.

    ROS's setup scripts read variables they have not set, so under `set -u`
    the first source aborts on AMENT_TRACE_SETUP_FILES and whatever was about
    to run does not. It was fixed in run_stack.sh, then reproduced in
    docker-entrypoint.sh, then reproduced again in a planner comparison that
    silently produced nothing.

    Each fix carried a comment explaining the trap, and each time the next
    script written from scratch hit it anyway. A lesson recorded in one file
    is not available to the next file, so the dance lives in tools/ros_env.sh
    and callers source that.
    """
    helper = REPO / 'tools' / 'ros_env.sh'
    assert helper.is_file(), 'tools/ros_env.sh is the one place that lifts -u'
    t = helper.read_text()
    assert 'set +u' in t and 'AMENT_TRACE_SETUP_FILES' in t
    assert 'BASH_SOURCE' in t, 'it must locate the workspace relative to itself'


def test_the_helper_restores_strictness_it_did_not_set():
    """A helper that leaves -u on in a script that never asked for it changes
    the caller's semantics, which is its own kind of surprise."""
    t = (REPO / 'tools' / 'ros_env.sh').read_text()
    assert '_ros_env_had_u' in t, 'it must remember whether -u was on'


def test_a_failed_mission_does_not_report_success():
    """V-40 again, in a different file.

    A run that completed 0 of 3 cycles having driven 0.0 m ended with "mission
    exited 0" and a zero from run_stack.sh, because `ros2 launch` returns 0
    whatever its nodes did. transport_task already returned non-zero for an
    incomplete cycle; nothing carried it out.

    A stage that fails and declares success is worse than one that fails,
    because the next thing measures on top of it.
    """
    t = text()
    assert 'MISSION_RC=$?' in t, (
        'the mission exit code is not captured, so $? has already been '
        'overwritten by the time anything looks at it')
    assert 'exit ${MISSION_RC:-0}' in t, (
        'run_stack.sh does not carry the mission exit code out to the shell')

    # NOT `on_exit=Shutdown()`. That was the first attempt and it does not
    # work: measured directly with a launch file whose process exits 3,
    # `ros2 launch` still returned 0. The test that accompanied it asserted the
    # SOURCE CONTAINED that line, which is true and useless, and is the same
    # mistake as every other check in this project that was correct about its
    # author's intent and blind to whether it did anything.
    #
    # The mission's own summary line is authoritative, so the verdict comes
    # from the log. The helper is executed below rather than grepped for.
    assert 'mission_verdict' in t, (
        'the mission verdict is not taken from the log, so it rests on a '
        'ros2 launch return code that is always 0')


def test_readiness_waits_are_tied_to_this_runs_bringup():
    """A node on the ROS graph is not evidence that THIS run put it there.

    Measured: a bringup died instantly on a bad parameter, its orchestrator sat
    in wait_active for three minutes, a second run was started in the meantime,
    and the first then reported "slam active" and carried on driving the second
    run's stack. Two orchestrators, one simulator, and neither log says
    anything is wrong. Every number either run produced would have been a
    measurement of the other.

    `ros2 lifecycle get` cannot tell them apart, so the wait watches the
    bringup process it belongs to instead.
    """
    t = text()
    assert 'BRINGUP_PID=$!' in t, (
        'the bringup launch pid is not captured, so no wait can tell whether '
        'the stack it is waiting for is still being brought up')

    gate = t.index('wait_active() {')
    body = t[gate:t.index('\n}\n', gate)]
    assert 'kill -0 "$BRINGUP_PID"' in body, (
        'wait_active does not check that the bringup is still alive, so a '
        'dead launch is indistinguishable from a slow one')

    # And the capture must come before the first wait, or it guards nothing.
    assert t.index('BRINGUP_PID=$!') < t.index('wait_active /slam_toolbox'), (
        'the pid is captured after the first readiness wait, which is after '
        'the window it exists to protect')


STOP_ALL = REPO / 'tools' / 'stop_all.sh'


def test_teardown_matches_a_relatively_invoked_orchestrator():
    """The invocation the handover documents is the one teardown could not see.

    stop_all.sh matched a process by looking for the absolute workspace path in
    its command line. `tools/run_stack.sh --run mission`, which is how every
    example in HANDOVER.md and this file's own usage block invokes it, puts no
    absolute path there at all.

    Measured: two orchestrators and their four probes survived a teardown that
    reported "all stopped" having killed 29 processes, and one was found alive
    fifteen hours and fifty one minutes later, still counting as a running
    stack to whats_running.sh. That is the collision this script exists to
    prevent, surviving the script that exists to prevent it.

    Resolving command line tokens against the process's own working directory
    turns `tools/run_stack.sh` back into a path inside the workspace. A plain
    shell sitting in the repository carries no such token, which is why the cwd
    alone is not the test.
    """
    t = STOP_ALL.read_text()
    assert 'readlink -f "/proc/$pid/cwd"' in t, (
        'teardown does not look at the working directory, so it cannot '
        'resolve a relative command line')
    assert 'if [ -e "$cwd/$tok" ]' in t, (
        'teardown does not resolve command line tokens against the cwd, so a '
        'relatively invoked orchestrator is invisible to it')


def test_teardown_still_refuses_to_kill_itself_or_its_ancestors():
    """The guard that must survive every change to the matcher.

    A teardown that matches more processes is one edit away from matching the
    shell that launched it. It has done exactly that once: stop_all.sh killed
    tools/run_stack.sh's own shell mid-run.
    """
    t = STOP_ALL.read_text()
    assert 'ANCESTORS' in t, 'the ancestor exclusion is gone'
    assert "grep -q 'stop_all'" in t, (
        'teardown no longer excludes itself by name, so a wider matcher can '
        'kill the script doing the killing')


def test_the_mission_verdict_helper_actually_works():
    """Run it, do not read it.

    Every previous version of this check asserted that run_stack.sh contained
    some string. A string is not behaviour, and the behaviour was wrong for as
    long as the string was present.
    """
    import subprocess
    import tempfile

    helper = subprocess.run(
        ['sed', '-n', '/^mission_verdict()/,/^}/p', str(RUN_STACK)],
        capture_output=True, text=True, check=True).stdout
    assert 'grep' in helper, 'mission_verdict was not extracted'

    def verdict(contents):
        with tempfile.NamedTemporaryFile('w', suffix='.log', delete=False) as f:
            f.write(contents)
            path = f.name
        r = subprocess.run(['bash', '-c', f'{helper}\nmission_verdict {path} 0'],
                           capture_output=True, text=True)
        return r.stdout.strip()

    assert verdict('3 of 3 cycle(s) completed\n') == '0'
    assert verdict('0 of 3 cycle(s) completed\n') == '1'
    assert verdict('2 of 3 cycle(s) completed\n') == '1'
    # A mission that never reported is not a pass. It is the case that looks
    # most like success in a log nobody reads.
    assert verdict('nothing useful here\n') == '2'
    # The LAST summary wins, because a run can log more than one.
    assert verdict('0 of 3 cycle(s) completed\n3 of 3 cycle(s) completed\n') == '0'
