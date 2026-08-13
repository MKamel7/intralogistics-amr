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
