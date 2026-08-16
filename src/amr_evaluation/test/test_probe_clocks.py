"""Every probe that reads a message stamp runs on the simulated clock.

WHY THIS EXISTS

`measure_control_latency.py` splits each sample at the collision monitor: the
part before the monitor announced a stop, and the part after. The first half
is a clock reading minus a message stamp. The node did not set `use_sim_time`,
so the clock returned epoch seconds while every stamp came from a node running
on sim time, and the "sensor half" printed as 1786870613342 ms.

That is not a subtle error, and it still survived a full run and a test suite,
because the number it corrupted is one nobody had a prior expectation for. The
original metric in the same file was unaffected: it subtracts one message
stamp from another, and two wrong-but-consistent clocks cancel.

Four probes in tools/ already set it. Two did not. Nothing compared them.

This walks tools/ rather than naming files, so a new probe is covered the day
it is written.
"""

import re
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[3] / 'tools'

# A probe is anything that constructs an rclpy Node. Scripts that only read
# files or drive the shell are not probes and are skipped by that test.
def probes():
    out = []
    for p in sorted(TOOLS.glob('*.py')):
        text = p.read_text()
        if 'rclpy.init' in text and 'Node' in text:
            out.append(p)
    return out


@pytest.mark.parametrize('path', probes(), ids=lambda p: p.stem)
def test_the_probe_runs_on_the_simulated_clock(path):
    text = path.read_text()
    assert "Parameter('use_sim_time', value=True)" in text, (
        f'{path.name} constructs a ROS node without setting use_sim_time, so '
        f'self.get_clock() returns wall time while every message stamp it '
        f'reads is simulated time. Any subtraction across the two is wrong by '
        f'the epoch')


@pytest.mark.parametrize('path', probes(), ids=lambda p: p.stem)
def test_the_override_is_on_the_node_not_just_imported(path):
    """Importing Parameter is not setting it.

    The override has to reach super().__init__, which is the only place rclpy
    reads it early enough to affect the clock the node builds.
    """
    text = path.read_text()
    i = text.find('super().__init__(')
    assert i != -1, f'{path.name} has no recognisable Node constructor call'
    # The override has to be inside the constructor call. A generous window
    # rather than a brace matcher, because the alternative is parsing Python
    # with a regex and the failure being guarded is coarse.
    assert 'use_sim_time' in text[i:i + 800], (
        f'{path.name} mentions use_sim_time somewhere but not in the Node '
        f'constructor, where it has to be to affect the clock')


def test_there_is_at_least_one_probe_to_check():
    """Otherwise the parametrised tests above pass by collecting nothing.

    An empty parametrisation is a green tick that checked nothing, which is
    the failure mode V-50 was about.
    """
    assert len(probes()) >= 4, (
        f'only {len(probes())} probes found in {TOOLS}; the discovery rule '
        f'has probably stopped matching')
