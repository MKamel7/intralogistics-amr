"""The docking control law, as arithmetic, before any robot drives on it.

A differential drive cannot correct lateral offset directly. A controller that
simply drives at the dock apex arrives beside it, aligned with nothing, and the
only way to find that out by experiment is to watch a vehicle do it.
"""

import importlib.util
import math
import re
from pathlib import Path


SRC = Path(__file__).resolve().parents[1] / 'amr_mission' / 'dock_approach.py'


def load():
    spec = importlib.util.spec_from_file_location('dock_approach', SRC)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


GAINS = (0.6, 0.8, 0.9)
LIMITS = (0.08, 0.30, 0.03)      # v_max, w_max, lateral_ok
STANDOFF = 0.55


def cmd(dx, dy, dyaw):
    return load().approach_command(dx, dy, dyaw, STANDOFF, GAINS, LIMITS)


def test_a_large_lateral_offset_turns_without_driving():
    """Translating while badly misaligned increases the lateral error.

    This is the whole reason the corrections are ordered rather than summed.
    """
    vx, wz = cmd(1.0, 0.25, 0.0)
    assert vx == 0.0, 'the vehicle drove while off the approach line'
    assert wz > 0.0, 'the turn is toward the wrong side'


def test_the_turn_direction_follows_the_side_the_dock_is_on():
    _, left = cmd(1.0, +0.25, 0.0)
    _, right = cmd(1.0, -0.25, 0.0)
    assert left > 0.0 and right < 0.0, (
        'the lateral correction has the wrong sign on one side, which drives '
        'the vehicle away from the dock')


def test_on_the_line_it_drives_and_keeps_correcting():
    vx, wz = cmd(1.0, 0.01, 0.0)
    assert vx > 0.0
    assert abs(wz) > 0.0, 'no residual lateral correction while driving'


def test_it_stops_translating_at_the_standoff():
    """The final rotation must not translate the vehicle off the point it just
    reached, which is what a controller that keeps driving while turning does."""
    vx, wz = cmd(STANDOFF, 0.0, 0.15)
    assert vx == 0.0
    assert wz > 0.0


def test_it_does_not_reverse_past_the_standoff():
    """Overshoot is corrected by heading, not by backing up.

    Reversing here would put the dock outside the forward sector the detector
    searches, so the vehicle would lose sight of what it is docking to.
    """
    vx, _ = cmd(STANDOFF - 0.05, 0.0, 0.0)
    assert vx == 0.0


def test_every_command_is_inside_the_limits():
    """A gain multiplied by a large error must not produce a lunge."""
    for dx, dy, dyaw in ((5.0, 0.0, 0.0), (1.0, 2.0, 0.0),
                         (STANDOFF, 0.0, math.pi), (0.3, -1.5, -math.pi)):
        vx, wz = cmd(dx, dy, dyaw)
        assert abs(vx) <= LIMITS[0] + 1e-9, f'{vx} exceeds v_max'
        assert abs(wz) <= LIMITS[1] + 1e-9, f'{wz} exceeds w_max'


def test_a_dock_at_zero_range_does_not_divide_by_zero():
    vx, wz = cmd(0.0, 0.0, 0.0)
    assert math.isfinite(vx) and math.isfinite(wz)


def test_commands_go_to_the_first_link_of_the_chain():
    """cmd_vel_nav, so the smoother and the collision monitor stay in the chain.

    Publishing further down would be faster to write and would put a docking
    manoeuvre outside the only layer that can stop it. The monitor's rotation
    polygons cover a spot turn and stop_reverse covers backing out.
    """
    # ASSERTED ON THE CALL, not on the absence of a word in the file. This
    # module's docstring draws the whole command chain, cmd_vel_raw included,
    # so a text search for the topic it must NOT publish to finds the diagram
    # explaining why. That mistake has now been made four times in this
    # project; the fix is to check behaviour rather than prose.
    t = SRC.read_text()
    calls = re.findall(r'create_publisher\(\s*(\w+)\s*,\s*[\'"]([^\'"]+)', t)
    assert calls == [('TwistStamped', 'cmd_vel_nav')], (
        f'the controller publishes {calls}; it must publish exactly one '
        f'command topic and it must be the first link of the chain')


def test_it_refuses_to_drive_on_a_stale_detection():
    """A vehicle in motion with an old dock pose is driving at where the dock
    was. Absence is a separate topic for the same reason."""
    t = SRC.read_text()
    assert 'max_pose_age_s' in t
    assert 'dock_found' in t


def test_arrival_requires_more_than_one_frame_inside_tolerance():
    """One frame inside tolerance is a measurement, not an arrival."""
    t = SRC.read_text()
    assert 'self.settled >= 5' in t
