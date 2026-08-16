"""The recovery for a planner that refuses to start. V-58.

WHAT THE DEFECT WAS

    GridBased plugin failed to plan from (4.68, 5.68) to (-2.00, 0.00):
      "Start occupied"

Three cycles, three failures, three seconds each, 0.0 m driven. The vehicle
finished its survey in a pose whose own cell reads as occupied, and from there
the planner refuses, so no command reaches the wheels, so the vehicle does not
move, so the start stays occupied. The mission retried the same goal from the
same pose and failed identically.

WHY THE GUARD IS A DISTANCE AND NOT AN ERROR STRING

Matching on "Start occupied" would tie the mission to one planner's wording.
SmacPlanner2D says that; NavFn says "Failed to create plan with tolerance";
ThetaStar says "Either of the start or goal pose are an obstacle" (V-47). A leg
that failed having driven nothing is the observable common to all three and it
does not care what the planner called it.

WHY NOT CLEAR THE FOOTPRINT FROM THE COSTMAP

That is the obvious fix and it trades a stall for a collision risk. V-42 and
V-45 are what this project has to show for safety changes argued from
arithmetic rather than measured, and both were reverted.
"""

import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / 'amr_mission' / 'transport_task.py'


def source():
    return SRC.read_text()


def test_a_leg_that_moved_is_not_nudged():
    """The bound that keeps this from firing on ordinary navigation failures.

    A leg that drove and then failed is one where the planner engaged. Whatever
    went wrong is not a refusal to start, and reversing into a corridor the
    vehicle was already driving down would be the wrong answer to it.
    """
    t = source()
    m = re.search(r'if not arrived and \(self\.odom_total - leg_start\) < ([\d.]+):', t)
    assert m, 'the nudge is not gated on the distance the leg drove'
    threshold = float(m.group(1))
    assert threshold <= 0.10, (
        f'the nudge fires on legs that drove up to {threshold} m, which is '
        f'ordinary navigation failure rather than a refusal to start')


def test_the_nudge_reports_whether_it_moved():
    """A nudge that did not move the vehicle has not changed the condition.

    Retrying after one would be the same failure with an extra step, and the
    log would show a recovery that "ran" before every failure.
    """
    t = source()
    assert 'moved = self.odom_total - before' in t
    assert 'return moved > 0.01' in t, (
        'nudge() reports success without checking the vehicle moved')


def test_the_nudge_count_is_always_reported():
    """Zero included.

    "No nudges" and "nudges not counted" look identical in a log otherwise, and
    a recovery that fires constantly is a different problem wearing a solution.
    """
    t = source()
    assert 'self.nudges += 1' in t
    assert re.search(r'nudged out of a stuck start \{self\.nudges\}', t), (
        'the nudge count is not printed in the run summary')


def test_it_uses_the_configured_backup_behaviour():
    """Not a raw cmd_vel.

    Publishing a reverse command directly would bypass the collision monitor's
    command chain. Going through the BackUp action keeps the monitor in the
    loop, and stop_reverse is the one polygon with real rearward margin,
    0.4560 m against a chassis half length of 0.2950 m.
    """
    t = source()
    assert 'from nav2_msgs.action import BackUp' in t
    assert "ActionClient(self, BackUp, 'backup')" in t
    m = re.search(r'goal\.target = Point\(x=([\d.]+)\)', t)
    assert m, 'the nudge distance is not stated'
    assert float(m.group(1)) <= 0.30, (
        f'a {m.group(1)} m reverse is beyond the rearward margin the reverse '
        f'protective field carries')
