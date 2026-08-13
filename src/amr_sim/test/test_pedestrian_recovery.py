"""The recovery that has to actually move a walker.

`_nearest_clear` is the path taken by a walker that spawns in a cell without
clearance. It only reads `self.grid` and `self.goal_tolerance`, so it can be
exercised directly without a running node, which is the whole reason it is
worth testing here rather than only in simulation.

The fault it now guards against ran for the entire life of this project:
every pedestrian in every recorded run stood still, because the recovery goal
was returned closer than the arrival tolerance and so was satisfied without
the walker moving. See V-32.
"""

import math

import pytest

from amr_sim.pedestrian_driver import PedestrianDriver


class Grid:
    """A floor that is obstructed inside a disc of the given radius."""

    res = 0.05

    def __init__(self, blocked_radius):
        self.blocked_radius = blocked_radius

    def clear(self, x, y, clearance):
        return math.hypot(x, y) > self.blocked_radius


class Driver:
    """Just enough of the node for the ring search to run."""

    def __init__(self, blocked_radius, goal_tolerance=0.30, clearance=0.45):
        self.grid = Grid(blocked_radius)
        self.goal_tolerance = goal_tolerance
        # The radius the walker needs around it. Passed straight through to
        # grid.clear, so the stub grid ignores it, but the method reads it.
        self.clearance = clearance


@pytest.mark.parametrize('blocked_radius', [0.05, 0.15, 0.30, 0.60, 1.20])
def test_escape_is_further_away_than_the_arrival_tolerance(blocked_radius):
    """The property that was violated.

    A goal within goal_tolerance is reached the moment it is set, so the
    walker is commanded nothing, re-picks, is still obstructed, and recovers
    to the same spot on every tick for the length of the run.
    """
    d = Driver(blocked_radius)
    p = PedestrianDriver._nearest_clear(d, 0.0, 0.0)
    assert p is not None, 'clear ground exists here, so a point must be found'
    assert math.hypot(*p) > d.goal_tolerance, (
        f'recovery goal {math.hypot(*p):.3f} m away is inside the '
        f'{d.goal_tolerance} m arrival tolerance, so the walker registers as '
        f'arrived without moving and never escapes')


@pytest.mark.parametrize('blocked_radius', [0.05, 0.30, 0.60])
def test_the_escape_point_is_itself_clear(blocked_radius):
    """Moving further out must not mean moving somewhere unusable."""
    d = Driver(blocked_radius)
    p = PedestrianDriver._nearest_clear(d, 0.0, 0.0)
    assert d.grid.clear(p[0], p[1], 0.45)


def test_it_gives_up_rather_than_returning_a_bad_point():
    """Wholly obstructed ground within the limit must return None.

    Returning a point that is not clear would move a walker into a wall and
    look like the recovery working.
    """
    d = Driver(blocked_radius=1e9)
    assert PedestrianDriver._nearest_clear(d, 0.0, 0.0, limit=3.0) is None


def test_a_tighter_tolerance_still_escapes():
    """The first ring is derived from goal_tolerance, not hard coded to it."""
    d = Driver(blocked_radius=0.10, goal_tolerance=0.05)
    p = PedestrianDriver._nearest_clear(d, 0.0, 0.0)
    assert math.hypot(*p) > 0.05
