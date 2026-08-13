"""The three pedestrian kinds, tested as pure logic.

`_route_goal` and `_cross_goal` read only the walker dict and, for crossings,
the recorded poses, so both run without a node. That is deliberate: the
behaviour of a scenario is the thing every comparison in this project rests on,
and it should not need a simulator to check.

Phase 6 of the plan asks for a mix rather than one loop, and names the crossing
pedestrian as the case worth demonstrating. It is also the case that no
scenario here has ever produced, because every walker yielded to the vehicle
long before it could step in front of one.
"""

import math

from amr_sim.pedestrian_driver import PedestrianDriver


def route_walker(mode='pingpong', pts=None):
    return {
        'kind': 'route',
        'pose': (0.0, 0.0, 0.0),
        'waypoints': pts or [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)],
        'leg': 0,
        'step': 1,
        'mode': mode,
    }


class Driver:
    """Enough of the node for the goal selectors to run."""

    robot_frame = 'amr'

    def __init__(self, robot=None):
        self.poses = {'amr': robot} if robot else {}


def visit(w, n):
    return [PedestrianDriver._route_goal(None, w) for _ in range(n)]


def test_a_route_visits_its_waypoints_in_order():
    w = route_walker()
    assert visit(w, 2) == [(5.0, 0.0), (5.0, 5.0)]


def test_pingpong_turns_round_at_the_end_without_repeating_the_endpoint():
    """Walking to the end and immediately back to the end would leave the
    walker standing still for one leg, which reads as a stall."""
    w = route_walker()
    seq = visit(w, 5)
    assert seq == [(5.0, 0.0), (5.0, 5.0), (5.0, 0.0), (0.0, 0.0), (5.0, 0.0)]


def test_loop_returns_to_the_start():
    w = route_walker(mode='loop')
    assert visit(w, 3) == [(5.0, 0.0), (5.0, 5.0), (0.0, 0.0)]


def test_a_route_is_deterministic():
    """The whole reason routes exist. Two walkers with the same route must
    produce the same sequence, or no metric measured under one scenario can be
    compared against the same metric measured later."""
    assert visit(route_walker(), 12) == visit(route_walker(), 12)


def test_a_crossing_walker_waits_until_the_vehicle_is_close():
    d = Driver(robot=(50.0, 50.0, 0.0))
    w = {'kind': 'cross', 'pose': (0.0, 0.0, 0.0), 'home': (0.0, 0.0),
         'far': (0.0, 4.0), 'trigger': 6.0, 'resets': True, 'phase': 'waiting'}
    assert PedestrianDriver._cross_goal(d, w) is None
    assert w['phase'] == 'waiting', 'a distant vehicle must not start a crossing'


def test_a_crossing_walker_steps_out_when_the_vehicle_arrives():
    d = Driver(robot=(3.0, 0.0, 0.0))
    w = {'kind': 'cross', 'pose': (0.0, 0.0, 0.0), 'home': (0.0, 0.0),
         'far': (0.0, 4.0), 'trigger': 6.0, 'resets': True, 'phase': 'waiting'}
    assert PedestrianDriver._cross_goal(d, w) == (0.0, 4.0)
    assert w['phase'] == 'crossing'


def test_a_crossing_walker_returns_and_can_cross_again():
    d = Driver(robot=(3.0, 0.0, 0.0))
    w = {'kind': 'cross', 'pose': (0.0, 4.0, 0.0), 'home': (0.0, 0.0),
         'far': (0.0, 4.0), 'trigger': 6.0, 'resets': True, 'phase': 'crossing'}
    assert PedestrianDriver._cross_goal(d, w) == (0.0, 0.0)
    assert w['phase'] == 'returning'
    assert PedestrianDriver._cross_goal(d, w) is None
    assert w['phase'] == 'waiting', 'it must be able to cross again'


def test_a_one_shot_crossing_stays_put_afterwards():
    d = Driver(robot=(3.0, 0.0, 0.0))
    w = {'kind': 'cross', 'pose': (0.0, 4.0, 0.0), 'home': (0.0, 0.0),
         'far': (0.0, 4.0), 'trigger': 6.0, 'resets': False, 'phase': 'crossing'}
    assert PedestrianDriver._cross_goal(d, w) is None


def test_a_crossing_walker_without_a_vehicle_does_nothing():
    """Before the ground truth feed arrives there is no vehicle to cross in
    front of. Stepping out anyway would put a person in the aisle at startup
    for no reason."""
    w = {'kind': 'cross', 'pose': (0.0, 0.0, 0.0), 'home': (0.0, 0.0),
         'far': (0.0, 4.0), 'trigger': 6.0, 'resets': True, 'phase': 'waiting'}
    assert PedestrianDriver._cross_goal(Driver(), w) is None


def test_the_crossing_distance_is_worth_crossing():
    """A crossing that ends where it started is not a crossing. Guards the
    scenario rather than the code, and it is the sort of thing a generator
    change breaks silently."""
    home, far = (0.0, 0.0), (0.0, 4.0)
    assert math.dist(home, far) > 1.0
