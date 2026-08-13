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


def test_a_crossing_walker_returns_home_and_then_waits_out_the_vehicle():
    """Crosses, goes home, and is spent until the vehicle leaves.

    This previously asserted the walker went straight back to `waiting`, which
    is the behaviour that produced the pacing seen live: four round trips in
    26 seconds while the vehicle sat inside the trigger radius. The old
    assertion described the bug, so it was changed rather than kept.
    """
    d = Driver(robot=(3.0, 0.0, 0.0))
    w = {'kind': 'cross', 'pose': (0.0, 4.0, 0.0), 'home': (0.0, 0.0),
         'far': (0.0, 4.0), 'trigger': 6.0, 'resets': True, 'phase': 'crossing'}
    assert PedestrianDriver._cross_goal(d, w) == (0.0, 0.0)
    assert w['phase'] == 'returning'
    assert PedestrianDriver._cross_goal(d, w) is None
    assert w['phase'] == 'spent', 'it must not immediately cross again'


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


def test_the_launch_bridges_every_behaviour_the_driver_understands():
    """The contract between three programs, asserted in one place.

    The generator writes behaviour keys, people.launch.py decides who gets a
    cmd_vel bridge by looking for them, and the driver moves whoever has one.
    When they disagree nothing errors: the models spawn, the driver runs,
    no bridge is created, and the entire crowd stands still.

    That has happened twice. Once when `path` became `wander`, and again when
    `route` and `cross` were added to the driver and the launch still named
    only two keys. Both times a scenario that looked full produced nobody
    moving.
    """
    from pathlib import Path

    from amr_sim.pedestrian_driver import BEHAVIOUR_KEYS
    launch = (Path(__file__).resolve().parents[1]
              / 'launch' / 'people.launch.py').read_text()
    assert 'BEHAVIOUR_KEYS' in launch, (
        'the launch must take the keys from the driver rather than repeat '
        'them, or the two will drift again')
    assert "q.get('wander') or q.get('path')" not in launch, (
        'a hand written key list here is the exact fault this guards')
    for kind in ('wander', 'route', 'cross'):
        assert kind in BEHAVIOUR_KEYS, f'{kind} is implemented but not declared'


def test_every_generated_scenario_only_uses_known_behaviours():
    """A scenario key the driver does not know is a person who never moves."""
    from pathlib import Path

    import yaml

    from amr_sim.pedestrian_driver import BEHAVIOUR_KEYS
    scen_dir = Path(__file__).resolve().parents[1] / 'scenarios'
    reserved = {'name', 'x', 'y', 'yaw'}
    for f in scen_dir.glob('*.yaml'):
        spec = yaml.safe_load(f.read_text())
        for person in spec.get('people', []):
            unknown = set(person) - reserved - set(BEHAVIOUR_KEYS)
            assert not unknown, (
                f'{f.name}: {person["name"]} carries {sorted(unknown)}, which '
                f'no behaviour in the driver reads, so it will never move')


def cross_walker(phase='waiting'):
    return {'kind': 'cross', 'pose': (0.0, 0.0, 0.0), 'home': (0.0, 0.0),
            'far': (0.0, 4.0), 'trigger': 6.0, 'resets': True, 'phase': phase}


def test_it_does_not_cross_again_while_the_vehicle_is_still_there():
    """Observed live: four round trips in 26 seconds.

    The walker re-armed as soon as it got home, and the vehicle was still
    inside the trigger radius, so it shuttled back and forth across the aisle.
    That is not a crossing, it is a metronome, and it manufactures cheap
    encounters that would inflate any later count.
    """
    d = Driver(robot=(2.0, 0.0, 0.0))          # vehicle parked well inside 6 m
    w = cross_walker('returning')
    assert PedestrianDriver._cross_goal(d, w) is None
    assert w['phase'] == 'spent'
    for _ in range(50):
        assert PedestrianDriver._cross_goal(d, w) is None, (
            'a second crossing started while the vehicle had not left')
        assert w['phase'] == 'spent'


def test_it_re_arms_once_the_vehicle_has_gone():
    """One crossing per approach, not one per run."""
    w = cross_walker('spent')
    far = Driver(robot=(40.0, 0.0, 0.0))
    assert PedestrianDriver._cross_goal(far, w) is None
    assert w['phase'] == 'waiting', 'it must be ready for the next approach'
    near = Driver(robot=(3.0, 0.0, 0.0))
    assert PedestrianDriver._cross_goal(near, w) == (0.0, 4.0)
