"""The dispatcher, tested without a broker or a robot.

A fleet is not several robots, it is the thing that decides which robot does
what and what happens when one stops answering. Everything worth testing is in
the second half.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_fleet.dispatcher import Dispatcher  # noqa: E402


def fleet():
    d = Dispatcher()
    a = d.track('Neobotix', 'mp400-01')
    b = d.track('MiR', 'mir250-01')
    for veh, (x, y) in ((a, (0.0, 0.0)), (b, (20.0, 0.0))):
        veh.online = True
        veh.x, veh.y = x, y
    return d, a, b


def test_the_nearest_free_vehicle_takes_the_order():
    d, a, b = fleet()
    d.submit('dispatch', 19.0, 0.0)
    made = d.assign()
    assert [k for k, _ in made] == [b.key]


def test_a_busy_vehicle_is_not_given_a_second_order():
    d, a, b = fleet()
    b.driving = True
    d.submit('dispatch', 19.0, 0.0)
    made = d.assign()
    assert [k for k, _ in made] == [a.key], 'the far but free vehicle should take it'


def test_orders_wait_rather_than_being_dropped_when_nobody_is_free():
    """A queue that silently discards is a fleet that loses jobs."""
    d, a, b = fleet()
    a.driving = b.driving = True
    d.submit('dispatch', 1.0, 0.0)
    assert d.assign() == []
    assert len(d.pending) == 1


def test_an_offline_vehicle_hands_its_order_back():
    """Leaving it assigned is how a fleet quietly loses a job: the order is
    neither running nor waiting, and nothing reports it."""
    d, a, b = fleet()
    d.submit('dispatch', 19.0, 0.0)
    d.assign()
    assert b.key in d.assigned
    d.on_connection(b.key, {'connectionState': 'CONNECTIONBROKEN'})
    assert b.key not in d.assigned
    assert len(d.pending) == 1, 'the order must return to the queue'


def test_a_reclaimed_order_goes_to_the_front():
    """It was already accepted once; making it queue behind newer work would
    starve exactly the job that has already been delayed."""
    d, a, b = fleet()
    first = d.submit('dispatch', 19.0, 0.0)
    d.assign()
    second = d.submit('goods_in', 19.0, 0.0)
    d.on_connection(b.key, {'connectionState': 'OFFLINE'})
    assert d.pending[0]['orderId'] == first['orderId']
    assert d.pending[1]['orderId'] == second['orderId']


def test_a_silent_vehicle_is_treated_as_offline():
    """The last will covers a clean broker disconnect. It does NOT cover a
    vehicle whose process is alive and wedged, which this project has produced
    more than once: the node held its connection and published nothing for
    thirty seconds. Silence is the more reliable signal.
    """
    d, a, b = fleet()
    d.submit('dispatch', 19.0, 0.0)
    d.assign()
    b.last_seen = 1000.0
    dropped = d.sweep(now=1000.0 + 11.0)
    assert b.key in dropped
    assert b.online is False
    assert len(d.pending) == 1


def test_a_vehicle_still_publishing_is_not_swept():
    d, a, b = fleet()
    b.last_seen = 1000.0
    assert d.sweep(now=1000.0 + 1.0) == []


def test_state_comes_from_the_vehicle_not_from_memory():
    """A dispatcher holding its own belief about a fleet will eventually route
    two vehicles into the same aisle because it has not noticed one stopped."""
    d, a, b = fleet()
    d.on_state(b.key, {'agvPosition': {'x': 5.0, 'y': 6.0}, 'driving': True,
                       'orderId': 'o9', 'nodeStates': [{'nodeId': 'n1'}],
                       'batteryState': {'batteryCharge': 44.0}})
    assert (b.x, b.y) == (5.0, 6.0)
    assert b.driving is True and b.nodes_left == 1 and b.battery == 44.0
    assert b.free is False


def test_completion_is_recorded_when_the_vehicle_says_so():
    d, a, b = fleet()
    order = d.submit('dispatch', 19.0, 0.0)
    d.assign()
    d.on_state(b.key, {'agvPosition': {'x': 19.0, 'y': 0.0}, 'driving': False,
                       'orderId': order['orderId'], 'nodeStates': []})
    assert d.completed and d.completed[0]['orderId'] == order['orderId']
    assert b.key not in d.assigned


def test_order_ids_are_unique():
    """Two orders sharing an id makes the vehicle's continuity check reject the
    second as a bad update to the first."""
    d, _, _ = fleet()
    ids = {d.submit('dispatch', 1.0, 0.0)['orderId'] for _ in range(5)}
    assert len(ids) == 5
