"""VDA 5050 message construction.

The protocol's difficulty is bookkeeping, not transport: header sequences that
must not skip, action states that must follow a lifecycle, and order updates
that must join the order they extend. All of that is pure data and is tested
here without a broker, a simulator or a robot.
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_fleet import vda5050 as v  # noqa: E402


@pytest.fixture
def header():
    return v.Header('Neobotix', 'mp400-01')


def test_topic_follows_the_standard_structure():
    assert v.topic('Neobotix', 'mp400-01', 'state') == \
        'uagv/v2/Neobotix/mp400-01/state'


def test_an_unknown_topic_is_refused():
    """A typo in a topic name publishes into the void and looks like a broker
    problem."""
    with pytest.raises(ValueError):
        v.topic('Neobotix', 'mp400-01', 'staet')


def test_header_ids_are_per_topic_and_gapless(header):
    """THE COUNTER IS PER TOPIC. Sharing one across state and connection makes
    both sequences appear to skip, and a master control reading a skip
    concludes it has lost messages."""
    assert [header.next('state')['headerId'] for _ in range(3)] == [0, 1, 2]
    assert header.next('connection')['headerId'] == 0
    assert header.next('state')['headerId'] == 3


def test_the_timestamp_is_schema_shaped(header):
    """Not a bare isoformat: that emits +00:00 and a master control validating
    against the published schema rejects it."""
    ts = header.next('state')['timestamp']
    assert ts.endswith('Z')
    assert '+00:00' not in ts
    assert ts[10] == 'T' and ts[-5] == '.'


def test_every_message_carries_the_identifying_quartet(header):
    m = v.state(header)
    for k in ('headerId', 'timestamp', 'version', 'manufacturer', 'serialNumber'):
        assert k in m, f'{k} missing; a master control cannot route this'


def test_connection_states_are_constrained(header):
    for s in ('ONLINE', 'OFFLINE', 'CONNECTIONBROKEN'):
        assert v.connection(header, s)['connectionState'] == s
    with pytest.raises(ValueError):
        v.connection(header, 'UP')


def test_action_states_are_constrained():
    with pytest.raises(ValueError):
        v.action_state('a1', 'pick', 'DONE')
    assert v.action_state('a1', 'pick', 'RUNNING')['actionStatus'] == 'RUNNING'


def test_an_order_update_must_advance():
    """A repeated or older update would silently re-run nodes the vehicle has
    already passed."""
    prev = {'orderId': 'o1', 'orderUpdateId': 3, 'nodes': [{'nodeId': 'n1'}]}
    with pytest.raises(v.SequenceError):
        v.order_is_continuous(prev, {'orderId': 'o1', 'orderUpdateId': 3})
    with pytest.raises(v.SequenceError):
        v.order_is_continuous(prev, {'orderId': 'o1', 'orderUpdateId': 2})


def test_an_order_update_must_join_the_current_order():
    """Accepting an update whose base does not join the current one leaves a
    gap: the vehicle drives to a node it was never routed to, through space
    nobody checked."""
    prev = {'orderId': 'o1', 'orderUpdateId': 1,
            'nodes': [{'nodeId': 'n1'}, {'nodeId': 'n2'}]}
    good = {'orderId': 'o1', 'orderUpdateId': 2, 'nodes': [{'nodeId': 'n2'}]}
    assert v.order_is_continuous(prev, good) is True
    bad = {'orderId': 'o1', 'orderUpdateId': 2, 'nodes': [{'nodeId': 'n9'}]}
    with pytest.raises(v.SequenceError):
        v.order_is_continuous(prev, bad)


def test_a_brand_new_order_is_allowed(header):
    prev = {'orderId': 'o1', 'orderUpdateId': 5, 'nodes': [{'nodeId': 'n1'}]}
    assert v.order_is_continuous(prev, {'orderId': 'o2', 'orderUpdateId': 0}) is True
    assert v.order_is_continuous(None, {'orderId': 'o1', 'orderUpdateId': 0}) is True


def test_state_reports_what_a_master_control_schedules_on(header):
    """driving, lastNodeId and actionStates are what a fleet manager plans
    against. A state without them is not a state."""
    m = v.state(header, order_id='o1', last_node='n3', driving=True)
    assert m['driving'] is True
    assert m['lastNodeId'] == 'n3'
    for k in ('nodeStates', 'edgeStates', 'actionStates', 'batteryState',
              'operatingMode', 'safetyState', 'agvPosition'):
        assert k in m


def test_position_is_marked_uninitialised_without_a_map(header):
    """A pose with no map behind it is a number, not a position, and a master
    control that plots it puts the vehicle somewhere it has never been."""
    assert v.state(header)['agvPosition']['positionInitialized'] is False
    assert v.state(header, map_id='track')['agvPosition']['positionInitialized'] is True


def test_safety_state_values_are_constrained():
    with pytest.raises(ValueError):
        v.safety_state('PRESSED')
    assert v.safety_state('MANUAL', True)['fieldViolation'] is True


def test_messages_serialise_compactly(header):
    """A fleet at 1 Hz each is a lot of bytes a broker does not need to
    carry."""
    s = v.dumps(v.state(header))
    assert ', ' not in s and '": ' not in s
    assert json.loads(s)['operatingMode'] == 'AUTOMATIC'
