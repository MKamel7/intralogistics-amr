#!/usr/bin/env python3
"""The load is picked up at the first stop and set down at the last one.

`config/stations.yaml` promises that "a three-stop or a loop route needs no
code change". Nothing tested that promise, and it was false: the task decided
the load state with `leg == 0`, so a three-stop route set the box down at the
middle station and drove the last leg empty while still reporting a completed
laden cycle. These assertions cost microseconds and catch a wrong answer that
reports success, which is the kind a simulator run will not show you.
"""

import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
STATIONS = PKG / 'config' / 'stations.yaml'

# The module under test is deliberately free of ROS imports so this runs in the
# fast CI gate, with no graph and no simulator.
sys.path.insert(0, str(PKG))
from amr_mission.route import carrying_after, handling_label  # noqa: E402


@pytest.fixture(scope='module')
def route():
    return yaml.safe_load(STATIONS.read_text())['route']


def test_the_shipped_route_loads_first_and_unloads_last(route):
    stops = len(route)
    assert carrying_after(0, stops) is True, 'the first stop must load'
    assert carrying_after(stops - 1, stops) is False, 'the last stop must unload'


def test_a_three_stop_route_keeps_the_load_through_the_middle():
    # The regression. Under the old `leg == 0` rule the middle stop returned
    # False here, unloading the vehicle a leg early.
    assert carrying_after(0, 3) is True
    assert carrying_after(1, 3) is True, (
        'the middle stop of a three-stop route is a transfer, so the load '
        'stays on the plate; unloading here drives the final leg empty while '
        'the cycle still reports a laden delivery')
    assert carrying_after(2, 3) is False


def test_the_load_is_set_down_exactly_once_on_any_route():
    for stops in range(2, 8):
        states = [carrying_after(leg, stops) for leg in range(stops)]
        assert states.count(False) == 1, (
            f'a {stops}-stop route puts the load down {states.count(False)} '
            f'times: {states}')
        assert states[-1] is False, 'the one set-down must be the last stop'


def test_the_words_match_the_behaviour():
    # A run that says "unloading" at a stop where the load stays on is a
    # report that disagrees with the vehicle.
    for stops in range(2, 6):
        for leg in range(stops):
            label = handling_label(leg, stops)
            if label == 'unloading':
                assert carrying_after(leg, stops) is False
            else:
                assert carrying_after(leg, stops) is True
    assert handling_label(1, 3) == 'passing through, still laden'


def test_a_route_that_goes_nowhere_is_rejected():
    for bad in (0, 1):
        with pytest.raises(ValueError):
            carrying_after(0, bad)
    with pytest.raises(ValueError):
        carrying_after(3, 3)
