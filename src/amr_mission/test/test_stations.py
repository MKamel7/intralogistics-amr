#!/usr/bin/env python3
"""Checks on the station definitions.

A station the vehicle cannot reach is a mission that fails at run time, in a
simulator, after several minutes of driving. These assertions cost milliseconds
and catch the same thing.
"""

import math
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
STATIONS = PKG / 'config' / 'stations.yaml'
MAPS = PKG.parent / 'amr_navigation' / 'maps'

# The vehicle's inscribed radius, 810 by 590 mm envelope. A station closer than
# this to anything is a station it cannot stand in.
INSCRIBED = math.hypot(0.405, 0.295)


@pytest.fixture(scope='module')
def spec():
    return yaml.safe_load(STATIONS.read_text())


def test_every_route_stop_is_a_defined_station(spec):
    names = {s['name'] for s in spec['stations']}
    for stop in spec['route']:
        assert stop in names, f'route names {stop}, which is not defined'


def test_the_route_visits_at_least_two_places(spec):
    assert len(spec['route']) >= 2, 'a transport task needs somewhere to go'
    assert len(set(spec['route'])) >= 2, 'the route never leaves one station'


def test_stations_have_room_for_the_vehicle(spec):
    for s in spec['stations']:
        assert s['clearance'] > INSCRIBED, (
            f'{s["name"]} claims {s["clearance"]} m of clearance against an '
            f'inscribed radius of {INSCRIBED:.3f} m, so the vehicle does not '
            f'fit in it')


def test_stations_are_far_enough_apart_to_be_a_journey(spec):
    """Two stations three metres apart do not test navigation.

    The pair in use was chosen by searching the surveyed map for the two
    positions furthest apart that still have 0.90 m of clearance, precisely so
    the task crosses the building instead of shuffling.
    """
    by_name = {s['name']: s for s in spec['stations']}
    stops = [by_name[n] for n in spec['route']]
    worst = min(math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                for a, b in zip(stops, stops[1:]))
    assert worst > 5.0, f'the shortest leg is {worst:.1f} m, which is a shuffle'


@pytest.mark.skipif(not (MAPS / 'keepout_mask.yaml').exists(),
                    reason='keepout mask not built')
def test_no_station_sits_inside_a_keepout_zone(spec):
    """A station inside a no-go zone is unreachable by construction.

    Worth asserting rather than trusting, because the two files are authored
    separately and nothing else would connect them until the vehicle failed to
    arrive.
    """
    import sys
    sys.path.insert(0, str(PKG.parent))
    from amr_sim.occupancy import load_map

    keep = load_map(MAPS / 'keepout_mask.yaml')
    for s in spec['stations']:
        value = keep.cell(s['x'], s['y'])
        assert value <= 30, (
            f'{s["name"]} at ({s["x"]}, {s["y"]}) is inside a keepout zone '
            f'(mask value {value})')


def station_keys_read_by(source):
    """Every station field the code actually reads, from the SYNTAX TREE.

    Not from a text search. This file and transport_task.py both discuss the
    key mismatch they exist to prevent, so any regex looking for the wrong key
    finds the comment explaining why it is wrong. That trap has now caught five
    tests in this project, and parsing is the structural answer: a comment is
    not in the tree.
    """
    import ast
    keys = set()
    for node in ast.walk(ast.parse(source)):
        if (isinstance(node, ast.Subscript)
                and isinstance(node.value, ast.Name)
                and node.value.id == 'station'
                and isinstance(node.slice, ast.Constant)
                and isinstance(node.slice.value, str)):
            keys.add(node.slice.value)
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'get'
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'station'
                and node.args
                and isinstance(node.args[0], ast.Constant)):
            keys.add(node.args[0].value)
    return keys


def test_the_mission_reads_the_key_the_generator_writes():
    """Every goal this project sent used yaw 0 for its whole life.

    The generator writes `yaw`. The mission read `approach_yaw` with a default
    of 0.0, so the computed orientation was never used and a plausible number
    was substituted silently. A subscript without a default would have raised
    KeyError on the first run.

    This walks the generated files rather than naming a key, so the two cannot
    drift apart again.
    """
    from pathlib import Path

    import yaml

    src = (Path(__file__).resolve().parents[1] / 'amr_mission'
           / 'transport_task.py').read_text()
    read = station_keys_read_by(src)
    assert read, 'the mission reads no station fields at all'

    cfg = Path(__file__).resolve().parents[1] / 'config'
    files = sorted(cfg.glob('stations.*.yaml'))
    assert files, 'no stations files to check against'
    for f in files:
        written = set()
        for s in yaml.safe_load(f.read_text())['stations']:
            written |= set(s.keys())
        missing = read - written
        assert not missing, (
            f'{f.name} does not carry {sorted(missing)}, which the mission '
            f'reads; a silent default would substitute a plausible number')


def test_the_station_orientation_has_no_silent_default():
    """A default turns a key mismatch into a plausible wrong answer.

    Checked on the tree: `station.get('yaw', ...)` would be a default and
    `station['yaw']` is not.
    """
    import ast
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'amr_mission'
           / 'transport_task.py').read_text()
    for node in ast.walk(ast.parse(src)):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'get'
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'station'):
            key = node.args[0].value if node.args else '?'
            raise AssertionError(
                f'the mission reads station {key!r} with a default, which '
                f'substitutes a plausible number when the key is wrong')
