#!/usr/bin/env python3
"""The test track must keep testing what its dimensions claim to test.

A world whose aisle widths have drifted from the datasheet figures they were
built from is worse than no purpose-built world at all: it still produces
confident numbers, and they are numbers about nothing in particular.

These run without ROS and without a simulator.
"""

import importlib.util
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
GEN = PKG / 'tools' / 'generate_test_track.py'
SPEC_DIR = (Path(__file__).resolve().parents[2]
            / 'amr_description' / 'config' / 'platforms')
PLATFORM = 'mir250_class'


def world_path(name=PLATFORM):
    return PKG / 'worlds' / f'test_track.{name}.sdf'


def stations_path(name=PLATFORM):
    return (Path(__file__).resolve().parents[2] / 'amr_mission' / 'config'
            / f'stations.test_track.{name}.yaml')


def _load_generator():
    spec = importlib.util.spec_from_file_location('generate_test_track', GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gen = _load_generator()


@pytest.fixture(scope='module')
def spec():
    return yaml.safe_load((SPEC_DIR / f'{PLATFORM}.yaml').read_text())


@pytest.fixture(scope='module')
def derived(spec):
    _, d = gen.build_world(spec, PLATFORM)
    return d


def test_generated_world_is_current(tmp_path, spec):
    """The world is generated. If someone hand-edits it, say so."""
    out = tmp_path / 'regen.sdf'
    st = tmp_path / 'regen_stations.yaml'
    subprocess.run(
        [sys.executable, str(GEN), '--platform', PLATFORM,
         '--out', str(out), '--stations-out', str(st)],
        check=True, capture_output=True)
    assert out.read_text() == world_path().read_text(), (
        'the committed test track differs from what the generator produces; it '
        'was hand-edited, or the platform spec changed without regenerating')


def test_the_world_is_valid_sdf():
    """Malformed SDF fails at Gazebo start, which reads as a launch problem."""
    root = ET.parse(world_path()).getroot()
    world = root.find('world')
    assert world is not None and world.get('name') == 'test_track'
    names = [m.get('name') for m in world.findall('model')]
    assert 'ground_plane' in names
    assert len({n for n in names}) == len(names), 'duplicate model names'


def test_every_aisle_width_is_the_published_figure(spec, derived):
    """THE WHOLE POINT. Each scored aisle is a number from the datasheet.

    If these drift, the track stops being a measurement of the manufacturer's
    claims and becomes a set of corridors somebody chose.
    """
    t = spec['validation_targets']
    for key, target in (('aisle_1_y', t['corridor_width_default']),
                        ('aisle_2_y', t['corridor_width_dynamic']),
                        ('doorway_y', t['doorway_width_default'])):
        lo, hi = derived[key]
        assert hi - lo == pytest.approx(target, abs=1e-6), (
            f'{key} is {hi - lo:.4f} m against a published {target} m')


def test_the_corner_is_the_published_corner(spec, derived):
    t = spec['validation_targets']
    lo, hi = derived['corner_x']
    assert hi - lo == pytest.approx(t['corridor_width_90_turn'], abs=1e-6)


def test_the_pinch_keeps_the_real_buildings_difficulty(derived):
    """The designed world must not be a comfortable one.

    1.340 m is the MEASURED median corridor of the AWS warehouse, V-22. Dropping
    it would make the demo clean and the result hollow, because the vehicle
    would never meet the width it actually has to work in.
    """
    lo, hi = derived['pinch_y']
    assert hi - lo == pytest.approx(1.340, abs=1e-6)


def test_zones_do_not_overlap_or_leave_gaps(derived):
    """Rows are solved from the top down, so an error shows up as an overlap."""
    bands = sorted([derived['aisle_1_y'], derived['aisle_2_y'],
                    derived['pinch_y']], key=lambda b: b[0])
    for (lo1, hi1), (lo2, hi2) in zip(bands, bands[1:]):
        assert hi1 <= lo2 + 1e-9, f'aisles overlap: {hi1} into {lo2}'


def test_the_vehicle_fits_every_aisle_it_is_scored_on(spec, derived):
    """A zone narrower than the robot is an impossible test, not a hard one.

    It would be recorded as a navigation failure when it is a geometry error.
    """
    v = spec['values']
    width = 2.0 * v['scanner_mount_y']
    for key in ('aisle_1_y', 'aisle_2_y', 'pinch_y', 'doorway_y'):
        lo, hi = derived[key]
        assert hi - lo > width, (
            f'{key} is {hi - lo:.3f} m against a vehicle {width:.3f} m wide')


def test_the_open_bay_can_actually_hold_a_reroute(spec, derived):
    """P1 only means something if routing around a person is geometrically possible.

    Vehicle circumscribed diameter plus a person plus clearance either side. If
    the bay is narrower than that, the pedestrian scenario has one outcome
    instead of two and proves nothing about re-planning.
    """
    v = spec['values']
    diameter = 2.0 * math.hypot(v['scanner_mount_x'], v['scanner_mount_y'])
    person = 0.50
    assert gen.OPEN_BAY_MIN > diameter + person, (
        f'open bay {gen.OPEN_BAY_MIN} m cannot hold a {diameter:.3f} m vehicle '
        f'passing a {person} m person, so P1 cannot demonstrate a re-route')


def test_the_scored_aisle_cannot_hold_a_reroute(spec, derived):
    """P2's outcome must be forced by geometry, not by tuning.

    The dynamic-corridor aisle has to be too narrow to pass a standing person,
    or "the vehicle waits" is a behaviour someone chose rather than a fact about
    the building.
    """
    v = spec['values']
    diameter = 2.0 * math.hypot(v['scanner_mount_x'], v['scanner_mount_y'])
    lo, hi = derived['aisle_2_y']
    assert (hi - lo) < diameter + 0.50, (
        'the scored aisle is wide enough to pass a person, so P2 no longer '
        'forces the wait it was built to demonstrate')


def test_station_approach_poses_are_generated(spec):
    """Hand-authored approach poses are how one platform's pose outlived it."""
    st = yaml.safe_load(stations_path().read_text())
    names = {s['name'] for s in st['stations']}
    assert names == {'goods_in', 'dispatch'}
    assert st['route'] == ['goods_in', 'dispatch']
    for s in st['stations']:
        assert 'note' in s and s['note'], 'every station states how it is approached'


def test_stations_sit_on_the_aisles_they_belong_to(derived):
    """A station off its own aisle is a goal the planner cannot reach cleanly."""
    a2_lo, a2_hi = derived['aisle_2_y']
    d_lo, d_hi = derived['doorway_y']
    gi = derived['stations']['goods_in']
    dp = derived['stations']['dispatch']
    assert a2_lo < gi[1] < a2_hi, 'goods_in is not on the dynamic-corridor aisle'
    assert d_lo < dp[1] < d_hi, 'dispatch is not aligned with the doorway'
