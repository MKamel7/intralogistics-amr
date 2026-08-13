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


def test_every_aisle_is_derived_from_the_vehicle(spec, derived):
    """THE WHOLE POINT, and the basis changed deliberately.

    The track used the published corridor figures until two of the four turned
    out to be unachievable with this stack and the vehicle trapped itself in
    the corner, V-26 and V-27. It is now a capability demonstrator: every width
    is derived from what THIS vehicle needs to turn round, so the widths are
    still traceable rather than chosen, and a cycle can actually complete.
    """
    turn = gen.rotation_width(spec)
    widths = {n: w for n, w, _ in gen.zones(spec)}
    for key, name in (('aisle_1_y', 'aisle_1'), ('aisle_2_y', 'aisle_2'),
                      ('doorway_y', 'doorway')):
        lo, hi = derived[key]
        assert hi - lo == pytest.approx(widths[name], abs=1e-6)
    assert widths['aisle_2'] == pytest.approx(turn, abs=1e-9), (
        'the scored aisle must be exactly the turning width, so the track '
        'still tests the narrowest corridor the vehicle can work in')


def test_the_cross_aisle_is_wide_enough_to_turn_in(spec, derived):
    """THE TRACK MUST NOT BE ABLE TO TRAP THE VEHICLE.

    The cross aisle used to be the published 0.950 m corner figure. The
    MiR250's circumscribed diameter is 1.0021 m, so it drove in and could not
    rotate out: fifteen survey rounds timed out at one pose, every recovery
    aborting on Collision Ahead, and fifty minutes of survey produced one data
    point because every other zone went unmeasured. See V-27.

    A scored zone the vehicle fails must record a failure and let the run
    continue. This asserts the route cannot swallow the vehicle again.
    """
    v = spec['values']
    diameter = 2.0 * math.hypot(v['scanner_mount_x'], v['scanner_mount_y'])
    lo, hi = derived['cross_aisle_x']
    assert (hi - lo) > diameter, (
        f'the cross aisle is {hi - lo:.4f} m against a {diameter:.4f} m '
        f'circumscribed diameter, so the vehicle cannot turn round in it and '
        f'the route can trap it')


def test_the_corner_claim_is_still_recorded_even_though_it_fails(spec, derived):
    """Widening the cross aisle must not quietly drop the claim.

    0.950 m is still a figure the datasheet publishes and the vehicle still
    cannot achieve it. The track no longer depends on it; the number stays.
    """
    t = spec['validation_targets']
    assert derived['corner_claim_m'] == pytest.approx(
        t['corridor_width_90_turn'], abs=1e-6)


def test_the_adversarial_case_has_not_been_deleted(spec):
    """Widening this track must not remove the hard case from the project.

    The AWS warehouse is still here and still has a 25th percentile corridor of
    0.64 m, narrower than the robot. Impossible geometry is measured THERE.
    This world measures what the vehicle can do. Losing the other world would
    turn a division of labour into a demo that only ever succeeds.
    """
    other = PKG / 'worlds' / 'warehouse.sdf'
    assert other.is_file(), (
        'the AWS warehouse is gone, so the only remaining world is one sized '
        'for the vehicle and nothing adversarial is measured anywhere')


def test_no_strip_is_too_narrow_for_the_vehicle_to_use(spec, derived):
    """Nothing on the track may be enterable but not usable.

    Widening the aisles pushed the racking down and left 0.543 m between the
    bottom rack and the south wall, against a 0.590 m vehicle. There is no rack
    west of the racking field, so the vehicle drove into that strip from the
    open bay and wedged: no valid path for the rest of the run.

    This is the corner fault in a different place, so it gets the same rule.
    Every band between a solid and the wall is either wide enough to turn in or
    does not exist.
    """
    turn = gen.rotation_width(spec)
    lows = [y0 for _, _, y0, _ in derived['solids']]
    bottom = min(lows)
    assert bottom <= 0.0 + 1e-9 or bottom >= turn, (
        f'a {bottom:.3f} m strip is left below the lowest solid, against a '
        f'{turn:.3f} m turning requirement; the vehicle can enter it and stop')


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


def test_the_scored_aisle_is_tight_but_passable(spec, derived):
    """P2 is now a TIGHT re-route rather than a forced wait.

    That is a decision, not a drift. The track was asked to make every
    manoeuvre possible for the vehicle, so the forced-wait case moved to the
    AWS world, where a 0.64 m corridor makes it unavoidable. Here the scored
    aisle has to be wide enough to pass a person and narrow enough that doing
    so is not trivial, or the pedestrian scenario proves nothing either way.
    """
    v = spec['values']
    width = 2.0 * v['scanner_mount_y']
    lo, hi = derived['aisle_2_y']
    person = 0.50
    assert (hi - lo) > width + person, (
        'the scored aisle cannot pass a person at all, so the vehicle is '
        'trapped rather than tested')
    assert (hi - lo) < width + person + 0.60, (
        'the scored aisle is so wide that passing a person is trivial')


def test_every_aisle_can_be_turned_round_in(spec, derived):
    """THE RULE THIS TRACK IS NOW BUILT TO. Nothing may trap the vehicle.

    Requiring a manoeuvre the vehicle cannot perform does not test it, it ends
    the run: fifteen survey rounds timed out at one pose and every other zone
    went unmeasured, V-27. Turning needs more room than driving through, so
    this is checked against the widest all-round field the vehicle can select
    while rotating, not against its footprint.
    """
    turn = gen.rotation_width(spec)
    for key in ('aisle_1_y', 'aisle_2_y', 'pinch_y', 'doorway_y'):
        lo, hi = derived[key]
        assert (hi - lo) >= turn - 1e-9, (
            f'{key} is {hi - lo:.4f} m against a turning requirement of '
            f'{turn:.4f} m, so the vehicle can be trapped there')
    lo, hi = derived['cross_aisle_x']
    assert (hi - lo) >= turn - 1e-9


def _superseded_test_the_track_contains_an_aisle_the_vehicle_cannot_turn_in(spec, derived):
    """A designed property, not an accident, and the track's first result.

    The MiR250's circumscribed diameter is 1.0021 m and its published
    `corridor_width_dynamic` is 1.0000 m, so it is 2.1 mm too large to rotate in
    the corridor its own datasheet claims. That is not a contradiction in the
    sheet: MiR quote the figure "with dynamic footprint and SICK safety
    configuration", and this stack plans with a STATIC footprint. The claim is
    therefore unreachable as configured, by arithmetic rather than by tuning.

    Asserted so the fact survives. It is also what makes the pedestrian pair
    work: an aisle a vehicle cannot turn in is one it cannot route around a
    person in either, so waiting is the correct behaviour there rather than a
    choice someone made.
    """
    v = spec['values']
    diameter = 2.0 * math.hypot(v['scanner_mount_x'], v['scanner_mount_y'])
    lo, hi = derived['aisle_2_y']
    assert (hi - lo) < diameter, (
        f'the scored aisle is {hi - lo:.4f} m against a circumscribed diameter '
        f'of {diameter:.4f} m, so the vehicle can now turn in it and the track '
        f'no longer tests the claim it was built for')


def test_the_widest_aisle_does_allow_a_turn(spec, derived):
    """The track must not be uniformly impossible.

    If every aisle forbade rotation the vehicle could never recover anywhere,
    and every failure would look the same. The default-footprint corridor is
    the one where turning is expected to work.
    """
    v = spec['values']
    diameter = 2.0 * math.hypot(v['scanner_mount_x'], v['scanner_mount_y'])
    lo, hi = derived['aisle_1_y']
    assert (hi - lo) > diameter, (
        f'aisle 1 is {hi - lo:.4f} m against a {diameter:.4f} m diameter, so '
        f'there is nowhere on the track the vehicle can turn')


def test_station_approach_poses_are_generated(spec):
    """Hand-authored approach poses are how one platform's pose outlived it."""
    st = yaml.safe_load(stations_path().read_text())
    names = {s['name'] for s in st['stations']}
    assert names == {'goods_in', 'dispatch'}
    assert st['route'] == ['goods_in', 'dispatch']
    for s in st['stations']:
        assert 'note' in s and s['note'], 'every station states how it is approached'


def test_stations_are_in_the_map_frame_not_world_coordinates(derived):
    """Goals are sent in `map`, whose origin SLAM puts at the spawn pose.

    Emitting world coordinates put the vehicle at map (0, 0) and asked it to
    drive to a point outside the building it had mapped. Three cycles, 0.3 m
    each, "no valid path found", which reads as a navigation failure and is a
    coordinate error. The offset is asserted rather than trusted.
    """
    st = yaml.safe_load(stations_path().read_text())
    sx, sy, _ = derived['spawn']
    # The file records poses to millimetre precision, so compare at that
    # precision rather than at floating point equality.
    assert st['spawn']['x'] == pytest.approx(sx, abs=1e-3)
    assert st['spawn']['y'] == pytest.approx(sy, abs=1e-3)
    for s in st['stations']:
        wx, wy = s['world_xy']
        assert s['x'] == pytest.approx(wx - sx, abs=2e-3)
        assert s['y'] == pytest.approx(wy - sy, abs=2e-3)


def test_the_vehicle_does_not_start_on_its_first_goal(derived):
    """A vehicle spawned on the station never demonstrates the leg."""
    sx, sy, _ = derived['spawn']
    gx, gy, _ = derived['stations_world']['goods_in']
    assert math.hypot(gx - sx, gy - sy) > 1.0, (
        'the spawn pose is on top of goods_in, so the first leg proves nothing')


def test_stations_sit_on_the_aisles_they_belong_to(derived):
    """A station off its own aisle is a goal the planner cannot reach cleanly."""
    a2_lo, a2_hi = derived['aisle_2_y']
    d_lo, d_hi = derived['doorway_y']
    gi = derived['stations_world']['goods_in']
    dp = derived['stations_world']['dispatch']
    assert a2_lo < gi[1] < a2_hi, 'goods_in is not on the dynamic-corridor aisle'
    assert d_lo < dp[1] < d_hi, 'dispatch is not aligned with the doorway'
