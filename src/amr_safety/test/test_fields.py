#!/usr/bin/env python3
"""Checks on the generated protective and warning fields.

These assert properties a protective field must have. They are not tests of the
collision monitor, which is upstream and tested there; they are tests that the
geometry handed to it is derived correctly and cannot silently drift when the
platform spec changes.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
GEN = PKG / 'tools' / 'generate_fields.py'
CFG = PKG / 'config' / 'collision_monitor.yaml'
SPEC = (Path(__file__).resolve().parents[2]
        / 'amr_description' / 'config' / 'platforms' / 'mir250_class.yaml')


def _load_generator():
    spec = importlib.util.spec_from_file_location('generate_fields', GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gen = _load_generator()


@pytest.fixture(scope='module')
def platform():
    return yaml.safe_load(SPEC.read_text())['values']


@pytest.fixture(scope='module')
def cfg():
    return yaml.safe_load(CFG.read_text())['collision_monitor']['ros__parameters']


def _polys(cfg, group):
    block = cfg[group]
    return {name: block[name] for name in block['velocity_polygons']}


def _points(poly):
    """Nav2 carries polygon points as a string, so parse them back."""
    import ast
    return ast.literal_eval(poly['points'])


def _reach(poly):
    """How far the field extends ahead of the robot centre."""
    return max(x for x, _ in _points(poly))


def test_stopping_distance_matches_the_hand_calculation(platform):
    v = 2.0
    expected = (
        v * (platform['scanner_response_time'] + platform['control_latency']
             + platform['brake_actuation_delay'])
        + v * v / (2.0 * platform['emergency_decel'])
        + platform['scanner_protective_supplement'])
    assert gen.stopping_distance(v, platform) == pytest.approx(expected)


def test_stopping_distance_grows_faster_than_speed(platform):
    """Braking is quadratic, so doubling the speed more than doubles the field.

    This is why a single fixed protective field cannot serve a vehicle with a
    wide speed range, and therefore why the fields are velocity switched.
    """
    a = gen.stopping_distance(1.0, platform)
    b = gen.stopping_distance(2.0, platform)
    assert b > 2.0 * a


def test_zero_speed_still_leaves_the_supplement(platform):
    """Even stationary, the scanner cannot resolve an object at the boundary."""
    assert gen.stopping_distance(0.0, platform) == pytest.approx(
        platform['scanner_protective_supplement'])


def test_every_protective_field_covers_its_stopping_distance(platform, cfg):
    """The property that makes it a protective field rather than a decoration.

    For each speed band, the field must reach at least the stopping distance at
    the TOP of that band, measured from the front face.
    """
    half_length = platform['chassis_length'] / 2.0
    for name, poly in _polys(cfg, 'protective').items():
        needed = gen.stopping_distance(poly['linear_max'], platform)
        have = _reach(poly) - half_length
        assert have >= needed - 1e-6, (
            f'{name} reaches {have:.3f} m ahead of the front face but needs '
            f'{needed:.3f} m to stop from {poly["linear_max"]} m/s')


def test_warning_field_always_encloses_the_protective_field(cfg):
    """Otherwise the vehicle would stop before it ever slowed down."""
    stop = _polys(cfg, 'protective')
    warn = _polys(cfg, 'warning')
    for s, w in zip(sorted(stop.values(), key=lambda p: p['linear_max']),
                    sorted(warn.values(), key=lambda p: p['linear_max'])):
        assert _reach(w) > _reach(s), (
            f'warning field reaches {_reach(w):.3f} m, protective '
            f'{_reach(s):.3f} m; the slowdown would never fire')


def test_speed_bands_are_contiguous_and_cover_the_full_range(platform, cfg):
    """A gap between bands is a speed at which NO protective field applies."""
    polys = sorted(_polys(cfg, 'protective').values(), key=lambda p: p['linear_max'])
    assert polys[0]['linear_min'] == pytest.approx(0.0)
    for a, b in zip(polys, polys[1:]):
        assert b['linear_min'] == pytest.approx(a['linear_max']), (
            f'gap between {a["linear_max"]} and {b["linear_min"]} m/s')
    assert polys[-1]['linear_max'] == pytest.approx(platform['max_linear_speed']), (
        'the fastest band does not reach the platform top speed, so the vehicle '
        'could travel at a speed no field covers')


def test_fields_are_at_least_as_wide_as_the_vehicle(platform, cfg):
    """A field narrower than the robot lets it drive its own corner into
    something the field never covered."""
    half_width = platform['chassis_width'] / 2.0
    for group in ('protective', 'warning'):
        for name, poly in _polys(cfg, group).items():
            widest = max(abs(y) for _, y in _points(poly))
            assert widest >= half_width, (
                f'{name} is {widest:.3f} m half-width against a vehicle '
                f'half-width of {half_width:.3f} m')


def test_polygon_points_are_the_string_form_nav2_requires(cfg):
    """Nav2 rejects a numeric array here, and the failure is opaque.

    Configuring with a double array fails with 'parameter points has invalid
    type', and the lifecycle manager reports only 'failed to change state'. The
    node had to be run standalone to see the real message.
    """
    for group in ('protective', 'warning'):
        for name, poly in _polys(cfg, group).items():
            assert isinstance(poly['points'], str), (
                f'{name} points is {type(poly["points"]).__name__}; nav2 needs a string')
            parsed = _points(poly)
            assert len(parsed) >= 3, f'{name} is not a polygon'
            assert all(len(pt) == 2 for pt in parsed)


def test_command_output_is_stamped(cfg):
    """The controller takes TwistStamped only, and the mismatch is silent.

    With plain Twist, both message types end up advertised on the same topic,
    diff_drive_controller ignores the one it does not want, and the robot never
    moves with no error logged anywhere. It presented as a permanent protective
    stop.
    """
    assert cfg['enable_stamped_cmd_vel'] is True


def test_the_source_is_the_merged_scan_and_not_the_classifier(cfg):
    """The architectural rule, asserted so it cannot be quietly changed.

    A protective function must not depend on classification. The people
    detector measures about 0.18 precision, while raw returns from a pedestrian
    are present in 100 percent of frames. Wiring the classifier in here would
    make safety depend on the weaker signal.
    """
    sources = cfg['observation_sources']
    assert sources == ['merged_scan']
    assert cfg['merged_scan']['topic'] == 'scan'
    for name in sources:
        topic = cfg[name]['topic']
        assert 'detection' not in topic and 'track' not in topic and 'people' not in topic, (
            f'observation source {name} consumes {topic}, which is a '
            f'classifier output; safety must not depend on it')


def test_generated_file_is_current(platform, tmp_path):
    """The config is generated. If someone edits it by hand, say so."""
    out = tmp_path / 'regenerated.yaml'
    import subprocess
    import sys
    subprocess.run(
        [sys.executable, str(GEN), '--out', str(out)],
        check=True, capture_output=True)
    fresh = yaml.safe_load(out.read_text())
    current = yaml.safe_load(CFG.read_text())
    assert fresh == current, (
        'config/collision_monitor.yaml differs from what the generator '
        'produces; it was hand-edited, or the spec changed without regenerating')
