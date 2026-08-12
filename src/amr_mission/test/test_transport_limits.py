#!/usr/bin/env python3
"""The transport task must drive the vehicle it is actually on.

WHAT THIS CATCHES. `transport_task.py` declares its acceleration limits as ROS
parameters with the MiR250's 0.3 and 1.0 m/s2 as defaults. Nothing passed them,
so the default WAS the value, and the MP-400 ran a full five-cycle transport
task at a fifth of its own published rating while every log line reported the
MiR250's figure as though it had come from the spec.

Nothing failed. The cycles completed. That is what makes it worth a test: the
symptom of driving a vehicle on another vehicle's dynamics is not an error, it
is a slower number that looks like a result.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'transport.launch.py'
SPEC_DIR = (Path(__file__).resolve().parents[2]
            / 'amr_description' / 'config' / 'platforms')
NAV2_DIR = Path(__file__).resolve().parents[2] / 'amr_navigation' / 'config'


def _load_launch():
    spec = importlib.util.spec_from_file_location('transport_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


launch_mod = _load_launch()


@pytest.fixture(params=sorted(p.stem for p in SPEC_DIR.glob('*.yaml')))
def platform_name(request):
    return request.param


def test_acceleration_limits_come_from_the_platform_spec(platform_name):
    """The spec is the source, clamped by the navigation envelope."""
    values = yaml.safe_load(
        (SPEC_DIR / f'{platform_name}.yaml').read_text())['values']
    cap = launch_mod.envelope_cap(platform_name, nav2_dir=NAV2_DIR)
    laden, unladen = launch_mod.accel_limits(
        platform_name, spec_dir=SPEC_DIR, nav2_dir=NAV2_DIR)
    assert laden == pytest.approx(min(values['max_linear_accel'], cap))
    assert unladen == pytest.approx(min(values['max_linear_accel_unladen'], cap))


def test_the_mission_cannot_widen_the_navigation_envelope(platform_name):
    """set_payload writes max_accel onto the smoother at RUNTIME.

    It does not touch max_decel, so a figure larger than the envelope replaces
    the symmetric limits the generator derived with asymmetric ones, on the
    last rate limiter before the wheels. Measured on the MP-400: the generated
    configuration held 1.00 against -1.00 and the mission set 2.40 the moment
    it reported "unloaded". Both directions of the switch are checked, because
    the laden figure is written by the same call.
    """
    cap = launch_mod.envelope_cap(platform_name, nav2_dir=NAV2_DIR)
    for value in launch_mod.accel_limits(
            platform_name, spec_dir=SPEC_DIR, nav2_dir=NAV2_DIR):
        assert value <= cap + 1e-9, (
            f'the mission would set the smoother to {value} m/s2 against a '
            f'navigation envelope of {cap} m/s2')


def test_laden_is_never_the_more_permissive_of_the_two(platform_name):
    """A payload cannot make a vehicle accelerate harder.

    On a platform whose manual publishes a single rating, as the MP-400's does,
    the two are equal and the switching is a no-op. That is correct rather than
    a special case, so this allows equality and forbids only the direction that
    would mean the load-retention limit had been read backwards.
    """
    laden, unladen = launch_mod.accel_limits(platform_name, spec_dir=SPEC_DIR, nav2_dir=NAV2_DIR)
    assert laden <= unladen


def test_an_unknown_platform_fails_loudly(tmp_path):
    """No fallback. A missing spec must stop the launch, not default to one.

    Defaulting is what produced the fault this file exists for.
    """
    with pytest.raises(RuntimeError, match='no platform spec'):
        launch_mod.accel_limits('no_such_platform', spec_dir=SPEC_DIR,
                                nav2_dir=NAV2_DIR)


def test_the_defaults_in_the_node_are_not_silently_authoritative():
    """The node's own defaults must stay labelled as a bare-run fallback.

    They are kept deliberately, so the node runs standalone, and the risk is
    that someone reads them as the source again. The comment beside them is the
    guard, so this asserts the comment is still there and still says so.
    """
    node = (Path(__file__).resolve().parents[1] / 'amr_mission'
            / 'transport_task.py').read_text()
    assert 'PASSED IN from the platform spec' in node, (
        'the note marking the acceleration defaults as a bare-run fallback has '
        'gone; without it the next reader will take them for the source again')
