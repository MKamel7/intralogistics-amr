"""The braking arithmetic every protective field rests on, and the probe's guards.

`iso_13855_distance` is the `v^2 / 2a` term. It is pure on purpose: it is the
part of the safety concept that can be checked without a simulator, and every
field in this project is sized through it.
"""

import importlib.util
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_braking.py'


def load():
    spec = importlib.util.spec_from_file_location('measure_braking', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def d(v, a):
    return load().iso_13855_distance(v, a)


def test_it_is_the_iso_13855_deceleration_term():
    # 0.75 m/s at 2.4 m/s2: 0.5625 / 4.8
    assert d(0.75, 2.4) == pytest.approx(0.1172, abs=1e-4)


def test_it_grows_with_the_square_of_speed():
    """The reason a commissioned speed is a safety decision and not a
    preference: doubling it quadruples this term."""
    assert d(1.5, 2.4) == pytest.approx(4.0 * d(0.75, 2.4), rel=1e-9)


def test_a_gentler_vehicle_needs_more_room():
    """The MiR250 publishes 0.3 m/s2 with maximum payload against the MP-400's
    single 2.4 m/s2 rating, and the difference is eightfold in this term."""
    assert d(0.75, 0.3) > d(0.75, 2.4)
    assert d(0.75, 0.3) / d(0.75, 2.4) == pytest.approx(8.0, rel=1e-9)


def test_nonsense_is_zero_rather_than_infinite():
    assert d(0.75, 0.0) == 0.0
    assert d(0.75, -1.0) == 0.0
    assert d(0.0, 2.4) == 0.0


def test_the_probe_arms_only_on_a_moving_vehicle():
    """V-56 in a different file.

    A stop command issued to a stationary vehicle brakes nothing. Arming there
    would leave the sample open until some later, unrelated stop and measure
    the interval between two events that were never connected, which is exactly
    how the latency tail was manufactured.
    """
    t = PROBE.read_text()
    assert 'if self.speed < self.min_speed' in t
    assert 'armed_while_slow' in t


def test_a_sample_that_never_closes_expires_and_is_counted():
    t = PROBE.read_text()
    assert 'self.expired += 1' in t
    assert 'expired after' in t, (
        'expiries are not reported, so the sample count cannot be reconciled '
        'with the number of protective stops in the run'
    )


def test_a_vehicle_told_to_drive_on_is_not_a_braking_sample():
    """It never finished stopping, so its distance is not a stopping distance
    and counting it would understate the figure."""
    t = PROBE.read_text()
    assert 'self.resumed += 1' in t


def test_distance_comes_from_ground_truth_and_not_from_odometry():
    """A braking wheel is the one most likely to be slipping.

    V-33 exists because wheel odometry disagreed with the world by 2.5 percent,
    and that error would land directly in this number.
    """
    t = PROBE.read_text()
    assert '/ground_truth/poses' in t
    code = '\n'.join(ln for ln in t.splitlines() if not ln.strip().startswith('#'))
    assert '/odom' not in code, 'the probe reads odometry, which slips under braking'


def test_it_says_this_is_the_optimistic_figure():
    """A service stop is not an emergency stop, and a safety argument built on
    the softer of the two without saying so is the kind of claim this project
    exists to avoid making."""
    t = PROBE.read_text()
    assert 'optimistic figure' in t
