"""Parked accuracy, and the arithmetic that says what limits it."""

import importlib.util
import math
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_docking.py'


def load():
    spec = importlib.util.spec_from_file_location('measure_docking', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def err(vx, vy, vyaw, sx, sy, syaw):
    return load().pose_error(vx, vy, vyaw, sx, sy, syaw)


def test_a_vehicle_on_the_station_has_no_error():
    d, dyaw = err(2.5, 5.5, 0.0, 2.5, 5.5, 0.0)
    assert d == pytest.approx(0.0)
    assert dyaw == pytest.approx(0.0)


def test_distance_is_euclidean_and_heading_is_separate():
    """They do not trade off. A vehicle perfectly placed and facing backwards
    has docked at nothing, and averaging the two would hide it."""
    d, dyaw = err(2.6, 5.5, 0.0, 2.5, 5.5, 0.0)
    assert d == pytest.approx(0.1)
    assert dyaw == pytest.approx(0.0)


def test_heading_error_wraps_the_short_way():
    """A vehicle 359 degrees off is 1 degree off.

    Without the wrap every mean over a set of arrivals is meaningless, because
    one wrapped sample dominates the rest.
    """
    _, dyaw = err(0, 0, math.radians(359), 0, 0, 0.0)
    assert abs(math.degrees(dyaw)) == pytest.approx(1.0, abs=1e-6)
    _, dyaw = err(0, 0, -math.pi + 0.01, 0, 0, math.pi - 0.01)
    assert abs(dyaw) < 0.05


def test_the_probe_requires_the_vehicle_to_settle():
    """Stopped is not parked.

    A vehicle pausing mid manoeuvre is stationary for a moment, and counting
    that pose would mix an intermediate position into the arrival figures.
    """
    t = PROBE.read_text()
    assert 'settle' in t
    assert 'now - self.since < self.settle' in t


def test_truth_comes_from_the_oracle_and_the_station_from_the_generator():
    """Both halves of the comparison have to be right.

    The vehicle's true pose can only come from the oracle. The station's pose
    has to come from the same file the mission drives to, or the probe and the
    mission would disagree about where the station is and the error would be
    that disagreement.
    """
    t = PROBE.read_text()
    assert '/ground_truth/poses' in t
    assert 'stations_file' in t and 'world_xy' in t


def test_it_says_the_localisation_error_is_the_floor():
    """The finding this probe exists to support or refute.

    A vehicle cannot park more accurately than it can locate itself while the
    goal is expressed in the map frame, so tightening xy_goal_tolerance below
    the localisation error buys nothing and costs goal-reached timeouts.
    """
    t = PROBE.read_text()
    assert 'cannot park more accurately' in t
