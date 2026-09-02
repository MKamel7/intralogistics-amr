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


def test_the_final_turn_is_recorded():
    """The quantity that discriminates the goods_in asymmetry.

    goods_in sits west and dispatch east on an alternating route, so the
    vehicle arrives at goods_in having driven west and must end facing east: a
    180 degree spot turn AT the goal. At dispatch it turns nothing.

    If the spot turn is what costs the accuracy, the heading error should
    OPPOSE the direction of that turn, because the goal checker stops the
    rotation as soon as it is inside tolerance and therefore undershoots.
    Without recording the turn the two explanations are indistinguishable.
    """
    t = PROBE.read_text()
    assert 'yaw_history' in t
    assert 'after turning' in t, 'the turn is computed but never reported'
    assert 'OPPOSING the turn' in t, (
        'the report does not test the sign relationship the hypothesis rests on')


def at(history, t):
    return load().pose_at(history, t)


def test_a_pose_between_two_samples_is_interpolated_not_snapped():
    """The fix V-66 asked for: score a detection against where the vehicle WAS.

    Taking the latest sample instead injects the vehicle's own motion into the
    detector's error, which is how a 43.4 mm p50 became an upper bound rather
    than a measurement.
    """
    history = [(10.0, 0.0, 0.0, 0.0), (10.1, 0.30, 0.0, 0.0)]

    assert at(history, 10.05)[0] == pytest.approx(0.15)
    assert at(history, 10.0)[0] == pytest.approx(0.0)
    assert at(history, 10.1)[0] == pytest.approx(0.30)


def test_the_skew_it_removes_is_the_size_v66_predicted():
    """At 0.3 m/s, half a 100 ms ground truth period is 15 mm of error."""
    history = [(0.0, 0.0, 0.0, 0.0), (0.1, 0.03, 0.0, 0.0)]

    interpolated = at(history, 0.05)[0]
    latest_sample = history[-1][1]

    assert abs(latest_sample - interpolated) == pytest.approx(0.015, abs=1e-9)


def test_yaw_interpolates_the_short_way_round():
    """A sample either side of pi must not produce a pose facing backwards."""
    history = [(0.0, 0.0, 0.0, math.pi - 0.05), (1.0, 0.0, 0.0, -math.pi + 0.05)]

    yaw = at(history, 0.5)[2]

    assert abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))) < 0.02


def test_a_stamp_outside_the_history_is_refused_rather_than_extrapolated():
    """Extrapolating would reinstate the invented number this replaces."""
    history = [(5.0, 0.0, 0.0, 0.0), (6.0, 1.0, 0.0, 0.0)]

    assert at(history, 4.9) is None
    assert at(history, 6.1) is None
    assert at([], 5.0) is None
    assert at([(5.0, 0.0, 0.0, 0.0)], 5.0) is None
