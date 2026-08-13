"""Guards on the path efficiency probe.

The arithmetic is trivial; what matters is that the two overheads stay
separated, because conflating them is how a project tunes the planner for a
week to fix a controller.
"""

import importlib.util
import math
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_path_efficiency.py'


def load():
    spec = importlib.util.spec_from_file_location('mpe', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_polyline_length_of_a_straight_line_is_its_length():
    assert load().polyline_length([(0, 0), (3, 4)]) == pytest.approx(5.0)


def test_polyline_length_sums_segments():
    assert load().polyline_length(
        [(0, 0), (1, 0), (1, 1), (0, 1)]) == pytest.approx(3.0)


def test_a_single_point_has_no_length():
    assert load().polyline_length([(2, 2)]) == pytest.approx(0.0)


def test_a_detour_is_longer_than_the_straight_line():
    """The property the planner overhead term relies on."""
    straight = math.dist((0, 0), (10, 0))
    detour = load().polyline_length([(0, 0), (5, 3), (10, 0)])
    assert detour > straight


def test_the_two_overheads_are_reported_separately():
    """Planner and controller overhead answer different questions and imply
    different fixes. A single combined number cannot be acted on."""
    t = PROBE.read_text()
    assert 'planner overhead' in t
    assert 'controller overhead' in t
    assert 'THE CONTROLLER OWNS THE OVERHEAD' in t
    assert 'THE PLANNER OWNS THE OVERHEAD' in t


def test_it_says_when_neither_is_at_fault():
    """Re-routing around a person is legitimate overhead. A probe that always
    blames something would send someone tuning a healthy system."""
    assert 'the system working rather than a' in PROBE.read_text()


def test_a_new_goal_is_distinguished_from_a_re_plan():
    """Nav2 re-plans continuously toward the same goal. Treating each re-plan
    as a new leg would reset the driven distance constantly and report an
    overhead near 1.0 whatever the vehicle did."""
    t = PROBE.read_text()
    assert 'NEW_GOAL' in t
    assert 'self.replans' in t


def test_no_legs_is_reported_as_no_measurement():
    assert 'NO COMPLETE LEGS' in PROBE.read_text()


def test_it_never_writes_anything():
    t = PROBE.read_text()
    for bad in ('.write_text(', 'yaml.dump', 'open('):
        assert bad not in t
