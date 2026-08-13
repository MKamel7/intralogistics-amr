"""The contact geometry, and the guards on the probe that reports it.

`clearance_to_footprint` is a pure function on purpose: it is the arithmetic
every safety claim in this project would rest on, and it should be checkable
without a simulator.
"""

import importlib.util
import math
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_contacts.py'


def load():
    spec = importlib.util.spec_from_file_location('measure_contacts', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def text():
    return PROBE.read_text()


HL, HW = 0.300, 0.2845          # MP-400 footprint half extents


def clear(x, y):
    return load().clearance_to_footprint(x, y, HL, HW)


def test_the_centre_is_deepest_inside():
    assert clear(0.0, 0.0) == pytest.approx(-HW)


def test_a_point_on_the_edge_is_zero():
    assert clear(HL, 0.0) == pytest.approx(0.0)
    assert clear(0.0, HW) == pytest.approx(0.0)


def test_a_point_ahead_measures_from_the_front_face():
    assert clear(HL + 0.5, 0.0) == pytest.approx(0.5)


def test_a_point_beside_measures_from_the_side_face():
    assert clear(0.0, HW + 0.25) == pytest.approx(0.25)


def test_a_diagonal_point_measures_from_the_CORNER_not_the_face():
    """The reason this is a rectangle and not a circle.

    A circular approximation of a 0.59 by 0.559 m vehicle is wrong by about
    12 percent at the corners, and every margin in this project is of that
    order.
    """
    d = clear(HL + 0.3, HW + 0.4)
    assert d == pytest.approx(math.hypot(0.3, 0.4))
    assert d > 0.4, 'a corner is further away than either face alone'


def test_it_is_symmetric_in_both_axes():
    for x, y in ((0.8, 0.5), (0.2, 0.9), (1.5, 0.0)):
        assert clear(x, y) == pytest.approx(clear(-x, y))
        assert clear(x, y) == pytest.approx(clear(x, -y))


def test_inside_is_negative_and_outside_is_positive():
    assert clear(0.1, 0.1) < 0.0
    assert clear(2.0, 2.0) > 0.0


# --- guards on the probe ---------------------------------------------------

def test_a_person_is_not_treated_as_a_point():
    """The person model carries a 0.22 m collision puck. Treating them as a
    point understates every clearance by that much."""
    t = text()
    assert 'PERSON_RADIUS = 0.22' in t
    assert 'gap -= PERSON_RADIUS' in t


def test_contacts_are_counted_on_the_rising_edge():
    """A person standing inside the vehicle for three seconds is one contact,
    not sixty. Counting samples would make the number a function of the
    publication rate."""
    assert 'in_contact' in text()


def test_it_does_not_count_the_vehicle_against_itself():
    """The ground truth stream carries the vehicle's own body link."""
    assert "'body'" in text(), 'the vehicle body link must be excluded'


def test_no_samples_is_reported_as_no_measurement():
    """Reporting zero contacts from zero samples would be the most flattering
    possible bug: a perfect safety record from a probe that heard nothing."""
    t = text()
    assert 'NO GROUND TRUTH RECEIVED' in t
    assert 'it is no run at all' in t


def test_zero_contacts_is_not_overclaimed():
    """People here cannot physically stop the vehicle, so an absence of
    contact is evidence the stack kept clear, not evidence that anything
    would have prevented a collision."""
    t = text()
    assert 'evidence the stack kept clear' in t


def test_it_never_writes_anything():
    t = text()
    for bad in ('.write_text(', 'yaml.dump', 'open('):
        assert bad not in t, f'the probe must not write ({bad})'
