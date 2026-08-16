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


def test_contacts_are_attributed_by_vehicle_speed():
    """A person walking into a parked robot is not the robot's failure.

    Measured: of four contacts in one mission, two happened with the vehicle
    at 0.00 m of travel for the whole minute around them. These pedestrians do
    not avoid the vehicle by design, because a crowd that dodges never tests
    anything, so a probe that blames the robot for every touch is measuring
    the scenario rather than the stack.
    """
    t = text()
    assert 'contact_speeds' in t
    assert 'they walked into the vehicle' in t
    assert 'THE VEHICLE DROVE INTO THEM' in t
    # And the speed alone is no longer what decides it. See the closing_split
    # tests below: 0.03 m/s counted as "moving" and was labelled as the
    # vehicle's fault while the person did all of the closing.
    assert 'contact_closing' in t


def test_the_headline_separates_moving_contacts():
    """The number that matters is contacts with the vehicle moving. Reporting
    a single total invites the reading that the safety layer failed four times
    when it may have failed none."""
    t = text()
    assert 'of which the vehicle was MOVING' in t


def test_a_stationary_contact_is_named_a_scenario_artefact():
    t = text()
    assert 'scenario' in t and 'not a safety failure' in t


# ---------------------------------------------------------------------------
# Who closed the distance. The vehicle's own speed cannot answer this, and it
# was being asked to: a contact at 0.03 m/s was labelled DRIVING INTO THEM
# while the person was walking at over a metre a second.

def split(v_vehicle, v_person, offset):
    return load().closing_split(v_vehicle, v_person, offset)


def blame(vs, ps):
    return load().blame(vs, ps)


def test_a_stationary_vehicle_gets_none_of_the_blame():
    """Person stands 2 m west of a parked robot and walks east into it."""
    vs, ps = split((0.0, 0.0), (1.2, 0.0), (-2.0, 0.0))
    assert vs == pytest.approx(0.0)
    assert ps == pytest.approx(1.2)
    assert blame(vs, ps) == 'they walked into the vehicle'


def test_a_person_walking_away_is_not_closing():
    """Same geometry, opposite direction. The sign must follow the motion and
    not the fact that somebody is nearby."""
    vs, ps = split((0.0, 0.0), (-1.2, 0.0), (-2.0, 0.0))
    assert ps == pytest.approx(-1.2)


def test_a_driving_vehicle_takes_the_blame():
    vs, ps = split((0.9, 0.0), (0.0, 0.0), (3.0, 0.0))
    assert vs == pytest.approx(0.9)
    assert ps == pytest.approx(0.0)
    assert blame(vs, ps) == 'THE VEHICLE DROVE INTO THEM'


def test_the_creeping_case_that_prompted_this():
    """0.03 m/s toward a person walking at 1.2 m/s toward the vehicle.

    The old rule called this DRIVING INTO THEM because 0.03 exceeds the 0.02
    movement threshold. It is a person walking into a vehicle that is barely
    moving, and the difference matters to every claim built on the count.
    """
    vs, ps = split((0.03, 0.0), (-1.2, 0.0), (2.0, 0.0))
    assert vs == pytest.approx(0.03)
    assert ps == pytest.approx(1.2)
    assert blame(vs, ps) == 'they walked into the vehicle'


def test_moving_apart_is_negative_not_zero():
    """A party retreating must not read as one standing still.

    Otherwise a vehicle reversing away from someone who runs after it would
    score the same as one that stood there, and the sign is the whole point.
    """
    vs, ps = split((-0.5, 0.0), (0.0, 0.0), (2.0, 0.0))
    assert vs == pytest.approx(-0.5)


def test_both_closing_names_neither():
    vs, ps = split((0.6, 0.0), (-0.6, 0.0), (2.0, 0.0))
    assert blame(vs, ps) == 'both were closing; neither dominates'


def test_an_unknown_velocity_is_not_reported_as_zero():
    """The first ground truth frame for a body has no previous sample.

    Returning a confident (0, 0) there would silently credit the other party
    with all of the closing on the very sample where a contact is most likely
    to be first seen.
    """
    assert split(None, (1.0, 0.0), (2.0, 0.0)) == (0.0, 0.0)
    assert split((1.0, 0.0), None, (2.0, 0.0)) == (0.0, 0.0)
    assert blame(0.0, 0.0).startswith('neither was closing')


def test_a_coincident_pose_does_not_divide_by_zero():
    assert split((1.0, 0.0), (0.0, 0.0), (0.0, 0.0)) == (0.0, 0.0)


def test_closing_is_projected_onto_the_line_not_the_speed():
    """Motion across the line closes nothing.

    A vehicle passing a person at a metre a second on a parallel track is not
    approaching them, and counting its speed would say it was.
    """
    vs, _ = split((0.0, 1.0), (0.0, 0.0), (2.0, 0.0))
    assert vs == pytest.approx(0.0)
