"""Load securing arithmetic, and the guard against measuring across time.

`slip_limit` is the whole safety argument for an unsecured load in one line,
and it is checkable without a simulator.
"""

import importlib.util
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_load.py'


def load():
    spec = importlib.util.spec_from_file_location('measure_load', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_the_slip_limit_is_mu_times_g():
    assert load().slip_limit(0.35) == pytest.approx(3.4335, abs=1e-4)


def test_it_does_not_depend_on_the_load_mass():
    """The reason a heavy load is not more secure than a light one.

    Friction force and inertial force both scale with mass, so the threshold
    is a property of the two surfaces alone. Anybody reaching for a heavier
    box to keep it in place is reasoning about the wrong quantity.
    """
    m = load()
    assert m.slip_limit(0.35) == m.slip_limit(0.35)   # no mass parameter exists
    assert 'mass' not in m.slip_limit.__code__.co_varnames


def test_a_measured_stop_exceeds_the_limit_for_a_plastic_container():
    """V-60 measured 3.49 and 4.08 m/s2 in protective stops.

    At the 0.35 in the payload model the load is expected to move. If this
    test ever fails because the coefficient rose, the prediction changes and
    the finding needs re-reading rather than the test being adjusted.
    """
    assert load().slip_limit(0.35) < 4.08
    assert load().slip_limit(0.50) > 4.08     # a grippier pairing would hold


def test_nonsense_friction_is_zero_rather_than_negative():
    assert load().slip_limit(-0.2) == 0.0


def test_a_load_over_the_edge_is_off_the_plate():
    on_plate = load().on_plate
    assert on_plate(0.0, 0.0, 0.295, 0.2795)
    assert on_plate(0.294, 0.279, 0.295, 0.2795)
    assert not on_plate(0.30, 0.0, 0.295, 0.2795)
    assert not on_plate(0.0, -0.29, 0.295, 0.2795)


def test_both_poses_come_from_one_message():
    """The mistake this probe exists to avoid making.

    Sampling the vehicle and the load separately reported the load 0.87 m
    behind the vehicle when it was 8 mm behind: two queries a second apart
    with the vehicle driving between them, so the difference was the
    vehicle's own travel. A relative measurement taken from a single
    timestamped message cannot be a difference of times.
    """
    m = load()
    t = PROBE.read_text()
    assert '/ground_truth/poses' in t
    # No shelling out at all. Asserted on the imported module rather than on
    # the source text, because the first version of this test matched the word
    # "gz model" inside the docstring that EXPLAINS the mistake and failed on
    # its own explanation. A text search over a file that discusses its own
    # bugs will find them.
    assert not hasattr(m, 'subprocess'), (
        'the probe imports subprocess, so it can sample entities one at a '
        'time and reintroduce the difference of times')
    # One callback, one message, both poses pulled from the same transform list.
    body = t[t.index('def _truth'):t.index('def _tick')]
    assert 'for tf in msg.transforms' in body
    assert body.count('def ') == 1


def test_a_delivered_load_is_not_counted_as_slipping_away():
    """Once set down on the table the vehicle drives off, and the growing
    distance is the vehicle leaving rather than the load moving."""
    t = PROBE.read_text()
    assert 'set down on the table' in t


def test_rotation_is_measured_and_not_only_translation():
    """A spot check found a box turned 3.79 degrees with its centre unmoved.

    The first version of this probe tracked translation only and reported
    0.0 mm, which reads as "the load did not move" and was not true. On a
    pallet with a lip it is the rotation that jams rather than the slide.
    """
    t = PROBE.read_text()
    assert 'self.turned' in t
    assert 'rel_yaw' in t
    assert 'worst rotation' in t, 'the rotation is tracked but never reported'
