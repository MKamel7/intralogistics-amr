"""Social navigation metrics: the pure arithmetic, and the guards.

Every one of these corresponds to a way a metric in this project has already
been made meaningless: a denominator that flattered the result, a verdict
written on a correct measurement, or an integration that counted time the
probe was starved.
"""

import importlib.util
import math
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_social.py'


def load():
    spec = importlib.util.spec_from_file_location('measure_social', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def text():
    return PROBE.read_text()


def test_time_to_collision_is_infinite_when_not_closing():
    """A vehicle driving away has no time to collision. Returning a large
    number instead would drag any median toward a value that means nothing."""
    assert math.isinf(load().time_to_collision(1.0, 0.0))
    assert math.isinf(load().time_to_collision(1.0, 0.0005))


def test_time_to_collision_is_distance_over_closing_speed():
    assert load().time_to_collision(2.0, 0.5) == pytest.approx(4.0)


def test_time_to_collision_is_never_negative():
    """Inside the footprint the gap is negative; the answer is zero seconds,
    not a negative time."""
    assert load().time_to_collision(-0.2, 1.0) == pytest.approx(0.0)


def test_clearance_matches_the_contact_probe():
    """Two probes disagreeing about what clearance means would be worse than
    duplicating the function."""
    import importlib.util as iu
    other = PROBE.parent / 'measure_contacts.py'
    s = iu.spec_from_file_location('mc', other)
    mc = iu.module_from_spec(s)
    s.loader.exec_module(mc)
    for x, y in ((0.0, 0.0), (0.5, 0.0), (0.0, 0.5), (1.0, 1.0), (0.31, 0.29)):
        assert load().clearance_to_footprint(x, y, 0.3, 0.2845) == pytest.approx(
            mc.clearance_to_footprint(x, y, 0.3, 0.2845))


def test_zones_are_ordered_and_labelled():
    z = load().ZONES
    radii = [r for _, r in z]
    assert radii == sorted(radii), 'zones must nest, not overlap arbitrarily'
    assert [n for n, _ in z] == ['intimate', 'personal', 'social']


def test_the_denominator_is_time_in_range_not_run_length():
    """A run spent in an empty aisle must not score as polite. An earlier
    probe here counted people twenty metres away behind racking and produced
    numbers that looked ordinary and meant nothing."""
    t = text()
    assert 'attention_range' in t
    assert 'is not being polite, it is being absent' in t


def test_starved_samples_are_discarded_not_integrated():
    """A gap in the probe's own scheduling is not time the vehicle spent
    beside somebody."""
    t = text()
    assert 'dt > 0.5' in t


def test_an_empty_run_is_not_reported_as_a_good_result():
    t = text()
    assert 'NOBODY CAME WITHIN RANGE' in t
    assert 'it is an empty run' in t


def test_the_zones_are_labelled_as_anthropology():
    """Hall's zones are not a datasheet, and this project fails a build when a
    constant loses its provenance."""
    t = text()
    assert 'anthropology' in t
    assert 'No safety' in t and 'derives from them' in t


def test_it_never_writes_anything():
    t = text()
    for bad in ('.write_text(', 'yaml.dump', 'open('):
        assert bad not in t


def test_it_announces_itself_on_startup():
    """A probe that prints nothing until it reports is indistinguishable from
    one that died on import. Every other probe in this project says it
    started; this one did not, and a zero byte log was briefly read as a
    crash."""
    assert 'social metrics for' in text()
