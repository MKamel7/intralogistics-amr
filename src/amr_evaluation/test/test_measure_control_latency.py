"""The split that attributes the latency tail, checked without a simulator.

V-44 measured p50 68 ms against p99 1260 ms and named two candidates for the
tail without being able to separate them. The probe now splits each sample at
the collision monitor, into the part before the decision and the part after,
and records how long the scan and command streams stalled alongside it.

`_widest_gap` is the stall detector and it is a pure function, so it is checked
here. The arithmetic that attributes a safety number should be checkable
without bringing up Gazebo.
"""

import importlib.util
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[3] / 'tools' / 'measure_control_latency.py'


def load():
    spec = importlib.util.spec_from_file_location('measure_control_latency', PROBE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def gap(arrivals, now):
    return load().LatencyProbe._widest_gap(arrivals, now)


def test_an_empty_history_is_not_a_stall():
    """Zero reads as "nothing seen", which is what it is.

    Reporting an unknown as a large gap would attribute every early sample to
    a stall that was really just the probe having started.
    """
    assert gap([], 10.0) == 0.0


def test_a_steady_stream_has_a_small_gap():
    arrivals = [10.0 + 0.07 * i for i in range(14)]
    assert gap(arrivals, arrivals[-1]) == pytest.approx(0.07, abs=1e-9)


def test_a_hole_in_the_middle_is_found():
    assert gap([10.0, 10.07, 10.62, 10.69], 10.69) == pytest.approx(0.55)


def test_a_stream_that_stopped_is_the_case_that_matters():
    """The gap to NOW, not just the gaps between arrivals.

    A stream that stopped dead half a second ago has small pairwise gaps and is
    the whole signature being looked for. Diffing history alone would report it
    as healthy, which is the failure mode this test exists to prevent.
    """
    assert gap([10.0, 10.07, 10.14], 10.64) == pytest.approx(0.50)


def test_only_the_last_second_counts():
    """Older arrivals must not dominate. The tail being attributed is about a
    second long, so a stall that explains a sample is inside that window."""
    assert gap([1.0, 9.8, 9.87, 9.94], 10.0) == pytest.approx(0.07, abs=1e-9)


def test_the_probe_still_refuses_to_write_the_spec():
    """The guard that matters more than any number here.

    control_latency feeds every protective field through ISO 13855. A probe
    that edited the spec from its own measurement would be one bad run away
    from shrinking every field in the project.
    """
    text = PROBE.read_text()
    assert 'CANDIDATE, not a decision' in text
    code = '\n'.join(ln for ln in text.splitlines() if not ln.strip().startswith('#'))
    assert 'yaml' not in code.lower(), (
        'the probe imports or writes yaml, which is how it would come to edit '
        'the platform spec it is measuring')


def test_the_attribution_reports_no_tail_rather_than_inventing_one():
    """A quiet run must say so.

    The failure this guards is the one V-38 already made once: reading a clean
    number as a good result when it actually meant nothing was measured.
    """
    text = PROBE.read_text()
    assert 'NO TAIL IN THIS RUN' in text
    assert 'not evidence the tail is gone' in text
