#!/usr/bin/env python3
"""The experiment runner must not average away the thing that invalidates a run.

Its whole purpose is to stop a conclusion being drawn from one run. It would be
a poor tool if it then quietly folded an unhealthy run into the mean, which is
precisely the mistake it exists to prevent: the run that appeared to confirm a
fix was the only one whose keepout mask never published.
"""

import importlib.util
from pathlib import Path


TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'experiment.py'


def _load():
    spec = importlib.util.spec_from_file_location('experiment', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


exp = _load()

MISSION = """
[transport_task-1] [INFO] cycle 1: complete in 76 s, 19.7 m driven, 0 protective stop(s), 0 s held up
[transport_task-1] [INFO] cycle 2: complete in 107 s, 25.5 m driven, 1 protective stop(s), 0 s held up
[transport_task-1] [INFO] cycle 3: INCOMPLETE in 70 s, 15.1 m driven, 1 protective stop(s), 2 s held up
[transport_task-1] [INFO] 2 of 3 cycle(s) completed
"""


def _make_run(tmp_path, mission=MISSION, nav='', stage='preflight exit 0\n'):
    d = tmp_path / 'run'
    d.mkdir(exist_ok=True)
    (d / 'mission.log').write_text(mission)
    (d / 'nav.log').write_text(nav)
    (d / 'stage.log').write_text(stage)
    return d


def test_it_reads_only_completed_cycles(tmp_path):
    """An INCOMPLETE cycle has no cycle time worth averaging.

    Folding a failed cycle's 70 s into the mean would make a run that failed
    look faster than one that succeeded.
    """
    r = exp.parse_run(_make_run(tmp_path))
    assert r['cycle_times'] == [76, 107]
    assert r['cycle_distances'] == [19.7, 25.5]
    assert r['completed'] == 2 and r['attempted'] == 3


def test_a_run_without_its_keepout_mask_is_excluded(tmp_path):
    """THE RETRACTION, asserted.

    A run whose mask never published is not a sample of the same system. It is
    reported rather than dropped, so the exclusion is visible.
    """
    r = exp.parse_run(_make_run(
        tmp_path, nav='[costmap] KeepoutFilter: Filter mask was not received\n'))
    assert r['healthy'] is False
    assert any('keepout' in n for n in r['notes'])


def test_a_run_that_failed_preflight_is_excluded(tmp_path):
    r = exp.parse_run(_make_run(tmp_path, stage='preflight exit 1\n'))
    assert r['healthy'] is False
    assert any('preflight' in n for n in r['notes'])


def test_a_run_that_never_started_is_excluded_not_zeroed(tmp_path):
    """No mission log means no data, which is not the same as a zero."""
    d = tmp_path / 'empty'
    d.mkdir()
    r = exp.parse_run(d)
    assert r['healthy'] is False
    assert r['completed'] is None, 'a missing result must not become 0'


def test_spread_reports_range_and_refuses_a_stdev_of_one_sample():
    """One sample has no spread, and saying 0.0 would imply it did."""
    d = exp.spread([76.0, 107.0, 90.0])
    assert d['n'] == 3 and d['min'] == 76.0 and d['max'] == 107.0
    assert d['stdev'] is not None
    assert exp.spread([76.0])['stdev'] is None
    assert exp.spread([]) is None


def test_the_summary_line_names_the_range_not_just_the_mean():
    """A mean alone is what made n=1 look like a result in the first place."""
    text = exp.fmt(exp.spread([10.0, 30.0]), ' m')
    assert '10.0' in text and '30.0' in text and 'n=2' in text
