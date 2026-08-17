#!/usr/bin/env python3
"""Where the PROTECTIVE STOP badge lands.

The badge is the only thing in the demo video that makes a claim about a
specific moment, so its position is arithmetic and the arithmetic is testable.
Getting it wrong is not visible: a badge a second late still sits over a
vehicle that is stopped, because the vehicle is stopped for a while either
side, and it would be captioning the wrong instant with nothing to show it.

Two conversions have to hold at once. The clip starts `ss` SIMULATED seconds
after the camera's first frame, and the output runs on a uniform grid from
there, so a frame's output time is its own stamp minus the clip's start stamp
and never its index divided by the rate. That second form is the mistake this
project already made once, in the shot selector, where it put the cut two to
four times further into the recording than the window that had been ranked.
"""

import importlib.util
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'demo_cut.py'


def load():
    spec = importlib.util.spec_from_file_location('demo_cut', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def make(actions, hz=10.0, t_start=100.0):
    """Rows and stamps for a camera running at `hz`, one action per frame."""
    ts = [t_start + i / hz for i in range(len(actions))]
    rows = [{'action': a, 'polygon': 'protective' if a == 'stop' else ''}
            for a in actions]
    return rows, ts


def test_a_stop_lands_at_its_own_offset_into_the_clip():
    """A stop three seconds into an eight second clip shows up at three."""
    d = load()
    actions = ['clear'] * 100 + ['stop'] * 10 + ['clear'] * 100
    rows, ts = make(actions)                    # stop runs 10.0 s to 11.0 s
    got = d.stop_intervals(rows, ts, 7.0, 8.0)  # clip covers 7.0 s to 15.0 s
    assert len(got) == 1, got
    start, end = got[0]
    assert start == pytest.approx(3.0 - 0.15, abs=0.05)
    assert end == pytest.approx(4.0 + 0.6, abs=0.15)


def test_a_stop_before_the_clip_starts_is_not_shown():
    d = load()
    actions = ['stop'] * 10 + ['clear'] * 200
    rows, ts = make(actions)                    # stop is over by 1.0 s
    assert d.stop_intervals(rows, ts, 5.0, 8.0) == []


def test_a_stop_after_the_clip_ends_is_not_shown():
    d = load()
    actions = ['clear'] * 200 + ['stop'] * 10   # stop starts at 20.0 s
    rows, ts = make(actions)
    assert d.stop_intervals(rows, ts, 0.0, 8.0) == []


def test_two_stops_a_moment_apart_become_one_badge():
    """Otherwise the badge blinks, which reads as a rendering fault."""
    d = load()
    actions = (['clear'] * 50 + ['stop'] * 3 + ['clear'] * 2 + ['stop'] * 3
               + ['clear'] * 100)
    rows, ts = make(actions)
    got = d.stop_intervals(rows, ts, 4.0, 8.0)
    assert len(got) == 1, got


def test_stops_far_apart_stay_separate():
    d = load()
    actions = (['clear'] * 50 + ['stop'] * 3 + ['clear'] * 60 + ['stop'] * 3
               + ['clear'] * 60)
    rows, ts = make(actions)
    got = d.stop_intervals(rows, ts, 4.0, 14.0)
    assert len(got) == 2, got


def test_a_three_frame_stop_is_still_readable():
    """The real case: a stop that is over in a tenth of a second.

    Shown for exactly its own duration the badge would flash for three frames
    of output and could not be read. It is held to a floor, and the card that
    introduces the shot says it is held.
    """
    d = load()
    actions = ['clear'] * 60 + ['stop'] * 3 + ['clear'] * 100
    rows, ts = make(actions)
    got = d.stop_intervals(rows, ts, 3.0, 8.0)
    assert len(got) == 1
    start, end = got[0]
    assert end - start >= 1.0, (start, end)


def test_the_badge_never_runs_past_the_end_of_the_clip():
    d = load()
    actions = ['clear'] * 70 + ['stop'] * 40
    rows, ts = make(actions)
    got = d.stop_intervals(rows, ts, 4.0, 5.0)
    assert got
    assert all(end <= 5.0 + 1e-9 for _, end in got), got


def test_an_irregular_camera_does_not_shift_the_badge():
    """The frame rate collapses when the scene is expensive; the clock does not.

    Here the camera stalls for a second in the middle of the clip. The badge
    must follow the STAMPS. Counting frames instead would put it a second early.
    """
    d = load()
    actions = ['clear'] * 40 + ['clear'] * 20 + ['stop'] * 5 + ['clear'] * 60
    ts = []
    t = 100.0
    for i in range(len(actions)):
        ts.append(t)
        t += 1.0 if i == 45 else 0.1       # one long stall before the stop
    rows = [{'action': a, 'polygon': 'protective' if a == 'stop' else ''}
            for a in actions]
    got = d.stop_intervals(rows, ts, 2.0, 10.0)
    assert len(got) == 1, got
    # First stop frame is index 60: 100 + 6.0 s of nominal gaps + 0.9 s stall
    # = 106.9, and the clip starts at 102.0, so 4.9 s in.
    assert got[0][0] == pytest.approx(4.9 - 0.15, abs=0.05), got
