"""The camera geometry and the timing conform, for the demo footage tool.

WHY THESE THREE THINGS ARE TESTED AND NOTHING ELSE

`tools/film.py` produces the demo video. Nothing it does is on the control
path, so it gets no safety scrutiny, but two of its parts are arithmetic that
is wrong silently and one of them once shipped:

`aim` and `frustum` must AGREE. They were written minutes apart and disagreed
about the sign of pitch, which is invisible while every camera is level and
sends a high camera's view into the ceiling the moment one is not. The shot
list looked perfectly reasonable either way. The check is that a camera can see
the point it was aimed at, which sounds too obvious to write down and was false.

`hold_indices` is the one that decides whether the video tells the truth about
speed. The first demo ran 2.3 times fast because a 4.3 fps screen grab was
encoded at 10 fps. Filming in the simulator does not fix that by itself: a
camera asked for 30 Hz delivered a median gap of 36 ms and a ninetieth
percentile of 132 ms, so encoding its frames at 30 fps would run about 1.75
times fast. Only the resample makes playback real time, and its failure mode is
a plausible looking video at the wrong speed, which no viewer can detect and
no other test in this project would catch.
"""

import importlib.util
import math
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'film.py'


def load():
    spec = importlib.util.spec_from_file_location('film', TOOL)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# --- aim and frustum have to agree -----------------------------------------

def test_a_camera_above_its_target_pitches_down():
    f = load()
    _, pitch = f.aim((0.0, 0.0, 10.0), (10.0, 0.0, 0.0))
    # Gazebo's convention: positive pitch about +Y takes +X toward -Z.
    assert pitch == pytest.approx(math.radians(45.0), abs=1e-6)
    assert pitch > 0.0


def test_every_camera_can_see_the_point_it_is_aimed_at():
    """The bug: frustum used -pitch, so this failed for any tilted camera.

    Every shot in every list is checked, so a new one cannot be added with an
    aim point behind its own lens.
    """
    f = load()
    for shots in (f.WAREHOUSE, f.TEST_TRACK):
        for s in shots:
            yaw, pitch = f.aim(s['pos'], s['look'])
            visible, _, _ = f.frustum(s['pos'], yaw, pitch, s['fov'], 16 / 9,
                                      s['look'], 0.55)
            assert visible, f"{s['name']} cannot see its own aim point"


def test_the_aim_point_lands_in_the_middle_of_the_frame():
    """Not merely inside the frustum: dead centre, to a rounding error.

    `visible` alone would pass with the aim point in a corner, which is the
    kind of agreement that hides a small angular error until a shot is framed
    around it.
    """
    f = load()
    for shots in (f.WAREHOUSE, f.TEST_TRACK):
        for s in shots:
            yaw, pitch = f.aim(s['pos'], s['look'])
            px, py, pz = s['pos']
            dx, dy, dz = (s['look'][0] - px, s['look'][1] - py,
                          s['look'][2] - pz)
            cy, sy = math.cos(yaw), math.sin(yaw)
            x1, y1 = dx * cy + dy * sy, -dx * sy + dy * cy
            cp, sp = math.cos(pitch), math.sin(pitch)
            fwd, up = x1 * cp - dz * sp, x1 * sp + dz * cp
            off = math.hypot(math.atan2(y1, fwd), math.atan2(up, fwd))
            assert off < 1e-9, f"{s['name']} aim point is {off:.2e} rad off axis"


def test_a_target_behind_the_lens_is_not_visible():
    f = load()
    visible, _, _ = f.frustum((0, 0, 1), 0.0, 0.0, 1.05, 16 / 9, (-10, 0, 1),
                              0.55)
    assert not visible


def test_apparent_size_falls_off_with_distance():
    """`px_across` is what rejects a shot of a building containing a speck."""
    f = load()
    _, _, near = f.frustum((0, 0, 1), 0.0, 0.0, 1.05, 16 / 9, (5, 0, 1), 0.55)
    _, _, far = f.frustum((0, 0, 1), 0.0, 0.0, 1.05, 16 / 9, (20, 0, 1), 0.55)
    assert near > far * 3.5


# --- the resample is what makes playback real time -------------------------

def test_a_uniform_source_at_the_output_rate_is_the_identity():
    f = load()
    stamps = [i / 10.0 for i in range(50)]
    assert f.hold_indices(stamps, 0.0, 2.0, 10) == list(range(20))


def test_a_half_rate_source_holds_each_frame_twice():
    """The real case: the renderer falls behind and frames must be REPEATED.

    Encoding them one per slot instead is exactly how a video ends up running
    at double speed while looking entirely normal.
    """
    f = load()
    stamps = [i / 5.0 for i in range(30)]
    assert f.hold_indices(stamps, 0.0, 1.0, 10) == [0, 0, 1, 1, 2, 2, 3, 3,
                                                    4, 4]


def test_a_double_rate_source_drops_every_other_frame():
    f = load()
    stamps = [i / 20.0 for i in range(60)]
    assert f.hold_indices(stamps, 0.0, 1.0, 10) == [0, 2, 4, 6, 8, 10, 12, 14,
                                                    16, 18]


def test_a_stall_repeats_rather_than_skipping_ahead():
    """A 0.5 s gap in the source must become 0.5 s of held frame.

    If it skipped, the vehicle would jump and the clip would be short by the
    length of the stall while claiming its nominal duration.
    """
    f = load()
    stamps = [0.0, 0.1, 0.2, 0.7, 0.8]
    assert f.hold_indices(stamps, 0.0, 0.8, 10) == [0, 1, 2, 2, 2, 2, 2, 3]


def test_frames_sharing_a_stamp_are_handled():
    """The tenth percentile gap in a real recording was 0 ms."""
    f = load()
    stamps = [0.0, 0.0, 0.0, 0.1, 0.1, 0.2]
    assert len(f.hold_indices(stamps, 0.0, 0.3, 10)) == 3


def test_the_start_offset_is_measured_from_the_first_frame():
    """`--report` prints offsets from the start of the recording."""
    f = load()
    stamps = [i / 10.0 for i in range(50)]
    assert f.hold_indices(stamps, 1.0, 0.3, 10) == [10, 11, 12]


def test_output_duration_matches_the_sim_time_it_covers():
    """The property the whole resample exists to guarantee.

    An irregular source, deliberately nastier than the measured one: alternate
    36 ms and 132 ms gaps with a stall in the middle. Ten seconds of output
    must span ten seconds of simulated time.
    """
    f = load()
    stamps, t = [], 0.0
    for i in range(400):
        stamps.append(t)
        t += 0.036 if i % 2 else 0.132
        if i == 200:
            t += 0.4
    idx = f.hold_indices(stamps, 2.0, 10.0, 30)
    covered = stamps[idx[-1]] - stamps[idx[0]]
    # Within one output slot: the hold can only land on a frame at or before
    # each grid point, so it is short by at most the last gap.
    assert covered == pytest.approx(10.0, abs=0.15), covered


def test_the_clip_never_runs_past_the_end_of_the_footage():
    """Asking for more than exists must clamp, not wrap or raise."""
    f = load()
    stamps = [i / 10.0 for i in range(20)]      # 2 s of footage
    idx = f.hold_indices(stamps, 0.0, 10.0, 30)
    assert len(idx) == 300
    assert max(idx) == len(stamps) - 1


def test_a_window_is_measured_in_simulated_seconds_not_frames():
    """The bug that disconnected shot selection from cutting.

    `slices` offers a start offset that `conform` will read as simulated
    seconds. When it counted frames and divided by the nominal rate instead,
    the two disagreed by whatever the camera was behind by: measured on a real
    recording, 610 frames spanning 43.0 s of simulated time, so an offset of
    "90" meant 90 s to the cutter and 20 s to the selector.

    Here the camera delivers 10 frames a second while the output rate is 30.
    A window of 8 s must therefore span 8 s of stamps, not 240 frames.
    """
    f = load()
    ts = [i / 10.0 for i in range(600)]         # 60 s at 10 Hz
    got = f.slices(ts, 8.0, 0.5)
    assert got, 'no windows offered'
    for lo, hi in got:
        assert ts[hi - 1] - ts[lo] == pytest.approx(8.0, abs=0.2)
    # And the last window must still fit inside the footage.
    lo, hi = got[-1]
    assert ts[lo] + 8.0 <= ts[-1] + 1e-9


def test_windows_advance_by_the_step_in_simulated_seconds():
    f = load()
    ts = [i / 10.0 for i in range(600)]
    got = f.slices(ts, 8.0, 0.5)
    starts = [ts[lo] for lo, _ in got]
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert all(g == pytest.approx(0.5, abs=1e-6) for g in gaps), gaps


def test_a_stalled_camera_still_advances_and_terminates():
    """Frames sharing a stamp must not wedge the scan in place.

    A tenth percentile gap of 0 ms means duplicate stamps are normal, and a
    step computed by seeking forward in time can land on the frame it started
    from. Without the forced advance this loops forever.
    """
    f = load()
    ts = [0.0] * 50 + [i / 10.0 for i in range(200)]
    got = f.slices(ts, 4.0, 0.5)
    assert got
    assert all(hi > lo for lo, hi in got)


def test_a_recording_shorter_than_the_window_offers_nothing():
    f = load()
    ts = [i / 10.0 for i in range(30)]          # 3 s of footage
    assert f.slices(ts, 8.0, 0.5) == []


def test_a_camera_aimed_along_minus_x_is_not_a_failed_spawn():
    """+pi and -pi are the same heading, and the pose check said otherwise.

    A camera aimed straight down an east-west aisle asks for a yaw of exactly
    pi; the simulator reports -pi. Compared by subtraction that is 6.28 rad
    against a 1e-3 tolerance, so a correctly placed camera was rejected as a
    silently failed spawn on every run.
    """
    f = load()
    assert f.angle_error(-math.pi, math.pi) == pytest.approx(0.0, abs=1e-9)
    assert f.angle_error(math.pi, -math.pi) == pytest.approx(0.0, abs=1e-9)


def test_a_genuinely_wrong_heading_is_still_caught():
    f = load()
    assert f.angle_error(0.0, math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert f.angle_error(1.0, 1.5) == pytest.approx(0.5, abs=1e-9)
