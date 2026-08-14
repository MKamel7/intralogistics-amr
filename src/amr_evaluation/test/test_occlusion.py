"""Line of sight against the ground truth map.

Recall counted a person behind a rack as a miss, which made it a measurement of
the building rather than of the tracker: move the racking and the number
changes with nothing else different. These tests cover the ray cast that fixes
it, including the case it exists to get right.
"""

import importlib.util
from pathlib import Path

import pytest

TOOL = (Path(__file__).resolve().parents[2] / 'amr_evaluation' / 'tools'
        / 'score_tracks.py')


def load():
    spec = importlib.util.spec_from_file_location('score_tracks', TOOL)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture
def wall_map(tmp_path):
    """A 2 m by 2 m room at 0.05 m per cell with a one cell wall at x = 1.0."""
    res, w, h = 0.05, 40, 40
    cells = bytearray(b'\xff' * (w * h))
    col = int(1.0 / res)
    for row in range(h):
        cells[row * w + col] = 0
    pgm = tmp_path / 'm.pgm'
    pgm.write_bytes(b'P5\n%d %d\n255\n' % (w, h) + bytes(cells))
    (tmp_path / 'm.yaml').write_text(
        f'image: m.pgm\nresolution: {res}\norigin: [0.0, 0.0, 0.0]\n')
    return load().TruthMap(tmp_path / 'm.yaml')


def test_a_clear_line_is_clear(wall_map):
    assert wall_map.line_of_sight(0.1, 0.5, 0.9, 0.5) is True


def test_a_wall_blocks_the_line(wall_map):
    assert wall_map.line_of_sight(0.5, 0.5, 1.5, 0.5) is False


def test_a_one_cell_wall_is_not_tunnelled(wall_map):
    """THE CASE THIS EXISTS FOR. Racking is drawn one cell thick, and a ray
    stepping a full cell at a time steps straight over it and reports clear.
    """
    for y in (0.15, 0.42, 0.77, 1.31, 1.88):
        assert wall_map.line_of_sight(0.2, y, 1.8, y) is False, (
            f'the ray tunnelled through a one cell wall at y={y}')


def test_off_map_points_are_treated_as_free():
    """Absence of map is not evidence of a wall. Treating unmapped space as
    solid would exclude every person the survey had not reached, which is the
    same error as counting the occluded."""
    m = load()
    class Fake(m.TruthMap):
        def __init__(self):
            self.res, self.ox, self.oy, self.w, self.h = 0.05, 0.0, 0.0, 10, 10
            self.data = bytes([255]) * 100
    assert Fake().occupied(-5.0, -5.0) is False
    assert Fake().occupied(99.0, 99.0) is False


def test_zero_length_line_is_clear(wall_map):
    assert wall_map.line_of_sight(0.5, 0.5, 0.5, 0.5) is True


def test_the_gate_is_optional_and_says_so():
    """Without a truth map the gate cannot run, and the tool must say recall is
    a lower bound rather than quietly report the weaker number as final."""
    t = TOOL.read_text()
    assert '--truth-map' in t
    assert 'LOWER BOUND' in t


def test_occluded_people_are_excluded_not_counted_as_misses():
    t = TOOL.read_text()
    assert "occluded += len(in_range) - len(visible)" in t
    assert 'person-frames excluded as occluded' in t
