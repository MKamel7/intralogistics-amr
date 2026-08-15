"""When the survey is allowed to say the building is surveyed.

The exit condition is the whole contract between the survey and the mission:
the survey exists to produce a map the mission can plan in. Both rules here
were added after a survey reported success on a map that could not support a
single delivery.
"""

import importlib.util
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
RUNNER = PKG / 'amr_navigation' / 'survey_runner.py'


def text():
    return RUNNER.read_text()


class Grid:
    """Minimal OccupancyGrid stand-in: origin, resolution and extent."""

    class _Info:
        class _Origin:
            class _Pos:
                x = 0.0
                y = 0.0
            position = _Pos()
        origin = _Origin()
        resolution = 0.05
        width = 400          # 20 m
        height = 200         # 10 m
    info = _Info()


class Runner:
    """Enough of the node for the map extent check to run."""

    def __init__(self, stations_file='', grid=None):
        self.stations_file = stations_file
        self.map = grid


def runner_cls():
    spec = importlib.util.spec_from_file_location('survey_runner', RUNNER)
    m = importlib.util.module_from_spec(spec)
    # The module imports rclpy at import time; only the pure methods are used.
    spec.loader.exec_module(m)
    return m.SurveyRunner


def on_map(x, y):
    return runner_cls().on_map(Runner(grid=Grid()), x, y)


def test_a_point_inside_the_grid_is_on_the_map():
    assert on_map(10.0, 5.0) is True


def test_a_point_beyond_the_grid_is_not():
    """The exact failure: dispatch at map x 35.0 against a map 20 m wide.

    The planner rejected it with "Goal Coordinates of(35.000000, 0.975000)
    was outside bounds" three times, once per cycle.
    """
    assert on_map(35.0, 0.975) is False


def test_the_grid_edges_are_inclusive():
    assert on_map(0.0, 0.0) is True
    assert on_map(20.0, 10.0) is True


def test_no_map_means_nothing_is_on_it():
    assert runner_cls().on_map(Runner(grid=None), 1.0, 1.0) is False


def test_one_quiet_round_is_not_convergence():
    """The rule was a single round below the growth threshold, so one unlucky
    leg ended a survey with eleven productive rounds left. The round before
    the early stop had added 47.7 m2."""
    t = text()
    assert 'quiet_rounds_needed' in t
    assert 'self.quiet_rounds = 0' in t, 'the counter must reset on a good round'


def test_the_survey_will_not_finish_with_a_station_off_the_map():
    """The survey's whole purpose. A map that does not cover the stations is
    not a finished survey, however little it is still growing."""
    t = text()
    assert 'stations_off_map' in t
    assert 'are still off it' in t, 'it must say which stations, not just how many'


def test_the_runner_is_told_which_stations_to_cover():
    """A survey that is never given the stations cannot check them, and would
    silently fall back to the old behaviour."""
    rs = (PKG.parents[1] / 'tools' / 'run_stack.sh').read_text()
    assert rs.count('-p stations_file:=') >= 2, (
        'both survey invocations in run_stack.sh must pass the stations file')


def test_a_leg_shorter_than_the_vehicle_ends_the_survey():
    """The frontier search returned a goal 0.1 m away and kept returning it.

    Rounds 22, 23 and 24 were each "driving 0.1 m through free space" from the
    same pose, each arrived instantly, each added 0.0 m2. The survey burned to
    max_rounds learning nothing and then reported success on 246.3 m2 of a
    544 m2 building.
    """
    t = text()
    assert 'min_leg' in t
    assert 'under the' in t and 'minimum leg' in t


def test_a_survey_that_missed_a_station_returns_failure():
    """The station check previously guarded only the quiet rounds exit. Ending
    any other way fell through to a success message, and the mission that
    followed failed every cycle with a goal outside bounds, which is the
    survey's fault reported as the vehicle's."""
    t = text()
    assert 'SURVEY FAILED' in t
    assert 'return 1' in t
    # Match the CODE line, not the phrase: the comment above it quotes the
    # old success message while explaining the bug, and an earlier version of
    # this assertion matched that comment and failed on a correct fix.
    i = t.find('SURVEY FAILED')
    j = t.find("self.get_logger().info(f'survey finished")
    assert -1 < i < j, 'the failure path must precede and pre-empt the success line'
