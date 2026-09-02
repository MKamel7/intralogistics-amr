"""The rules that drive the vehicle at the dock.

This tool publishes velocity to a 250 kg vehicle, so its decisions are worth
testing without a simulator. The sign of the turn especially: inverting it
drives away from the dock while looking exactly like a controller that works,
and the collision monitor downstream would not object, because driving away
from something is not a hazard.
"""

import importlib.util
import math
from pathlib import Path


TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'drive_to_dock.py'


def load():
    spec = importlib.util.spec_from_file_location('drive_to_dock', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command(distance, bearing, mode='approach', standoff=1.2, retreat_to=2.6):
    return load().approach_command(
        distance, bearing, mode=mode, standoff=standoff, retreat_to=retreat_to,
        turn_speed=0.5, drive_speed=0.25)


def test_it_turns_toward_the_dock_and_not_away_from_it():
    """The one that would look like a working controller while being wrong."""
    _, angular_left, _ = command(3.0, 0.8)
    _, angular_right, _ = command(3.0, -0.8)

    assert angular_left > 0
    assert angular_right < 0


def test_it_turns_in_place_before_driving():
    """Driving while badly misaligned traces an arc into whatever is beside the dock."""
    linear, angular, _ = command(3.0, 0.8)

    assert linear == 0.0
    assert abs(angular) > 0


def test_it_drives_forward_once_aligned():
    linear, angular, mode = command(3.0, 0.02)

    assert linear > 0
    assert abs(angular) < 0.05
    assert mode == 'approach'


def test_it_stops_at_the_standoff_rather_than_at_the_dock():
    linear, angular, mode = command(1.19, 0.0)

    assert (linear, angular) == (0.0, 0.0)
    assert mode == 'arrived'


def test_arrived_stays_arrived_and_commands_nothing():
    """A latched stop, because re-triggering counted 624 approaches once."""
    assert command(0.5, 0.0, mode='arrived') == (0.0, 0.0, 'arrived')
    assert command(9.0, 3.0, mode='arrived') == (0.0, 0.0, 'arrived')


def test_retreat_backs_off_and_then_hands_back_to_the_approach():
    reversing, _, mode = command(1.5, 0.0, mode='retreat')
    assert reversing < 0
    assert mode == 'retreat'

    linear, angular, mode = command(2.7, 0.0, mode='retreat')
    assert (linear, angular) == (0.0, 0.0)
    assert mode == 'approach'


def test_the_bearing_tolerance_is_the_one_the_module_documents():
    module = load()
    just_over = module.BEARING_TOLERANCE + 0.01
    just_under = module.BEARING_TOLERANCE - 0.01

    assert command(3.0, just_over)[0] == 0.0     # still turning
    assert command(3.0, just_under)[0] > 0       # driving


def test_a_bearing_at_pi_turns_rather_than_reversing():
    """The vehicle starts facing away from the dock on this track."""
    linear, angular, _ = command(2.9, math.pi - 0.01)

    assert linear == 0.0
    assert angular > 0
