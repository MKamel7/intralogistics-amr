"""The pose maths, including the case that made the shortcut form wrong."""

import math

import pytest
from amr_common.pose import (
    normalise_angle,
    quaternion_from_yaw,
    yaw_error,
    yaw_from_quaternion,
)


class Q:
    """A quaternion with the attribute names the ROS message uses."""

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


def _from_yaw(yaw):
    x, y, z, w = quaternion_from_yaw(yaw)
    return Q(x, y, z, w)


@pytest.mark.parametrize("yaw", [0.0, 0.5, -0.5, 1.5, -1.5, 3.0, -3.0, math.pi - 1e-6])
def test_a_planar_rotation_round_trips(yaw):
    assert yaw_from_quaternion(_from_yaw(yaw)) == pytest.approx(yaw, abs=1e-12)


def test_the_case_the_shortcut_form_got_wrong():
    """A compound tilt, where `2 * atan2(z, w)` stops being the heading.

    This is the whole argument for the file, and the condition is narrower than
    it looks: roll alone or pitch alone leaves the shortcut exact, so it takes
    BOTH to break it. A vehicle tilting one way over a ramp reads correctly and
    the same vehicle taking a pallet edge at an angle does not. Here the error
    is 5.4 degrees, against a mission layer that works to a yaw tolerance
    smaller than that.
    """
    roll, pitch, yaw = math.radians(30.0), math.radians(20.0), math.radians(40.0)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Q(x=sr * cp * cy - cr * sp * sy,
          y=cr * sp * cy + sr * cp * sy,
          z=cr * cp * sy - sr * sp * cy,
          w=cr * cp * cy + sr * sp * sy)

    correct = yaw_from_quaternion(q)
    shortcut = 2.0 * math.atan2(q.z, q.w)

    assert correct == pytest.approx(yaw, abs=1e-9)
    assert abs(correct - shortcut) > math.radians(5.0)


@pytest.mark.parametrize("roll,pitch", [(30.0, 0.0), (0.0, 30.0)])
def test_a_single_axis_tilt_is_exactly_where_the_shortcut_survived(roll, pitch):
    """Why the wrong form went unnoticed: one axis of tilt is not enough."""
    roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(40.0)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Q(x=sr * cp * cy - cr * sp * sy,
          y=cr * sp * cy + sr * cp * sy,
          z=cr * cp * sy - sr * sp * cy,
          w=cr * cp * cy + sr * sp * sy)

    assert yaw_from_quaternion(q) == pytest.approx(yaw, abs=1e-12)
    assert 2.0 * math.atan2(q.z, q.w) == pytest.approx(yaw, abs=1e-12)


def test_normalise_wraps_into_the_half_open_turn():
    # Either end of the turn is a correct answer at the boundary, and which one
    # comes back is an atan2 sign detail rather than a property worth pinning.
    assert abs(normalise_angle(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(normalise_angle(-3 * math.pi)) == pytest.approx(math.pi)
    assert normalise_angle(0.3) == pytest.approx(0.3)
    assert normalise_angle(-0.3) == pytest.approx(-0.3)


def test_yaw_error_takes_the_short_way_round_and_keeps_its_sign():
    """Positive means turn counter-clockwise, and the wrap must not flip it."""
    assert yaw_error(math.radians(10), math.radians(350)) == pytest.approx(math.radians(20))
    assert yaw_error(math.radians(350), math.radians(10)) == pytest.approx(math.radians(-20))
    assert abs(yaw_error(math.pi - 0.01, -math.pi + 0.01)) == pytest.approx(0.02, abs=1e-9)
