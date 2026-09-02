"""One quaternion to yaw, and the angle arithmetic that goes with it.

WHY THIS FILE EXISTS, WITH THE COUNT

Before it, quaternion to yaw was written twelve times across ten files, in two
forms that are not the same function:

    atan2(2(wz + xy), 1 - 2(yy + zz))      ten sites, correct for any orientation
    2 * atan2(z, w)                        two sites, correct only sometimes

The second is not a sloppy version of the first, it is a different function,
and the condition under which they agree is narrower than "roughly flat".
Measured: with roll 30 degrees and pitch 0 they agree exactly, with roll 0 and
pitch 30 they agree exactly, and with roll 30 AND pitch 20 the shortcut reports
34.59 degrees where the heading is 40.00. It needs BOTH roll and pitch to be
non-zero to go wrong, which is why it survived: a vehicle tilting in one axis
over a ramp still reads correctly, and only a compound tilt, a pallet edge
taken at an angle, breaks it. That is an assumption that holds until the day
it does not, and it fails by 5 degrees rather than by something obvious.

So this module keeps the general form and the shortcut is gone. The cost is a
few extra multiplications per pose, which nothing here is close to noticing.
"""

from __future__ import annotations

import math


def yaw_from_quaternion(q) -> float:
    """Heading in radians from a quaternion, for any orientation.

    Accepts anything with x, y, z, w attributes, which covers
    `geometry_msgs/Quaternion` and any local stand-in a test wants to pass.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float):
    """A planar rotation as (x, y, z, w), the inverse of the above for flat poses."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalise_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi].

    Written as atan2 of its own sine and cosine rather than with a modulo,
    because that is what the call sites this replaced all did, and because it
    has no branch to get wrong at the boundary.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_error(target: float, current: float) -> float:
    """Shortest signed rotation from current to target, in [-pi, pi].

    The sign matters and is easy to invert by accident, so it is fixed here
    once: positive means the vehicle must turn counter-clockwise to face the
    target.
    """
    return normalise_angle(target - current)
