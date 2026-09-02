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


def to_frame(x: float, y: float, origin_x: float, origin_y: float,
             origin_yaw: float = 0.0) -> tuple[float, float]:
    """Express a world point in a frame whose origin sits at a known world pose.

    WHY THIS IS SHARED RATHER THAN INLINE

    `stations.*.yaml` mixes frames on purpose and says so per entry: `spawn` is
    world, the stations are map frame relative to it, and the `dock` block is
    world because it is the scorer's ground truth. Anything that reads one of
    those and compares it with `map -> base_link` has to do this conversion, and
    doing it inline is how it gets done once and forgotten the second time.

    It was forgotten the first time: the dock detector's proximity gate was
    handed the dock's WORLD coordinates and compared them against the vehicle's
    MAP pose, 7.1 m apart on the test track. The gate never opened, so the
    detector published nothing anywhere, and a gate that is always shut looks
    exactly like a building with no docks in it.
    """
    dx, dy = x - origin_x, y - origin_y
    c, s = math.cos(-origin_yaw), math.sin(-origin_yaw)
    return c * dx - s * dy, s * dx + c * dy
