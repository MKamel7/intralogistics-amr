"""The names the nodes talk over, in one place.

WHY A CONSTANT AND NOT A STRING LITERAL

A subscriber to a topic nobody publishes is not an error. It is silence, and
silence looks exactly like a robot that has nothing to say yet. So a renamed
topic does not fail the build, fail a test, or log a warning: it produces a
node that runs forever and never receives anything.

The controller name is the sharp case. `diff_drive_controller` appeared in five
Python files and in the controller and collision monitor YAML, and renaming it
in the YAML would have left those five subscribing to a dead topic. Everything
derived from the controller is now derived from one constant, and
`test_topics.py` checks the YAML against it, so the two cannot drift without
the build going red.

Names are absolute, with the leading slash, because that is what the call sites
this replaced used and mixing absolute and relative names inside one system is
its own source of silent mismatches.
"""

from __future__ import annotations

#: The ros2_control controller. Rename here and the derived topics follow.
CONTROLLER = "diff_drive_controller"


class Topics:
    """Topic names, grouped by who owns them."""

    #: Where velocity commands enter the safety layer, and where they leave it.
    #: The collision monitor sits between these two, which is the whole point of
    #: the design: nothing reaches the wheels without passing it.
    CMD_VEL_NAV = "/cmd_vel_nav"
    CMD_VEL_RAW = "/cmd_vel_raw"
    CMD_VEL = "/cmd_vel"
    CONTROLLER_CMD_VEL = f"/{CONTROLLER}/cmd_vel"

    #: Wheel odometry, published by the controller and therefore named after it.
    ODOM = f"/{CONTROLLER}/odom"

    #: Perception and navigation.
    SCAN = "/scan"
    MAP = "/map"
    PLAN = "/plan"
    TRAJECTORIES = "/trajectories"
    ROBOT_DESCRIPTION = "/robot_description"

    #: The safety layer's own outputs.
    PROTECTIVE_FIELD = "/protective_field"
    WARNING_FIELD = "/warning_field"

    #: Evaluation oracles. ADR 0006: ground truth is scored against, never
    #: consumed by the control path, so these belong to the tools and to
    #: nothing that drives.
    GROUND_TRUTH_POSES = "/ground_truth/poses"
    GROUND_TRUTH_MAP = "/ground_truth/map"
