#!/usr/bin/env python3
"""Close the last metre onto a dock the vehicle can see.

WHY THIS EXISTS

V-62 measured the vehicle parking a median 117 mm from a station, worst 212 mm,
against a goal tolerance of 200 mm and a parked localisation error of 0.055 m.
Docking needs roughly 10 mm. A goal expressed in the map frame cannot beat the
map, so the error has to stop being a localisation error and become a sensor
error. `dock_detector` supplies the sensor half; this closes on it.

WHERE THE COMMANDS GO, WHICH IS THE IMPORTANT DECISION

    controller -> cmd_vel_nav -> smoother -> cmd_vel_raw -> monitor -> wheels

This publishes to `cmd_vel_nav`, the FIRST link, so the velocity smoother and
the collision monitor both stay in the chain. Publishing further down would be
faster to write and would put a docking manoeuvre outside the only layer that
can stop it, which is not a trade this project is willing to make for a
convenience. The monitor's rotation polygons cover a spot turn and
`stop_reverse` covers the backing out.

It also means the docking approach obeys the same acceleration limits as
everything else, so the last centimetres are not a special case with its own
dynamics.

WHY IT RUNS ONLY WHEN NAV2 IS IDLE

Nav2's controller publishes to the same topic. Two publishers on one command
topic is a race whose winner depends on timing, so this is a stage AFTER the
navigation goal completes, not a layer running alongside it. The mission drives
to the station, the goal completes, and then this closes the gap.

WHAT IT REFUSES TO DO

It will not drive on a stale detection. `dock_found` is a separate topic from
`dock_pose` precisely so that absence is explicit: a controller that infers
"no dock" from an old pose drives at a dock that is no longer there, and a
topic that stops publishing looks the same as a node that died.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def approach_command(dx, dy, dyaw, standoff, gains, limits):
    """The velocity command for one control step, as pure arithmetic.

    `dx, dy` is the dock apex in the vehicle's own frame and `dyaw` is the
    heading error against the dock's bisector. Returns (vx, wz).

    THE ORDER OF THE THREE CORRECTIONS IS THE DESIGN.

    A differential drive cannot correct lateral offset directly, so a docking
    controller that drives at the apex arrives beside it, aligned with nothing.
    This turns to face the approach line first, drives along it second, and
    settles heading last:

      1. while the lateral offset is large, turn to reduce it
      2. once roughly on the line, drive down it and keep correcting
      3. inside the standoff, stop translating and fix heading only

    Written as a pure function because the alternative is discovering the sign
    of a gain by watching a robot drive into a dock.
    """
    k_lin, k_lat, k_yaw = gains
    v_max, w_max, lat_ok = limits

    # Distance still to run along the approach direction, positive is ahead.
    along = dx - standoff

    if abs(dy) > lat_ok:
        # Off the line. Turn toward it without translating, because translating
        # while badly misaligned increases the lateral error.
        return 0.0, _clamp(k_lat * math.atan2(dy, max(dx, 1e-3)), w_max)

    if along > 0.0:
        # On the line and short of the standoff. Drive, correcting gently.
        return (_clamp(k_lin * along, v_max),
                _clamp(k_lat * math.atan2(dy, max(dx, 1e-3)), w_max))

    # At the standoff. Heading only, so the final rotation does not translate
    # the vehicle back off the point it just reached.
    return 0.0, _clamp(k_yaw * dyaw, w_max)


def _clamp(v, limit):
    return max(-limit, min(limit, v))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockApproach(Node):
    def __init__(self):
        super().__init__('dock_approach',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.standoff = self.declare_parameter('standoff', 0.55).value
        self.xy_tol = self.declare_parameter('xy_tolerance', 0.010).value
        self.yaw_tol = self.declare_parameter('yaw_tolerance', 0.020).value
        self.lat_ok = self.declare_parameter('lateral_ok', 0.03).value
        self.v_max = self.declare_parameter('v_max', 0.08).value
        self.w_max = self.declare_parameter('w_max', 0.30).value
        self.gains = (self.declare_parameter('k_linear', 0.6).value,
                      self.declare_parameter('k_lateral', 0.8).value,
                      self.declare_parameter('k_yaw', 0.9).value)
        # A detection older than this is not evidence about now.
        self.max_age = self.declare_parameter('max_pose_age_s', 0.5).value
        self.timeout = self.declare_parameter('timeout_s', 60.0).value

        self.pose = None
        self.pose_t = None
        self.found = False
        self.done = False
        self.settled = 0

        self.cmd = self.create_publisher(TwistStamped, 'cmd_vel_nav', 10)
        self.create_subscription(PoseStamped, 'dock_pose', self._pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, 'dock_found', self._found,
                                 qos_profile_sensor_data)
        self.t0 = self.get_clock().now()
        self.create_timer(0.05, self._step)
        self.get_logger().info(
            f'docking: standoff {self.standoff:.3f} m, tolerance '
            f'{self.xy_tol * 1000:.0f} mm and {math.degrees(self.yaw_tol):.1f} deg')

    def _pose(self, msg):
        self.pose = msg
        self.pose_t = self.get_clock().now().nanoseconds * 1e-9

    def _found(self, msg):
        self.found = msg.data

    def _stop(self, why):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        self.cmd.publish(out)
        if not self.done:
            self.done = True
            self.get_logger().info(f'docking finished: {why}')

    def _step(self):
        if self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 > self.timeout:
            self._stop('timed out')
            return
        if not self.found or self.pose is None or self.pose_t is None:
            self._stop('no dock in view') if self.pose is None else None
            return
        if now - self.pose_t > self.max_age:
            # Stale. Stop rather than coast: the vehicle is moving and the last
            # thing it knew about the dock is no longer current.
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            self.cmd.publish(out)
            return

        dx = self.pose.pose.position.x
        dy = self.pose.pose.position.y
        # The detector's yaw points from the apex back at the sensor, so a
        # perfectly aligned vehicle sees pi. The error is the departure from it.
        dyaw = math.atan2(math.sin(yaw_of(self.pose.pose.orientation) - math.pi),
                          math.cos(yaw_of(self.pose.pose.orientation) - math.pi))

        if abs(dx - self.standoff) <= self.xy_tol and abs(dy) <= self.xy_tol \
                and abs(dyaw) <= self.yaw_tol:
            # SETTLED, not merely inside tolerance for one scan. A single frame
            # inside tolerance is a measurement, not an arrival, and stopping on
            # it is how a controller reports success from noise.
            self.settled += 1
            if self.settled >= 5:
                self._stop(
                    f'arrived, {abs(dx - self.standoff) * 1000:.1f} mm along, '
                    f'{abs(dy) * 1000:.1f} mm across, '
                    f'{math.degrees(abs(dyaw)):.2f} deg off')
            return
        self.settled = 0

        vx, wz = approach_command(
            dx, dy, dyaw, self.standoff, self.gains,
            (self.v_max, self.w_max, self.lat_ok))
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist.linear.x = vx
        out.twist.angular.z = wz
        self.cmd.publish(out)


def main():
    rclpy.init()
    node = DockApproach()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node._stop('shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
