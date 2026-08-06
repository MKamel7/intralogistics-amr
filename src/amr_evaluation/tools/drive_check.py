#!/usr/bin/env python3
"""Drive the robot and check the result against the platform data sheet.

Two things are established here, in order:

  1. That the robot actually moves. A stack can report every controller active
     and still not turn a wheel; the predecessor project had exactly that
     failure mode and it was silent.

  2. That its acceleration behaviour matches the reference platform. The data
     sheet publishes an acceleration limit AND a distance to reach top speed,
     which is an over-determined pair: they are a cross-check on each other and
     on the model. This is a validation, not a tuning target, so a mismatch is
     reported rather than fitted away.

Run it against a live bringup:

    ros2 launch amr_bringup robot.launch.py gui:=false cameras:=false
    python3 drive_check.py
"""

import math
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

SPEC = (Path(__file__).resolve().parents[2]
        / 'amr_description' / 'config' / 'platforms' / 'mir250_class.yaml')

CMD_TOPIC = '/diff_drive_controller/cmd_vel'
ODOM_TOPIC = '/diff_drive_controller/odom'


class DriveCheck(Node):
    def __init__(self):
        super().__init__('drive_check')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, qos)
        self.samples = []
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom, qos)

    def _odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((
            t,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ))

    def command(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(vx)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def drive(self, vx, wz, seconds):
        """Hold a command for `seconds` of SIMULATED time."""
        self.samples.clear()
        while rclpy.ok() and not self.samples:
            self.command(vx, wz)
            rclpy.spin_once(self, timeout_sec=0.1)
        start = self.samples[0][0]
        while rclpy.ok() and self.samples[-1][0] - start < seconds:
            self.command(vx, wz)
            rclpy.spin_once(self, timeout_sec=0.02)
        out = list(self.samples)
        self.command(0.0, 0.0)
        for _ in range(40):
            self.command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        return out


def main():
    spec = yaml.safe_load(SPEC.read_text())
    v = spec['values']
    targets = spec['validation_targets']

    rclpy.init()
    node = DriveCheck()
    failures = []

    print('waiting for odometry...')
    while rclpy.ok() and not node.samples:
        node.command(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.2)
    print('odometry live\n')

    # ---- 1. does it move at all --------------------------------------
    s = node.drive(0.5, 0.0, 6.0)
    dx = math.hypot(s[-1][1] - s[0][1], s[-1][2] - s[0][2])
    dt = s[-1][0] - s[0][0]
    v_mean = dx / dt if dt else 0.0
    print(f'straight line     commanded 0.50 m/s, travelled {dx:.2f} m in '
          f'{dt:.1f} s, mean {v_mean:.2f} m/s')
    if dx < 0.5:
        failures.append(f'robot barely moved ({dx:.3f} m in {dt:.1f} s)')

    # ---- 2. does it turn ----------------------------------------------
    s = node.drive(0.0, 0.6, 5.0)
    w_peak = max(abs(r[4]) for r in s)
    print(f'spot turn         commanded 0.60 rad/s, peak {w_peak:.2f} rad/s')
    if w_peak < 0.3:
        failures.append(f'robot did not turn in place (peak {w_peak:.3f} rad/s)')

    # ---- 3. acceleration, against the data sheet ----------------------
    # Command top speed from rest and see how far it takes to get there. The
    # sheet gives both the acceleration limit and the distance, so the model can
    # be checked against each independently.
    s = node.drive(v['max_linear_speed'], 0.0, 20.0)
    t0 = s[0][0]
    # Path length, not the change in x. After the spot turn above the robot is
    # pointing somewhere else entirely, and an earlier version of this check
    # duly reported that it reached top speed after MINUS 6.47 metres.
    path = [0.0]
    for a, b in zip(s, s[1:]):
        path.append(path[-1] + math.hypot(b[1] - a[1], b[2] - a[2]))
    reached_i = next((i for i, r in enumerate(s)
                      if r[3] >= 0.98 * v['max_linear_speed']), None)
    reached = s[reached_i] if reached_i is not None else None
    if reached is None:
        v_top = max(r[3] for r in s)
        print(f'accel ramp        never reached {v["max_linear_speed"]} m/s; '
              f'peak {v_top:.2f} m/s')
        failures.append(f'top speed not reached, peak {v_top:.2f} m/s')
    else:
        dist = path[reached_i]
        t_acc = reached[0] - t0
        a_eff = v['max_linear_speed'] / t_acc if t_acc else float('nan')
        print(f'accel ramp        reached {v["max_linear_speed"]} m/s after '
              f'{dist:.2f} m in {t_acc:.2f} s, effective a = {a_eff:.2f} m/s2')
        print(f'                  data sheet says {targets["distance_to_max_speed"]} m '
              f'and an acceleration limit of {v["max_linear_accel"]} m/s2')
        # The two published figures do not agree with each other under constant
        # acceleration: v^2 / (2 d) with v=2.0 and d=9.5 gives 0.21 m/s2, not
        # the 0.3 m/s2 the same sheet lists. Report both comparisons and let the
        # discrepancy be visible rather than choosing whichever one passes.
        implied = v['max_linear_speed'] ** 2 / (2 * targets['distance_to_max_speed'])
        expected = v['max_linear_speed'] ** 2 / (2 * v['max_linear_accel'])
        print(f'                  those two sheet figures are NOT consistent with each '
              f'other under constant acceleration:')
        print(f'                    {targets["distance_to_max_speed"]} m implies '
              f'{implied:.2f} m/s2, while the stated limit of '
              f'{v["max_linear_accel"]} m/s2 implies {expected:.2f} m')
        print(f'                  the model follows the ACCELERATION figure, so it '
              f'reaches speed sooner than the distance figure suggests.')
        print(f'                  most likely the real platform ramps with a jerk '
              f'limit (an S-curve), which stretches the distance without raising')
        print(f'                  the peak acceleration. Not modelled; recorded '
              f'rather than tuned away.')
        if abs(dist - expected) > 0.5:
            failures.append(
                f'ramp distance {dist:.2f} m does not match the {expected:.2f} m '
                f'implied by the configured {v["max_linear_accel"]} m/s2 limit')

    node.destroy_node()
    rclpy.shutdown()

    print()
    if failures:
        print('FAILED:')
        for f in failures:
            print(f'  {f}')
        return 1
    print('robot drives, turns and accelerates')
    return 0


if __name__ == '__main__':
    sys.exit(main())
