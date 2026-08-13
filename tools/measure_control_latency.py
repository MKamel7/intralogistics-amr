#!/usr/bin/env python3
"""Measure the sensor-to-command latency that protective field sizing assumes.

WHY THIS IS THE MOST IMPORTANT UNMEASURED NUMBER IN THE PROJECT

Every protective field in this stack is sized by the ISO 13855 shape:

    S = v * (t_scanner + t_control + t_brake) + v^2 / (2a) + C

`t_scanner` is the SICK sheet's 70 ms response time and `t_brake` is an
estimate. `t_control` is `control_latency` in the platform spec, and on both
platforms it reads:

    control_latency: 0.10   # s, sensor to command; NOT YET MEASURED

An estimate feeding a protective field is the first thing a functional safety
assessor asks about, and "0.10 s, we think" is where they stop reading. At
1.0 m/s an error of 50 ms is 50 mm of stopping distance, which is the difference
between a field that covers the vehicle and one that does not.

WHAT IS ACTUALLY BEING MEASURED

The wall clock, or rather the simulated clock, between:

    the STAMP on the scan that first shows a protective field violation
    the arrival of the command that acts on it

That is the whole chain the number stands for: scan transport, the merge, the
collision monitor's own cycle, and publication. It does NOT include the
scanner's internal response time, which is `scanner_response_time` and is
already counted separately, and it does not include brake actuation, which is
`brake_actuation_delay`. Double counting any of them would inflate every field
in the stack.

HOW A SAMPLE IS TAKEN

Passively, from an ordinary run. The vehicle drives, a person or a rack enters
the protective field, the monitor reports `stop`, and the command going to the
wheels drops to zero. Each such event is one sample. No obstacles are injected
and nothing is commanded, so this can be attached to any mission run without
changing what it measures.

    tools/run_stack.sh --cameras off --run mission --cycles 5 &
    tools/measure_control_latency.py --ros-args -p duration_s:=400.0

WHY THE DISTRIBUTION AND NOT THE MEAN

A protective field sized on a mean latency is under-sized half the time. The
figure that belongs in the spec is a high percentile, and the spread is the
interesting part: a tight distribution means a predictable system, a long tail
means the number is a scheduling artefact and the fix is in the stack rather
than in the spec.

This prints p50, p95 and the maximum, and says plainly that the p95 is the
candidate rather than writing it anywhere itself. A tool that edited the
platform spec from its own measurement would be one bad run away from shrinking
every protective field in the project.
"""

import statistics
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class LatencyProbe(Node):
    def __init__(self):
        super().__init__('control_latency_probe')
        self.duration = self.declare_parameter('duration_s', 300.0).value
        # A command below this is a stop rather than a slow-down. The drive's
        # own deadband, matching min_x_velocity_threshold in the Nav2 config.
        self.stop_eps = self.declare_parameter('stop_epsilon', 0.02).value

        self.last_scan = None        # stamp of the most recent scan
        self.pending = None          # stamp of the scan that triggered a stop
        self.polygon = ''            # which field fired, for context
        self.stopping = False
        self.was_moving = False
        self.states_seen = 0
        self.samples = []
        self.rejected = 0

        self.create_subscription(LaserScan, '/scan', self._scan, SENSOR_QOS)
        # nav2_msgs/CollisionMonitorState, NOT std_msgs/String. The first
        # version of this subscribed with the wrong type and therefore received
        # nothing at all, so it reported no samples across thirty four
        # protective stops. The message was honest and the cause was mine.
        self.create_subscription(CollisionMonitorState,
                                 '/collision_monitor_state', self._state, 20)
        self.create_subscription(TwistStamped,
                                 '/diff_drive_controller/cmd_vel',
                                 self._cmd, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'measuring sensor to command latency for {self.duration:.0f} s. '
            f'Drive the vehicle somewhere people are.')

    def _scan(self, msg):
        self.last_scan = stamp_s(msg.header)

    def _state(self, msg):
        """The monitor announcing a protective stop starts the clock.

        The scan that caused it is the most recent one, which is the closest
        attribution available without reaching inside the monitor. It is an
        upper bound on the scan-to-decision half and is honest about that.

        The RISING EDGE only. While the field stays violated the monitor keeps
        publishing STOP, and treating every message as a fresh event would time
        the wrong thing.
        """
        self.states_seen += 1
        stopping = msg.action_type == CollisionMonitorState.STOP
        if stopping and not self.stopping and self.last_scan is not None:
            self.pending = self.last_scan
            self.polygon = msg.polygon_name
        self.stopping = stopping

    def _cmd(self, msg):
        moving = abs(msg.twist.linear.x) > self.stop_eps
        # The falling edge only. A vehicle that was already stopped tells us
        # nothing about how fast the stack reacts.
        if self.was_moving and not moving and self.pending is not None:
            dt = stamp_s(msg.header) - self.pending
            if 0.0 < dt < 2.0:
                self.samples.append(dt)
                self.get_logger().info(
                    f'  sample {len(self.samples)}: {dt * 1000:.1f} ms '
                    f'({self.polygon or "field"})')
            else:
                # Out of range means the pairing is wrong, not that the system
                # is slow. Counted, never averaged in.
                self.rejected += 1
            self.pending = None
        self.was_moving = moving

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        n = len(self.samples)
        print('\n' + '=' * 70)
        print(f'  sensor to command latency, {n} sample(s), '
              f'{self.rejected} rejected as unpairable')
        if n == 0:
            print(f'  NO SAMPLES. {self.states_seen} monitor state message(s) '
                  f'were seen.')
            if self.states_seen == 0:
                print('  ZERO STATE MESSAGES, so the probe heard nothing at')
                print('  all. Check the monitor is active and that this is')
                print('  subscribed with the right message type; getting that')
                print('  wrong is silent and looks exactly like a quiet run.')
            else:
                print('  The monitor was heard but the vehicle never stopped')
                print('  for it while this was attached.')
            print('=' * 70)
            return
        s = sorted(self.samples)
        p50 = statistics.median(s)
        p95 = s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))]
        print(f'  p50   {p50 * 1000:7.1f} ms')
        print(f'  p95   {p95 * 1000:7.1f} ms')
        print(f'  max   {max(s) * 1000:7.1f} ms')
        print(f'  min   {min(s) * 1000:7.1f} ms')
        if n > 1:
            print(f'  sd    {statistics.stdev(s) * 1000:7.1f} ms')
        print()
        print('  control_latency currently reads 0.10 s in the platform spec.')
        print(f'  The p95 of {p95:.3f} s is the CANDIDATE, not a decision, and')
        print('  nothing here writes it. A mean would under-size the field')
        print('  half the time; a long tail means the fix is in the stack.')
        if n < 20:
            print(f'  {n} samples is thin. Prefer 20 or more before changing a spec.')
        print('=' * 70)


def main():
    rclpy.init()
    node = LatencyProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when the context is
        # shut down from outside, which is what stop_all.sh does. Without it
        # here the probe dies with a traceback and the samples it spent the
        # whole run collecting are never printed. Measured: a full run's data
        # lost on teardown.
        pass
    finally:
        # ALWAYS report. Reporting only when there is something to say meant a
        # run that heard nothing printed nothing, which is indistinguishable
        # from the probe not having run.
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
