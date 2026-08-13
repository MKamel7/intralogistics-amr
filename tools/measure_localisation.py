#!/usr/bin/env python3
"""Measure how far the vehicle's believed pose is from where it actually is.

WHY THIS EXISTS

The vehicle stopped outside a painted delivery bay, and the three obvious
explanations were all wrong. The goal pose and the bay marking coincide to
0.4 mm. The bay is 1.6 m across, so its edge is 0.80 m from the centre. The
controller's `xy_goal_tolerance` is 0.20 m. A vehicle that satisfies its own
goal cannot end up outside that square.

So it stopped where it BELIEVED the bay was, and the gap between believing and
being is the quantity nobody in this project has ever measured. It was 5.53 m
in V-31, which is how the robot came to be planning from inside a wall. After
the wheel geometry was fixed the map looks right, and "looks right" is not a
number.

This is the number. Everything the vehicle claims about arriving somewhere
rests on it.

WHAT IS COMPARED

    /ground_truth/poses      where the vehicle is, from the simulator
    map -> base_link         where the stack believes it is

Both are sampled continuously and paired in one place, for the reason recorded
at length in measure_slip.py: two reads taken at different instants, each with
its own latency, produce confident nonsense. That mistake cost a retraction
here already.

THE FRAME ASSUMPTION, STATED RATHER THAN BURIED

slam_toolbox starts its map frame at the vehicle's initial pose, so world and
map differ by the spawn pose. That is read from the stations file rather than
typed in, because a hand copied constant is exactly what this project keeps
finding at the bottom of its faults.

The assumption is then CHECKED rather than trusted: the first paired sample
must show a small error, since at startup belief and truth agree by
construction. If it does not, the frames are not related the way this thinks
and every number below would be an offset rather than an error. It says so and
refuses to report a verdict.

WHY THE PARKED ERROR IS REPORTED SEPARATELY

Error while driving is interesting; error at the moment the vehicle declares
arrival is the one that decides whether a pallet ends up in the bay or beside
it. They are not the same distribution, because a stationary vehicle has
stopped feeding the scan matcher new geometry.

    tools/measure_localisation.py --ros-args \
        -p stations_file:=src/amr_mission/config/stations.test_track.mp400_class.yaml
"""

import math
import statistics
import sys

import rclpy
import tf2_ros
import yaml
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

TRUTH_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)

# Below this speed the vehicle counts as stopped, matching the deadband used by
# the other probes in this project so the three agree about what "moving" means.
STOP_EPS = 0.02

# The first paired sample must agree to better than this, or the frame
# assumption is wrong and nothing below means what it says.
STARTUP_AGREEMENT = 0.50


class LocalisationProbe(Node):
    def __init__(self):
        super().__init__('localisation_probe')
        self.duration = self.declare_parameter('duration_s', 300.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        stations = self.declare_parameter('stations_file', '').value
        self.spawn = self._spawn_from(stations)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.truth = None
        self.moving = False
        self.errors = []
        self.parked_errors = []
        self.first_error = None
        self.tf_failures = 0
        self.truth_msgs = 0

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, 20)
        self.t0 = self.get_clock().now()
        self.create_timer(0.05, self._sample)
        self.create_timer(1.0, self._tick)
        self.reported = False
        self.get_logger().info(
            f'comparing map -> base_link against ground truth for '
            f'{self.duration:.0f} s, spawn {self.spawn}')

    def _spawn_from(self, path):
        """Read the spawn pose the map frame is anchored to.

        Read rather than typed. A constant copied by hand into a diagnostic is
        how this project produced a wrong answer more than once.
        """
        if not path:
            self.get_logger().warn(
                'no stations_file given, assuming the map frame is anchored at '
                'the world origin. If it is not, every number here is an '
                'offset rather than an error.')
            return (0.0, 0.0)
        d = yaml.safe_load(open(path))
        s = d['spawn']
        return (float(s['x']), float(s['y']))

    def _truth(self, msg):
        tf = next((t for t in msg.transforms
                   if t.child_frame_id == self.vehicle_frame), None)
        if tf is None:
            return
        self.truth_msgs += 1
        self.truth = (tf.transform.translation.x, tf.transform.translation.y)

    def _odom(self, msg):
        self.moving = abs(msg.twist.twist.linear.x) > STOP_EPS

    def _sample(self):
        if self.truth is None:
            return
        try:
            tr = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            # Counted, not raised. A disconnected TF tree is itself a finding,
            # and one that has happened here: map and base_link ended up in two
            # unconnected trees mid run, which makes the vehicle unlocalisable
            # rather than badly localised.
            self.tf_failures += 1
            return
        bx = self.spawn[0] + tr.transform.translation.x
        by = self.spawn[1] + tr.transform.translation.y
        e = math.hypot(self.truth[0] - bx, self.truth[1] - by)
        if self.first_error is None:
            self.first_error = e
        self.errors.append(e)
        if not self.moving:
            self.parked_errors.append(e)

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    @staticmethod
    def _stats(xs):
        s = sorted(xs)
        return (statistics.median(s),
                s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))],
                max(s))

    def report(self):
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 70)
        print('  localisation error against ground truth')
        print(f'  {len(self.errors)} paired sample(s), '
              f'{self.truth_msgs} truth message(s), '
              f'{self.tf_failures} TF lookup failure(s)')

        if not self.errors:
            print('  NO PAIRED SAMPLES, so nothing was measured.')
            if self.tf_failures and not self.truth_msgs:
                print('  Neither source arrived. Is the stack up?')
            elif self.tf_failures:
                print('  Ground truth arrived but map -> base_link never')
                print('  resolved. That is a broken TF tree, not a badly')
                print('  localised vehicle, and it is a fault in itself.')
            print('=' * 70)
            return

        if self.first_error is not None and self.first_error > STARTUP_AGREEMENT:
            print(f'  FRAME ASSUMPTION LOOKS WRONG. The first sample differs by '
                  f'{self.first_error:.2f} m,')
            print('  and at startup belief and truth agree by construction. The')
            print('  spawn pose given here probably is not what the map frame is')
            print('  anchored to, so these are offsets, not errors. No verdict.')
            print('=' * 70)
            return

        p50, p95, mx = self._stats(self.errors)
        print(f'  while driving   p50 {p50:6.3f} m   p95 {p95:6.3f} m   max {mx:6.3f} m')
        if self.parked_errors:
            q50, q95, qmx = self._stats(self.parked_errors)
            print(f'  while stopped   p50 {q50:6.3f} m   p95 {q95:6.3f} m   max {qmx:6.3f} m'
                  f'   ({len(self.parked_errors)} sample(s))')
        else:
            print('  the vehicle never stopped, so there is no parked figure')
        print()
        # 0.80 m is the half extent of a 1.6 m delivery bay marking, which is
        # the concrete thing this number decides.
        worst = self._stats(self.parked_errors)[1] if self.parked_errors else p95
        print('  A delivery bay marking is 1.6 m across, so its edge is 0.80 m')
        print(f'  from the centre. At p95 {worst:.3f} m the vehicle parks '
              f'{"INSIDE" if worst < 0.80 else "OUTSIDE"} it.')
        if worst >= 0.80:
            print('  The goal and the marking coincide and the goal tolerance is')
            print('  0.20 m, so this is the whole explanation for stopping short.')
        print('=' * 70)


def main():
    rclpy.init()
    node = LocalisationProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
