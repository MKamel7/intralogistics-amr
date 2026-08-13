#!/usr/bin/env python3
"""Compare wheel odometry against ground truth, to find slip and drift.

WHY THIS EXISTS

A vehicle that reports motion it is not making looks, from inside the stack,
exactly like a vehicle that is moving. Nav2 believes odometry between SLAM
corrections, so an odometry fault does not announce itself: the robot simply
fails to arrive, and every explanation on offer is about planning.

This measures the one thing that separates those cases. Over a window it
integrates the distance the wheels claim, and the distance the vehicle actually
covered according to the simulator, and reports the ratio:

    ratio ~ 1.0    the wheels are telling the truth
    ratio ~ 0.0    the wheels are turning and the vehicle is not moving,
                   which is a vehicle wedged against something
    ratio  > 1.0   the vehicle is moving further than the wheels report,
                   which is a vehicle being pushed, or a wheel radius that is
                   too small in the description

WHY NOT SINGLE SHOT SAMPLING

Because it does not work, and the way it fails is convincing. Sampling
`gz model -p` and `ros2 topic echo --once` a few seconds apart gave a ratio of
0.0 on one pair and an apparent 6 m odometry jump on another, on a vehicle
whose odometry was in fact continuous to within 7.2 mm per sample. The two
reads happen at different instants, each carries its own latency, and `--once`
can hand back a message older than the other reading. Nothing about the output
looks wrong.

Both series here are subscribed continuously and integrated over the same
window, which is the only comparison that means anything.

WHAT IT DOES NOT DO

It does not touch the control path. `/ground_truth/poses` is a measurement
channel and stays one; this only ever reads it, and nothing it prints is
written to any configuration.

    tools/measure_slip.py --ros-args -p duration_s:=60.0
"""

import math
import sys

import rclpy
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

# Below this, a sample is noise rather than motion. Integrating every jitter
# would inflate both path lengths and drive the ratio toward 1.0 whatever the
# vehicle is doing, which would hide exactly the fault this looks for.
MIN_STEP = 0.001


class SlipProbe(Node):
    def __init__(self):
        super().__init__('slip_probe')
        self.duration = self.declare_parameter('duration_s', 60.0).value
        self.truth_topic = self.declare_parameter(
            'truth_topic', '/ground_truth/poses').value
        # The vehicle's child_frame_id in the ground truth stream. Everything
        # else on that topic is a pedestrian.
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value

        self.odom_path = 0.0
        self.truth_path = 0.0
        self.odom_last = None
        self.truth_last = None
        self.odom_first = None
        self.truth_first = None
        self.odom_n = 0
        self.truth_n = 0
        self.max_odom_step = 0.0
        # report() runs from the timer AND from the finally block, so without
        # this the summary prints twice and the second copy looks like a
        # second measurement.
        self.reported = False

        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, 20)
        # tf2_msgs/TFMessage, NOT PoseArray. This was written against the
        # wrong type first, which is silent: the subscription is created, no
        # message ever matches, and the probe reports no data as though the
        # vehicle had not moved. The same mistake cost this project thirty
        # four protective stops of latency data. Checked with `ros2 topic
        # info` before running, which is the only reason it is right.
        self.create_subscription(TFMessage, self.truth_topic,
                                 self._truth, TRUTH_QOS)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'comparing wheel odometry against {self.truth_topic} '
            f'for {self.duration:.0f} s')

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.odom_n += 1
        if self.odom_first is None:
            self.odom_first = (p.x, p.y)
        if self.odom_last is not None:
            d = math.hypot(p.x - self.odom_last[0], p.y - self.odom_last[1])
            self.max_odom_step = max(self.max_odom_step, d)
            if d > MIN_STEP:
                self.odom_path += d
        self.odom_last = (p.x, p.y)

    def _truth(self, msg):
        """Pick the vehicle out of the ground truth transforms.

        The publisher emits one transform per tracked model, the vehicle
        among the pedestrians, so the frame has to be selected by name rather
        than by position in the list. Selecting index 0 would silently start
        measuring a pedestrian the moment the publication order changed.
        """
        tf = next((t for t in msg.transforms
                   if t.child_frame_id == self.vehicle_frame), None)
        if tf is None:
            return
        p = tf.transform.translation
        self.truth_n += 1
        if self.truth_first is None:
            self.truth_first = (p.x, p.y)
        if self.truth_last is not None:
            d = math.hypot(p.x - self.truth_last[0], p.y - self.truth_last[1])
            if d > MIN_STEP:
                self.truth_path += d
        self.truth_last = (p.x, p.y)

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 70)
        print(f'  wheel odometry against ground truth, '
              f'{self.odom_n} odom / {self.truth_n} truth sample(s)')
        if self.odom_n == 0 or self.truth_n == 0:
            missing = '/diff_drive_controller/odom' if self.odom_n == 0 else self.truth_topic
            print(f'  NO DATA on {missing}, so nothing was compared.')
            print('  This is not a result. Check the topic is being published.')
            print('=' * 70)
            return

        print(f'  wheels claim   {self.odom_path:7.2f} m of path')
        print(f'  vehicle moved  {self.truth_path:7.2f} m of path')
        if self.odom_first and self.odom_last:
            print(f'  odom  net      {math.hypot(self.odom_last[0] - self.odom_first[0], self.odom_last[1] - self.odom_first[1]):7.2f} m')
        if self.truth_first and self.truth_last:
            print(f'  truth net      {math.hypot(self.truth_last[0] - self.truth_first[0], self.truth_last[1] - self.truth_first[1]):7.2f} m')
        print(f'  largest odom step {self.max_odom_step * 1000:.1f} mm')

        if self.odom_path < 0.10:
            print('  The vehicle barely moved, so the ratio would be noise.')
            print('  Not a verdict either way; run it while the vehicle drives.')
            print('=' * 70)
            return

        ratio = self.truth_path / self.odom_path
        print()
        print(f'  RATIO truth/odom = {ratio:.3f}')
        if ratio < 0.7:
            print('  The wheels are turning further than the vehicle travels.')
            print('  That is slip, and a vehicle held against something shows')
            print('  it first. Look at where it is, not at the planner.')
        elif ratio > 1.3:
            print('  The vehicle travels further than the wheels report, so it')
            print('  is being pushed, or the wheel radius in the description is')
            print('  smaller than the one in the world.')
        else:
            print('  Odometry is consistent with ground truth over this window.')
            print('  Whatever else is wrong, it is not the wheels.')
        print('=' * 70)


def main():
    rclpy.init()
    node = SlipProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        # ALWAYS report, including on an external shutdown. Three probes in
        # this project lost a full run's data by reporting only on the happy
        # path.
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
