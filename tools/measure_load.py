#!/usr/bin/env python3
"""Whether the load stays where it was put, and how far it creeps if not.

WHY THIS EXISTS

V-60 modelled the payload as a welded URDF link, which measures what the mass
does to the vehicle and makes the question of whether the load stays on
unanswerable by construction. The load is now a separate body on the deck held
by friction alone, so sliding, rotating and falling off are outcomes the physics
is allowed to produce, and this measures which of them happens.

THE ARITHMETIC IT IS TESTING

An unsecured load begins to slide when the deceleration exceeds mu * g. At the
0.35 in the payload model that is 3.43 m/s2, and V-60 measured protective stops
at 3.49 m/s2 unladen and 4.08 laden. So the load is expected to move, and the
interesting question is how much:

    a stop at 4.08 m/s2 exceeds the limit by 0.65 m/s2 and lasts about 190 ms,
    which is 11.5 mm of relative travel

That predicts a CREEP of millimetres per hard stop rather than a load being
thrown off, and it accumulates over a duty cycle. Whether millimetres matter
depends on how close the load starts to the edge, which is why the distance to
the edge of the plate is reported rather than only the displacement.

BOTH POSES FROM ONE MESSAGE, WHICH IS THE POINT

Sampling the vehicle and the load separately reported the load 0.87 m behind
the vehicle while it was in fact 8 mm behind. Two `gz model` calls a second
apart, with the vehicle driving between them, and the difference is the
vehicle's own travel rather than any slip at all. `/ground_truth/poses` carries
every model in a single timestamped message, so a relative measurement taken
from it cannot be a difference of times.

That is the same error the project has now made in several forms: comparing two
things measured at different moments. See V-52 and V-56.
"""

import math
import statistics
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

TRUTH_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)


def slip_limit(mu, g=9.81):
    """The deceleration above which an unsecured load starts to move.

    Pure, and independent of the load's mass: the friction force and the
    inertial force both scale with it, which is why a heavier load is not more
    secure than a light one on the same surfaces.
    """
    return max(0.0, mu) * g


def on_plate(fore, lateral, half_length, half_width):
    """Whether the load's centre is still over the deck."""
    return abs(fore) <= half_length and abs(lateral) <= half_width


class LoadProbe(Node):
    def __init__(self):
        super().__init__('load_probe',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 2400.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        # The plate, not the chassis envelope. A load whose centre passes the
        # plate edge is going over even if its far corner is still above deck.
        self.half_length = self.declare_parameter('half_length', 0.295).value
        self.half_width = self.declare_parameter('half_width', 0.2795).value
        self.mu = self.declare_parameter('mu', 0.35).value

        self.start = {}          # name -> (fore, lateral) when first seen
        self.worst = {}          # name -> largest displacement from that
        self.turned = {}         # name -> largest rotation on the deck
        self.left = set()        # loads whose centre went past the plate edge
        self.samples = 0
        self.seen = 0

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False
        self.get_logger().info(
            f'watching the load for {self.duration:.0f} s; at mu {self.mu:.2f} '
            f'it starts to move above {slip_limit(self.mu):.2f} m/s2')

    def _truth(self, msg):
        poses = {}
        vehicle = None
        for tf in msg.transforms:
            p = tf.transform.translation
            if tf.child_frame_id == self.vehicle_frame:
                q = tf.transform.rotation
                vehicle = (p.x, p.y, math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            elif tf.child_frame_id.startswith('payload'):
                q = tf.transform.rotation
                poses[tf.child_frame_id] = (p.x, p.y, p.z, math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        if vehicle is None or not poses:
            return
        self.samples += 1
        vx, vy, vyaw = vehicle
        c, s = math.cos(-vyaw), math.sin(-vyaw)
        for name, (x, y, _z, lyaw) in poses.items():
            dx, dy = x - vx, y - vy
            fore, lateral = c * dx - s * dy, s * dx + c * dy
            # A load that has been set down on the table is no longer being
            # carried, and its distance from the vehicle is the vehicle
            # driving away rather than anything sliding.
            if not on_plate(fore, lateral, self.half_length + 0.6,
                            self.half_width + 0.6):
                continue
            # RELATIVE YAW TOO, and it is where the motion actually is.
            # A spot check found a carried box rotated 3.79 degrees on the
            # plate while its centre had not moved measurably, and the first
            # version of this probe tracked translation only and duly reported
            # 0.0 mm with nothing else to say. A load that has turned on the
            # deck has moved, and on a pallet with a lip it is the rotation
            # that jams rather than the slide.
            rel_yaw = math.atan2(math.sin(lyaw - vyaw), math.cos(lyaw - vyaw))
            if name not in self.start:
                self.start[name] = (fore, lateral, rel_yaw)
                self.seen += 1
            f0, l0, y0 = self.start[name]
            moved = math.hypot(fore - f0, lateral - l0)
            if moved > self.worst.get(name, 0.0):
                self.worst[name] = moved
            turned = abs(math.atan2(math.sin(rel_yaw - y0),
                                    math.cos(rel_yaw - y0)))
            if turned > self.turned.get(name, 0.0):
                self.turned[name] = turned
            if not on_plate(fore, lateral, self.half_length, self.half_width):
                self.left.add(name)

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 70)
        print(f'  load securing, {self.samples} sample(s), {self.seen} load(s)')
        if not self.seen:
            print('  NO LOAD SEEN. Either the run carried nothing, or the box')
            print('  was never placed. Those look identical from here and the')
            print('  mission log is the place to tell them apart.')
            print('=' * 70)
            return
        for name in sorted(self.worst):
            edge = min(self.half_length, self.half_width) - self.worst[name]
            print(f'    {name:12s} crept {self.worst[name] * 1000:6.1f} mm, '
                  f'turned {math.degrees(self.turned.get(name, 0.0)):5.2f} deg, '
                  f'{edge * 1000:6.1f} mm to the plate edge'
                  f'{"   LEFT THE PLATE" if name in self.left else ""}')
        crept = list(self.worst.values())
        turns = list(self.turned.values()) or [0.0]
        print(f'  worst creep {max(crept) * 1000:.1f} mm, '
              f'median {statistics.median(crept) * 1000:.1f} mm; '
              f'worst rotation {math.degrees(max(turns)):.2f} deg')
        print()
        if self.left:
            print(f'  {len(self.left)} load(s) went over the edge. An unsecured')
            print('  load does not survive this duty cycle.')
        else:
            print('  No load left the plate. It CREPT rather than being thrown,')
            print(f'  which is what a stop marginally above the {slip_limit(self.mu):.2f} m/s2')
            print('  friction limit predicts: millimetres per stop, accumulating.')
            print('  Whether that matters depends on where the load starts, and')
            print('  a load placed near the edge would reach it sooner.')
        print('=' * 70)


def main():
    rclpy.init()
    node = LoadProbe()
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
