#!/usr/bin/env python3
"""Count how close the vehicle came to people, and whether it touched anyone.

WHY THIS EXISTS

The pedestrians in this simulation carry no collision geometry. That is
deliberate and documented in models/person/model.sdf: a driven figure with a
body reaching 1.2 m snags on shelf beams the route probe never saw, and two of
three walkers were stopped dead by it. The lidar renders visuals, so the
scanner sees them exactly as before.

The consequence was never accounted for. **A person cannot be hit in this
simulation.** They pass through the vehicle. So every safety claim this project
might make about the collision monitor preventing contact is unfalsifiable:
no run can ever produce a collision, and "zero collisions" is a property of the
model rather than a result.

Observed directly: a pedestrian walked through the robot while an operator was
watching.

This measures contact geometrically instead, from the ground truth oracle. The
safety layer can then be judged by whether it kept people out of the vehicle's
footprint, which is what it is for.

WHAT COUNTS AS WHAT

    contact     the person is inside the vehicle footprint, expanded by their
                own 0.22 m collision puck. In a real cell this is an injury.
    near miss   inside `near_miss` of the footprint. Not a failure, but the
                distribution matters more than the count: a stack that only
                ever misses by 50 mm is not safe, it is lucky.
    clearance   the distance from the person to the footprint, whose MINIMUM
                over a run is the honest headline number.

The footprint is the real polygon from the platform spec, not a circle. A
circular approximation of a 0.59 by 0.559 m vehicle is 12 percent wrong at the
corners, and every figure here is about margins of that order.

WHAT IT DOES NOT DO

It does not touch the control path, and it never writes a configuration.
`/ground_truth/poses` is a measurement channel and stays one.

    tools/measure_contacts.py --ros-args -p duration_s:=600.0 \
        -p half_length:=0.300 -p half_width:=0.2845
"""

import math
import statistics
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

TRUTH_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)

# The person model's collision puck, from models/person/model.sdf. A person is
# not a point, and treating them as one understates every clearance by this.
PERSON_RADIUS = 0.22


def clearance_to_footprint(px, py, half_length, half_width):
    """Distance from a point to a centred, axis aligned rectangle.

    Negative inside. The point is given in the VEHICLE frame, so the caller
    does the rotation and this stays a pure function that can be tested.
    """
    dx = abs(px) - half_length
    dy = abs(py) - half_width
    if dx > 0.0 and dy > 0.0:
        return math.hypot(dx, dy)          # nearest corner
    return max(dx, dy)                     # nearest edge, negative inside


class ContactProbe(Node):
    def __init__(self):
        super().__init__('contact_probe')
        self.duration = self.declare_parameter('duration_s', 600.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        # Footprint half extents. Defaults are the MP-400's; pass the platform's
        # own rather than trusting these.
        self.half_length = self.declare_parameter('half_length', 0.300).value
        self.half_width = self.declare_parameter('half_width', 0.2845).value
        self.near_miss = self.declare_parameter('near_miss', 0.30).value

        self.min_clear = {}
        self.contacts = {}
        self.near_misses = {}
        self.samples = 0
        self.in_contact = set()

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False
        self.get_logger().info(
            f'watching for contact against a '
            f'{2 * self.half_length:.3f} by {2 * self.half_width:.3f} m footprint')

    def _truth(self, msg):
        poses = {}
        yaw = None
        for tf in msg.transforms:
            p = tf.transform.translation
            poses[tf.child_frame_id] = (p.x, p.y)
            if tf.child_frame_id == self.vehicle_frame:
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = poses.get(self.vehicle_frame)
        if v is None or yaw is None:
            return
        self.samples += 1
        c, s = math.cos(-yaw), math.sin(-yaw)
        for name, (x, y) in poses.items():
            # `body` is the vehicle's own link, not a person.
            if name in (self.vehicle_frame, 'body'):
                continue
            dx, dy = x - v[0], y - v[1]
            px, py = c * dx - s * dy, s * dx + c * dy
            gap = clearance_to_footprint(px, py,
                                         self.half_length, self.half_width)
            gap -= PERSON_RADIUS
            prev = self.min_clear.get(name)
            if prev is None or gap < prev:
                self.min_clear[name] = gap
            if gap <= 0.0:
                # Rising edge only. A person standing inside the vehicle for
                # three seconds is one contact, not sixty.
                if name not in self.in_contact:
                    self.contacts[name] = self.contacts.get(name, 0) + 1
                    self.in_contact.add(name)
                    self.get_logger().error(
                        f'CONTACT with {name}, {-gap:.3f} m inside the footprint')
            else:
                self.in_contact.discard(name)
                if gap <= self.near_miss:
                    self.near_misses[name] = self.near_misses.get(name, 0) + 1

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 70)
        print(f'  contact and clearance, {self.samples} sample(s)')
        if not self.samples:
            print('  NO GROUND TRUTH RECEIVED, so nothing was measured.')
            print('  This is not a clean run, it is no run at all.')
            print('=' * 70)
            return

        total = sum(self.contacts.values())
        print(f'  CONTACTS: {total}')
        for name in sorted(self.min_clear):
            n = self.contacts.get(name, 0)
            print(f'    {name:18s} min clearance {self.min_clear[name]:+7.3f} m'
                  f'   contacts {n}   near misses {self.near_misses.get(name, 0)}')
        gaps = [g for g in self.min_clear.values()]
        if gaps:
            print(f'  closest anyone came: {min(gaps):+.3f} m')
            if len(gaps) > 1:
                print(f'  median of the per person minima: '
                      f'{statistics.median(gaps):+.3f} m')
        print()
        if total:
            print('  A CONTACT IS A FAILURE OF THE SAFETY LAYER, and in this')
            print('  simulation it does not stop the vehicle, because the person')
            print('  model carries no collision geometry and is walked through.')
            print('  The run continues looking normal. That is why this is')
            print('  measured here rather than left to physics.')
        else:
            print('  No contact. Note what this does and does not say: the')
            print('  people here cannot physically stop the vehicle, so this is')
            print('  evidence the stack kept clear, not evidence that anything')
            print('  would have prevented it if the stack had not.')
        print('=' * 70)


def main():
    rclpy.init()
    node = ContactProbe()
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
