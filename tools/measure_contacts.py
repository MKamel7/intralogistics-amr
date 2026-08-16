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
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
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


def closing_split(v_vehicle, v_person, offset):
    """How fast each party was closing the gap, in m/s along the line.

    `offset` points from the vehicle to the person. A POSITIVE share means that
    party moved toward the other; negative means away. The two sum to the total
    closing rate.

    This is the difference between "the stack drove into somebody" and
    "somebody walked into the back of a vehicle that was barely moving", and
    the vehicle's own speed cannot tell them apart: a contact at 0.03 m/s was
    labelled DRIVING INTO THEM when the person was doing over a metre a second.

    Returns (0.0, 0.0) when either velocity is unknown, because an unknown is
    not a zero and must not be reported as one.
    """
    if v_vehicle is None or v_person is None:
        return 0.0, 0.0
    ox, oy = offset
    n = math.hypot(ox, oy)
    if n < 1e-9:
        return 0.0, 0.0
    ux, uy = ox / n, oy / n
    # The vehicle closes by moving along +u, the person by moving along -u.
    return (v_vehicle[0] * ux + v_vehicle[1] * uy,
            -(v_person[0] * ux + v_person[1] * uy))


def blame(v_share, p_share):
    """Plain words for the split, with a stated threshold rather than a feel."""
    if v_share <= 0.02 and p_share <= 0.02:
        return 'neither was closing; contact by drift or by a pose jump'
    if v_share > 2.0 * max(p_share, 0.0):
        return 'THE VEHICLE DROVE INTO THEM'
    if p_share > 2.0 * max(v_share, 0.0):
        return 'they walked into the vehicle'
    return 'both were closing; neither dominates'


class ContactProbe(Node):
    def __init__(self):
        super().__init__('contact_probe',
                         # THE SIMULATED CLOCK. Without it get_clock() returns
                         # epoch seconds while every stamp read here is sim
                         # time, and the difference between them is not a
                         # duration. See V-52.
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 600.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        # Footprint half extents. Defaults are the MP-400's; pass the platform's
        # own rather than trusting these.
        self.half_length = self.declare_parameter('half_length', 0.300).value
        self.half_width = self.declare_parameter('half_width', 0.2845).value
        self.near_miss = self.declare_parameter('near_miss', 0.30).value

        self.min_clear = {}
        self.min_bearing = {}
        self.contacts = {}
        self.near_misses = {}
        self.samples = 0
        self.in_contact = set()
        # WHO MOVED INTO WHOM. A contact while the vehicle is stationary is a
        # person walking into a parked robot, which is not a failure of the
        # robot and must not be counted as one. Measured: of four contacts in
        # one mission, two happened with the vehicle at 0.00 m of travel for
        # the whole minute around them.
        self.speed = 0.0
        self.contact_speeds = {}
        # ...AND THAT RULE WAS TOO CRUDE. It labelled a contact at 0.03 m/s
        # "DRIVING INTO THEM", which is the vehicle creeping while a person
        # walks at over a metre a second. The vehicle's own speed says how fast
        # it was going, not who closed the distance, and those are the
        # difference between the stack driving into somebody and somebody
        # walking into the back of a nearly stopped vehicle.
        #
        # So the closing rate is split. Each person's own velocity comes from
        # the ground truth poses, differentiated over the interval between
        # frames, and both velocities are projected onto the line between them.
        self.prev_person = {}       # name -> (x, y, t) in the world frame
        self.prev_vehicle = None
        self.contact_closing = {}   # name -> [(v_share, p_share), ...]

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.create_subscription(TwistStamped,
                                 '/diff_drive_controller/cmd_vel',
                                 self._cmd, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False
        self.get_logger().info(
            f'watching for contact against a '
            f'{2 * self.half_length:.3f} by {2 * self.half_width:.3f} m footprint')

    def _cmd(self, msg):
        self.speed = abs(msg.twist.linear.x)

    @staticmethod
    def _velocity(prev, now_xy, t):
        """World frame velocity from two ground truth samples.

        None on the first sample for a given body, and on a zero or backward
        time step, which the simulated clock does produce. Returning (0, 0)
        there would read as "stood still" and quietly credit the other party
        with all of the closing.
        """
        if prev is None:
            return None
        dt = t - prev[2]
        if dt <= 1e-6:
            return None
        return ((now_xy[0] - prev[0]) / dt, (now_xy[1] - prev[1]) / dt)

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
        now = self.get_clock().now().nanoseconds * 1e-9
        vveh = self._velocity(self.prev_vehicle, v, now)
        self.prev_vehicle = (v[0], v[1], now)
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
                # WHERE, not just how close. A person 89 mm from the footprint
                # with no protective stop is a different fault depending on
                # bearing: dead astern is a known blind zone between two corner
                # scanners, dead ahead is the safety layer failing at the one
                # thing it exists for. Without this the two are
                # indistinguishable and the argument goes in circles.
                self.min_bearing[name] = math.degrees(math.atan2(py, px))
            if gap <= 0.0:
                # Rising edge only. A person standing inside the vehicle for
                # three seconds is one contact, not sixty.
                if name not in self.in_contact:
                    self.contacts[name] = self.contacts.get(name, 0) + 1
                    self.in_contact.add(name)
                    self.contact_speeds.setdefault(name, []).append(self.speed)
                    vp = self._velocity(self.prev_person.get(name), (x, y), now)
                    v_share, p_share = closing_split(vveh, vp, (dx, dy))
                    self.contact_closing.setdefault(name, []).append(
                        (v_share, p_share))
                    self.get_logger().error(
                        f'CONTACT with {name}, {-gap:.3f} m inside the '
                        f'footprint, vehicle at {self.speed:.2f} m/s. '
                        f'Closing: vehicle {v_share:+.2f} m/s, person '
                        f'{p_share:+.2f} m/s ({blame(v_share, p_share)})')
            else:
                self.in_contact.discard(name)
                if gap <= self.near_miss:
                    self.near_misses[name] = self.near_misses.get(name, 0) + 1
            self.prev_person[name] = (x, y, now)

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
        driven = sum(1 for v in self.contact_speeds.values() for s in v if s > 0.02)
        # WHO CLOSED THE DISTANCE, which is not the same question as whether
        # the vehicle happened to be moving. A contact at 0.03 m/s counts as
        # "moving" and can still be a person walking into the back of it.
        shares = [pair for v in self.contact_closing.values() for pair in v]
        drove_in = sum(1 for vs, ps in shares if vs > 2.0 * max(ps, 0.0))
        walked_in = sum(1 for vs, ps in shares if ps > 2.0 * max(vs, 0.0))
        print(f'  CONTACTS: {total}   of which the vehicle was MOVING: {driven}')
        if shares:
            print(f'    by who closed the distance: vehicle drove in '
                  f'{drove_in}, person walked in {walked_in}, '
                  f'neither dominant {len(shares) - drove_in - walked_in}')
        for name in sorted(self.min_clear):
            n = self.contacts.get(name, 0)
            b = self.min_bearing.get(name)
            side = ('ahead' if abs(b) <= 45 else
                    'astern' if abs(b) >= 135 else
                    'port' if b > 0 else 'starboard')
            print(f'    {name:18s} min clearance {self.min_clear[name]:+7.3f} m'
                  f'   at {b:+6.0f} deg ({side})'
                  f'   contacts {n}   near misses {self.near_misses.get(name, 0)}')
        gaps = [g for g in self.min_clear.values()]
        if gaps:
            print(f'  closest anyone came: {min(gaps):+.3f} m')
            if len(gaps) > 1:
                print(f'  median of the per person minima: '
                      f'{statistics.median(gaps):+.3f} m')
        print()
        if total and not driven:
            print('  Every contact happened with the vehicle stationary, so a')
            print('  person walked into a parked robot. That is a scenario')
            print('  artefact, not a safety failure: these pedestrians do not')
            print('  avoid the vehicle by design, because a crowd that dodges')
            print('  never tests anything.')
        elif drove_in:
            print(f'  {drove_in} CONTACT(S) THE VEHICLE DROVE INTO. That is the')
            print('  safety layer failing, and in this')
            print('  simulation it does not stop the vehicle, because the person')
            print('  model carries no collision geometry and is walked through.')
            print('  The run continues looking normal. That is why this is')
            print('  measured here rather than left to physics.')
        elif driven:
            print(f'  {driven} contact(s) with the vehicle moving, but NONE of')
            print('  them driven into: the person closed the distance in every')
            print('  case. A creeping vehicle counts as moving and that is not')
            print('  the same as it being the party at fault. The split above')
            print('  is the honest number; the MOVING count on its own is not.')
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
