#!/usr/bin/env python3
"""Work out WHAT the safety layer is stopping for.

WHY THIS EXISTS

A transport cycle recorded 32 protective stops in 122 seconds. That number on
its own says nothing useful, because two completely different situations
produce it and they lead to opposite fixes:

    a person stepped in front of the vehicle
        The system is working exactly as designed. The answer is about
        scenario density, and possibly a human-aware costmap layer so the
        planner routes around people rather than driving at them and stopping.

    the vehicle stopped for a rack, a wall, or its own geometry
        The system is not working. The answer is in the field geometry, the
        margins, or the self filter, and the vehicle is being prevented from
        doing its job by furniture.

Guessing between them is how the previous several days were spent. So this
measures it.

HOW IT CLASSIFIES

On every transition into a protective stop it takes the current scan, transforms
the returns into the world frame, and asks whether the nearest returns sit close
to a pedestrian's true position. The pose oracle is legitimate here: this is a
measurement tool, it publishes nothing, and nothing in the control path reads
it. See ADR 0006.

A return is attributed to a person if it lies within `person_radius` of a walker
whose true pose came from `/ground_truth/poses`. Everything else is structure.
The verdict for a stop is taken from the CLOSEST return, because that is the one
that entered the field first and therefore the one that caused the stop.
"""

import math
import sys
import time
from collections import Counter

import rclpy
from rclpy.executors import ExternalShutdownException
import tf2_ros
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# Bearing sectors, in degrees from straight ahead. Defined once because the
# forward, side and rear counts and every split derived from them must use the
# same boundaries; when they did not, the rear split printed a negative count.
SIDE_BEARING = 60.0
REAR_BEARING = 120.0


ACTION = {0: 'clear', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}


class StopClassifier(Node):
    def __init__(self):
        super().__init__('classify_stops',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        # A walker's legs are two cylinders about 0.2 m apart, so returns from a
        # person spread over roughly a 0.3 m circle. 0.55 m allows for that plus
        # the pose oracle being the model origin rather than the leg surface.
        self.person_radius = self.declare_parameter('person_radius', 0.55).value
        self.duration = self.declare_parameter('duration_s', 300.0).value

        self.scan = None
        self.people = {}
        self.robot = None
        self.create_subscription(LaserScan, '/scan', self._scan,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/ground_truth/poses', self._poses, 20)
        self.create_subscription(CollisionMonitorState,
                                 '/collision_monitor_state', self._state, 20)
        # WHAT THE VEHICLE WAS DOING when the field fired. A stop for something
        # behind is correct if the vehicle was turning on the spot, because a
        # spot turn sweeps the rear, and wrong if it was driving forward. The
        # summary could not tell those apart.
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.create_subscription(
            TwistStamped, '/cmd_vel_raw',
            lambda m: (setattr(self, 'cmd_vx', m.twist.linear.x),
                       setattr(self, 'cmd_wz', m.twist.angular.z)), 20)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)

        # THE COSTMAPS, so a stop can be attributed to one of two causes that
        # need opposite fixes:
        #
        #   the offending point IS in the costmap
        #       the planner routed the vehicle to within centimetres of an
        #       obstacle it knew about. The fault is in the margins or in how
        #       closely the controller tracks the path.
        #
        #   the offending point is NOT in the costmap
        #       the costmap is missing an obstacle the scanner can see. The
        #       fault is in the layers, their update rate, or their range.
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.costmaps = {}
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: self.costmaps.__setitem__('global', m),
                                 latched)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: self.costmaps.__setitem__('local', m),
                                 latched)

        self.action = 'clear'
        self.verdicts = Counter()
        self.detail = []
        self.costmap_says = Counter()

    def _scan(self, msg):
        self.scan = msg

    def _poses(self, msg):
        for t in msg.transforms:
            xy = (t.transform.translation.x, t.transform.translation.y)
            if t.child_frame_id.startswith('walker') or 'worker' in t.child_frame_id:
                self.people[t.child_frame_id] = xy
            elif t.child_frame_id == 'amr':
                self.robot = xy

    def _state(self, msg):
        action = ACTION.get(msg.action_type, 'unknown')
        if action == 'stop' and self.action != 'stop':
            # WHICH FIELD FIRED, recorded because the summary could not tell a
            # rear field live during forward motion from an all-round rotation
            # field doing its job. Seventeen of forty five stops were for
            # something behind while driving forward and the summary could only
            # say "the field is being too generous", which is a guess. The
            # polygon name says which band the monitor actually selected, and
            # the commanded twist says what the vehicle was doing at the time.
            self.classify(msg.polygon_name)
        self.action = action

    def classify(self, polygon=''):
        if self.scan is None or self.robot is None:
            self.verdicts['no data'] += 1
            return
        try:
            tr = self.tf.lookup_transform('map', self.scan.header.frame_id,
                                          rclpy.time.Time())
        except Exception:
            self.verdicts['no transform'] += 1
            return
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tx, ty = tr.transform.translation.x, tr.transform.translation.y

        m = self.scan
        best = None
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < m.range_max) or r > 1.5:
                continue
            a = m.angle_min + i * m.angle_increment
            # Scan frame to map frame.
            lx, ly = r * math.cos(a), r * math.sin(a)
            wx = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = ty + lx * math.sin(yaw) + ly * math.cos(yaw)
            if best is None or r < best[0]:
                best = (r, math.degrees(a), wx, wy)
        if best is None:
            # Nothing within 1.5 m at all. The stop came from something the
            # scan does not show, which is itself a finding.
            self.verdicts['nothing in the scan'] += 1
            return

        r, bearing, wx, wy = best
        near = min(((math.hypot(wx - px, wy - py), name)
                    for name, (px, py) in self.people.items()),
                   default=(1e9, None))
        kind = 'person' if near[0] <= self.person_radius else 'structure'
        self.verdicts[kind] += 1
        self.detail.append((kind, r, bearing, near[1], near[0],
                            polygon, self.cmd_vx, self.cmd_wz))

        # Was the thing that stopped us already in the costmap?
        for which in ('global', 'local'):
            self.costmap_says[f'{which}: {self._costmap_value(which, wx, wy)}'] += 1

    def _costmap_value(self, which, wx, wy):
        """Describe a world point's status in one of the costmaps."""
        grid = self.costmaps.get(which)
        if grid is None:
            return 'not received'
        info = grid.info
        # THE TWO COSTMAPS ARE IN DIFFERENT FRAMES. The global one is in `map`,
        # the local one is in `odom` so it stays smooth through a SLAM loop
        # closure. The offending point is in `map`, so it has to be converted
        # before being looked up, or the local costmap answer is quietly wrong
        # by however far the two frames have diverged. A diagnostic that lies is
        # worse than no diagnostic.
        px, py = wx, wy
        frame = grid.header.frame_id or 'map'
        if frame != 'map':
            try:
                tr = self.tf.lookup_transform(frame, 'map', rclpy.time.Time())
            except Exception:
                return f'no transform map to {frame}'
            q = tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            px = (tr.transform.translation.x
                  + wx * math.cos(yaw) - wy * math.sin(yaw))
            py = (tr.transform.translation.y
                  + wx * math.sin(yaw) + wy * math.cos(yaw))
        i = int((px - info.origin.position.x) / info.resolution)
        j = int((py - info.origin.position.y) / info.resolution)
        if not (0 <= i < info.width and 0 <= j < info.height):
            return 'outside the costmap'
        v = grid.data[j * info.width + i]
        if v < 0:
            return 'UNKNOWN'
        if v >= 99:
            return 'lethal or inscribed'
        if v > 0:
            return f'inflated ({v})'
        return 'FREE (costmap did not know)'

    def run(self):
        end = time.monotonic() + self.duration
        self.get_logger().info(
            f'classifying protective stops for {self.duration:.0f} s')
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.report()
        return 0

    def report(self):
        total = sum(self.verdicts.values())
        print('\n' + '=' * 66)
        print(f'  {total} protective stop(s) classified')
        if not total:
            print('  no stops occurred; nothing to explain')
            print('=' * 66)
            return
        for verdict, n in self.verdicts.most_common():
            print(f'    {verdict:22s} {n:4d}   {n / total * 100:5.1f} %')
        struct = [d for d in self.detail if d[0] == 'structure']
        if struct:
            rng = [d[1] for d in struct]
            bearings = [d[2] for d in struct]
            print(f'\n  structure stops: range {min(rng):.2f} to {max(rng):.2f} m, '
                  f'median {sorted(rng)[len(rng) // 2]:.2f} m')
            fwd = sum(1 for b in bearings if abs(b) < SIDE_BEARING)
            side = sum(1 for b in bearings
                       if SIDE_BEARING <= abs(b) < REAR_BEARING)
            rear = sum(1 for b in bearings if abs(b) >= REAR_BEARING)
            print(f'    ahead {fwd}, to the side {side}, behind {rear}')
            if rear:
                # Behind, split by what the vehicle was doing and which band
                # the monitor had selected. Turning on the spot sweeps the rear
                # and a stop there is correct; driving forward it is not.
                #
                # ONE PREDICATE FOR "BEHIND", used everywhere below. It used to
                # be `>= 120` for the count and `> 90` for the split, so the
                # split ran over a larger set than the thing it was splitting
                # and printed "and -5 while NOT turning". A negative count is
                # the only reason anybody looked; the same mismatch was also
                # quietly inflating the band histogram beside it.
                behind = [d for d in self.detail
                          if d[0] == 'structure' and abs(d[2]) >= REAR_BEARING]
                assert len(behind) == rear, (
                    f'{len(behind)} rear stops by one rule and {rear} by the '
                    f'other; the two disagree and the split below is not real')
                turning = sum(1 for d in behind if abs(d[7]) > 0.10)
                straight = rear - turning
                print(f'      of those, {turning} while turning on the spot '
                      f'(correct: a spot turn sweeps the rear)')
                print(f'      and {straight} while NOT turning, which is the '
                      f'rear field live during forward motion')
                bands = Counter(d[5] for d in behind if d[5])
                for band, n in bands.most_common(4):
                    print(f'      band {band}: {n}')
            print('    a stop for something BEHIND while driving forward is the '
                  'field being too generous, not an obstacle')
        if self.costmap_says:
            print('\n  was the offending point already in the costmap?')
            for key, n in sorted(self.costmap_says.items()):
                print(f'    {key:42s} {n:4d}')
            print('    FREE or UNKNOWN means the costmap missed it; lethal means')
            print('    the planner drove the vehicle up against something it knew about')
        print('=' * 66)


def main():
    rclpy.init()
    node = StopClassifier()
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.report()
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
