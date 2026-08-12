#!/usr/bin/env python3
"""Drive scenario pedestrians on random, collision-free walks.

HOW A PEDESTRIAN DECIDES WHERE TO GO

Each one picks a random goal, walks to it, then picks another. A goal is only
accepted if the straight line to it is clear in the GROUND TRUTH floorplan,
sampled every few centimetres and requiring a clearance radius around each
sample. So a pedestrian physically cannot be handed a route that passes through
a rack, a pallet or a wall: the route is rejected before it is walked, rather
than being corrected afterwards by a collision the physics engine has to resolve.

WHICH MAP, AND WHY NOT THE ROBOT'S

An earlier version planned against the robot's SLAM map on `/map`. That was the
wrong map. `/map` is what the ROBOT has discovered, so at startup it is nearly
nothing: measured, 88 m2 of which only 6.4 percent had the 0.45 m of clearance a
walker needs, and all three walkers spawned on cells that failed the test. They
correctly refused to move and stood still for the whole run.

The floorplan built from the world's own collision meshes has 99 m2 clear at the
same radius and exists in full before the robot has turned a wheel. It is also
the accurate model of the situation: a person working in a warehouse knows the
building, and only the robot has to learn it. Nothing here is fed back to the
robot, so its map stays honestly earned.

WHY NOT A HAND-WRITTEN LANE LIST

The version before that walked fixed straight lanes whose endpoints were
guessed, and two of three spawned inside the racking. Reading a map removes the
guesswork and means the scenario file encodes no knowledge of the layout.

STEERING

Heading error drives yaw rate, and forward speed is scaled by how well the figure
already faces its goal, so it turns before it walks rather than crabbing
sideways. Walkers also yield to each other: if another is close and roughly
ahead, this one stops and keeps turning, so a pair untangles instead of
deadlocking nose to nose.

Motion is seeded. Random has to be repeatable or none of the detection numbers
measured against these scenarios are comparable between runs.
"""

import math
import random
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from amr_sim.occupancy import load_map

SCENARIOS = Path(__file__).resolve().parent.parent / 'scenarios'


class PedestrianDriver(Node):
    def __init__(self):
        super().__init__('pedestrian_driver')
        name = self.declare_parameter('scenario', 'walking_people').value
        path = self.declare_parameter('scenario_path', '').value
        seed = self.declare_parameter('seed', 7).value
        # Clearance kept from anything in the map. The body is about 0.25 m
        # across; the rest is margin so a walker never brushes a rack.
        self.clearance = self.declare_parameter('clearance', 0.45).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.30).value
        self.personal_space = self.declare_parameter('personal_space', 1.1).value
        # HOW FAR PEOPLE STAY FROM THE VEHICLE.
        #
        # Walkers used to ignore the robot completely, because they have no
        # collision geometry and nothing told them it was there. Measured, one
        # passed within 0.38 m of it and one was inside 1.5 m for 31 percent of
        # the run. Every one of those was a correct protective stop, and
        # together they made it impossible for the vehicle to depart at all: it
        # was stopped 46 percent of the time and averaged 0.03 m/s.
        #
        # That is not what a warehouse looks like. Staff who work alongside AMRs
        # see them and give way, and ISO 3691-4 assumes trained personnel in the
        # operating area. Modelling people who walk blindly into a moving
        # vehicle does not make the test harder, it makes it meaningless: the
        # protective stop is exercised constantly and nothing else ever is.
        #
        # So walkers avoid the vehicle, and the protective stop is left to prove
        # itself in the scenario written for it rather than by accident here.
        self.robot_clearance = self.declare_parameter('robot_clearance', 1.6).value
        self.robot_frame = self.declare_parameter('robot_frame', 'amr').value

        self.rng = random.Random(seed)

        source = Path(path) if path else (SCENARIOS / f'{name}.yaml')
        spec = yaml.safe_load(source.read_text())

        self.walkers = {}
        for person in spec.get('people', []):
            w = person.get('wander')
            if not w:
                continue
            self.walkers[person['name']] = {
                'pub': self.create_publisher(
                    Twist, f'/model/{person["name"]}/cmd_vel', 10),
                'speed': float(w.get('speed', 1.0)),
                'range': float(w.get('range', 6.0)),
                'goal': None,
                'pose': None,
                'stuck_for': 0.0,
                'dwell': 0.0,
            }
        self.dwell_range = (
            self.declare_parameter('dwell_min', 0.5).value,
            self.declare_parameter('dwell_max', 3.0).value)

        default_map = str(Path(get_package_share_directory('amr_sim'))
                          / 'maps' / 'warehouse_truth.yaml')
        map_file = self.declare_parameter('map_file', default_map).value
        self.grid = load_map(map_file)
        x0, y0, x1, y1 = self.grid.bounds
        self.poses = {}
        self.create_subscription(TFMessage, '/ground_truth/poses', self._poses, 10)
        self.dt = 0.05
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'wandering {len(self.walkers)} pedestrian(s) from {source.name}, '
            f'seed {seed}, routes checked against {Path(map_file).name} '
            f'({self.grid.w}x{self.grid.h} at {self.grid.res:.3f} m, '
            f'x[{x0:.1f},{x1:.1f}] y[{y0:.1f},{y1:.1f}]) '
            f'with {self.clearance} m clearance')

    def _poses(self, msg):
        for tf in msg.transforms:
            p = tf.transform.translation
            q = tf.transform.rotation
            self.poses[tf.child_frame_id] = (p.x, p.y, 2.0 * math.atan2(q.z, q.w))
            if tf.child_frame_id in self.walkers:
                self.walkers[tf.child_frame_id]['pose'] = self.poses[tf.child_frame_id]

    def _pick_goal(self, w):
        """A random point this walker can reach in a straight line."""
        if w['pose'] is None:
            return None
        x, y, _ = w['pose']

        # A walker that is not itself standing clear can never satisfy the
        # segment test, because the segment starts underneath it, so it would
        # stand still forever. Send it to the nearest clear spot first. This
        # only fires if a scenario places someone badly; it is a recovery, not
        # the normal path.
        if not self.grid.clear(x, y, self.clearance):
            escape = self._nearest_clear(x, y)
            if escape is not None:
                self.get_logger().warn(
                    f'spawn at ({x:.2f}, {y:.2f}) has less than '
                    f'{self.clearance} m clearance, stepping out to '
                    f'({escape[0]:.2f}, {escape[1]:.2f})')
            return escape

        for _ in range(60):
            ang = self.rng.uniform(-math.pi, math.pi)
            dist = self.rng.uniform(1.5, w['range'])
            gx, gy = x + dist * math.cos(ang), y + dist * math.sin(ang)
            if not self.grid.clear(gx, gy, self.clearance):
                continue
            if not self.grid.segment_clear(x, y, gx, gy, self.clearance):
                continue
            # Do not set off towards a spot the vehicle is occupying. Yielding
            # while walking handles the vehicle arriving; this stops a walker
            # choosing to walk at it in the first place.
            if self._too_near_robot(gx, gy):
                continue
            return (gx, gy)
        return None

    def _nearest_clear(self, x, y, limit=3.0):
        """Closest point with full clearance, searched outward in rings."""
        step = self.grid.res * 2
        r = step
        while r <= limit:
            n = max(8, int(2 * math.pi * r / step))
            for k in range(n):
                ang = 2 * math.pi * k / n
                cx, cy = x + r * math.cos(ang), y + r * math.sin(ang)
                if self.grid.clear(cx, cy, self.clearance):
                    return (cx, cy)
            r += step
        return None

    def stop_all(self):
        """Command every walker to a halt, and give it time to leave."""
        for _ in range(5):
            for w in self.walkers.values():
                w['pub'].publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def _blocked_by_person(self, name, x, y, heading):
        """Is another pedestrian, or the vehicle, close and roughly ahead?"""
        for other, pose in self.poses.items():
            if other == name:
                continue
            if other == self.robot_frame:
                gap, arc = self.robot_clearance, 90.0
            elif other in self.walkers:
                gap, arc = self.personal_space, 70.0
            else:
                continue
            dx, dy = pose[0] - x, pose[1] - y
            if math.hypot(dx, dy) > gap:
                continue
            bearing = math.atan2(dy, dx) - heading
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            # A wider arc for the vehicle than for another person: someone
            # stepping round a colleague only has to miss them, but nobody
            # walks close past the side of a moving AMR on purpose.
            if abs(bearing) < math.radians(arc):
                return True
        return False

    def _too_near_robot(self, x, y):
        """Is this point inside the space kept clear around the vehicle?"""
        robot = self.poses.get(self.robot_frame)
        if robot is None:
            return False
        return math.hypot(robot[0] - x, robot[1] - y) < self.robot_clearance

    def _tick(self):
        for name, w in self.walkers.items():
            if w['pose'] is None:
                continue
            x, y, yaw = w['pose']

            # Workers stop at a rack, read a label, move on. Without a pause
            # they ricochet from goal to goal, which reads as machinery rather
            # than people and gives the tracker an unrealistically easy target:
            # a person who stands still and then sets off again is the harder
            # and more honest case.
            if w['dwell'] > 0.0:
                w['dwell'] -= self.dt
                w['pub'].publish(Twist())
                continue

            if w['goal'] is None:
                w['goal'] = self._pick_goal(w)
                if w['goal'] is None:
                    w['pub'].publish(Twist())
                    continue

            gx, gy = w['goal']
            dx, dy = gx - x, gy - y
            if math.hypot(dx, dy) < self.goal_tolerance:
                w['goal'] = None
                w['dwell'] = self.rng.uniform(*self.dwell_range)
                w['pub'].publish(Twist())
                continue

            err = math.atan2(dy, dx) - yaw
            err = math.atan2(math.sin(err), math.cos(err))

            msg = Twist()
            if self._blocked_by_person(name, x, y, yaw):
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            else:
                msg.angular.z = max(-1.5, min(1.5, 2.0 * err))
                msg.linear.x = w['speed'] * max(0.0, math.cos(err))
            w['pub'].publish(msg)

            # A goal that stops being reachable, because the map grew or another
            # walker parked on it, is abandoned rather than pursued forever.
            if msg.linear.x < 0.05:
                w['stuck_for'] += self.dt
                if w['stuck_for'] > 6.0:
                    w['goal'] = None
                    w['stuck_for'] = 0.0
            else:
                w['stuck_for'] = 0.0


def main():
    rclpy.init()
    node = PedestrianDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # STOP EVERYONE ON THE WAY OUT.
        #
        # The velocity plugin holds the last twist it was given, and the figures
        # have gravity off and no collision, so nothing slows them. When this
        # node was killed mid-walk the walkers kept their last commanded speed
        # indefinitely: measured after one such kill, they had travelled to
        # x = 143 m and y = -176 m, far outside a 15 by 22 m building, and were
        # still going. Publishing zero before exiting leaves them where they
        # stand instead.
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
