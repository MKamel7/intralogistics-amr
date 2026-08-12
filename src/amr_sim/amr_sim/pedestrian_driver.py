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

MEETING THE VEHICLE IS DIFFERENT, and is the point of the scenario. A walker
that sees the vehicle within `yield_distance` and roughly ahead STOPS DEAD and
holds its ground, keeping the goal it was walking to. It becomes a stationary
obstacle in the aisle that the robot has to plan around and then continue past
to its original target. See the long note beside `robot_response` for the two
earlier models and why both were wrong.

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
        # WHAT A PERSON DOES WHEN A VEHICLE COMES AT THEM.
        #
        # THE HISTORY MATTERS, because both earlier answers were wrong in
        # opposite directions and the second one quietly invalidated the
        # headline result.
        #
        # First, walkers ignored the vehicle entirely, because they have no
        # collision geometry and nothing told them it was there. Measured, one
        # passed within 0.38 m of it and one was inside 1.5 m for 31 percent of
        # the run. Every resulting protective stop was correct, and together
        # they made it impossible for the vehicle to depart at all: stopped 46
        # percent of the time, averaging 0.03 m/s. Modelling people who walk
        # blindly into a moving AMR does not make the test harder, it makes it
        # meaningless, because the protective stop is exercised constantly and
        # nothing else ever is.
        #
        # Then walkers AVOIDED the vehicle, keeping 1.6 m from it and refusing
        # to even pick a goal near it. That is what this file did until now, and
        # it is worse, because it looks reasonable. The largest protective field
        # the MiR250 uses reaches 1.43 m from the vehicle centre, so a 1.6 m
        # exclusion radius puts people exactly outside the region that would
        # trigger a stop. It made the safety layer look cheap by construction,
        # and, more damaging, it meant THE ROBOT NEVER HAD TO REPLAN AROUND A
        # PERSON. The whole reason MPPI is used here rather than Pure Pursuit is
        # that it reacts to an obstacle appearing in the local costmap, and no
        # scenario in this repository ever produced one.
        #
        # WHAT PEOPLE ACTUALLY DO, and what is modelled now: they see the
        # vehicle and STOP, and they stay stopped while it deals with them. A
        # stationary person in an aisle is a dynamic obstacle that then
        # persists, which is the case that forces the local costmap to mark it,
        # the global planner to re-route, and the vehicle to reach its original
        # goal by another way. That is the behaviour worth demonstrating, and it
        # is also what a trained warehouse worker does, which is what ISO 3691-4
        # assumes is in the operating area.
        #
        # They are NOT steered away from the vehicle's path any more, so they
        # genuinely get in the way.
        self.robot_response = self.declare_parameter(
            'robot_response', 'stop').value
        # Far enough out that stopping produces a RE-ROUTE rather than a
        # protective stop: the protective field reaches at most 1.43 m from the
        # vehicle centre, so a person halting at 3 m is an obstacle the planner
        # has time to route around. The protective stop stays as the backstop
        # for when the aisle is too narrow to route around at all, which is the
        # other half of what this scenario measures.
        self.yield_distance = self.declare_parameter('yield_distance', 3.0).value
        self.yield_arc = self.declare_parameter('yield_arc', 75.0).value
        # NOBODY STANDS THERE FOREVER. In an aisle at this building's measured
        # median width of 1.34 m there is often no room to route around a
        # standing person at all, and without a release the vehicle waits and
        # the person waits and the cycle times out on a deadlock neither of
        # them would sustain in reality. After this long the person gives up and
        # moves on, which is also what happens when someone notices an AMR
        # patiently waiting for them.
        self.yield_release_s = self.declare_parameter(
            'yield_release_s', 20.0).value
        # Retained for the old behaviour, selected with robot_response: avoid,
        # so the runs measured before this change stay reproducible.
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
                # How long this walker has been standing still for the vehicle.
                'holding_for': 0.0,
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
            # In AVOID mode, do not set off towards a spot the vehicle is
            # occupying. In STOP mode this check is deliberately skipped: a
            # walker that will not even choose a goal near the vehicle never
            # gets in its way, and getting in its way is the entire point.
            if self.robot_response == 'avoid' and self._too_near_robot(gx, gy):
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

    def _vehicle_ahead(self, x, y, heading):
        """Is the vehicle close enough and far enough forward to stop for?

        Distance and bearing only. Whether the VEHICLE is heading this way is
        deliberately not considered: a person who has seen an AMR nearby stops
        for it, and does not first estimate its heading to decide whether they
        need to.
        """
        robot = self.poses.get(self.robot_frame)
        if robot is None:
            return False
        dx, dy = robot[0] - x, robot[1] - y
        if math.hypot(dx, dy) > self.yield_distance:
            return False
        bearing = math.atan2(dy, dx) - heading
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return abs(bearing) < math.radians(self.yield_arc)

    def _blocked_by_person(self, name, x, y, heading):
        """Is another pedestrian, or the vehicle, close and roughly ahead?"""
        for other, pose in self.poses.items():
            if other == name:
                continue
            if other == self.robot_frame:
                if self.robot_response != 'avoid':
                    continue        # handled by _vehicle_ahead, which stops
                                    # dead rather than turning away
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

            # STAND STILL FOR THE VEHICLE.
            #
            # Zero twist and no turn, so the figure holds its ground and its
            # footprint stays where the costmap put it. A walker that spun on
            # the spot here would smear its returns across neighbouring cells
            # and read as a moving obstacle rather than a standing person.
            #
            # The goal is KEPT, not abandoned, and stuck_for is not advanced, so
            # the walker resumes the journey it was on once the vehicle has
            # gone, rather than wandering off and taking the obstacle with it.
            if self.robot_response == 'stop' and self._vehicle_ahead(x, y, yaw):
                if w['holding_for'] < self.yield_release_s:
                    w['holding_for'] += self.dt
                    w['pub'].publish(Twist())
                    continue
                # Held long enough that the vehicle is evidently not getting
                # past. Move on, and do not stop for it again until clear.
            else:
                w['holding_for'] = 0.0

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
