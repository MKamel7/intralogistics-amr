#!/usr/bin/env python3
"""Split path overhead into the planner's share and the controller's.

WHY THIS EXISTS

A transport cycle drove 58.3 m on a journey whose straight line is about 39 m,
a factor of 1.49. That is real and worth fixing, and "make path planning more
efficient" is not yet an actionable statement, because two entirely different
faults produce the same number:

    planner overhead     plan length / straight line distance
        The global plan itself is long. Fix the planner: inflation pushing
        routes wide, a keepout mask that is too generous, a map with phantom
        obstacles in it.

    controller overhead  distance driven / plan length
        The plan is short and the vehicle does not follow it. Fix the
        controller: MPPI missing its control rate, oscillating near goals,
        weaving between critics that disagree.

Tuning the planner when the controller is the problem is how a project spends a
week making things worse. This measures both and says which one owns the
overhead.

CAVEAT, and it matters for reading the result

Re-routing around a person is legitimate overhead. A run with pedestrians
SHOULD exceed 1.0 on the planner term, because the straight line ignores the
people standing in it. The figure to watch is the controller term, which no
obstacle explains: a vehicle that drives 1.4 times its own plan is not avoiding
anything, it is failing to track.

    tools/measure_path_efficiency.py --ros-args -p duration_s:=400.0
"""

import math
import statistics
import sys

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

TRUTH_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)

# Below this, a step is jitter rather than travel. Integrating every wobble
# would inflate the driven distance and blame the controller for noise.
MIN_STEP = 0.01

# A new plan whose endpoint moved by more than this is a new GOAL rather than a
# re-plan toward the same one. Re-plans are continuous and constant; goals are
# metres apart.
NEW_GOAL = 1.0


def polyline_length(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


class PathProbe(Node):
    def __init__(self):
        super().__init__('path_efficiency_probe')
        self.duration = self.declare_parameter('duration_s', 400.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value

        self.pos = None
        self.driven = 0.0
        self.legs = []              # (straight, plan_len, driven) per goal
        self.goal = None
        self.leg_start = None
        self.leg_driven = 0.0
        self.plan_len = None
        self.replans = 0

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False

    def _truth(self, msg):
        tf = next((t for t in msg.transforms
                   if t.child_frame_id == self.vehicle_frame), None)
        if tf is None:
            return
        p = (tf.transform.translation.x, tf.transform.translation.y)
        if self.pos is not None:
            d = math.dist(p, self.pos)
            if d > MIN_STEP:
                self.driven += d
                self.leg_driven += d
        self.pos = p

    def _plan(self, msg):
        if not msg.poses or self.pos is None:
            return
        pts = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        end = pts[-1]
        if self.goal is None or math.dist(end, self.goal) > NEW_GOAL:
            # A new goal. Bank the leg just finished.
            self._bank()
            self.goal = end
            self.leg_start = self.pos
            self.leg_driven = 0.0
            # The plan length AT THE START of the leg is the one to compare
            # driven distance against. Later re-plans are shorter simply
            # because the vehicle has made progress.
            self.plan_len = polyline_length(pts)
        else:
            self.replans += 1

    def _bank(self):
        if self.goal is None or self.leg_start is None or self.plan_len is None:
            return
        straight = math.dist(self.leg_start, self.goal)
        # Legs shorter than a vehicle length are docking wobble, not journeys.
        if straight > 1.0 and self.leg_driven > 0.5:
            self.legs.append((straight, self.plan_len, self.leg_driven))

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        self._bank()
        print('\n' + '=' * 70)
        print(f'  path efficiency, {len(self.legs)} completed leg(s), '
              f'{self.replans} re-plan(s), {self.driven:.1f} m driven in total')
        if not self.legs:
            print('  NO COMPLETE LEGS, so nothing was measured. The vehicle')
            print('  needs to be given goals and to travel between them.')
            print('=' * 70)
            return

        print()
        print('    straight   plan   driven   planner   controller')
        for s, p, d in self.legs:
            print(f'    {s:7.1f}  {p:6.1f}  {d:6.1f}     '
                  f'{p / s:5.2f}      {d / p:5.2f}')
        planner = [p / s for s, p, _ in self.legs]
        control = [d / p for _, p, d in self.legs]
        print()
        print(f'  planner overhead    median {statistics.median(planner):.2f}   '
              f'worst {max(planner):.2f}')
        print(f'  controller overhead median {statistics.median(control):.2f}   '
              f'worst {max(control):.2f}')
        print()
        # Which one owns the problem. Said plainly, because the whole point is
        # to stop the next change being a guess.
        pm, cm = statistics.median(planner), statistics.median(control)
        if cm > 1.25:
            print('  THE CONTROLLER OWNS THE OVERHEAD. The vehicle is driving')
            print('  well beyond its own plan, which no obstacle explains. Look')
            print('  at the control loop rate before touching the planner.')
        elif pm > 1.25:
            print('  THE PLANNER OWNS THE OVERHEAD. The routes themselves are')
            print('  long. Inflation, the keepout mask and phantom obstacles in')
            print('  the map are the candidates, in that order.')
        else:
            print('  Neither term is dominant. Overhead here is re-routing')
            print('  around people, which is the system working rather than a')
            print('  fault, and the straight line was never available.')
        print('=' * 70)


def main():
    rclpy.init()
    node = PathProbe()
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
