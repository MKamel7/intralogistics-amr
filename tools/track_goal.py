#!/usr/bin/env python3
"""Log how the distance to the goal evolves, to see WHY a leg fails to arrive.

WHY THIS EXISTS

A transport leg drove 12.1 m on an 8.1 m journey over 240 seconds, with zero
protective stops and no planner errors, and did not arrive. "Did not arrive" is
not a diagnosis. Three quite different behaviours produce it and they are
distinguishable only by watching the distance over time:

    it plateaus far out
        the vehicle is being held somewhere, or the path leads somewhere other
        than the goal. Look at the plan.

    it oscillates around a small value
        the vehicle is circling the goal without satisfying the goal checker.
        Look at the goal tolerances and the goal critics.

    it decreases and then jumps back up
        the goal is being re-sent, or the plan is being recomputed to a
        different place. Look at the behaviour tree.

WHAT IS TRACKED

The straight-line distance from the vehicle to the current goal, the length of
the current global plan, and the commanded speed. Distance comes from TF, which
is what the navigation stack itself believes, rather than from the pose oracle:
the question here is why NAVIGATION is not converging, so navigation's own view
of the world is the right one to inspect.
"""

import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter


class GoalTracker(Node):
    def __init__(self):
        super().__init__('track_goal',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 400.0).value
        self.period = self.declare_parameter('sample_period_s', 2.0).value

        self.goal = None
        self.plan_len = 0.0
        self.plan_end = None
        self.speed = 0.0
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self._cmd, 20)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.samples = []

    def _cmd(self, msg):
        self.speed = msg.twist.linear.x

    def _goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def _plan(self, msg):
        if not msg.poses:
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.plan_len = sum(math.dist(pts[i], pts[i + 1])
                            for i in range(len(pts) - 1))
        self.plan_end = pts[-1]
        # THE CURRENT PLAN'S ENDPOINT IS THE REFERENCE, refreshed every plan.
        #
        # The first version latched onto whatever `/goal_pose` carried, falling
        # back to the first plan's endpoint. The mission drives through the
        # NavigateToPose ACTION, so `/goal_pose` is never published at all, and
        # the fallback then held the FIRST leg's endpoint for the whole run.
        # Every later sample was measured against a stale reference, and the
        # tool duly reported that the plan ended 8.17 m from "the goal" when in
        # fact it was simply on a different leg. A diagnostic that lies is worse
        # than none, and this one lied on its first outing.
        #
        # Tracking the live plan endpoint measures the thing that matters: is
        # the vehicle closing on where navigation is currently taking it.
        self.goal = pts[-1]

    def robot(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def run(self):
        warm = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < warm:
            rclpy.spin_once(self, timeout_sec=0.05)

        end = time.monotonic() + self.duration
        last = 0.0
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - last < self.period:
                continue
            last = now
            r = self.robot()
            if r is None or self.goal is None:
                continue
            d = math.dist(r, self.goal)
            gap = (math.dist(self.plan_end, self.goal)
                   if self.plan_end else float('nan'))
            self.samples.append((now - start, d, self.plan_len, gap,
                                 self.speed, r))
            print(f'  t={now - start:6.1f}s  to goal {d:6.2f} m  '
                  f'plan {self.plan_len:6.2f} m  plan-end to goal {gap:5.2f} m  '
                  f'cmd {self.speed:+.3f} m/s  at ({r[0]:+.2f},{r[1]:+.2f})',
                  flush=True)
        self.report()
        return 0

    def report(self):
        print('\n' + '=' * 70)
        if len(self.samples) < 4:
            print('  too few samples to say anything')
            print('=' * 70)
            return
        d = [s[1] for s in self.samples]
        print(f'  {len(self.samples)} samples over {self.samples[-1][0]:.0f} s')
        print(f'  distance to goal: start {d[0]:.2f} m, end {d[-1]:.2f} m, '
              f'min {min(d):.2f} m, max {max(d):.2f} m')

        # Did it ever get close, and then leave again?
        i_min = d.index(min(d))
        after = d[i_min:]
        if min(d) < 1.0 and after[-1] > min(d) + 0.5:
            print(f'  CAME WITHIN {min(d):.2f} m at t={self.samples[i_min][0]:.0f}s '
                  f'and then moved away to {after[-1]:.2f} m')
            print('    the vehicle reached the goal and did not stop there, so '
                  'the goal checker is not being satisfied')
        elif max(d[-5:]) - min(d[-5:]) < 0.3 and d[-1] > 1.0:
            print(f'  PLATEAUED at about {d[-1]:.2f} m')
            print('    the vehicle stopped approaching well short of the goal')
        elif min(d) < 1.0:
            print(f'  approached to {min(d):.2f} m')
        else:
            print('  never got close')

        gaps = [s[3] for s in self.samples if not math.isnan(s[3])]
        if gaps and max(gaps) > 0.5:
            print(f'  the PLAN does not end at the goal: plan endpoint is up to '
                  f'{max(gaps):.2f} m away from it')
            print('    the planner is routing somewhere other than where it was '
                  'asked to go, which no amount of controller tuning will fix')
        speeds = [abs(s[4]) for s in self.samples]
        moving = sum(1 for v in speeds if v > 0.02)
        print(f'  commanding motion in {moving}/{len(speeds)} samples, '
              f'peak {max(speeds):.2f} m/s')
        print('=' * 70)


def main():
    rclpy.init()
    node = GoalTracker()
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.report()
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
