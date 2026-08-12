#!/usr/bin/env python3
"""Drive the building to build the map, without being told the layout.

THE PROBLEM THIS SOLVES

SLAM had mapped 88 m2, of which only 6.4 percent had a walker's clearance, and
it was never going to map more, because the robot was parked. A map is not
something a stationary vehicle produces. It is the record of where the vehicle
has been.

A hand-written waypoint tour would work and would be a lie: the tour would
encode the warehouse layout, so the "mapping" result would be a consequence of
me knowing the answer. This picks its goals from the ROBOT'S OWN MAP instead.

HOW IT PICKS

Each round it takes the map as it currently stands, finds every free cell
reachable from the robot by flood fill, and drives to the reachable cell FURTHEST
from where it is. Going to the far end of known space parks the scanner against
the frontier, so the next round's map is bigger, and the round after that reaches
further still. It is frontier exploration's cheaper cousin: not optimal in path
length, but it needs no frontier clustering and it cannot pick a goal the vehicle
cannot reach, which is the failure that makes naive frontier exploration thrash.

It stops when the map stops growing, which is the honest termination condition:
the building is surveyed when driving further stops teaching anything.

WHAT IT DOES NOT DO. It never reads the ground truth map. If it did, the
resulting SLAM map would be an answer copied from the back of the book.
"""

import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from tf2_ros import Buffer, TransformListener

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class SurveyRunner(Node):
    def __init__(self):
        super().__init__('survey_runner')
        # use_sim_time is declared by rclpy itself, so it is passed on the
        # command line rather than declared here.
        # CLEARANCE MUST EXCEED THE INSCRIBED RADIUS, and 0.45 m did not.
        #
        # The vehicle is 810 by 590 mm, so the circle that just contains it has
        # radius hypot(0.405, 0.295) = 0.501 m. Nav2's costmap marks every cell
        # within the inscribed radius of an obstacle as INSCRIBED_INFLATED, and
        # Smac treats that as lethal, because a cell that close cannot hold the
        # vehicle at any orientation.
        #
        # Goals were being chosen with 0.45 m of clearance, which is INSIDE that
        # radius, so the planner refused essentially every goal this tool
        # offered: "GridBased plugin failed to plan ... no valid path found",
        # over and over. With no path there was nothing to follow, so the
        # behaviour tree ran its recoveries, and the spinning that looked like a
        # controller tuning problem was the `spin` recovery doing its job. Three
        # rounds of controller and safety debugging were spent on a number that
        # was 51 mm too small.
        #
        # 0.70 m is the inscribed radius plus about 200 mm. It is deliberately
        # MORE than the planner demands, and that asymmetry is the design.
        #
        # The planner's own margin is a hard constraint and cannot express a
        # preference: raise it and legitimate floor becomes illegal, lower it
        # and routes get threaded through gaps with millimetres to spare. So
        # the preference for roomy routes lives here instead. This tool only
        # targets floor with 0.70 m of clearance and only walks its reachability
        # search through such floor, which leaves the planner 0.157 m of margin
        # on every journey it is asked to make while still allowing it to use
        # the whole building when it has to.
        #
        # On this warehouse that leaves 116.3 m2 of connected floor to survey.
        self.clearance = self.declare_parameter('clearance', 0.70).value
        self.max_rounds = self.declare_parameter('max_rounds', 24).value
        # Below this the round taught us nothing worth another trip.
        self.growth_stop = self.declare_parameter('growth_stop_m2', 2.0).value
        self.goal_timeout = self.declare_parameter('goal_timeout_s', 180.0).value

        self.map = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._map, qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _map(self, msg):
        self.map = msg

    def sync_clock(self, timeout=30.0):
        """Block until simulated time is actually running.

        With use_sim_time the node clock reads zero until the first /clock
        arrives. A deadline computed from that zero is already long past by the
        time the simulator's real value shows up, so every wait failed
        instantly: the first attempt reported a /map timeout after no wait at
        all, on a topic that was publishing perfectly well.
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > 0:
                return True
        self.get_logger().error('no clock; is the simulator publishing /clock?')
        return False

    def wait_for(self, predicate, timeout, label):
        # Timed on the WALL clock, deliberately. These are waits for something
        # to appear, and if the simulator stalls, a deadline measured in
        # simulated time stalls with it and the wait never returns.
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.get_logger().error(f'timed out waiting for {label}')
        return False

    def settle(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def free_area(self):
        m = self.map
        n = sum(1 for v in m.data if 0 <= v <= 30)
        return n * m.info.resolution ** 2

    def farthest_reachable(self, start, exclude=()):
        """Breadth-first over cells the VEHICLE FITS IN; the deepest one.

        Breadth-first because it is measuring travel distance through free
        space, not straight-line distance. A cell on the far side of a rack can
        be near in a straight line and a long way to drive, and driving to the
        latter is what actually extends the map.

        THE FILL TRAVERSES CLEAR CELLS, NOT MERELY FREE ONES, and that
        distinction was worth several rounds of debugging. The first version
        flooded through any free cell and only checked clearance at the goal.
        That let it route through a gap narrower than the vehicle and offer a
        goal on the far side, which the planner then refused, because Nav2
        needs a corridor of at least twice the inscribed radius, about 1.0 m
        here. The log said only "GridBased plugin failed to plan ... no valid
        path found", the behaviour tree ran its recoveries, and the resulting
        spinning looked convincingly like a controller tuning problem.

        Reachability here now means what it means to the planner.
        """
        m = self.map
        info = m.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        # Distance from every cell to the nearest obstacle or unknown cell, in
        # metres. A Euclidean distance transform does in one pass what testing a
        # disc of offsets around every cell would do in hundreds, which matters
        # because this runs on a 451 by 455 grid between every leg of the survey.
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        passable = (grid >= 0) & (grid <= 30)
        clearance_m = distance_transform_edt(passable) * res
        clear = clearance_m >= self.clearance

        si = int((start[0] - ox) / res)
        sj = int((start[1] - oy) / res)
        if not (0 <= si < w and 0 <= sj < h) or not clear[sj, si]:
            # The vehicle can be standing somewhere that does not meet the
            # survey's own clearance rule: at its parking spot, or right after a
            # loop closure shifts the map under it. Start the fill from the
            # nearest cell that does qualify rather than abandoning the round.
            qualifying = np.argwhere(clear)
            if len(qualifying) == 0:
                return None
            d2 = (qualifying[:, 0] - sj) ** 2 + (qualifying[:, 1] - si) ** 2
            sj, si = qualifying[int(np.argmin(d2))]
            si, sj = int(si), int(sj)

        seen = np.zeros((h, w), dtype=bool)
        seen[sj, si] = True
        q = deque([(si, sj, 0)])
        best = None
        best_d = -1
        while q:
            i, j, d = q.popleft()
            if d > best_d:
                wx = ox + (i + 0.5) * res
                wy = oy + (j + 0.5) * res
                # Skip goals the planner has already refused, and everything
                # within a metre of them, so a single unreachable spot cannot
                # be re-offered every round while the survey makes no progress.
                if not any((wx - fx) ** 2 + (wy - fy) ** 2 < 1.0
                           for fx, fy in exclude):
                    best, best_d = (i, j), d
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < w and 0 <= b < h and clear[b, a] and not seen[b, a]:
                    seen[b, a] = True
                    q.append((a, b, d + 1))
        if best is None:
            return None
        return (ox + (best[0] + 0.5) * res, oy + (best[1] + 0.5) * res,
                best_d * res)

    def drive_to(self, x, y, heading):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)

        send = self.nav.send_goal_async(goal)
        if not self.wait_for(lambda: send.done(), 10.0, 'goal acceptance'):
            return False
        handle = send.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected by the planner')
            return False
        result = handle.get_result_async()
        if not self.wait_for(lambda: result.done(), self.goal_timeout, 'arrival'):
            handle.cancel_goal_async()
            return False
        # CHECK THE STATUS, not just that a result arrived.
        #
        # An aborted goal returns a result immediately, and treating that as
        # arrival made a 5.9 m drive "succeed" in three seconds without the
        # wheels turning. The round then reported no map growth and the survey
        # concluded the building was fully mapped from 88 m2.
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'goal ended with status {STATUS_NAMES.get(status, status)}')
            return False
        return True

    def run(self):
        if not self.sync_clock():
            return 1
        if not self.wait_for(lambda: self.map is not None, 60.0, '/map'):
            return 1
        if not self.wait_for(lambda: self.nav.server_is_ready()
                             or self.nav.wait_for_server(timeout_sec=0.1),
                             60.0, 'navigate_to_pose'):
            return 1
        if not self.wait_for(lambda: self.robot_xy() is not None, 30.0,
                             'map to base_link'):
            return 1

        self.get_logger().info(
            f'survey starting, {self.free_area():.1f} m2 mapped so far')
        failed = []
        for rnd in range(1, self.max_rounds + 1):
            before = self.free_area()
            here = self.robot_xy()
            target = self.farthest_reachable(here, failed)
            if target is None:
                self.get_logger().info('no reachable goal with clearance, done')
                break
            x, y, dist = target
            heading = math.atan2(y - here[1], x - here[0])
            self.get_logger().info(
                f'round {rnd}: at ({here[0]:.2f}, {here[1]:.2f}), driving '
                f'{dist:.1f} m through free space to ({x:.2f}, {y:.2f}), '
                f'{before:.1f} m2 mapped')
            reached = self.drive_to(x, y, heading)
            if not reached:
                failed.append((x, y))

            # Let the scan settle at the far end before measuring. The map is
            # still being integrated for a moment after the wheels stop.
            self.settle(3.0)

            after = self.free_area()
            self.get_logger().info(
                f'round {rnd}: {"arrived" if reached else "did not arrive"}, '
                f'map {before:.1f} -> {after:.1f} m2 '
                f'({after - before:+.1f} m2)')
            if reached and after - before < self.growth_stop:
                self.get_logger().info(
                    f'map grew by less than {self.growth_stop} m2, '
                    f'the building is surveyed')
                break

        self.get_logger().info(f'survey finished, {self.free_area():.1f} m2 mapped')
        return 0


def main():
    rclpy.init()
    node = SurveyRunner()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
