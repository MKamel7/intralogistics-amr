#!/usr/bin/env python3
"""How accurately the vehicle parks at a station, and what limits it.

WHY THIS IS NOT THE SAME AS THE GOAL TOLERANCE

Nav2 stops when it believes it is within `xy_goal_tolerance` of the goal. Two
different errors sit between that belief and where the vehicle actually is:

    the controller stops somewhere inside the tolerance      0.20 m
    localisation is wrong about where that is                0.055 m parked

The first is a decision and can be tightened by editing a number. The second is
a property of the stack, and it is the floor: **a vehicle cannot park more
accurately than it can locate itself**, as long as the goal is expressed in the
map frame. Tightening the tolerance below the localisation error buys nothing
and costs goal-reached timeouts.

This measures the total, from the ground truth oracle, so the two can be
compared against each other rather than argued about.

WHAT WOULD BEAT IT

A dock the vehicle can SEE. Aligning to a feature in the scan makes the error a
sensor error rather than a localisation error, and the scan is good to a few
millimetres at these ranges. That is what precision docking means and it is not
what this project does today: the vehicle parks by navigation goal, and the
README says so.

This probe exists to put a number on the gap before anything is built, because
the alternative is arguing from arithmetic and V-46 is what that costs.

MEASUREMENT ONLY

`/ground_truth/poses` never reaches the control path. The station's own world
pose comes from the stations file the generator writes, which is the same file
the mission drives to, so the two cannot disagree about where the station is.
"""

import math
import statistics
import sys
from pathlib import Path

import rclpy
import yaml
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


def pose_error(vx, vy, vyaw, sx, sy, syaw):
    """Distance and heading error between a parked vehicle and a station.

    Pure, so the arithmetic can be checked without a simulator. The heading
    error is wrapped to (-pi, pi]: a vehicle 359 degrees off is 1 degree off,
    and reporting 359 would make every mean meaningless.
    """
    d = math.hypot(vx - sx, vy - sy)
    dyaw = math.atan2(math.sin(vyaw - syaw), math.cos(vyaw - syaw))
    return d, dyaw


class DockingProbe(Node):
    def __init__(self):
        super().__init__('docking_probe',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 2400.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        stations = self.declare_parameter('stations_file', '').value
        # Close enough to be AT the station rather than passing it, and the
        # vehicle must also be stopped. Both are needed: a vehicle driving
        # through the bay is within a metre of the station for a second or two.
        self.near = self.declare_parameter('near', 1.0).value
        self.stopped = self.declare_parameter('stopped_speed', 0.02).value
        self.settle = self.declare_parameter('settle_s', 2.0).value

        self.stations = {}
        if stations and Path(stations).is_file():
            spec = yaml.safe_load(Path(stations).read_text())
            for s in spec['stations']:
                self.stations[s['name']] = (s['world_xy'][0], s['world_xy'][1],
                                            float(s.get('yaw', 0.0)))
        if not self.stations:
            self.get_logger().warn(
                'no stations file, so there is nothing to measure against')

        self.last = None
        self.speed = 0.0
        # THE FINAL TURN, which is the quantity that discriminates.
        #
        # goods_in sits west and dispatch east on a route that alternates, so
        # the vehicle arrives at goods_in having driven WEST and must end
        # facing EAST: a 180 degree spot turn AT the goal. At dispatch it
        # arrives already facing the right way and turns nothing.
        #
        # If a spot turn is what costs the accuracy, the error sign should
        # follow the direction of that turn, because the goal checker stops the
        # rotation as soon as it is inside tolerance and therefore undershoots.
        # Recording the turn lets that be checked rather than argued.
        self.yaw_history = []       # (t, yaw), a few seconds of it
        self.since = None          # when the vehicle stopped near a station
        self.at = None
        self.arrivals = {}         # station -> [(distance, heading error)]

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False
        self.get_logger().info(
            f'measuring parked accuracy at {len(self.stations)} station(s)')

    def _truth(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        for tf in msg.transforms:
            if tf.child_frame_id != self.vehicle_frame:
                continue
            p = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if self.last is not None:
                dt = now - self.last[2]
                if dt > 1e-6:
                    self.speed = math.hypot(p.x - self.last[0],
                                            p.y - self.last[1]) / dt
            self.last = (p.x, p.y, now)
            self.yaw_history.append((now, yaw))
            self.yaw_history = [h for h in self.yaw_history if now - h[0] <= 12.0]

            near = None
            for name, (sx, sy, _syaw) in self.stations.items():
                if math.hypot(p.x - sx, p.y - sy) <= self.near:
                    near = name
                    break

            if near is None or self.speed > self.stopped:
                self.since = None
                self.at = None
                return
            if self.since is None:
                self.since = now
                self.at = near
                return
            # SETTLED, not merely stopped. A vehicle pausing mid manoeuvre is
            # stationary for a moment and is not parked, and counting it would
            # mix an intermediate pose into the arrival figures.
            if now - self.since < self.settle or self.at != near:
                return
            if self.arrivals.get(near) and self.arrivals[near][-1][2] == self.since:
                return                      # one record per arrival
            sx, sy, syaw = self.stations[near]
            d, dyaw = pose_error(p.x, p.y, yaw, sx, sy, syaw)
            # How far it turned in the ten seconds before settling.
            turn = 0.0
            if len(self.yaw_history) > 1:
                oldest = self.yaw_history[0][1]
                turn = math.atan2(math.sin(yaw - oldest),
                                  math.cos(yaw - oldest))
            self.arrivals.setdefault(near, []).append(
                (d, dyaw, self.since, turn))
            self.get_logger().info(
                f'parked at {near}: {d * 1000:.0f} mm, '
                f'{math.degrees(dyaw):+.1f} deg from the station pose, '
                f'after turning {math.degrees(turn):+.1f} deg')
            return

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 70)
        n = sum(len(v) for v in self.arrivals.values())
        print(f'  parked accuracy, {n} arrival(s)')
        if not n:
            print('  NO ARRIVALS. Either the vehicle never reached a station')
            print('  and settled, or no stations file was given. Those look')
            print('  the same from here.')
            print('=' * 70)
            return
        for name in sorted(self.arrivals):
            ds = [a[0] for a in self.arrivals[name]]
            ys = [abs(math.degrees(a[1])) for a in self.arrivals[name]]
            print(f'    {name:12s} n={len(ds):2d}  '
                  f'median {statistics.median(ds) * 1000:5.0f} mm  '
                  f'worst {max(ds) * 1000:5.0f} mm  '
                  f'heading worst {max(ys):4.1f} deg')
        # DOES THE ERROR SIGN FOLLOW THE TURN? If a spot turn undershoots, the
        # heading error opposes the direction of rotation on every sample where
        # a real turn happened. Reported rather than asserted, because one run
        # with six arrivals is a hypothesis test and not a proof.
        turned = [a for v in self.arrivals.values() for a in v
                  if len(a) > 3 and abs(math.degrees(a[3])) > 20.0]
        if turned:
            opposed = sum(1 for a in turned if a[1] * a[3] < 0)
            print()
            print(f'  of {len(turned)} arrival(s) that turned more than 20 deg '
                  f'before settling,')
            print(f'  {opposed} had a heading error OPPOSING the turn, which is '
                  f'the signature')
            print('  of a rotation stopped by a tolerance rather than completed.')
        allds = [a[0] for v in self.arrivals.values() for a in v]
        print(f'  median {statistics.median(allds) * 1000:.0f} mm, '
              f'worst {max(allds) * 1000:.0f} mm')
        print()
        print('  This is TRUE error, from the oracle, not what the vehicle')
        print('  believes. It contains the goal tolerance the controller was')
        print('  allowed AND the localisation error underneath it, and the')
        print('  second is the floor: a vehicle cannot park more accurately')
        print('  than it can locate itself while the goal is in the map frame.')
        print('=' * 70)


def main():
    rclpy.init()
    node = DockingProbe()
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
