#!/usr/bin/env python3
"""How far the vehicle actually travels after it is told to stop.

WHY THIS IS THE NUMBER THE SAFETY CONCEPT RESTS ON

Every protective field in this stack is sized by ISO 13855:

    S = v (t_scanner + t_control + t_brake) + v^2 / (2a) + C

Two of those terms are braking. `t_brake` is an estimate and `a` comes from
`max_linear_accel` in the platform spec, which for the MP-400 is a single
published rating of 2.4 m/s2 that the manual does NOT distinguish by load. The
MiR250 sheet does distinguish, at 0.3 m/s2 with maximum payload.

So on this platform the fields are sized identically whether the vehicle is
empty or carrying its rated 100 kg. That is an assumption, it has never been
measured, and this measures it.

HOW A SAMPLE IS TAKEN

Passively, from a run that was going to happen anyway. The collision monitor
issues on the order of 120 protective stops per cycle, and every one of them is
a braking event: the commanded speed drops to zero while the vehicle is moving.
The probe records where the vehicle was when the command went to zero, and how
much further it travelled before it actually stopped.

Distance comes from `/ground_truth/poses`, which is a MEASUREMENT channel and
never reaches the control path. Wheel odometry would be the wrong source here
by construction: a braking wheel is the one most likely to be slipping, and
V-33 exists because odometry disagreed with the world by 2.5 percent.

THE PAIRING GUARD, LEARNED THE EXPENSIVE WAY

V-56 was a latency tail that turned out to be a probe arming a sample while the
vehicle was stationary, so nothing closed it until an unrelated later event and
the interval spanned both. The same shape of error is available here, so:

  * a sample is armed only when the vehicle is genuinely moving, above
    `min_speed`, because a stop command to a stationary vehicle brakes nothing
  * it is closed when the vehicle's own measured speed reaches zero
  * it EXPIRES if that takes longer than `max_brake_s`, and the expiries are
    counted and printed rather than dropped, because a probe that quietly
    discards its awkward samples is how a clean number gets believed
  * a sample is discarded if the command goes non-zero again before the vehicle
    stops, since the vehicle was told to drive on and never finished braking

WHAT IT DOES NOT MEASURE

Emergency braking. This is the service stop the collision monitor commands,
which is what the vehicle does in practice; a hardware emergency stop would be
harder and is not modelled. The figure here is therefore the OPTIMISTIC one for
a safety argument, and saying so is the point of the note.
"""

import math
import statistics
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
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


def iso_13855_distance(v, accel):
    """The `v^2 / 2a` term the protective fields are sized with.

    Pure, so the arithmetic every field in this project depends on is
    checkable without a simulator.
    """
    if accel <= 0.0 or v <= 0.0:
        return 0.0
    return (v * v) / (2.0 * accel)


class BrakingProbe(Node):
    def __init__(self):
        super().__init__('braking_probe',
                         # The simulated clock. See V-52: six probes here were
                         # reading wall time in a world that runs on sim time.
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 2400.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        # Below this the vehicle is not meaningfully moving and a stop command
        # brakes nothing. Matches the drive deadband used elsewhere.
        self.min_speed = self.declare_parameter('min_speed', 0.10).value
        self.stopped_speed = self.declare_parameter('stopped_speed', 0.02).value
        self.max_brake_s = self.declare_parameter('max_brake_s', 3.0).value

        self.pos = None
        self.speed = 0.0
        self.last = None            # (x, y, t) for measured speed
        self.commanded = 0.0
        self.armed = None           # (x, y, t, speed at command)
        self.samples = []           # (distance, time, speed at command)
        self.expired = 0
        self.armed_while_slow = 0
        self.resumed = 0

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.create_subscription(TwistStamped,
                                 '/diff_drive_controller/cmd_vel',
                                 self._cmd, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False
        self.get_logger().info(
            f'measuring braking distance for {self.duration:.0f} s, '
            f'arming above {self.min_speed:.2f} m/s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _truth(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id != self.vehicle_frame:
                continue
            p = tf.transform.translation
            now = self._now()
            if self.last is not None:
                dt = now - self.last[2]
                if dt > 1e-6:
                    self.speed = math.hypot(p.x - self.last[0],
                                            p.y - self.last[1]) / dt
            self.last = (p.x, p.y, now)
            self.pos = (p.x, p.y)

            if self.armed is not None:
                ax, ay, at, av = self.armed
                if self.speed <= self.stopped_speed:
                    dist = math.hypot(p.x - ax, p.y - ay)
                    self.samples.append((dist, now - at, av))
                    self.get_logger().info(
                        f'  stop {len(self.samples)}: {dist * 1000:.0f} mm in '
                        f'{now - at:.2f} s from {av:.2f} m/s')
                    self.armed = None
                elif now - at > self.max_brake_s:
                    # Not a slow stop, a sample nothing closed. Counted.
                    self.expired += 1
                    self.armed = None
            return

    def _cmd(self, msg):
        previous, self.commanded = self.commanded, abs(msg.twist.linear.x)
        if self.commanded <= self.stopped_speed < previous:
            if self.speed < self.min_speed or self.pos is None:
                self.armed_while_slow += 1
                return
            self.armed = (self.pos[0], self.pos[1], self._now(), self.speed)
        elif self.commanded > self.stopped_speed and self.armed is not None:
            # Told to drive on before it finished stopping. Not a braking
            # event, and counting it would understate the distance.
            self.resumed += 1
            self.armed = None

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        n = len(self.samples)
        print('\n' + '=' * 70)
        print(f'  braking distance, {n} sample(s)')
        print(f'  {self.armed_while_slow} stop command(s) ignored below '
              f'{self.min_speed:.2f} m/s, {self.resumed} abandoned when the')
        print(f'  vehicle was told to drive on, {self.expired} expired after '
              f'{self.max_brake_s:.1f} s.')
        if not n:
            print('  NO SAMPLES. Either the vehicle never braked from above')
            print(f'  {self.min_speed:.2f} m/s, or no ground truth arrived.')
            print('=' * 70)
            return
        d = sorted(s[0] for s in self.samples)
        print(f'  p50   {statistics.median(d) * 1000:7.0f} mm')
        print(f'  p95   {d[min(n - 1, int(round(0.95 * (n - 1))))] * 1000:7.0f} mm')
        print(f'  max   {max(d) * 1000:7.0f} mm')
        fastest = max(self.samples, key=lambda s: s[2])
        print(f'  from the highest speed seen, {fastest[2]:.2f} m/s: '
              f'{fastest[0] * 1000:.0f} mm')
        print()
        print('  This is the SERVICE stop the collision monitor commands, not')
        print('  an emergency stop, so it is the optimistic figure for a')
        print('  safety argument. Compare it against v^2/2a before believing')
        print('  any field is generously sized.')
        print('=' * 70)


def main():
    rclpy.init()
    node = BrakingProbe()
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
