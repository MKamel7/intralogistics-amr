#!/usr/bin/env python3
"""Measure the sensor-to-command latency that protective field sizing assumes.

WHY THIS IS THE MOST IMPORTANT UNMEASURED NUMBER IN THE PROJECT

Every protective field in this stack is sized by the ISO 13855 shape:

    S = v * (t_scanner + t_control + t_brake) + v^2 / (2a) + C

`t_scanner` is the SICK sheet's 70 ms response time and `t_brake` is an
estimate. `t_control` is `control_latency` in the platform spec, and on both
platforms it reads:

    control_latency: 0.10   # s, sensor to command; NOT YET MEASURED

An estimate feeding a protective field is the first thing a functional safety
assessor asks about, and "0.10 s, we think" is where they stop reading. At
1.0 m/s an error of 50 ms is 50 mm of stopping distance, which is the difference
between a field that covers the vehicle and one that does not.

WHAT IS ACTUALLY BEING MEASURED

The wall clock, or rather the simulated clock, between:

    the STAMP on the scan that first shows a protective field violation
    the arrival of the command that acts on it

That is the whole chain the number stands for: scan transport, the merge, the
collision monitor's own cycle, and publication. It does NOT include the
scanner's internal response time, which is `scanner_response_time` and is
already counted separately, and it does not include brake actuation, which is
`brake_actuation_delay`. Double counting any of them would inflate every field
in the stack.

HOW A SAMPLE IS TAKEN

Passively, from an ordinary run. The vehicle drives, a person or a rack enters
the protective field, the monitor reports `stop`, and the command going to the
wheels drops to zero. Each such event is one sample. No obstacles are injected
and nothing is commanded, so this can be attached to any mission run without
changing what it measures.

    tools/run_stack.sh --cameras off --run mission --cycles 5 &
    tools/measure_control_latency.py --ros-args -p duration_s:=400.0

WHY THE DISTRIBUTION AND NOT THE MEAN

A protective field sized on a mean latency is under-sized half the time. The
figure that belongs in the spec is a high percentile, and the spread is the
interesting part: a tight distribution means a predictable system, a long tail
means the number is a scheduling artefact and the fix is in the stack rather
than in the spec.

This prints p50, p95 and the maximum, and says plainly that the p95 is the
candidate rather than writing it anywhere itself. A tool that edited the
platform spec from its own measurement would be one bad run away from shrinking
every protective field in the project.

WHERE THE TAIL COMES FROM, WHICH IS THE POINT OF THE SPLIT

V-44 measured p50 68 ms against p99 1260 ms and called it a control path that
is occasionally starved rather than one that is slow. That reading was a
hypothesis and it was recorded as one. Two candidates were named: the executor
contention that produced 380 ms MPPI iterations in V-37, and the scan merger
lag that produced transient source rejections in V-41. Nothing distinguished
them, and no protective field can honestly be sized on an unattributed tail.

So each sample is also split at the collision monitor, which is the one point
in the chain that announces itself:

    sensor half    this node sees the scan  -> this node sees the STOP
    control half   this node sees the STOP  -> this node sees the command

MEASURED AT THIS NODE, ON RECEPTION, AND THAT IS DELIBERATE. The split does
not sum to the total above and is not meant to. `CollisionMonitorState` carries
no header, so the only time available for the decision is when this node
received it. Anchoring one half on that and the other on generation stamps
produced a sensor half larger than the total on nearly every sample and a
control half clamped to zero. Measuring all three on reception makes the
probe's own subscription delay common to them, which is what a split needs and
a total does not.

A tail in the SENSOR half is the scan pipeline: transport, the merge, the
monitor's own cycle. A tail in the CONTROL half is downstream of a decision
that had already been made, which is the executor. They are different faults
with different fixes and the total cannot tell them apart.

Two more quantities go with each sample, because a stalled stream is the
signature the halves are being read for:

    scan gap    the longest interval between scans in the second before
    cmd gap     the longest interval between commands in the second before

The monitor state message carries no header, so the moment it "arrived" is the
receive time on the simulated clock. That makes the split slightly pessimistic
about the sensor half and slightly optimistic about the control half, by one
executor wakeup. It is stated here rather than corrected for, because the
effect is single-digit milliseconds and the tail being attributed is 1260.
"""

import statistics
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan

# The simulated clock advances one physics step at a time, and every world
# here sets `max_step_size` to 0.004. Nothing measured against that clock can
# resolve finer, so an interval of 0.0 means "inside one step", not "zero".
# Reporting it as zero would be a precision this probe does not have, and the
# control half of every sample in the first good run came out at exactly 0.0.
SIM_STEP = 0.004

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class LatencyProbe(Node):
    def __init__(self):
        super().__init__('control_latency_probe',
                         # THE SIMULATED CLOCK. Without it get_clock() returns
                         # epoch seconds while every stamp read here is sim
                         # time, and the difference between them is not a
                         # duration. See V-52.
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 300.0).value
        # A command below this is a stop rather than a slow-down. The drive's
        # own deadband, matching min_x_velocity_threshold in the Nav2 config.
        self.stop_eps = self.declare_parameter('stop_epsilon', 0.02).value

        self.last_scan = None        # stamp of the most recent scan
        self.last_scan_rx = None     # ...and when this node received it
        self.pending = None          # stamp of the scan that triggered a stop
        self.pending_rx = None       # ...and when this node received that one
        self.decided = None          # when the monitor announced that stop
        self.polygon = ''            # which field fired, for context
        self.stopping = False
        self.was_moving = False
        self.states_seen = 0
        self.samples = []            # total, kept for continuity with V-44
        self.parts = []              # (total, sensor, control, scan_gap, cmd_gap)
        self.rejected = 0

        # Arrival times, for the stall signature. One second of history is
        # enough: the tail being attributed is on the order of a second, so a
        # stall that explains a sample is inside this window by construction.
        self.scan_arrivals = []
        self.cmd_arrivals = []

        self.create_subscription(LaserScan, '/scan', self._scan, SENSOR_QOS)
        # nav2_msgs/CollisionMonitorState, NOT std_msgs/String. The first
        # version of this subscribed with the wrong type and therefore received
        # nothing at all, so it reported no samples across thirty four
        # protective stops. The message was honest and the cause was mine.
        self.create_subscription(CollisionMonitorState,
                                 '/collision_monitor_state', self._state, 20)
        self.create_subscription(TwistStamped,
                                 '/diff_drive_controller/cmd_vel',
                                 self._cmd, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'measuring sensor to command latency for {self.duration:.0f} s. '
            f'Drive the vehicle somewhere people are.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _widest_gap(arrivals, now):
        """The longest interval between arrivals in the last second.

        Zero when there are fewer than two, which reads as "no stall seen"
        rather than as a stall of unknown size. A stream that stopped entirely
        shows up as the gap to `now`, which is the case worth catching and the
        one a pairwise diff over history alone would miss.
        """
        recent = [a for a in arrivals if now - a <= 1.0]
        if not recent:
            return 0.0
        gaps = [b - a for a, b in zip(recent, recent[1:])]
        gaps.append(now - recent[-1])
        return max(gaps)

    def _scan(self, msg):
        self.last_scan = stamp_s(msg.header)
        now = self._now()
        self.last_scan_rx = now
        self.scan_arrivals.append(now)
        self.scan_arrivals = [a for a in self.scan_arrivals if now - a <= 2.0]

    def _state(self, msg):
        """The monitor announcing a protective stop starts the clock.

        The scan that caused it is the most recent one, which is the closest
        attribution available without reaching inside the monitor. It is an
        upper bound on the scan-to-decision half and is honest about that.

        The RISING EDGE only. While the field stays violated the monitor keeps
        publishing STOP, and treating every message as a fresh event would time
        the wrong thing.
        """
        self.states_seen += 1
        stopping = msg.action_type == CollisionMonitorState.STOP
        if stopping and not self.stopping and self.last_scan is not None:
            self.pending = self.last_scan
            self.pending_rx = self.last_scan_rx
            self.decided = self._now()
            self.polygon = msg.polygon_name
        self.stopping = stopping

    def _cmd(self, msg):
        now = self._now()
        self.cmd_arrivals.append(now)
        self.cmd_arrivals = [a for a in self.cmd_arrivals if now - a <= 2.0]

        moving = abs(msg.twist.linear.x) > self.stop_eps
        # The falling edge only. A vehicle that was already stopped tells us
        # nothing about how fast the stack reacts.
        if self.was_moving and not moving and self.pending is not None:
            dt = stamp_s(msg.header) - self.pending
            if 0.0 < dt < 2.0:
                self.samples.append(dt)
                # The split. `decided` is on the same clock as `now`, while
                # `pending` is a message STAMP; both are the simulated clock,
                # so the subtraction is meaningful.
                # BOTH HALVES IN RECEPTION TIME, and that is not an
                # approximation of the total above, it is a different
                # quantity measured on purpose.
                #
                # `dt` is generation stamp to generation stamp: the scan's
                # own stamp to the command's own stamp. That is the number
                # a protective field is sized by and it stays as it is.
                #
                # The split cannot use it. CollisionMonitorState carries no
                # header, so the only time available for the decision is
                # when THIS node received it, which includes the probe's own
                # subscription delay. Mixing that with generation stamps
                # produced a sensor half LARGER than the total on nearly
                # every sample, and a control half clamped to zero, which is
                # how the mistake announced itself.
                #
                # Measured at the same node, on the same clock, the probe's
                # delay is common to all three and the split is honest about
                # what it is: where the time went between this node seeing
                # the scan and this node seeing the command that acted on it.
                sensor = max(0.0, self.decided - self.pending_rx)
                control = max(0.0, now - self.decided)
                self.parts.append((
                    dt, sensor, control,
                    self._widest_gap(self.scan_arrivals, now),
                    self._widest_gap(self.cmd_arrivals, now)))
                self.get_logger().info(
                    f'  sample {len(self.samples)}: {dt * 1000:.1f} ms; '
                    f'split {(sensor + control) * 1000:.1f} = '
                    f'{sensor * 1000:.1f} sensor + {control * 1000:.1f} '
                    f'control ({self.polygon or "field"})')
            else:
                # Out of range means the pairing is wrong, not that the system
                # is slow. Counted, never averaged in.
                self.rejected += 1
            self.pending = None
        self.was_moving = moving

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        n = len(self.samples)
        print('\n' + '=' * 70)
        print(f'  sensor to command latency, {n} sample(s), '
              f'{self.rejected} rejected as unpairable')
        if n == 0:
            print(f'  NO SAMPLES. {self.states_seen} monitor state message(s) '
                  f'were seen.')
            if self.states_seen == 0:
                print('  ZERO STATE MESSAGES, so the probe heard nothing at')
                print('  all. Check the monitor is active and that this is')
                print('  subscribed with the right message type; getting that')
                print('  wrong is silent and looks exactly like a quiet run.')
            else:
                print('  The monitor was heard but the vehicle never stopped')
                print('  for it while this was attached.')
            print('=' * 70)
            return
        s = sorted(self.samples)
        p50 = statistics.median(s)
        p95 = s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))]
        print(f'  p50   {p50 * 1000:7.1f} ms')
        print(f'  p95   {p95 * 1000:7.1f} ms')
        print(f'  max   {max(s) * 1000:7.1f} ms')
        print(f'  min   {min(s) * 1000:7.1f} ms')
        if n > 1:
            print(f'  sd    {statistics.stdev(s) * 1000:7.1f} ms')
        print()
        print('  control_latency currently reads 0.10 s in the platform spec.')
        print(f'  The p95 of {p95:.3f} s is the CANDIDATE, not a decision, and')
        print('  nothing here writes it. A mean would under-size the field')
        print('  half the time; a long tail means the fix is in the stack.')
        if n < 20:
            print(f'  {n} samples is thin. Prefer 20 or more before changing a spec.')
        self._attribute(s, p50)
        print('=' * 70)

    def _attribute(self, s, p50):
        """Say where the tail is, or say that this run did not produce one.

        The whole reason for the split. A total says the stack was slow; the
        halves say which half, and that is the difference between a finding and
        an observation.
        """
        if not self.parts:
            return
        # A tail sample is one at least four times the median. Arbitrary, and
        # deliberately stated: what matters is that the threshold is fixed
        # before the numbers are read, not that it is principled.
        tail = [row for row in self.parts if row[0] >= 4.0 * p50]
        print()
        print(f'  ATTRIBUTION, over {len(self.parts)} split sample(s)')
        sensor = [r[1] for r in self.parts]
        control = [r[2] for r in self.parts]
        print('    measured at this node on reception, so these do NOT sum to')
        print('    the totals above, which are generation stamp to generation')
        print('    stamp. The split is for attribution, the total is the figure.')
        below = sum(1 for c in control if c < SIM_STEP)
        if below:
            print(f'    {below} of {len(control)} control halves are under one '
                  f'{SIM_STEP * 1000:.0f} ms physics step, which is the')
            print('    resolution of the simulated clock. Under, not zero: the')
            print('    clock cannot say which.')
        print(f'    sensor half   p50 {statistics.median(sensor) * 1000:7.1f} ms   '
              f'max {max(sensor) * 1000:7.1f} ms')
        print(f'    control half  p50 {statistics.median(control) * 1000:7.1f} ms   '
              f'max {max(control) * 1000:7.1f} ms')
        if not tail:
            print(f'    NO TAIL IN THIS RUN. No sample reached 4x the p50 of '
                  f'{p50 * 1000:.0f} ms, so')
            print('    there is nothing here to attribute. That is a result')
            print('    about this run, not evidence the tail is gone.')
            return
        n_sensor = sum(1 for r in tail if r[1] > r[2])
        print(f'    {len(tail)} sample(s) above 4x the p50 of {p50 * 1000:.0f} ms:')
        print(f'      {n_sensor} dominated by the SENSOR half '
              f'(scan transport, merge, monitor cycle)')
        print(f'      {len(tail) - n_sensor} dominated by the CONTROL half '
              f'(downstream of the decision)')
        worst = max(tail, key=lambda r: r[0])
        print(f'    worst: {worst[0] * 1000:.0f} ms = {worst[1] * 1000:.0f} sensor '
              f'+ {worst[2] * 1000:.0f} control, with a scan gap of '
              f'{worst[3] * 1000:.0f} ms')
        print(f'           and a command gap of {worst[4] * 1000:.0f} ms '
              f'in the second before it')
        stalled_scan = sum(1 for r in tail if r[3] > 0.2)
        stalled_cmd = sum(1 for r in tail if r[4] > 0.2)
        print(f'    stream stalls over 200 ms alongside a tail sample: '
              f'{stalled_scan} scan, {stalled_cmd} command')


def main():
    rclpy.init()
    node = LatencyProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when the context is
        # shut down from outside, which is what stop_all.sh does. Without it
        # here the probe dies with a traceback and the samples it spent the
        # whole run collecting are never printed. Measured: a full run's data
        # lost on teardown.
        pass
    finally:
        # ALWAYS report. Reporting only when there is something to say meant a
        # run that heard nothing printed nothing, which is indistinguishable
        # from the probe not having run.
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
