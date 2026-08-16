#!/usr/bin/env python3
"""Social navigation metrics, computed from ground truth.

WHY THESE AND NOT A FRAMEWORK

The social navigation literature, and the Arena benchmark suite in particular,
scores a robot on how it behaves around people rather than on whether it
arrived. Those metrics are the valuable part; adopting the framework that
publishes them is not, because it would mean replacing a pedestrian system that
works with an external dependency whose Harmonic support is unproven.

So the metrics are computed here, from `/ground_truth/poses`, which is a
measurement channel and never enters the control path.

WHAT IS MEASURED, AND WHY EACH ONE

    minimum clearance       the closest the vehicle body came to any person,
                            over the whole run. One number, and the one a
                            safety assessor asks for first.

    proxemic intrusion      time spent inside each of Hall's zones, measured
                            from the person, not the vehicle centre:
                              intimate  < 0.45 m   a person would flinch
                              personal  < 1.20 m   a person would step back
                              social    < 3.60 m   a person would notice
                            Reported as a fraction of the time that person was
                            within sensing range at all, because time spent
                            comfortably far away is not a virtue when the
                            vehicle was in another aisle.

    time to collision       at the closest approach, the separation divided by
                            the closing speed. Distance alone does not separate
                            a vehicle creeping past someone from one arriving
                            at 0.75 m/s, and those are different events.

    speed at closest        what the vehicle was doing at that moment. A pass
                            at 50 mm and 0.05 m/s is a docking manoeuvre; the
                            same 50 mm at full speed is a near miss.

WHY THE DENOMINATOR MATTERS MORE THAN THE COUNT

An earlier probe in this project reported precision and recall over people it
could not possibly see, twenty metres away behind racking, and the numbers were
meaningless in a way that looked ordinary. Every proxemic figure here is
divided by the time that person was within `attention_range` of the vehicle, so
a run that spends ten minutes in an empty aisle does not flatter itself.

    tools/measure_social.py --ros-args -p duration_s:=600.0
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

# Hall's proxemic zones, measured from the PERSON. These are anthropology
# rather than engineering and are labelled as such: they are not a datasheet
# and no safety figure derives from them.
ZONES = (('intimate', 0.45), ('personal', 1.20), ('social', 3.60))

# The person model's collision puck, from models/person/model.sdf.
PERSON_RADIUS = 0.22


def clearance_to_footprint(px, py, half_length, half_width):
    """Distance from a point to a centred axis aligned rectangle, negative
    inside. Shared shape with measure_contacts.py deliberately: two probes
    disagreeing about what "clearance" means would be worse than duplication.
    """
    dx = abs(px) - half_length
    dy = abs(py) - half_width
    if dx > 0.0 and dy > 0.0:
        return math.hypot(dx, dy)
    return max(dx, dy)


def time_to_collision(gap, closing_speed):
    """Seconds until contact at the current closing speed.

    Infinite when not closing, which is the honest answer rather than a large
    number that would drag a median around.
    """
    if closing_speed <= 1e-3:
        return float('inf')
    return max(0.0, gap) / closing_speed


class SocialProbe(Node):
    def __init__(self):
        super().__init__('social_probe',
                         # THE SIMULATED CLOCK. Without it `duration_s` counts
                         # wall seconds while everything measured happens in
                         # simulated ones, so the probe runs for the wrong
                         # length of time by the real time factor. Six probes
                         # here were missing it and one of them, the latency
                         # split, was subtracting a clock reading from a
                         # message stamp and reporting the epoch as a duration.
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 600.0).value
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        self.half_length = self.declare_parameter('half_length', 0.300).value
        self.half_width = self.declare_parameter('half_width', 0.2845).value
        # Beyond this a person is not part of the encounter and counting the
        # time would only dilute every figure.
        self.attention = self.declare_parameter('attention_range', 5.0).value

        self.prev = {}
        self.prev_t = None
        self.speed = 0.0
        self.seen = {}          # name -> seconds within attention range
        self.zone_time = {}     # name -> {zone: seconds}
        self.min_gap = {}       # name -> closest approach to the footprint
        self.at_min = {}        # name -> (speed, ttc) at that moment
        self.samples = 0

        self.create_subscription(TFMessage, '/ground_truth/poses',
                                 self._truth, TRUTH_QOS)
        self.create_subscription(TwistStamped,
                                 '/diff_drive_controller/cmd_vel',
                                 self._cmd, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self._tick)
        self.reported = False
        # SAY THAT IT STARTED. Without this the tool prints nothing at all
        # until it reports, and a probe silent for fourteen minutes is
        # indistinguishable from one that died on import. That ambiguity cost
        # a few minutes of reading a zero byte log as a crash.
        self.get_logger().info(
            f'social metrics for {self.duration:.0f} s, attention range '
            f'{self.attention:.1f} m, footprint '
            f'{2 * self.half_length:.3f} by {2 * self.half_width:.3f} m')

    def _cmd(self, msg):
        self.speed = abs(msg.twist.linear.x)

    def _truth(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
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
        dt = (now - self.prev_t) if self.prev_t is not None else 0.0
        # A long gap means the probe was starved, not that the vehicle spent
        # that time next to somebody. Discard rather than integrate.
        if dt < 0.0 or dt > 0.5:
            dt = 0.0
        self.samples += 1
        c, s = math.cos(-yaw), math.sin(-yaw)
        for name, (x, y) in poses.items():
            if name in (self.vehicle_frame, 'body'):
                continue
            centre = math.dist((x, y), v)
            if centre > self.attention:
                self.prev[name] = (x, y)
                continue
            self.seen[name] = self.seen.get(name, 0.0) + dt
            dx, dy = x - v[0], y - v[1]
            px, py = c * dx - s * dy, s * dx + c * dy
            gap = clearance_to_footprint(px, py,
                                         self.half_length, self.half_width) - PERSON_RADIUS

            z = self.zone_time.setdefault(name, {k: 0.0 for k, _ in ZONES})
            for label, radius in ZONES:
                if gap < radius:
                    z[label] += dt

            if name not in self.min_gap or gap < self.min_gap[name]:
                closing = 0.0
                if name in self.prev and dt > 0.0:
                    was = math.dist(self.prev[name], v)
                    closing = max(0.0, (was - centre) / dt)
                self.min_gap[name] = gap
                self.at_min[name] = (self.speed, time_to_collision(gap, closing))
            self.prev[name] = (x, y)
        self.prev_t = now

    def _tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit(0)

    def report(self):
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 74)
        print(f'  social navigation, {self.samples} sample(s), '
              f'attention range {self.attention:.1f} m')
        if not self.min_gap:
            print('  NOBODY CAME WITHIN RANGE, so nothing was measured. This is')
            print('  not a good social navigation result, it is an empty run.')
            print('=' * 74)
            return

        print()
        print('  person            near(s)  min gap   speed    TTC    '
              'intimate personal social')
        for name in sorted(self.min_gap):
            seen = self.seen.get(name, 0.0)
            z = self.zone_time.get(name, {})
            spd, ttc = self.at_min.get(name, (0.0, float('inf')))
            frac = lambda k: (100.0 * z.get(k, 0.0) / seen) if seen > 0 else 0.0  # noqa: E731
            ttc_s = '   inf' if math.isinf(ttc) else f'{ttc:6.1f}'
            print(f'  {name:16s} {seen:7.1f}  {self.min_gap[name]:+7.3f}  '
                  f'{spd:5.2f}  {ttc_s}    {frac("intimate"):6.1f}% '
                  f'{frac("personal"):7.1f}% {frac("social"):6.1f}%')

        gaps = list(self.min_gap.values())
        print()
        print(f'  closest anyone came        {min(gaps):+.3f} m')
        print(f'  median of per person minima {statistics.median(gaps):+.3f} m')
        intruded = [n for n, z in self.zone_time.items() if z.get('intimate', 0) > 0]
        if intruded:
            print(f'  entered intimate space of  {len(intruded)}: '
                  f'{", ".join(sorted(intruded))}')
        print()
        print('  Zone percentages are of the time that person was within the')
        print('  attention range, not of the run. A vehicle in another aisle')
        print('  is not being polite, it is being absent.')
        print('  Hall\'s zones are anthropology, not a datasheet. No safety')
        print('  figure in this project derives from them.')
        print('=' * 74)


def main():
    rclpy.init()
    node = SocialProbe()
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
