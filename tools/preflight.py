#!/usr/bin/env python3
"""Check the running system is actually wired up, before trusting a result.

WHY THIS EXISTS

Four separate faults in this system presented identically: everything looked
healthy, every node was running, no error was logged anywhere, and the vehicle
did not move. Each one cost a round of rebuilding the wrong thing.

    /odom had no publisher, so the controller believed the vehicle was
    permanently stationary and its acceleration limit capped every command at
    0.015 m/s.

    Two /clock publishers, because an orphaned bridge from a previous launch
    survived, so simulated time jumped backwards and every TF lookup failed
    about a fifth of the time.

    The collision monitor was left INACTIVE by a lifecycle service timeout
    during a crowded start-up, so nothing forwarded commands to the wheels.

    The monitor received a velocity no polygon covered and responded by
    publishing nothing at all, rather than by stopping.

None of these is subtle once you look at the right thing. The cost was entirely
in not knowing which thing to look at. So this looks at all of them, in about
fifteen seconds, and prints a table. Run it before believing any measurement.

Exit code is 0 if everything passes, 1 otherwise, so it can gate a script.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# The command chain, in the order a velocity travels along it. Each entry is a
# topic that must have at least one publisher, because a broken link here is
# invisible: the upstream node keeps publishing happily into nothing.
CHAIN = [
    '/cmd_vel_nav',
    '/cmd_vel_raw',
    '/diff_drive_controller/cmd_vel',
]

LIFECYCLE = [
    '/slam_toolbox',
    '/collision_monitor',
    '/controller_server',
    '/planner_server',
    '/bt_navigator',
    '/behavior_server',
    '/velocity_smoother',
    # The costmap filter servers, which were missing from this list and are the
    # reason a run completed with no keepout zones while every check passed.
    # They sit on their own lifecycle manager precisely because they are the
    # ones that time out under load, so they are the ones most worth asserting.
    '/filter_mask_server',
    '/costmap_filter_info_server',
]

SENSORS = ['/scan', '/diff_drive_controller/odom', '/map']


class Preflight(Node):
    def __init__(self):
        super().__init__('preflight',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.results = []

    def check(self, label, ok, detail=''):
        self.results.append((label, bool(ok), detail))
        mark = 'PASS' if ok else 'FAIL'
        print(f'  [{mark}] {label:38s} {detail}')
        return ok

    def publishers(self, topic, patience=6.0):
        """Publisher count, RETRIED rather than sampled once.

        A single sample is a race. `/cmd_vel_nav` gets its publisher only when
        controller_server finishes activating, and a run with the cameras
        disabled shifted the timing enough that preflight sampled a moment too
        early, reported zero publishers, and failed a perfectly healthy stack.
        That is the third false alarm this script has produced by measuring
        something before it settled, after the discovery race and the TF warm-up.

        A diagnostic that cries wolf gets ignored, and an ignored diagnostic is
        worse than none, so anything that can legitimately arrive late is given
        time to arrive before being called missing.
        """
        deadline = time.monotonic() + patience
        n = self.count_publishers(topic)
        while n == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            n = self.count_publishers(topic)
        return n

    def settle(self, seconds=4.0):
        """Let discovery finish before counting anything.

        Publisher counts read zero for the first few seconds of a node's life
        regardless of the truth, because the graph has not propagated yet. The
        first version of this file reported "0 publishers" for /scan on the
        same run in which it measured /scan flowing at 14.4 Hz, which is a
        diagnostic that would send someone looking in the wrong place: exactly
        what this script exists to prevent.
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        print('\nPREFLIGHT\n')
        self.settle()

        # ONE CLOCK. Two publishers make simulated time jump backwards, which
        # clears every TF buffer several times a second and looks like a
        # navigation problem.
        n = self.publishers('/clock')
        self.check('single /clock publisher', n == 1, f'{n} publisher(s)')

        print()
        for topic in SENSORS:
            n = self.publishers(topic)
            self.check(f'{topic} has a publisher', n >= 1, f'{n} publisher(s)')

        print()
        for topic in CHAIN:
            n = self.publishers(topic)
            self.check(f'chain {topic}', n >= 1, f'{n} publisher(s)')

        print()
        for node in LIFECYCLE:
            state = self.lifecycle_state(node)
            self.check(f'{node} active', state == 'active', state)

        print()
        # THE KEEPOUT MASK, which nothing checked until it was found missing
        # for an entire five cycle run that this file had passed as healthy.
        #
        # The mask is the permanent no-go areas over the racking. When
        # filter_mask_server configures but does not activate, and that has
        # happened repeatedly under load, the mask is never published and both
        # costmaps fall back to having NO keepout zones. The vehicle then plans
        # straight through floor that was declared forbidden before it was
        # switched on.
        #
        # The only signal is `KeepoutFilter: Filter mask was not received`, at
        # WARN, from each costmap. Measured on one run: 441 of them, and this
        # script reported 17 of 17 checks passed while they were being printed.
        # A missing safety-relevant layer that announces itself only in a log
        # nobody greps is exactly what a preflight is for.
        n = self.publishers('/keepout_filter_mask')
        self.check('keepout mask is published', n >= 1, f'{n} publisher(s)')
        n = self.publishers('/costmap_filter_info')
        self.check('costmap filter info is published', n >= 1, f'{n} publisher(s)')

        print()
        ok, detail = self.tf_ok()
        self.check('map to base_link resolves', ok, detail)

        print()
        for topic in ('/scan', '/diff_drive_controller/odom'):
            rate, detail = self.rate(topic)
            self.check(f'{topic} is flowing', rate > 0.0, detail)

        print()
        # THE SAFETY NODE MUST BE PUBLISHING, NOT MERELY ACTIVE.
        #
        # `ros2 lifecycle get /collision_monitor` returned `active [3]` while
        # the node published neither its command output nor its state for 30
        # seconds, its scan source was healthy at 92 ms p95 against a 300 ms
        # timeout, and its input /cmd_vel_raw flowed at 16.7 Hz. The vehicle
        # sat still for eight minutes and the survey looped four rounds with
        # zero map growth, because the monitor sits in the command path and
        # nothing was reaching the wheels.
        #
        # Every other check in this file passed throughout. `active` is a
        # statement about a lifecycle transition that happened once, not about
        # whether the node is doing its job now. See V-41.
        # CONDITIONAL, and the condition matters. The collision monitor
        # processes ON an incoming command, so with no goal active both its
        # output and its state are legitimately silent and a bare "is flowing"
        # check would fail every healthy bringup. The fault is input flowing
        # while output does not.
        in_rate, in_detail = self.rate('/cmd_vel_raw')
        if in_rate > 0.0:
            out_rate, out_detail = self.rate('/diff_drive_controller/cmd_vel')
            self.check('collision monitor passes commands through',
                       out_rate > 0.0,
                       f'in {in_rate:.1f} Hz, out {out_detail}')
        else:
            self.check('collision monitor passes commands through', True,
                       'no commands in flight, nothing to pass (not a fault)')

        failed = [r for r in self.results if not r[1]]
        print(f'\n{len(self.results) - len(failed)} passed, {len(failed)} failed\n')
        if failed:
            print('FAILURES:')
            for label, _, detail in failed:
                print(f'  {label}: {detail}')
            print()
        return 1 if failed else 0

    def lifecycle_state(self, node):
        from lifecycle_msgs.srv import GetState
        cli = self.create_client(GetState, f'{node}/get_state')
        if not cli.wait_for_service(timeout_sec=3.0):
            return 'no service'
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return 'no response'
        return fut.result().current_state.label

    def tf_ok(self):
        import tf2_ros
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, self)
        # WARM UP BEFORE COUNTING. A listener starts with an empty buffer, so
        # the first second of lookups fails no matter how healthy the system
        # is. Counting those scored a perfectly good system at 11.7% failures
        # against a 10% threshold, which is a false alarm of exactly the kind
        # this script is meant to eliminate.
        warm = time.monotonic() + 2.0
        while time.monotonic() < warm:
            rclpy.spin_once(self, timeout_sec=0.02)

        ok = bad = 0
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                ok += 1
            except Exception:
                bad += 1
        total = ok + bad
        if total == 0:
            return False, 'no lookups attempted'
        fail_pct = bad / total * 100
        # A few percent is ordinary timing jitter. Twenty percent was the
        # signature of the duplicated clock.
        return fail_pct < 10.0, f'{fail_pct:.1f}% of lookups failed'

    def rate(self, topic):
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        types = dict(self.get_topic_names_and_types()).get(topic)
        if not types:
            return 0.0, 'topic not present'
        msg_type = get_message(types[0])
        count = [0]
        for qos in (qos_profile_sensor_data, 10):
            sub = self.create_subscription(
                msg_type, topic, lambda _m: count.__setitem__(0, count[0] + 1), qos)
            end = time.monotonic() + 2.5
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
            self.destroy_subscription(sub)
            if count[0]:
                break
        return count[0] / 2.5, f'{count[0] / 2.5:.1f} Hz'


def main():
    rclpy.init()
    node = Preflight()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
