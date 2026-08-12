#!/usr/bin/env python3
"""Measure how often map to odom is published, and whether that changes.

WHY

Five consecutive transport cycles: the first two completed in 69 s and 66 s, the
last three failed within 6 to 10 seconds each. Every failure was the same fault,
a goal transform refused because the request was newer than the latest
`map -> odom`. The failures DEGRADE rather than occurring at random, so
something accumulates over a run.

The hypothesis in ADR 0010 is that slam_toolbox's publication slows as its pose
graph grows, widening the window in which a request for "now" outruns the newest
transform. It is a plausible story and this project has been burned repeatedly
by plausible stories, so it gets measured before either fix in that ADR is built.

WHAT DECIDES IT

The interval between consecutive `map -> odom` publications, reported per
window across the run. `transform_publish_period` is 0.02, so a healthy system
publishes every 20 ms.

    intervals grow across the run     the hypothesis holds. ADR 0010 option B,
                                      localising on a saved map, removes the
                                      mechanism; option A only bounds it.

    intervals stay flat               the hypothesis is WRONG. Neither option in
                                      ADR 0010 is treating the real cause, and
                                      the degradation is something else that
                                      accumulates. Do not build either.

The worst interval in a window matters more than the mean. One 300 ms gap at the
wrong moment fails a goal; a slightly higher average does not.
"""

import statistics
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


class MapOdomTracker(Node):
    def __init__(self):
        super().__init__('track_map_odom',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.duration = self.declare_parameter('duration_s', 600.0).value
        self.window = self.declare_parameter('window_s', 60.0).value

        self.last = None
        self.intervals = []          # (wall time since start, interval seconds)
        self.start = time.monotonic()
        # /tf, not a listener: the question is when the message ARRIVES, which a
        # buffer hides by design.
        self.create_subscription(TFMessage, '/tf', self._tf, 100)

    def _tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') != 'map':
                continue
            if t.child_frame_id.lstrip('/') != 'odom':
                continue
            now = time.monotonic()
            if self.last is not None:
                self.intervals.append((now - self.start, now - self.last))
            self.last = now

    def run(self):
        self.get_logger().info(
            f'measuring map to odom publication for {self.duration:.0f} s')
        end = time.monotonic() + self.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.report()
        return 0

    def report(self):
        print('\n' + '=' * 72)
        if len(self.intervals) < 20:
            print(f'  only {len(self.intervals)} publications seen; '
                  f'is slam_toolbox running?')
            print('=' * 72)
            return

        total = self.intervals[-1][0]
        print(f'  {len(self.intervals)} publications over {total:.0f} s')
        print(f'  {"window":>14}  {"n":>6}  {"mean":>8}  {"median":>8}  '
              f'{"p95":>8}  {"worst":>8}')

        worsts = []
        w = 0.0
        while w < total:
            chunk = [d for t, d in self.intervals if w <= t < w + self.window]
            if len(chunk) >= 5:
                chunk_sorted = sorted(chunk)
                p95 = chunk_sorted[int(len(chunk_sorted) * 0.95)]
                worsts.append(max(chunk))
                print(f'  {w:6.0f}-{w + self.window:6.0f}s  {len(chunk):6d}  '
                      f'{statistics.mean(chunk) * 1000:7.1f}ms  '
                      f'{statistics.median(chunk) * 1000:7.1f}ms  '
                      f'{p95 * 1000:7.1f}ms  {max(chunk) * 1000:7.1f}ms')
            w += self.window

        print()
        if len(worsts) >= 3:
            first, last = worsts[0], worsts[-1]
            half = len(worsts) // 2
            early = statistics.mean(worsts[:half])
            late = statistics.mean(worsts[half:])
            print(f'  worst interval, first window {first * 1000:.1f} ms, '
                  f'last window {last * 1000:.1f} ms')
            print(f'  mean of worst intervals: early {early * 1000:.1f} ms, '
                  f'late {late * 1000:.1f} ms')
            # A goal transform fails when the gap exceeds what the consumer will
            # wait for. Growth of that gap is the thing ADR 0010 turns on.
            if late > early * 1.5 and late > 0.1:
                print('\n  VERDICT: the interval GROWS across the run.')
                print('  The ADR 0010 hypothesis holds. Option B removes the '
                      'mechanism; option A bounds it.')
            elif late > early * 1.5:
                print('\n  VERDICT: the interval grows but stays small.')
                print('  Directionally consistent, but too small on its own to '
                      'explain a refused transform. Look for a second cause.')
            else:
                print('\n  VERDICT: the interval does NOT grow.')
                print('  The ADR 0010 hypothesis is WRONG. Neither option there '
                      'addresses the real cause, and building either would be '
                      'fixing something that is not broken.')
        print('=' * 72)


def main():
    rclpy.init()
    node = MapOdomTracker()
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # BOTH, because the harness stops this with SIGINT once the mission
        # finishes and rclpy turns that into ExternalShutdownException rather
        # than KeyboardInterrupt. Catching only the latter lost an entire run's
        # measurement to a traceback, which is a poor way to learn that a
        # diagnostic has to survive its own shutdown.
        node.report()
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
