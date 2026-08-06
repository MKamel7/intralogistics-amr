#!/usr/bin/env python3
"""Drive scenario pedestrians back and forth along straight legs.

CLOSED LOOP on the simulator's own pose feed, deliberately.

An earlier version was open loop: it computed a traverse time from leg length
over speed and flipped direction on that timer. It did not work, and the way it
failed is worth recording. The driver starts with the launch while the people
are spawned a moment later, so every walker begins mid-phase; the errors never
correct, and they accumulate. Measured, one walker commanded a 4.0 m leg
travelled 5.6 m, walked off its aisle, climbed onto the warehouse clutter and
ended up hovering 0.28 m above the floor with its feet in a pallet. Three of
them left their lanes entirely.

Reversing on measured displacement instead means a walker cannot drift out of
its lane no matter when it spawned or what it bumped into. Runs are still
repeatable, because the leg endpoints are fixed rather than the timing.
"""

import math
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

SCENARIOS = Path(__file__).resolve().parent.parent / 'scenarios'


class PedestrianDriver(Node):
    def __init__(self):
        super().__init__('pedestrian_driver')
        name = self.declare_parameter('scenario', 'walking_people').value
        path = self.declare_parameter('scenario_path', '').value
        source = Path(path) if path else (SCENARIOS / f'{name}.yaml')
        spec = yaml.safe_load(source.read_text())

        self.walkers = {}
        for person in spec.get('people', []):
            leg = person.get('path')
            if not leg:
                continue                      # a standing worker needs no driver
            self.walkers[person['name']] = {
                'pub': self.create_publisher(
                    Twist, f'/model/{person["name"]}/cmd_vel', 10),
                'speed': float(leg['speed']),
                'length': float(leg['length']),
                'origin': None,               # captured on the first pose seen
                'forward': True,
                'pose': None,
            }

        self.create_subscription(TFMessage, '/ground_truth/poses', self._poses, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'driving {len(self.walkers)} pedestrian(s) from {source.name}, '
            f'reversing on measured displacement')

    def _poses(self, msg):
        for tf in msg.transforms:
            w = self.walkers.get(tf.child_frame_id)
            if w is None:
                continue
            p = tf.transform.translation
            w['pose'] = (p.x, p.y)
            if w['origin'] is None:
                w['origin'] = (p.x, p.y)

    def _tick(self):
        for name, w in self.walkers.items():
            if w['pose'] is None or w['origin'] is None:
                continue                      # not spawned yet; command nothing
            travelled = math.hypot(w['pose'][0] - w['origin'][0],
                                   w['pose'][1] - w['origin'][1])
            if w['forward'] and travelled >= w['length']:
                w['forward'] = False
            elif not w['forward'] and travelled <= 0.15:
                w['forward'] = True

            msg = Twist()
            msg.linear.x = w['speed'] if w['forward'] else -w['speed']
            w['pub'].publish(msg)


def main():
    rclpy.init()
    node = PedestrianDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
