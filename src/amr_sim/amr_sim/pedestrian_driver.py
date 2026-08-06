#!/usr/bin/env python3
"""Drive scenario pedestrians back and forth along straight legs.

Open loop by design. Each person's traverse time is computed from its leg length
and speed, and the direction flips on that timer. No pose feedback is used, so
two runs of the same scenario produce identical motion, which is what makes
detection and tracking numbers comparable between runs.

Velocities go out on /model/<name>/cmd_vel, which the bridge forwards to the
VelocityControl plugin carried by the person model.
"""

import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node

SCENARIOS = Path(__file__).resolve().parent.parent / 'scenarios'


class PedestrianDriver(Node):
    def __init__(self):
        super().__init__('pedestrian_driver')
        name = self.declare_parameter('scenario', 'walking_people').value
        path = self.declare_parameter('scenario_path', '').value
        source = Path(path) if path else (SCENARIOS / f'{name}.yaml')
        spec = yaml.safe_load(source.read_text())

        self.walkers = []
        for person in spec.get('people', []):
            leg = person.get('path')
            if not leg:
                continue          # a standing worker needs no driver
            self.walkers.append({
                'pub': self.create_publisher(Twist, f'/model/{person["name"]}/cmd_vel', 10),
                'speed': float(leg['speed']),
                'period': float(leg['length']) / float(leg['speed']),
                'name': person['name'],
            })

        self.t = 0.0
        self.dt = 0.1
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'driving {len(self.walkers)} pedestrian(s) from {source.name}')

    def _tick(self):
        self.t += self.dt
        for w in self.walkers:
            # Square wave: forward for one traverse, back for the next.
            phase = int(self.t // w['period']) % 2
            v = w['speed'] if phase == 0 else -w['speed']
            msg = Twist()
            msg.linear.x = v
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
