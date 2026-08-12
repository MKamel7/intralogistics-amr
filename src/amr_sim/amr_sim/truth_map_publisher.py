#!/usr/bin/env python3
"""Publish the ground truth floorplan, latched, on /ground_truth/map.

TWO USES, AND ONE THING THIS IS NOT FOR

It is displayable, so the true layout can be put beside the SLAM map in RViz and
the difference looked at directly rather than guessed at.

It is scoreable, so `tools/score_map.py` can put a number on how good the SLAM
map is instead of leaving it to the eye. Judging maps by eye is how earlier
mapping problems survived as long as they did.

What it is NOT is an input to the robot. Nothing in the navigation stack
subscribes to this topic, and nothing should. The moment the robot plans against
ground truth, every mapping and localisation result in this repository stops
meaning anything. The topic sits under `/ground_truth/` with the pose oracle for
exactly that reason: that namespace is measurement, never control.
"""

import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_sim.occupancy import load_map


class TruthMapPublisher(Node):
    def __init__(self):
        super().__init__('truth_map_publisher')
        default = str(Path(get_package_share_directory('amr_sim'))
                      / 'maps' / 'warehouse_truth.yaml')
        path = self.declare_parameter('map_file', default).value
        frame = self.declare_parameter('frame_id', 'map').value

        grid = load_map(path)
        msg = OccupancyGrid()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = grid.res
        msg.info.width = grid.w
        msg.info.height = grid.h
        msg.info.origin.position.x = grid.ox
        msg.info.origin.position.y = grid.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = list(grid.data)

        # Latched, so a display or a scoring tool started at any time still gets
        # it. The map never changes, so there is nothing to publish repeatedly.
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/ground_truth/map', qos)
        self.pub.publish(msg)

        free = sum(1 for v in grid.data if 0 <= v <= 30)
        occ = sum(1 for v in grid.data if v > 30)
        self.get_logger().info(
            f'ground truth map latched on /ground_truth/map from '
            f'{Path(path).name}: {grid.w}x{grid.h} at {grid.res:.3f} m, '
            f'{free * grid.res ** 2:.1f} m2 floor, {occ * grid.res ** 2:.1f} m2 '
            f'obstacle. Evaluation only, never an input to navigation.')


def main():
    rclpy.init()
    node = TruthMapPublisher()
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
