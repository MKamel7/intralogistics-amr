#!/usr/bin/env python3
"""Republish the simulator's pose feed as a NAMED ROS transform stream.

This is the LABEL ORACLE. It exists for evaluation only and must never be
consumed by anything in the control path. docs/validation.md explains why that
distinction is the difference between measuring a system and cheating.

Why not ros_gz_bridge. The bridge does convert gz.msgs.Pose_V to
tf2_msgs/TFMessage, but it leaves frame_id and child_frame_id EMPTY, so every
pose arrives anonymous and there is no way to tell which pedestrian is which.
Measured rather than assumed: the raw gz feed carries a name on every pose, the
bridged message does not. Reading the JSON form of the same topic keeps the
names, at the cost of one subprocess.
"""

import json
import subprocess
import sys
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# Link poses share the feed with model poses. Links are what the robot is made
# of; only whole models are subjects of evaluation.
LINK_SUFFIXES = ('_link', '_wheel', '_swivel', '_frame', '_deck')


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        world = self.declare_parameter('world', 'warehouse').value
        self.topic = f'/world/{world}/dynamic_pose/info'
        self.pub = self.create_publisher(TFMessage, 'ground_truth/poses', 10)
        self._stop = threading.Event()
        self._proc = None
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()
        self.get_logger().info(f'ground truth from {self.topic}')

    def _read(self):
        self._proc = subprocess.Popen(
            ['gz', 'topic', '-e', '--json-output', '-t', self.topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        for line in self._proc.stdout:
            if self._stop.is_set():
                break
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                data = json.loads(line)
            except ValueError:
                continue

            msg = TFMessage()
            for pose in data.get('pose', []):
                name = pose.get('name', '')
                if not name or name.endswith(LINK_SUFFIXES):
                    continue
                tf = TransformStamped()
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.header.frame_id = 'world'
                tf.child_frame_id = name
                p = pose.get('position', {})
                tf.transform.translation.x = float(p.get('x', 0.0))
                tf.transform.translation.y = float(p.get('y', 0.0))
                tf.transform.translation.z = float(p.get('z', 0.0))
                o = pose.get('orientation', {})
                tf.transform.rotation.x = float(o.get('x', 0.0))
                tf.transform.rotation.y = float(o.get('y', 0.0))
                tf.transform.rotation.z = float(o.get('z', 0.0))
                tf.transform.rotation.w = float(o.get('w', 1.0))
                msg.transforms.append(tf)

            if msg.transforms:
                self.pub.publish(msg)


def main():
    rclpy.init()
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        if node._proc is not None:
            node._proc.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
