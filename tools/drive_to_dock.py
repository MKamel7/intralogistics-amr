#!/usr/bin/env python3
"""Turn the vehicle to face the dock and creep in, for a docking measurement.

    python3 tools/drive_to_dock.py --ros-args -p stations_file:=<stations.yaml>

WHY THIS EXISTS RATHER THAN A TRANSPORT MISSION

Measuring the dock detector needs the vehicle in front of the dock. It does not
need a survey, a map, a planner or a transport cycle, and using those cost 40
minutes a run and failed twice for reasons that had nothing to do with docking:
once because a pedestrian standing beside the vehicle marked its own cell
occupied, once because the mission had no map yet.

On the generated track the dock is 2.9 m BEHIND the spawn pose, so the whole
approach is a turn and a short drive. This publishes velocity on cmd_vel_raw,
which is the safety layer's INPUT, so the collision monitor still sits between
this and the wheels and can still stop it. Driving the vehicle by hand is
legitimate for a sensor measurement and would not be for a navigation one, and
the report says which this is.
"""

import math

import rclpy
import yaml
from rclpy.parameter import Parameter
from amr_common.pose import to_frame, yaw_error, yaw_from_quaternion
from amr_common.topics import Topics
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_msgs.msg import TFMessage

#: The oracle publishes BEST_EFFORT, and a RELIABLE subscriber to a BEST_EFFORT
#: publisher receives nothing at all. Both sides look healthy: the topic has a
#: publisher, the node has a subscription, and no message ever arrives. Copied
#: deliberately from measure_docking, which had it right.
TRUTH_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10)


class DriveToDock(Node):
    def __init__(self):
        # SIM CLOCK, set at construction rather than after it, which is what
        # test_probe_clocks enforces across tools/. Setting it later leaves a
        # window where the node is on wall time, and this stamps every
        # TwistStamped it publishes: a wall-clock stamp against a stack running
        # on sim time is the mismatch that once printed a sensor latency of
        # 1786870613342 ms.
        super().__init__('drive_to_dock',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        path = self.declare_parameter('stations_file', '').value
        self.standoff = self.declare_parameter('standoff_m', 1.2).value
        # Approach, back off, approach again. One approach gives a handful of
        # moving detections and a window full of stationary ones, which is how
        # the first run produced a p95 that was really about a parked vehicle.
        self.cycles = self.declare_parameter('cycles', 4).value
        self.retreat_to = self.declare_parameter('retreat_m', 2.6).value
        # Start by backing off. The vehicle may already be at the dock from a
        # previous run, and counting that as an approach both inflates the
        # count and collects no moving detections at all: the first version of
        # this logged "624 of 4 approaches" without the wheels turning.
        self.retreating = True
        self.done_cycles = 0
        self.finished = False
        self.turn_speed = self.declare_parameter('turn_speed', 0.5).value
        self.drive_speed = self.declare_parameter('drive_speed', 0.25).value
        spec = yaml.safe_load(open(path))
        spawn, dock = spec['spawn'], spec['dock']
        # The dock block is world frame; everything the vehicle knows is map
        # frame. See amr_common.pose.to_frame for what forgetting that cost.
        self.dock = to_frame(dock['x'], dock['y'],
                             spawn['x'], spawn['y'], spawn.get('yaw', 0.0))
        # TwistStamped, because the collision monitor is configured with
        # enable_stamped_cmd_vel: true. Publishing a plain Twist puts a second
        # type on the same topic name: both nodes look connected, `ros2 topic
        # hz` shows commands flowing at 10 Hz, and the monitor receives nothing.
        # The vehicle sat still for three runs on that.
        self.pub = self.create_publisher(TwistStamped, Topics.CMD_VEL_RAW, 10)
        self.create_subscription(TFMessage, Topics.GROUND_TRUTH_POSES, self._truth, TRUTH_QOS)
        self.vehicle_frame = self.declare_parameter('vehicle_frame', 'amr').value
        self.pose = None
        self.spawn = (spawn['x'], spawn['y'], spawn.get('yaw', 0.0))
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'dock at map ({self.dock[0]:.2f}, {self.dock[1]:.2f}), '
            f'stopping {self.standoff:.2f} m short of it')

    def _truth(self, msg):
        for tf in msg.transforms:
            # The oracle names the vehicle 'amr', which is also what
            # measure_docking looks for. Guessing at 'base_link' here matched
            # nothing and the script sat still, looking exactly like a vehicle
            # that had decided it was already at the dock.
            if tf.child_frame_id != self.vehicle_frame:
                continue
            t = tf.transform.translation
            # Ground truth is world frame; put it in the map frame so it can be
            # compared with a dock the same way the gate does.
            x, y = to_frame(t.x, t.y, *self.spawn)
            self.pose = (x, y, yaw_from_quaternion(tf.transform.rotation) - self.spawn[2])
            return

    def _tick(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        dx, dy = self.dock[0] - x, self.dock[1] - y
        distance = math.hypot(dx, dy)
        bearing = yaw_error(math.atan2(dy, dx), yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.finished:
            self.pub.publish(cmd)
            return

        if self.retreating:
            if distance >= self.retreat_to:
                self.retreating = False
                self.get_logger().info(f'backed off to {distance:.2f} m, approaching again')
            else:
                cmd.twist.linear.x = -self.drive_speed
                self.pub.publish(cmd)
                return
        elif distance <= self.standoff:
            self.done_cycles += 1
            self.get_logger().info(
                f'reached the dock at {distance:.2f} m '
                f'({self.done_cycles} of {self.cycles} approaches)')
            if self.done_cycles >= self.cycles:
                self.finished = True
                self.pub.publish(cmd)
                self.get_logger().info(
                    f'{self.done_cycles} approaches done, holding. The moving '
                    f'detections are the ones a docking claim rests on.')
                return
            self.retreating = True
            return
        if abs(bearing) > 0.15:
            cmd.twist.angular.z = math.copysign(self.turn_speed, bearing)
        else:
            cmd.twist.linear.x = self.drive_speed
            cmd.twist.angular.z = 0.5 * bearing
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = DriveToDock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
