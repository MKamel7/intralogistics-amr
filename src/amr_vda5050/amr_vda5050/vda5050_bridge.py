#!/usr/bin/env python3
"""VDA 5050 vehicle interface: MQTT on one side, Nav2 on the other.

WHAT THIS IS FOR

A master control that cannot talk to a vehicle is a vehicle that cannot be
sold. VDA 5050 is what German intralogistics integrates against, and this node
is the vehicle half: it accepts orders over MQTT, drives them with the same
Nav2 action the transport task uses, and reports state the way a fleet manager
expects to read it.

THE DIVISION OF LABOUR IS DELIBERATE

All the protocol bookkeeping lives in vda5050.py with no I/O in it, and is
tested without a broker. This file is the thin part: sockets, an action client,
and the translation between a VDA 5050 node list and a sequence of navigation
goals. When something goes wrong in the field it is almost always here, in the
I/O, and keeping the reasoning out of it means the reasoning stays testable.

WHAT IT IMPLEMENTS

    connection      ONLINE on connect, OFFLINE on clean shutdown, and
                    CONNECTIONBROKEN as the BROKER'S last will. A vehicle
                    cannot send its own last will, which is the entire point:
                    the robot that has crashed is exactly the one that cannot
                    tell anybody it has.

    state           at 1 Hz AND on change. A master control schedules against
                    driving, lastNodeId and actionStates; a vehicle that only
                    reports on a timer gives it up to a second of stale truth
                    to schedule on.

    order           base nodes traversed in sequence. An update must join the
                    order it extends, or it is refused, because accepting a
                    disjoint update makes the vehicle drive to a node it was
                    never routed to.

    instantActions  cancelOrder and startPause, which are the two a transport
                    vehicle cannot operate without.
"""

import sys
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from amr_vda5050 import vda5050 as v

try:
    import paho.mqtt.client as mqtt
except ImportError:                                   # pragma: no cover
    mqtt = None


class Vda5050Bridge(Node):
    def __init__(self):
        super().__init__('vda5050_bridge')
        self.manufacturer = self.declare_parameter('manufacturer', 'Neobotix').value
        self.serial = self.declare_parameter('serial_number', 'mp400-01').value
        self.broker = self.declare_parameter('broker_host', 'localhost').value
        self.port = self.declare_parameter('broker_port', 1883).value
        self.map_id = self.declare_parameter('map_id', 'test_track').value

        self.header = v.Header(self.manufacturer, self.serial)
        # REENTRANT ON PURPOSE. _accept_order and _drive_next both hold the
        # lock and then call _publish_state, which takes it again to build the
        # message. With a plain Lock that deadlocks, and it deadlocks only on
        # the paths that report a problem: a refused order update, or a queue
        # that has run dry. The happy path never touched it, so the interface
        # looked healthy and went silent exactly when it had something to say.
        #
        # Measured: state publication stopped dead at 14 messages, one second
        # after a deliberately disjoint order update, and the error it should
        # have reported never left the vehicle.
        self.lock = threading.RLock()

        self.order = None
        self.node_queue = []
        self.last_node = ''
        self.last_node_seq = 0
        self.driving = False
        self.paused = False
        self.actions = []
        self.pose = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0)
        self.errors = []

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, 10)

        self.client = None
        if mqtt is None:
            # NOT FATAL, and it says so. The node is useful without a broker
            # for anyone reading the state it would publish, and a hard exit
            # here would make the whole stack fail to launch over an optional
            # interface.
            self.get_logger().error(
                'paho-mqtt is not installed, so the VDA 5050 interface is '
                'inert. Install python3-paho-mqtt. The rest of the stack is '
                'unaffected.')
        else:
            self._connect()

        # 1 Hz heartbeat. Change driven publications go out immediately from
        # wherever the change happens.
        self.create_timer(1.0, self._publish_state)

    # ---- MQTT ------------------------------------------------------------

    def _topic(self, name):
        return v.topic(self.manufacturer, self.serial, name)

    def _connect(self):
        self.client = mqtt.Client(client_id=f'{self.manufacturer}-{self.serial}')
        # THE LAST WILL IS THE POINT. Registered before connecting, so the
        # broker holds it from the first moment and publishes it if this
        # process dies without saying goodbye.
        self.client.will_set(
            self._topic('connection'),
            v.dumps(v.connection(self.header, 'CONNECTIONBROKEN')),
            qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        try:
            self.client.connect(self.broker, self.port, keepalive=15)
        except OSError as e:
            self.get_logger().error(
                f'no broker at {self.broker}:{self.port} ({e}). The interface '
                f'is inert; the vehicle still runs.')
            self.client = None
            return
        self.client.loop_start()

    def _on_connect(self, client, _userdata, _flags, rc):
        if rc != 0:
            self.get_logger().error(f'broker refused the connection, rc={rc}')
            return
        client.subscribe(self._topic('order'), qos=1)
        client.subscribe(self._topic('instantActions'), qos=1)
        client.publish(self._topic('connection'),
                       v.dumps(v.connection(self.header, 'ONLINE')),
                       qos=1, retain=True)
        self.get_logger().info(
            f'VDA 5050 {v.VERSION} online as {self.manufacturer}/{self.serial}')

    def _on_message(self, _client, _userdata, msg):
        import json
        try:
            payload = json.loads(msg.payload)
        except ValueError:
            self.get_logger().error('discarding malformed JSON from the broker')
            return
        if msg.topic.endswith('/order'):
            self._accept_order(payload)
        elif msg.topic.endswith('/instantActions'):
            self._instant_actions(payload)

    # ---- orders ----------------------------------------------------------

    def _accept_order(self, order):
        with self.lock:
            try:
                v.order_is_continuous(self.order, order)
            except v.SequenceError as e:
                # REFUSED AND REPORTED, not silently dropped. A master control
                # that never hears about a rejected order assumes the vehicle
                # is executing it.
                self.get_logger().error(f'order refused: {e}')
                self.errors.append({
                    'errorType': 'orderUpdateError',
                    'errorLevel': 'WARNING',
                    'errorDescription': str(e),
                })
                self._publish_state()
                return
            self.order = order
            self.node_queue = list(order.get('nodes', []))
            self.errors = [e for e in self.errors
                           if e['errorType'] != 'orderUpdateError']
            self.actions = [
                v.action_state(a['actionId'], a['actionType'], 'WAITING')
                for n in self.node_queue for a in n.get('actions', [])]
        self.get_logger().info(
            f"order {order.get('orderId')} update "
            f"{order.get('orderUpdateId')} accepted, "
            f"{len(self.node_queue)} node(s)")
        self._publish_state()
        self._drive_next()

    def _drive_next(self):
        with self.lock:
            if self.paused or not self.node_queue:
                self.driving = False
                self._publish_state()
                return
            node = self.node_queue[0]
        pos = node.get('nodePosition') or {}
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pos.get('x', 0.0))
        goal.pose.pose.position.y = float(pos.get('y', 0.0))
        goal.pose.pose.orientation.w = 1.0
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose is not available')
            return
        self.driving = True
        self._publish_state()
        self.nav.send_goal_async(goal).add_done_callback(self._goal_sent)

    def _goal_sent(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('navigation refused the goal')
            self.driving = False
            self._publish_state()
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, _future):
        with self.lock:
            if self.node_queue:
                done = self.node_queue.pop(0)
                self.last_node = done.get('nodeId', '')
                self.last_node_seq = int(done.get('sequenceId', 0))
                for a in done.get('actions', []):
                    for st in self.actions:
                        if st['actionId'] == a['actionId']:
                            st['actionStatus'] = 'FINISHED'
            more = bool(self.node_queue)
            self.driving = more
        self._publish_state()
        if more:
            self._drive_next()

    # ---- instant actions -------------------------------------------------

    def _instant_actions(self, payload):
        for action in payload.get('actions', []):
            kind = action.get('actionType')
            if kind == 'cancelOrder':
                with self.lock:
                    self.node_queue = []
                    self.driving = False
                    for st in self.actions:
                        if st['actionStatus'] in ('WAITING', 'RUNNING'):
                            st['actionStatus'] = 'FAILED'
                            st['resultDescription'] = 'cancelled by master control'
                if getattr(self, '_handle', None) is not None:
                    self._handle.cancel_goal_async()
                self.get_logger().warn('order cancelled by master control')
            elif kind == 'startPause':
                with self.lock:
                    self.paused = True
                    self.driving = False
                if getattr(self, '_handle', None) is not None:
                    self._handle.cancel_goal_async()
            elif kind == 'stopPause':
                with self.lock:
                    self.paused = False
                self._drive_next()
            else:
                self.get_logger().warn(f'unsupported instantAction {kind}')
            self._publish_state()

    # ---- state -----------------------------------------------------------

    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        import math
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _publish_state(self):
        with self.lock:
            msg = v.state(
                self.header,
                order_id=(self.order or {}).get('orderId', ''),
                order_update_id=(self.order or {}).get('orderUpdateId', 0),
                last_node=self.last_node, last_node_seq=self.last_node_seq,
                driving=self.driving, paused=self.paused,
                x=self.pose[0], y=self.pose[1], theta=self.pose[2],
                map_id=self.map_id,
                node_states=[{'nodeId': n.get('nodeId', ''),
                              'sequenceId': n.get('sequenceId', 0),
                              'released': True} for n in self.node_queue],
                action_states=self.actions,
                errors=self.errors,
                velocity=self.velocity)
        if self.client is not None:
            self.client.publish(self._topic('state'), v.dumps(msg), qos=0)

    def shutdown(self):
        if self.client is not None:
            self.client.publish(self._topic('connection'),
                                v.dumps(v.connection(self.header, 'OFFLINE')),
                                qos=1, retain=True)
            self.client.loop_stop()
            self.client.disconnect()


def main():
    rclpy.init()
    node = Vda5050Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
