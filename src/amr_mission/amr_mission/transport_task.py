#!/usr/bin/env python3
"""Fetch a load from one station and deliver it to another, repeatedly.

WHERE THIS SITS

Three layers, and keeping them apart is the whole design:

    mission      decides WHAT to do.        This file.
    navigation   decides HOW to get there.  Nav2, ADR 0008.
    safety       decides whether it MAY.    The collision monitor, ADR 0009.

The mission layer is allowed to be ambitious. It is not allowed to be trusted,
which is why it commands through `NavigateToPose` like any other client and has
no path to the wheels of its own.

WHAT A LOAD ACTUALLY CHANGES

Carrying 250 kg is not a flag on a state machine, it changes the vehicle's
dynamics, and the platform spec already carries both figures. The sheet gives
0.3 m/s2 as the acceleration limit WITH MAXIMUM PAYLOAD, which is a
load-retention number: it exists so the load does not slide. Unladen the vehicle
is allowed 1.0 m/s2. So this node switches the acceleration limit when it picks
up and puts down, which is the honest model of what a payload means to a
vehicle.

The limit is applied to the VELOCITY SMOOTHER, not to the controller. See
set_payload: applying it to MPPI's trajectory sampler instead destroyed the
optimiser's candidate diversity and reduced the laden leg to 0.015 m/s.

That coupling has a consequence this node does NOT yet act on, recorded here
rather than hidden: a laden vehicle takes longer to stop, so a fully correct
implementation switches PROTECTIVE FIELD SETS with load state too, exactly as a
real safety controller does. The fields here are sized for the unladen braking
rate. Since laden acceleration is gentler and the emergency braking rate is
unchanged, the current fields are not undersized by this omission, but the
coupling should be closed before any field figure is published. See
docs/validation.md.

WHAT IS MEASURED

A transport task that cannot say how long a cycle took is a demonstration, not
an experiment. Every cycle reports its duration, the distance driven, how many
times the safety layer stopped the vehicle, and how long it spent held up. The
last two are the interesting ones: they are the price of sharing a floor with
people, and they are the number an intralogistics customer actually asks about.
"""

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

# Collision monitor action codes, from nav2_msgs/CollisionMonitorState.
ACTION = {0: 'clear', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}


class Cycle:
    """One complete pick and deliver, with the numbers that describe it."""

    def __init__(self, index):
        self.index = index
        self.started = time.monotonic()
        self.finished = None
        self.legs = []
        self.distance_start = None
        self.distance_end = None
        self.stops = 0
        self.held_up = 0.0
        # Seconds spent in each collision monitor state, so a cycle can say
        # WHICH safety behaviour cost the time rather than only that some did.
        self.in_state = {}
        self.completed = False

    @property
    def duration(self):
        return (self.finished or time.monotonic()) - self.started

    @property
    def distance(self):
        if self.distance_start is None or self.distance_end is None:
            return 0.0
        return self.distance_end - self.distance_start


class TransportTask(Node):
    def __init__(self):
        super().__init__('transport_task')
        default = str(Path(get_package_share_directory('amr_mission'))
                      / 'config' / 'stations.yaml')
        path = self.declare_parameter('stations_file', default).value
        self.cycles_wanted = self.declare_parameter('cycles', 3).value
        self.dwell = self.declare_parameter('handling_time_s', 5.0).value
        self.leg_timeout = self.declare_parameter('leg_timeout_s', 240.0).value
        # Both from the platform spec. See the note at the top of this file.
        self.accel_laden = self.declare_parameter('accel_laden', 0.3).value
        self.accel_unladen = self.declare_parameter('accel_unladen', 1.0).value

        spec = yaml.safe_load(Path(path).read_text())
        self.stations = {s['name']: s for s in spec['stations']}
        self.route = spec['route']
        missing = [n for n in self.route if n not in self.stations]
        if missing:
            raise SystemExit(f'route names stations that do not exist: {missing}')

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self.odom_total = 0.0
        self._last_odom = None
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, 20)

        self._action = 'clear'
        self._action_since = time.monotonic()
        self.create_subscription(CollisionMonitorState,
                                 '/collision_monitor_state', self._monitor, 20)

        self.cycle = None
        self.carrying = False
        self.get_logger().info(
            f'transport task: {" -> ".join(self.route)}, '
            f'{self.cycles_wanted} cycle(s), from {Path(path).name}')

    # ---- telemetry ------------------------------------------------------

    def _odom(self, msg):
        """Distance travelled, integrated from odometry.

        Integrated from the vehicle's own wheel odometry rather than from the
        ground truth pose, deliberately. This is a number the real vehicle can
        report about itself; taking it from the simulator's oracle would make it
        unreproducible on hardware. It will drift, and that is the honest
        figure a fleet manager would receive.
        """
        p = msg.pose.pose.position
        if self._last_odom is not None:
            self.odom_total += math.hypot(p.x - self._last_odom[0],
                                          p.y - self._last_odom[1])
        self._last_odom = (p.x, p.y)

    def _monitor(self, msg):
        """Count protective stops, and time spent in each monitor state.

        ACCRUED ON EVERY MESSAGE, not only on transitions, and the difference
        is not pedantic. The first version added elapsed time only when the
        action CHANGED. A vehicle driving an entire leg with the warning field
        continuously breached never changes state, so it accrued nothing, and
        the cycle report claimed "0 s held up" for a leg that had been speed
        capped from end to end. The KPI said the safety layer cost nothing while
        it was in fact halving the speed.
        """
        now = time.monotonic()
        action = ACTION.get(msg.action_type, 'unknown')
        elapsed = now - self._action_since
        if self.cycle is not None:
            self.cycle.in_state[self._action] = (
                self.cycle.in_state.get(self._action, 0.0) + elapsed)
            if self._action in ('stop', 'slowdown', 'limit'):
                self.cycle.held_up += elapsed
        if action != self._action and action == 'stop' and self.cycle:
            self.cycle.stops += 1
        self._action, self._action_since = action, now

    def publish_state(self, text):
        self.state_pub.publish(String(data=text))
        self.get_logger().info(text)

    # ---- helpers --------------------------------------------------------

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, label):
        # Wall clock, not simulated time. These are waits for something to
        # appear, and a deadline in simulated time stalls with the simulator.
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f'timed out waiting for {label}')
        return False

    def set_payload(self, carrying):
        """Switch the acceleration limit with the load state.

        THE LIMIT GOES TO THE VELOCITY SMOOTHER, NOT TO MPPI, and putting it in
        the wrong place cost a full diagnosis.

        MPPI's `ax_max` is not a physical limit, it is the bound used to
        generate candidate trajectories. Set it low and every one of the 2000
        samples lands within `ax_max * model_dt` of the last command, the
        candidate set has no diversity, and the optimiser returns essentially
        its prior. ADR 0008 records this happening at 0.3 m/s2, and the fix was
        to raise it to the unladen 1.0.

        This method then set it straight back to 0.3 the moment the vehicle
        picked something up, reintroducing the fault it was supposed to have
        left behind. Measured on the laden delivery leg: commanded 0.013 to
        0.022 m/s for four minutes, which is 0.3 * 0.05 = 0.015 m/s, the
        acceleration-limited first step from rest. The unladen pick-up leg on
        the same run arrived normally.

        The velocity smoother is the right place. It is a rate limiter sitting
        between the controller and the wheels, so it shapes what the vehicle
        actually does without touching how the controller explores its options.
        A load that must not slide is a constraint on the commanded profile,
        which is exactly what a smoother enforces.
        """
        self.carrying = carrying
        target = self.accel_laden if carrying else self.accel_unladen
        cli = self.create_client(SetParameters,
                                 '/velocity_smoother/set_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'velocity_smoother parameter service missing; the acceleration '
                'limit was NOT changed, so this cycle does not model the load')
            return False
        # [x, y, theta]. The vehicle is differential, so the y entry is unused,
        # and the angular rate is not changed by a load on the deck.
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='max_accel',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(target), 0.0, 2.0]))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = bool(fut.result() and fut.result().results
                  and fut.result().results[0].successful)
        self.get_logger().info(
            f'{"loaded" if carrying else "unloaded"}: acceleration limit '
            f'{"set to" if ok else "NOT set to"} {target} m/s2')
        return ok

    def drive_to(self, station):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # STAMP ZERO, meaning "use the latest available transform", and NOT the
        # current time.
        #
        # Stamping with this node's clock made the controller reject goals
        # outright: "Lookup would require extrapolation into the future.
        # Requested time 326.928 but the latest data is at 326.912", followed by
        # "Unable to transform goal pose into costmap frame" and an aborted
        # goal. Sixteen milliseconds.
        #
        # It looked like a SLAM problem, and it is not. Measured over 15140
        # publications across a five cycle run, map to odom holds a steady
        # 20.0 ms mean and 22.9 ms p95 from start to finish, with the worst gap
        # occurring at STARTUP and improving thereafter. The transform is
        # healthy. The stamp was the problem: under sim time, nodes receive
        # /clock a few milliseconds apart, so a stamp taken from the mission
        # node's clock can be marginally ahead of what the controller's buffer
        # has seen, and a transform lookup in the future fails no matter how
        # generous the tolerance, because a tolerance reaches into the past.
        #
        # A goal pose in the map frame is not a time-varying quantity. There is
        # nothing to interpolate and no reason to demand a particular instant.
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(station['x'])
        goal.pose.pose.position.y = float(station['y'])
        yaw = float(station.get('approach_yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send = self.nav.send_goal_async(goal)
        if not self.wait_for(send.done, 15.0, 'goal acceptance'):
            return False
        handle = send.result()
        if not handle.accepted:
            self.get_logger().warn(f'{station["name"]}: goal rejected')
            return False
        result = handle.get_result_async()
        if not self.wait_for(result.done, self.leg_timeout, 'arrival'):
            handle.cancel_goal_async()
            return False
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'{station["name"]}: ended with status {status}')
            return False
        return True

    # ---- the task -------------------------------------------------------

    def run(self):
        if not self.wait_for(lambda: self.get_clock().now().nanoseconds > 0,
                             30.0, 'the clock'):
            return 1
        if not self.wait_for(
                lambda: self.nav.server_is_ready()
                or self.nav.wait_for_server(timeout_sec=0.1),
                60.0, 'navigate_to_pose'):
            return 1

        # Start unladen, and say so rather than assuming the controller is in
        # whatever state the last run left it in.
        self.set_payload(False)

        done = []
        for index in range(1, self.cycles_wanted + 1):
            self.cycle = Cycle(index)
            self.cycle.distance_start = self.odom_total
            ok = True
            for leg, name in enumerate(self.route):
                station = self.stations[name]
                self.publish_state(
                    f'cycle {index}: driving to {name} '
                    f'({"laden" if self.carrying else "empty"})')
                t0 = time.monotonic()
                arrived = self.drive_to(station)
                self.cycle.legs.append((name, time.monotonic() - t0, arrived))
                if not arrived:
                    self.publish_state(f'cycle {index}: failed to reach {name}')
                    ok = False
                    break
                # Handling. First station of the route loads, the last unloads.
                self.publish_state(
                    f'cycle {index}: at {name}, '
                    f'{"loading" if leg == 0 else "unloading"}')
                self.spin_for(self.dwell)
                self.set_payload(leg == 0)

            # Let the navigation stack settle before the next cycle. Firing
            # immediately caught controller_server still unwinding the previous
            # goal's recovery, so it could not acknowledge the new goal and the
            # cycle died in 9 seconds without the vehicle moving. A real fleet
            # manager does not dispatch the next task the instant the last one
            # reports either.
            self.spin_for(3.0)
            self.cycle.finished = time.monotonic()
            self.cycle.distance_end = self.odom_total
            self.cycle.completed = ok
            done.append(self.cycle)
            self.report(self.cycle)

        self.summary(done)
        return 0 if all(c.completed for c in done) else 1

    def report(self, c):
        legs = ', '.join(f'{n} {t:.0f}s{"" if a else " FAILED"}'
                         for n, t, a in c.legs)
        self.get_logger().info(
            f'cycle {c.index}: {"complete" if c.completed else "INCOMPLETE"} in '
            f'{c.duration:.0f} s, {c.distance:.1f} m driven, '
            f'{c.stops} protective stop(s), {c.held_up:.0f} s held up  [{legs}]')
        if c.in_state:
            share = ', '.join(
                f'{k} {v / max(1e-9, c.duration) * 100:.0f}%'
                for k, v in sorted(c.in_state.items(), key=lambda kv: -kv[1])
                if v > 0.5)
            self.get_logger().info(f'  monitor state: {share}')

    def summary(self, cycles):
        good = [c for c in cycles if c.completed]
        self.get_logger().info('=' * 68)
        self.get_logger().info(
            f'{len(good)} of {len(cycles)} cycle(s) completed')
        if not good:
            return
        dur = sum(c.duration for c in good) / len(good)
        dist = sum(c.distance for c in good) / len(good)
        stops = sum(c.stops for c in good) / len(good)
        held = sum(c.held_up for c in good) / len(good)
        self.get_logger().info(f'  mean cycle time      {dur:6.0f} s')
        self.get_logger().info(f'  mean distance        {dist:6.1f} m')
        self.get_logger().info(f'  mean speed           {dist / dur:6.2f} m/s')
        self.get_logger().info(f'  protective stops     {stops:6.1f} per cycle')
        self.get_logger().info(
            f'  held up by safety    {held:6.0f} s per cycle '
            f'({held / dur * 100:.0f} percent of the cycle)')
        self.get_logger().info(
            '  the last two are the price of sharing a floor with people')
        self.get_logger().info('=' * 68)


def main():
    rclpy.init()
    node = TransportTask()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
