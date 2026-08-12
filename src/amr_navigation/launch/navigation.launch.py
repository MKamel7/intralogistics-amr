#!/usr/bin/env python3
"""Nav2 planning and control, on top of the map SLAM is building.

WHAT THIS DOES NOT DO. It does not start SLAM, and it does not start the
collision monitor. Both belong to robot.launch.py, and both must already be
running: SLAM because there is nothing to plan on without it, and the monitor
because it is the last thing between a planned command and the wheels. Starting
navigation without the monitor would put a planner in direct control of a
vehicle that shares a floor with people, which is exactly the arrangement the
whole safety concept exists to prevent.

THE COMMAND CHAIN, in order:

    controller_server  -> /cmd_vel_nav
    velocity_smoother  -> /cmd_vel_smoothed
    collision_monitor  -> /diff_drive_controller/cmd_vel   (in robot.launch.py)

Every stage is TwistStamped. diff_drive_controller 4.x rejects an unstamped
Twist and says nothing about it, so a mismatch anywhere in that chain shows up
only as a vehicle that will not move.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# The costmap filter servers come FIRST in the lifecycle order, because a
# costmap that activates before its filter mask exists comes up without the
# keepout zones and never picks them up.
# (node name, package, executable). The node NAME and the EXECUTABLE are not
# always the same thing and conflating them fails at launch: the keepout mask is
# served by nav2's ordinary `map_server` binary running under the name
# `filter_mask_server`, which is the name its parameters are keyed on.
# THE FILTER SERVERS GET THEIR OWN LIFECYCLE MANAGER, and this is not tidiness.
#
# `filter_mask_server` reads a map file from disk when it configures. Under load
# that took long enough that its change_state response missed the manager's
# service timeout: "failed to send response to /filter_mask_server/change_state
# (timeout)". The manager was still waiting on it, so it never went on to
# configure anything else, and controller_server, planner_server and
# bt_navigator all sat unconfigured while the launch reported nothing wrong.
#
# Two managers means one slow node cannot stall the other group, and it matches
# what nav2's own costmap filter examples do.
FILTER_NODES = [
    ('filter_mask_server', 'nav2_map_server', 'map_server'),
    ('costmap_filter_info_server', 'nav2_map_server', 'costmap_filter_info_server'),
]

NAV_NODES = [
    ('controller_server', 'nav2_controller', 'controller_server'),
    ('smoother_server', 'nav2_smoother', 'smoother_server'),
    ('planner_server', 'nav2_planner', 'planner_server'),
    ('behavior_server', 'nav2_behaviors', 'behavior_server'),
    ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator'),
    ('waypoint_follower', 'nav2_waypoint_follower', 'waypoint_follower'),
    ('velocity_smoother', 'nav2_velocity_smoother', 'velocity_smoother'),
]


def generate_launch_description():
    share = get_package_share_directory('amr_navigation')
    params = PathJoinSubstitution([share, 'config', 'nav2.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Topic wiring is done by REMAPPING, not by editing the safety package. The
    # collision monitor's input is called cmd_vel_raw and its configuration is
    # generated from the stopping-distance calculation and covered by tests, so
    # navigation bends to fit it rather than the other way round.
    # ODOMETRY. ros2_control publishes on /diff_drive_controller/odom, and
    # every Nav2 default is plain /odom, which NOTHING publishes.
    #
    # This was the root cause of the crawl, and it is worth stating exactly
    # because nothing reported it. MPPI reads the measured velocity from the
    # odometry topic and limits each command to what the acceleration limit
    # allows from there. With no odometry it believed the vehicle was
    # permanently stationary, so every command was 0 + ax_max * model_dt =
    # 0.3 * 0.05 = 0.015 m/s, for ever. Measured: commanded 0.019 m/s,
    # travelled 0.24 m in 180 s. No node logged an error, the topic simply had
    # no publisher and the subscriber waited. I spent three rounds rebuilding
    # the protective fields before checking whether the topic existed.
    odom = [('/odom', '/diff_drive_controller/odom')]

    remaps = {
        'controller_server': [('/cmd_vel', '/cmd_vel_nav')] + odom,
        'bt_navigator': odom,
        'velocity_smoother': [('/cmd_vel', '/cmd_vel_nav'),
                              ('/cmd_vel_smoothed', '/cmd_vel_raw')] + odom,
        # Recoveries drive the vehicle directly, so they go through the monitor
        # too. A backup manoeuvre that bypassed it would be a blind reverse.
        'behavior_server': [('/cmd_vel', '/cmd_vel_raw')] + odom,
    }

    # The mask server needs an absolute path to the mask, and the parameter
    # file carries only a filename so it stays readable. Resolved here against
    # the installed maps directory.
    maps = PathJoinSubstitution([share, 'maps'])
    extra = {'filter_mask_server': [{
        'yaml_filename': PathJoinSubstitution([maps, 'keepout_mask.yaml'])}]}

    def make(entries):
        return [
            Node(package=pkg, executable=exe, name=name, output='screen',
                 parameters=([params, {'use_sim_time': use_sim_time}]
                             + extra.get(name, [])),
                 remappings=remaps.get(name, []))
            for name, pkg, exe in entries
        ]

    nodes = make(FILTER_NODES) + make(NAV_NODES)

    # The filter group first, and on its own manager.
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            # Generous, because this group reads map files from disk and the
            # default timeout is what stalled the whole stack.
            'bond_timeout': 20.0,
            'node_names': [name for name, _, _ in FILTER_NODES],
        }]))

    # One manager for the navigation nodes. SLAM and the safety nodes have their
    # own managers in robot.launch.py, so navigation can be restarted during
    # tuning without dropping the map or, more to the point, without dropping
    # the collision monitor.
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 20.0,
            'node_names': [name for name, _, _ in NAV_NODES],
        }]))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes,
    ])
