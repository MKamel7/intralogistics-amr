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
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  TextSubstitution)
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
# MANAGER DELAYS, and they are not padding.
#
# Nav2's lifecycle manager begins configuring as soon as it starts, and it does
# NOT wait for the nodes it manages to finish constructing. Measured on this
# machine with Gazebo and SLAM already running:
#
#   023.212  manager: "Configuring filter_mask_server"
#   023.370  filter_mask_server: "lifecycle node launched ... Creating"
#
# The configure request arrived 158 ms before the node existed to answer it. The
# navigation group is worse: controller_server builds two costmaps and the MPPI
# optimiser before it serves anything, and its manager gave up 20 ms after
# asking even with an 8 second head start.
#
# The manager has no wait-for-node option and no service-call timeout parameter,
# both checked against the installed library, so staggering the managers is the
# only lever available. The nodes themselves all start immediately; only the
# transitions wait, so this costs bringup time and nothing else.
#
# When the FILTER group loses this race the failure is silent and safety
# relevant: the keepout mask is never published, both costmaps run with no
# keepout zones, and the only symptom is a WARN per costmap update. See V-25.
FILTER_MANAGER_DELAY = 12.0
NAV_MANAGER_DELAY = 30.0

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
    # ONE CONFIGURATION PER PLATFORM, selected by name and generated from that
    # platform's spec by tools/generate_nav2.py. There is deliberately no
    # fallback file: a missing platform must fail the launch loudly rather than
    # quietly hand the vehicle another machine's footprint, speed limits and
    # inflation radius, which is what a single nav2.yaml did.
    params = PathJoinSubstitution([
        share, 'config',
        [TextSubstitution(text='nav2.'),
         LaunchConfiguration('platform'),
         TextSubstitution(text='.yaml')]])
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

    # The filter group first, and on its own manager, delayed so the manager
    # cannot ask before the node can answer. See MANAGER_DELAYS.
    nodes.append(TimerAction(period=FILTER_MANAGER_DELAY, actions=[Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            # Generous, because this group reads map files from disk and the
            # default timeout is what stalled the whole stack.
            'bond_timeout': 20.0,
            'node_names': [name for name, _, _ in FILTER_NODES],
        }])]))

    # One manager for the navigation nodes. SLAM and the safety nodes have their
    # own managers in robot.launch.py, so navigation can be restarted during
    # tuning without dropping the map or, more to the point, without dropping
    # the collision monitor.
    #
    # DELAYED, so the two managers do not configure at the same instant.
    #
    # Splitting the filters onto their own manager stopped one slow node
    # stalling the other group, but both managers still fired together and both
    # then contended for the same loaded machine. Measured: the two "Starting
    # managed nodes bringup" lines landed 110 ms apart, the navigation manager
    # gave up on controller_server SIX MILLISECONDS after asking, and
    # filter_mask_server printed "Configuring" 400 ms AFTER its own manager had
    # already declared it failed.
    #
    # The consequence when only the filter group loses that race is the worse
    # one, because it is silent: the mask is never published, both costmaps run
    # with NO keepout zones, and the only symptom is a WARN per costmap update.
    # One run completed five cycles that way while preflight reported every
    # check passing. See V-25.
    #
    # Nav2's lifecycle manager has no service-call timeout parameter, so this
    # cannot be tuned. Giving the filter group a clear head start is the
    # remaining lever, and it matches the order this file already documents as
    # the correct one. The nodes themselves all start immediately; only the
    # transitions are staggered, so this costs a few seconds of bringup and
    # nothing else.
    nodes.append(TimerAction(period=NAV_MANAGER_DELAY, actions=[Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 20.0,
            'node_names': [name for name, _, _ in NAV_NODES],
        }])]))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Must match the platform robot.launch.py was given. run_stack.sh
        # passes the same value to both.
        DeclareLaunchArgument('platform', default_value='mir250_class'),
        *nodes,
    ])
