#!/usr/bin/env python3
"""Bring up one MiR250-class AMR in the warehouse.

    ros2 launch amr_bringup robot.launch.py
    ros2 launch amr_bringup robot.launch.py gui:=false cameras:=false
    ros2 launch amr_bringup robot.launch.py rviz:=true

Sensor sets follow the simulation tiers in docs/adr/0003-three-tier-simulation.md,
so `cameras:=false` is the fleet-tier robot and the default is the perception
tier. The measured cost of each is in the top-level README.

Ordering matters here and is enforced rather than hoped for. The controllers
cannot be spawned until the simulator has created the robot, because
gz_ros2_control only starts its controller_manager when the model is inserted
into the world. The predecessor project papered over exactly this class of race
with fixed sleeps, and the last robot's stack would occasionally never come up.
Here the spawners are chained to the spawn process exiting.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
import yaml
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare('amr_description')
    sim_share = FindPackageShare('amr_sim')
    bringup_share = FindPackageShare('amr_bringup')

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='run the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('cameras', default_value='true',
                              description='false gives the fleet-tier robot'),
        DeclareLaunchArgument('scanners', default_value='true'),
        DeclareLaunchArgument('platform', default_value='mir250_class'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='-1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('payload', default_value='0.0',
                              description='kg on the load deck, for the energy model'),
        DeclareLaunchArgument(
            'battery_time_scale', default_value='1.0',
            description='simulated hours per hour; anything but 1.0 is flagged '
                        'in the log because the curve is then not real'),
    ]

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([desc_share, 'urdf', 'amr.urdf.xacro']), ' ',
            'platform:=', LaunchConfiguration('platform'), ' ',
            'use_cameras:=', LaunchConfiguration('cameras'), ' ',
            'use_scanners:=', LaunchConfiguration('scanners'), ' ',
            'sim:=true',
        ]),
        value_type=str)

    def make_gz(context, *_, **__):
        """Build the simulator arguments once `gui` is resolvable.

        `-s` runs the server alone. The GUI is a separate process that renders
        the scene continuously and costs real time on this machine, so headless
        is a first-class mode rather than an afterthought.
        """
        world = LaunchConfiguration('world').perform(context)
        headless = LaunchConfiguration('gui').perform(context).lower() != 'true'
        world_path = PathJoinSubstitution(
            [sim_share, 'worlds', f'{world}.sdf']).perform(context)
        gz_args = f'{world_path} -r' + (' -s' if headless else '')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': gz_args,
                              'on_exit_shutdown': 'true'}.items())]

    gz = OpaqueFunction(function=make_gz)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'amr',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-Y', LaunchConfiguration('yaw'),
                   '-z', '0.02'],
        parameters=[{'use_sim_time': True}])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': PathJoinSubstitution(
            [bringup_share, 'config', 'bridge.yaml']),
            'use_sim_time': True}])

    # Spawners, chained. joint_state_broadcaster first: without joint states
    # the diff drive controller has no feedback and robot_state_publisher
    # cannot place the wheels.
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster',
                          '--controller-manager', '/controller_manager'],
               output='screen')

    diff_drive = Node(package='controller_manager', executable='spawner',
                      arguments=['diff_drive_controller',
                                 '--controller-manager', '/controller_manager'],
                      output='screen')

    def make_battery(context, *_, **__):
        """Battery parameters come from the platform spec, not from defaults.

        The node carries defaults so it can run standalone, but the launch reads
        the same spec the robot description reads, so there is one source for
        the pack and the runtimes it is fitted to.
        """
        platform = LaunchConfiguration('platform').perform(context)
        spec_path = PathJoinSubstitution(
            [desc_share, 'config', 'platforms', f'{platform}.yaml']
        ).perform(context)
        spec = yaml.safe_load(open(spec_path))
        v, tg = spec['values'], spec['validation_targets']
        return [Node(
            package='amr_sim', executable='battery_model', output='screen',
            parameters=[{
                'use_sim_time': True,
                'capacity_wh': v['battery_capacity_kwh'] * 1000.0,
                'nominal_voltage': v['battery_nominal_voltage'],
                'max_speed': v['max_linear_speed'],
                'max_payload_kg': v['max_payload'],
                'runtime_standby_h': tg['runtime_standby_h'],
                'runtime_unloaded_h': tg['runtime_unloaded_h'],
                'runtime_loaded_h': tg['runtime_loaded_h'],
                'reference_speed': 1.0,
                'payload_kg': float(LaunchConfiguration('payload').perform(context)),
                'time_scale': float(LaunchConfiguration('battery_time_scale')
                                    .perform(context)),
            }])]

    battery = OpaqueFunction(function=make_battery)

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution(
            [bringup_share, 'rviz', 'robot.rviz'])],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription(args + [
        gz, rsp, bridge, spawn, rviz, battery,
        # Chain on spawn exiting rather than on a timer. `create` exits once the
        # model is in the world, which is exactly the precondition the
        # controller_manager needs.
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,
                                           on_exit=[diff_drive])),
    ])
