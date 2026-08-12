#!/usr/bin/env python3
"""Run the transport task.

Expects the vehicle, navigation and the safety layer to be up already. It is a
CLIENT of navigation, not a replacement for it, so starting it against a stack
that is not running simply times out waiting for the action server rather than
doing anything surprising.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='3'),
        DeclareLaunchArgument('handling_time_s', default_value='5.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='amr_mission', executable='transport_task',
            name='transport_task', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cycles': LaunchConfiguration('cycles'),
                'handling_time_s': LaunchConfiguration('handling_time_s'),
            }]),
    ])
