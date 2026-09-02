"""Start the VDA 5050 vehicle interface.

    export AMR_VDA5050_USER=vehicle AMR_VDA5050_PASSWORD=...
    export AMR_VDA5050_CA=/path/to/broker-ca.crt
    ros2 launch amr_vda5050 vda5050.launch.py broker_host:=fleet.example.com

WHY THERE WAS NO LAUNCH FILE UNTIL NOW

There was none, and the README called this package the vehicle half an
integrator connects to. A headline interface that no launch file can start is
a claim rather than a feature, and this is half of closing that gap; the other
half is that it now refuses to connect without credentials and TLS.

Nothing starts this from the main bringup on purpose. A vehicle that accepts
remote orders should be started deliberately, by somebody who has decided the
broker is trusted, and not as a side effect of bringing up a robot to test
navigation.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('broker_host', default_value='localhost'),
        DeclareLaunchArgument('broker_port', default_value='8883',
                              description='8883 is MQTT over TLS; 1883 is clear'),
        DeclareLaunchArgument('manufacturer', default_value='Neobotix'),
        DeclareLaunchArgument('serial_number', default_value='mp400-01'),
        DeclareLaunchArgument('map_id', default_value='test_track'),
        DeclareLaunchArgument(
            'allow_insecure', default_value='false',
            description='connect without credentials or TLS. For a local test '
                        'broker only: anyone who can reach it can drive the '
                        'vehicle'),
    ]
    return LaunchDescription(args + [
        Node(
            package='amr_vda5050', executable='vda5050_bridge',
            name='vda5050_bridge', output='screen',
            parameters=[{
                'use_sim_time': True,
                'broker_host': LaunchConfiguration('broker_host'),
                'broker_port': LaunchConfiguration('broker_port'),
                'manufacturer': LaunchConfiguration('manufacturer'),
                'serial_number': LaunchConfiguration('serial_number'),
                'map_id': LaunchConfiguration('map_id'),
                'allow_insecure': LaunchConfiguration('allow_insecure'),
            }]),
    ])
