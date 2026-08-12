#!/usr/bin/env python3
"""Run the transport task.

Expects the vehicle, navigation and the safety layer to be up already. It is a
CLIENT of navigation, not a replacement for it, so starting it against a stack
that is not running simply times out waiting for the action server rather than
doing anything surprising.

THE ACCELERATION LIMITS COME FROM THE PLATFORM SPEC, and they did not used to.
`transport_task.py` declared them with the MiR250's 0.3 and 1.0 m/s2 as
parameter defaults, under a comment saying both came from the platform spec.
Nothing passed them, so the default was the value, and the MP-400 drove its
first five cycles at a fifth of its own rating with every log line reporting
the MiR250's figure. It was not unsafe, it was slower than the vehicle can go
and it made the cycle times meaningless as a comparison.

This is the same fault as the one that made the whole Nav2 configuration
platform-specific, in a place nothing had looked: a number that is a property
of the vehicle, written where only one vehicle existed.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def envelope_cap(platform, nav2_dir=None):
    """The largest acceleration the navigation envelope permits, in m/s2.

    Read from the GENERATED Nav2 configuration rather than recomputed, because
    the rule that derives it lives in generate_nav2.py and a second copy of a
    rule is a second thing to get out of step.
    """
    root = Path(nav2_dir) if nav2_dir else (
        Path(get_package_share_directory('amr_navigation')) / 'config')
    cfg_file = root / f'nav2.{platform}.yaml'
    if not cfg_file.is_file():
        raise RuntimeError(f'no generated Nav2 configuration at {cfg_file}')
    cfg = yaml.safe_load(cfg_file.read_text())
    return float(cfg['velocity_smoother']['ros__parameters']['max_accel'][0])


def accel_limits(platform, spec_dir=None, nav2_dir=None):
    """The laden and unladen acceleration limits for a platform, in m/s2.

    A plain function taking a plain name, so it can be tested without a launch
    context. That matters more than it looks: the fault this replaces was a
    default value nothing overrode, which is exactly the kind of thing that is
    invisible until something asserts on it.

    BOTH ARE CLAMPED TO THE NAVIGATION ENVELOPE, and that is not belt and
    braces. `set_payload` writes `max_accel` onto the velocity smoother at
    RUNTIME and does not touch `max_decel`, so an uncapped figure here silently
    replaces the symmetric envelope the generator derived with an asymmetric
    one, on the last rate limiter before the wheels. Measured on the MP-400:
    the generated configuration held 1.00 against -1.00, the mission set the
    smoother to 2.40 the moment it reported "unloaded", and the vehicle went
    back to overshooting its goals. See V-25.
    """
    root = Path(spec_dir) if spec_dir else (
        Path(get_package_share_directory('amr_description'))
        / 'config' / 'platforms')
    spec_file = root / f'{platform}.yaml'
    if not spec_file.is_file():
        raise RuntimeError(
            f'no platform spec at {spec_file}. The transport task takes its '
            f'acceleration limits from the spec and there is deliberately no '
            f'fallback: driving one platform on another\'s limits is what the '
            f'platform argument exists to prevent.')
    values = yaml.safe_load(spec_file.read_text())['values']
    cap = envelope_cap(platform, nav2_dir)
    return (min(float(values['max_linear_accel']), cap),
            min(float(values['max_linear_accel_unladen']), cap))


def stations_file(context):
    """The stations file for this run, resolved here rather than in the node.

    An empty launch argument must not reach the node: it would replace the
    node's own default with an empty string and fail on the first open. So the
    package default is resolved here, where the fallback is visible.
    """
    given = LaunchConfiguration('stations_file').perform(context)
    if given:
        path = Path(given)
        if not path.is_file():
            raise RuntimeError(f'no stations file at {path}')
        return str(path)
    return str(Path(get_package_share_directory('amr_mission'))
               / 'config' / 'stations.yaml')


def make_node(context):
    laden, unladen = accel_limits(LaunchConfiguration('platform').perform(context))

    return [Node(
        package='amr_mission', executable='transport_task',
        name='transport_task', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cycles': LaunchConfiguration('cycles'),
            'handling_time_s': LaunchConfiguration('handling_time_s'),
            # STATIONS BELONG TO A WORLD. They are poses in a specific building,
            # so running one world's stations in another sends the vehicle to
            # coordinates that are inside a wall or outside the shell entirely.
            # The test track generates its own alongside the world.
            'stations_file': stations_file(context),
            # WITH MAXIMUM PAYLOAD and unladen respectively. On a platform
            # whose manual publishes a single acceleration rating, as the
            # MP-400's does, these are equal and the switching is a no-op,
            # which is the correct behaviour rather than a special case.
            'accel_laden': laden,
            'accel_unladen': unladen,
        }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='3'),
        DeclareLaunchArgument('handling_time_s', default_value='5.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Must match the platform the stack was brought up with. run_stack.sh
        # passes the same value to every launch that needs it.
        DeclareLaunchArgument('platform', default_value='mir250_class'),
        DeclareLaunchArgument(
            'stations_file', default_value='',
            description='absolute path; empty uses the package default'),
        OpaqueFunction(function=make_node),
    ])
