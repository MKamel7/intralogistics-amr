"""Bring up N vehicles in one world, each in its own namespace.

THIS DOES NOT RUN YET, AND THE REASON IS WORTH READING BEFORE USING IT.

It passes `namespace`, `tf_prefix` and `start_simulator` to
amr_bringup/robot.launch.py, and that file declares none of them. Every node
in this project publishes on fixed topic names, and the robot description
emits fixed frame names, so a second vehicle launched today does not run
beside the first, it fights it: both drive /cmd_vel_raw, both write /map, and
both publish odom -> base_link into one TF tree. The symptom is a vehicle that
appears to teleport between two positions, which reads as a localisation fault
and is not one.

The design here is complete and the dispatcher it serves is tested. What is
missing is namespacing the bringup, which touches the description (frame
prefixes), Nav2, the collision monitor, the scan merger and the spawn path.
That is a substantial piece of work and it is NOT done, so this file is kept
as the specification for it rather than presented as a working entry point.
`test_fleet_launch.py` asserts the gap and will fail the moment it closes,
which is the signal to remove this paragraph.


WHY NAMESPACES AND NOT N COPIES OF THE STACK

Every node in this project publishes on fixed topic names: /scan, /map,
/cmd_vel_raw. Two vehicles launched without namespaces do not run side by
side, they fight: both drive the same /cmd_vel, both write the same /map, and
the result looks like one robot behaving erratically rather than two robots
misconfigured. That failure is silent, which is why this file exists rather
than a shell loop over run_stack.sh.

WHAT IS SHARED AND WHAT IS NOT

    shared      the world, the clock, and the ground truth oracle. One
                simulator, one truth, because two would defeat the point.

    per vehicle namespace, TF prefix, robot description, Nav2 stack,
                collision monitor, and VDA 5050 interface with its own
                serial number.

THE TF PREFIX IS THE PART THAT BITES. Without it both robots publish
odom -> base_link, the tree has two nodes with the same name and different
parents, and every transform lookup returns whichever arrived last. The
symptom is a vehicle that appears to teleport between two positions, which
reads as a localisation fault and is not one.

    ros2 launch amr_fleet fleet.launch.py vehicles:=2 platform:=mp400_class
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction

# Spawn poses are spread along the open bay so two vehicles do not start
# inside each other's protective field, which would leave both stopped before
# either had a goal. Derived from the generated stations file at runtime would
# be better and is the obvious next step; these are the bay's own coordinates.
SPAWNS = [(2.5, 6.0), (2.5, 8.5), (2.5, 3.5), (5.0, 6.0)]


def vehicles(context, *_, **__):
    n = int(LaunchConfiguration('vehicles').perform(context))
    platform = LaunchConfiguration('platform').perform(context)
    world = LaunchConfiguration('world').perform(context)
    if n > len(SPAWNS):
        raise RuntimeError(
            f'{n} vehicles requested but only {len(SPAWNS)} spawn poses are '
            f'defined; adding more without checking they clear each other\'s '
            f'protective fields would leave them stopped on top of each other')

    actions = []
    for i in range(n):
        name = f'{platform.split("_")[0]}-{i + 1:02d}'
        x, y = SPAWNS[i]
        actions.append(GroupAction([
            PushRosNamespace(name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('amr_bringup'), 'launch', 'robot.launch.py'])),
                launch_arguments={
                    'platform': platform,
                    'world': world,
                    'x': str(x), 'y': str(y),
                    'gui': 'true' if i == 0 else 'false',
                    'rviz': 'false',
                    'cameras': 'false',
                    # ONE SIMULATOR. Only the first vehicle starts the world;
                    # the rest spawn into it. Two simulators would give two
                    # clocks, and simulated time jumping backwards clears every
                    # TF buffer several times a second.
                    'start_simulator': 'true' if i == 0 else 'false',
                    'namespace': name,
                    'tf_prefix': name,
                }.items()),
            # The vehicle's own VDA 5050 interface, one serial per robot.
            Node(package='amr_fleet', executable='vda5050_bridge',
                 name='vda5050_bridge', output='screen',
                 parameters=[{'use_sim_time': True,
                              'manufacturer': 'Neobotix',
                              'serial_number': name}]),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicles', default_value='2'),
        DeclareLaunchArgument('platform', default_value='mp400_class'),
        DeclareLaunchArgument('world', default_value='test_track.mp400_class'),
        OpaqueFunction(function=vehicles),
    ])
