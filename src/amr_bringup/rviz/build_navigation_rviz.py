#!/usr/bin/env python3
"""Generate the navigation RViz layout.

WHY GENERATED RATHER THAN HAND EDITED

An .rviz file is nine hundred lines of nested YAML in which a display's colour,
its topic and its QoS sit two hundred lines apart. Editing one by hand is how
you end up with a display subscribed to a topic that no longer exists, showing
nothing, with no error. Written here, each display is six lines and the intent
is legible.

WHAT THIS VIEW IS FOR

It answers one question: WHY DID THE VEHICLE DO THAT. Every layer is chosen to
show a decision, in the order the decision is made.

    the map SLAM has built              what the vehicle knows
    the global costmap                  what it treats as blocked, inflated by
                                        its own size
    the global plan                     the route it chose through that
    the candidate trajectories          the several thousand futures MPPI
                                        considered this cycle
    the optimal trajectory              the one it picked
    the local costmap                   what it can see right now, including
                                        people who were not there a moment ago
    the protective and warning fields   the speed-switched shapes that will
                                        stop it regardless of the above
    tracked people                      what the perception stack believes

Seeing the candidate trajectories bend around a pedestrian and the chosen one
follow them, while the protective field holds its distance, is the entire
argument of this project in one picture.
"""

from pathlib import Path

import yaml

OUT = Path(__file__).resolve().parent / 'navigation.rviz'

# Colours are picked so the layers stay distinguishable when several overlap,
# which they do constantly around the vehicle.
GLOBAL_PLAN = '25; 175; 65'      # green, the committed route
OPTIMAL = '255; 170; 0'          # amber, the trajectory being driven now
CANDIDATES = '120; 170; 255'     # pale blue, the futures considered
PROTECTIVE = '220; 40; 40'       # red, stop
WARNING = '245; 200; 40'         # yellow, slow
FOOTPRINT = '255; 255; 255'


def display(cls, name, topic=None, **kw):
    d = {'Class': cls, 'Name': name, 'Enabled': kw.pop('enabled', True)}
    if topic is not None:
        d['Topic'] = {
            'Depth': kw.pop('depth', 5),
            'Durability Policy': kw.pop('durability', 'Volatile'),
            'Reliability Policy': kw.pop('reliability', 'Reliable'),
            'History Policy': 'Keep Last',
            'Filter size': 10,
            'Value': topic,
        }
    d.update(kw)
    return d


def path(name, topic, colour, width, **kw):
    return display(
        'rviz_default_plugins/Path', name, topic,
        **{'Line Style': 'Lines', 'Color': colour, 'Alpha': kw.pop('alpha', 1.0),
           'Buffer Length': 1, 'Offset': {'X': 0, 'Y': 0, 'Z': kw.pop('z', 0.02)},
           'Pose Style': 'None', 'Head Diameter': 0.1, 'Head Length': 0.1,
           'Shaft Diameter': 0.05, 'Shaft Length': 0.1,
           'Line Width': width, **kw})


def polygon(name, topic, colour, **kw):
    return display('rviz_default_plugins/Polygon', name, topic,
                   Color=colour, Alpha=kw.pop('alpha', 1.0), **kw)


def costmap(name, topic, alpha, scheme='costmap', enabled=True):
    return display(
        'rviz_default_plugins/Map', name, topic,
        durability='Transient Local', enabled=enabled,
        **{'Color Scheme': scheme, 'Alpha': alpha, 'Draw Behind': False,
           'Update Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                            'History Policy': 'Keep Last',
                            'Reliability Policy': 'Reliable',
                            'Value': topic + '_updates'}})


def build():
    displays = [
        {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid', 'Enabled': True,
         'Cell Size': 1, 'Plane Cell Count': 40, 'Color': '80; 80; 80',
         'Alpha': 0.3, 'Line Style': {'Line Width': 0.03, 'Value': 'Lines'},
         'Plane': 'XY', 'Reference Frame': '<Fixed Frame>'},

        # WHAT THE VEHICLE KNOWS. The SLAM map underneath everything.
        costmap('SLAM map', '/map', 0.7, scheme='map'),

        # WHAT IT TREATS AS BLOCKED. The inflation here is the vehicle's own
        # size: the reason a legal-looking gap is refused is visible in this
        # layer and nowhere else.
        costmap('Global costmap', '/global_costmap/costmap', 0.45),
        costmap('Local costmap', '/local_costmap/costmap', 0.65),

        # GROUND TRUTH, off by default. Turning it on next to the SLAM map is
        # how the mapping quality is judged by eye; score_map.py does it by
        # number. It is never an input to navigation.
        costmap('Ground truth map (evaluation only)', '/ground_truth/map', 0.5,
                scheme='map', enabled=False),

        # THE ROUTE IT CHOSE.
        path('Global plan', '/plan', GLOBAL_PLAN, 0.06, z=0.03),

        # THE FUTURES IT CONSIDERED, this cycle. This is the display that
        # explains obstacle avoidance: the cloud visibly bends away from a
        # pedestrian before the vehicle has turned.
        display('rviz_default_plugins/MarkerArray', 'MPPI candidate trajectories',
                '/trajectories', Namespaces={}),

        # THE ONE IT PICKED.
        path('MPPI chosen trajectory', '/optimal_trajectory', OPTIMAL, 0.09, z=0.04),

        # THE SAFETY SHAPES. These are not navigation and do not consult it.
        polygon('Protective field (stop)', '/protective_field', PROTECTIVE),
        polygon('Warning field (limit speed)', '/warning_field', WARNING, alpha=0.8),
        display('rviz_default_plugins/MarkerArray', 'Collision points',
                '/collision_monitor/collision_points_marker', Namespaces={}),

        # THE VEHICLE.
        {'Class': 'rviz_default_plugins/RobotModel', 'Name': 'Robot', 'Enabled': True,
         'Description Topic': {'Depth': 5, 'Durability Policy': 'Transient Local',
                               'History Policy': 'Keep Last',
                               'Reliability Policy': 'Reliable',
                               'Value': '/robot_description'},
         'Alpha': 1.0, 'Collision Enabled': False, 'Visual Enabled': True,
         'Update Interval': 0, 'Links': {'All Links Enabled': True,
                                         'Expand Link Details': False,
                                         'Expand Joint Details': False,
                                         'Expand Tree': False,
                                         'Link Tree Style': 'Links in Alphabetic Order'}},
        polygon('Footprint', '/local_costmap/published_footprint', FOOTPRINT, alpha=0.9),

        # WHAT IT SEES.
        display('rviz_default_plugins/LaserScan', 'Merged scan', '/scan',
                reliability='Best Effort',
                **{'Size (m)': 0.03, 'Style': 'Flat Squares', 'Alpha': 1.0,
                   'Decay Time': 0, 'Color Transformer': 'FlatColor',
                   'Color': '255; 90; 200', 'Position Transformer': 'XYZ',
                   'Queue Size': 10}),

        # WHAT THE PERCEPTION STACK BELIEVES. Deliberately last, and
        # deliberately NOT wired to anything safety critical.
        display('rviz_default_plugins/MarkerArray', 'Tracked people',
                '/people_markers', Namespaces={}),

        {'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': False,
         'Show Names': True, 'Show Axes': True, 'Show Arrows': False,
         'Marker Scale': 0.4, 'Update Interval': 0, 'Frame Timeout': 15,
         'Frames': {'All Enabled': False}, 'Tree': {}},
    ]

    return {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Name': 'Displays',
             'Property Tree Widget': {'Expanded': [
                 '/Global Options1'], 'Splitter Ratio': 0.62},
             'Tree Height': 780},
            {'Class': 'rviz_common/Views', 'Name': 'Views',
             'Expanded': [''], 'Splitter Ratio': 0.5},
            # The tool panel carries "2D Goal Pose", which is how a goal is
            # given by hand during a demonstration.
            {'Class': 'rviz_common/Tool Properties', 'Name': 'Tool Properties',
             'Expanded': ['/2D Goal Pose1'], 'Splitter Ratio': 0.6},
        ],
        'Visualization Manager': {
            'Class': '',
            'Name': 'root',
            'Displays': displays,
            'Enabled': True,
            'Global Options': {
                'Background Color': '38; 40; 44',
                'Fixed Frame': 'map',
                'Frame Rate': 30,
            },
            'Tools': [
                {'Class': 'rviz_default_plugins/MoveCamera'},
                {'Class': 'rviz_default_plugins/Select'},
                {'Class': 'rviz_default_plugins/FocusCamera'},
                {'Class': 'rviz_default_plugins/Measure', 'Line color': '128; 128; 0'},
                {'Class': 'rviz_default_plugins/SetInitialPose',
                 'Covariance x': 0.25, 'Covariance y': 0.25,
                 'Covariance yaw': 0.068,
                 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                           'History Policy': 'Keep Last',
                           'Reliability Policy': 'Reliable',
                           'Value': '/initialpose'}},
                {'Class': 'rviz_default_plugins/SetGoal',
                 'Topic': {'Depth': 5, 'Durability Policy': 'Volatile',
                           'History Policy': 'Keep Last',
                           'Reliability Policy': 'Reliable',
                           'Value': '/goal_pose'}},
            ],
            'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
            'Value': True,
            'Views': {
                'Current': {
                    # Looking straight down from 22 m, which frames the whole
                    # 15 by 22 m building. A perspective view looks better in a
                    # screenshot and is useless for judging whether a path
                    # clears a rack.
                    'Class': 'rviz_default_plugins/TopDownOrtho',
                    'Name': 'Top down',
                    'Target Frame': 'map',
                    'Scale': 38,
                    'Angle': 0,
                    'X': 0, 'Y': 0,
                    'Invert Z Axis': False,
                    'Near Clip Distance': 0.01,
                },
                'Saved': [
                    {'Class': 'rviz_default_plugins/ThirdPersonFollower',
                     'Name': 'Chase the vehicle',
                     'Target Frame': 'base_link',
                     'Distance': 9, 'Pitch': 0.9, 'Yaw': 3.14,
                     'Focal Point': {'X': 0, 'Y': 0, 'Z': 0},
                     'Near Clip Distance': 0.01},
                ],
            },
        },
        'Window Geometry': {
            'Displays': {'collapsed': False},
            'Height': 1000,
            'Width': 1720,
            'Hide Left Dock': False,
            'Hide Right Dock': True,
            'QMainWindow State': '',
        },
    }


if __name__ == '__main__':
    OUT.write_text(yaml.safe_dump(build(), default_flow_style=False, sort_keys=False))
    print(f'wrote {OUT}')
