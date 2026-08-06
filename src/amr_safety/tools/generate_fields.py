#!/usr/bin/env python3
"""Generate protective and warning field geometry from the platform spec.

A protective field is only as good as the stopping distance behind it, so the
fields are CALCULATED rather than chosen. For each speed band:

    S = v * (t_scanner + t_control + t_brake)   reaction, the vehicle keeps going
      + v^2 / (2 * a)                           braking
      + C                                       the scanner's field supplement

which is the ISO 13855 shape of the calculation, applied to a vehicle. The
supplement is the SICK sheet's own 65 mm protective field supplement, which
exists because a scanner cannot resolve an object exactly at the field boundary.

The field is then that distance ahead of the FRONT FACE, so from the robot
centre it is half the chassis length further out.

Speed-dependent switching is the point of the bands. A single field sized for
2 m/s would be 2.2 m from the centre and the robot could never work in an aisle;
a single field sized for creeping would not stop it at speed. Real safety
controllers switch fields on measured speed and so does this.

Output is a nav2_collision_monitor configuration. Nothing here is hand-tuned:
change the spec, regenerate.
"""

import argparse
import math
import sys
from pathlib import Path

import yaml

SPEC_DIR = (Path(__file__).resolve().parents[2]
            / 'amr_description' / 'config' / 'platforms')


def stopping_distance(v, spec):
    """Reaction plus braking plus supplement, in metres."""
    t_react = (spec['scanner_response_time']
               + spec['control_latency']
               + spec['brake_actuation_delay'])
    return (v * t_react
            + v * v / (2.0 * spec['emergency_decel'])
            + spec['scanner_protective_supplement'])


def as_param(points):
    """Nav2 wants polygon points as a STRING, not a numeric array.

    Passing a double array configures cleanly right up to the point where the
    node refuses it: "parameter points has invalid type ... is of type {string},
    setting it to {double_array} is not allowed". Found by running the monitor
    standalone; the lifecycle manager only reported "failed to change state".
    """
    # Round OUTWARD, never to nearest. Rounding a protective field to the
    # nearest 0.1 mm can make it smaller than the stopping distance it was
    # derived from, and a field one micron short is a field that does not meet
    # its own requirement. Caught by the test that checks each field covers its
    # own stopping distance: at 0.8 m/s it read 0.454 m against 0.454 m needed.
    def out(value):
        return math.copysign(math.ceil(abs(value) * 1e4) / 1e4, value)

    return '[' + ', '.join(f'[{out(x):.4f}, {out(y):.4f}]' for x, y in points) + ']'


def field_polygon(reach, half_width, half_length):
    """A rectangle ahead of the vehicle, squared off at its own footprint.

    Deliberately not a shape tuned to look tidy: it is the footprint widened by
    the lateral margin and extended forward by the stopping distance.
    """
    front = half_length + reach
    return [
        (front, half_width), (front, -half_width),
        (-half_length, -half_width), (-half_length, half_width),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--platform', default='mir250_class')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()

    spec = yaml.safe_load((SPEC_DIR / f'{args.platform}.yaml').read_text())
    v = spec['values']

    half_length = v['chassis_length'] / 2.0
    # Lateral margin: the vehicle's own half width plus the field supplement.
    # A protective field narrower than the vehicle would let it drive its own
    # corner into something the field never covered.
    half_width = v['chassis_width'] / 2.0 + v['scanner_protective_supplement']

    bands = [0.30, 0.80, 1.40, v['max_linear_speed']]
    stop_polys, warn_polys = {}, {}
    lines = []

    for i, vmax in enumerate(bands):
        reach = stopping_distance(vmax, v)
        vmin = 0.0 if i == 0 else bands[i - 1]
        name = f'stop_{int(vmax * 100):03d}'
        stop_polys[name] = {
            'points': field_polygon(reach, half_width, half_length),
            'linear_min': vmin, 'linear_max': vmax,
        }
        # The warning field must give the vehicle room to slow to the next band
        # down BEFORE the protective field is breached, so it is sized on the
        # stopping distance at this speed plus the distance covered while
        # decelerating. Doubling would be arbitrary; this is derived.
        decel_dist = (vmax * vmax - (vmin * vmin)) / (2.0 * v['emergency_decel'])
        warn_polys[f'warn_{int(vmax * 100):03d}'] = {
            'points': field_polygon(reach + decel_dist + 0.30, half_width, half_length),
            'linear_min': vmin, 'linear_max': vmax,
        }
        lines.append(
            f'  {vmin:.2f} to {vmax:.2f} m/s: stop reach {reach:.3f} m, '
            f'warn reach {reach + decel_dist + 0.30:.3f} m')

    cfg = {
        'collision_monitor': {
            'ros__parameters': {
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                # diff_drive_controller 4.x consumes TwistStamped only. The
                # monitor defaults to plain Twist, and the failure is silent
                # and total: both types appear on the same topic, the
                # controller ignores the one it does not want, and the robot
                # simply never moves with no error anywhere. Found by noticing
                # `ros2 topic hz` refusing the topic because it carried two
                # types.
                'enable_stamped_cmd_vel': True,
                'cmd_vel_in_topic': 'cmd_vel_raw',
                'cmd_vel_out_topic': 'diff_drive_controller/cmd_vel',
                'state_topic': 'collision_monitor_state',
                'transform_tolerance': 0.2,
                'source_timeout': 0.3,
                # A source that stops publishing must STOP the vehicle, not be
                # ignored. Default is permissive; safety is not.
                'base_shift_correction': True,
                'stop_pub_timeout': 2.0,
                'polygons': ['protective', 'warning'],
                'protective': {
                    'type': 'velocity_polygon',
                    'action_type': 'stop',
                    'min_points': 2,
                    'visualize': True,
                    'enabled': True,
                    'polygon_pub_topic': 'protective_field',
                    'velocity_polygons': list(stop_polys),
                    'holonomic': False,
                    **{k: {'points': as_param(val['points']),
                           'linear_min': val['linear_min'],
                           'linear_max': val['linear_max'],
                           'theta_min': -1.0, 'theta_max': 1.0}
                       for k, val in stop_polys.items()},
                },
                'warning': {
                    'type': 'velocity_polygon',
                    'action_type': 'slowdown',
                    'slowdown_ratio': 0.3,
                    'min_points': 2,
                    'visualize': True,
                    'enabled': True,
                    'polygon_pub_topic': 'warning_field',
                    'velocity_polygons': list(warn_polys),
                    'holonomic': False,
                    **{k: {'points': as_param(val['points']),
                           'linear_min': val['linear_min'],
                           'linear_max': val['linear_max'],
                           'theta_min': -1.0, 'theta_max': 1.0}
                       for k, val in warn_polys.items()},
                },
                'observation_sources': ['merged_scan'],
                'merged_scan': {
                    # The MERGED SCAN, never the people detector. A protective
                    # function must not depend on classification: measured
                    # precision of the classifier is about 0.18, while returns
                    # from a pedestrian are present in 100 percent of frames.
                    # See V-11 in docs/validation.md.
                    'type': 'scan',
                    'topic': 'scan',
                    'enabled': True,
                },
            }
        }
    }

    text = ('# GENERATED by amr_safety/tools/generate_fields.py from the\n'
            '# platform spec. Do not hand-edit: change the spec and regenerate.\n'
            '#\n# Derived field reaches:\n'
            + '\n'.join(f'#{line}' for line in lines) + '\n\n'
            + yaml.safe_dump(cfg, sort_keys=False, default_flow_style=False))

    if args.out:
        args.out.write_text(text)
        print(f'wrote {args.out}')
    else:
        print(text)
    for line in lines:
        print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
