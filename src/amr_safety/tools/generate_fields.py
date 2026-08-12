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


def rear_polygon(reach, half_width, half_length):
    """The same rectangle, extended BACKWARD instead."""
    back = -(half_length + reach)
    return [
        (half_length, half_width), (half_length, -half_width),
        (back, -half_width), (back, half_width),
    ]


def all_round_polygon(reach, half_width_raw, half_length):
    """The footprint grown in every direction, for turning on the spot.

    A spot turn sweeps the vehicle's corners through space with no linear
    velocity at all, so neither a forward nor a rear field covers what actually
    moves. The corner furthest from the turn centre is the circumscribed radius,
    and this grows the whole footprint by the distance that corner travels
    before the vehicle can be stopped.

    TAKES THE RAW HALF WIDTH, not the supplemented one, and the distinction is
    not cosmetic. `reach` is a stopping distance and already contains the
    scanner's 65 mm protective field supplement. The forward fields add that
    supplement laterally on its own, because their sides are not a stopping
    distance. Growing the ALREADY supplemented half width by a reach that also
    contains it applied the supplement twice: the field for a vehicle creeping
    almost straight came out 0.432 m half width instead of 0.367 m, which is a
    third wider than it should be, on the single dimension that decides whether
    the vehicle fits an aisle.
    """
    return [
        (half_length + reach, half_width_raw + reach),
        (half_length + reach, -(half_width_raw + reach)),
        (-(half_length + reach), -(half_width_raw + reach)),
        (-(half_length + reach), half_width_raw + reach),
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
    half_width_raw = v['chassis_width'] / 2.0
    half_width = half_width_raw + v['scanner_protective_supplement']

    # THE BANDS MUST COVER EVERY VELOCITY THE VEHICLE CAN BE COMMANDED.
    #
    # If a commanded velocity falls outside every band, nav2_collision_monitor
    # logs "Velocity is not covered by any of the velocity polygons" and then
    # publishes NOTHING. It does not pass the command through and it does not
    # stop the vehicle: the output topic goes silent and the wheels get no
    # command at all. Found when navigation commanded -0.15 m/s to reverse: the
    # controller and the smoother were both publishing at 20 Hz and
    # /diff_drive_controller/cmd_vel had no publisher at all.
    #
    # So the forward bands below are not the whole story. Two more cases exist
    # and both are ordinary, not exotic:
    #
    #   REVERSING. The controller may reverse out of a dead end and the backup
    #   recovery reverses by design. Uncovered, both would go silent.
    #
    #   TURNING ON THE SPOT. A differential drive rotates with zero linear
    #   velocity, and the platform's maximum is 1.5 rad/s. The forward bands
    #   previously declared theta_min/theta_max of -1.0 to 1.0, so any turn
    #   faster than 1 rad/s fell through the gap.
    #
    # Every band's angular range is therefore the platform's full angular
    # capability, and reverse and spot-turn bands are added explicitly.
    w_max = v['max_angular_speed']
    bands = [0.30, 0.80, 1.40, v['max_linear_speed']]
    stop_polys, warn_polys = {}, {}
    lines = []

    for i, vmax in enumerate(bands):
        reach = stopping_distance(vmax, v)
        # The lowest forward band starts at the spot-turn band's upper edge, so
        # the two do not overlap. Overlapping ranges are not an error to the
        # monitor, it simply takes the first match, which would silently give a
        # rotating vehicle a forward-only field.
        vmin = 0.05 if i == 0 else bands[i - 1]
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
            f'  forward {vmin:.2f} to {vmax:.2f} m/s: stop reach {reach:.3f} m, '
            f'warn reach {reach + decel_dist + 0.30:.3f} m')

    # REVERSE. Sized on the reverse speed limit, which is lower than forward
    # because the cameras face forward and a reversing vehicle is navigating on
    # 2D scan alone. The rear scanner covers this arc.
    v_rev = v['max_reverse_speed']
    rev_reach = stopping_distance(v_rev, v)
    rev_decel = v_rev * v_rev / (2.0 * v['emergency_decel'])
    stop_polys['stop_reverse'] = {
        'points': rear_polygon(rev_reach, half_width, half_length),
        'linear_min': -v_rev, 'linear_max': 0.0,
    }
    warn_polys['warn_reverse'] = {
        'points': rear_polygon(rev_reach + rev_decel + 0.30, half_width, half_length),
        'linear_min': -v_rev, 'linear_max': 0.0,
    }
    lines.append(
        f'  reverse {-v_rev:.2f} to 0.00 m/s: stop reach {rev_reach:.3f} m, '
        f'warn reach {rev_reach + rev_decel + 0.30:.3f} m')

    # ROTATING WITH NO LINEAR SPEED. There is no linear velocity to put into
    # the ISO 13855 formula, so the speed that matters is the tangential speed
    # of the corner furthest from the turn centre: w * circumscribed radius.
    #
    # THIS IS SPLIT IN TWO, and the reason is the most instructive failure in
    # this file.
    #
    # A single all-round field covering every rotation rate up to 1.5 rad/s
    # reaches 0.70 m to each side. The requirements call for working in a
    # 1.00 m dynamic corridor, whose half width is 0.50 m, so that field is
    # inside the racking at all times. The consequence was not a slow robot, it
    # was a DEADLOCKED one: the controller starts from rest, its first commands
    # are a few millimetres per second, anything under 0.05 m/s selected the
    # all-round field, the field was violated by the racking, so the vehicle was
    # held stopped, so it never left the speed band that held it. Measured, the
    # controller published 0.011 m/s for three minutes while the vehicle
    # travelled 0.28 m.
    #
    # So the near-zero linear band is split by ROTATION RATE. Creeping and
    # turning gently gets a field sized for that gentle turn, which fits the
    # corridor. Turning hard gets the big field, and a vehicle that wants to
    # turn hard in a narrow aisle is correctly stopped until it slows down,
    # which is exactly the speed-switching behaviour the bands exist to provide.
    r_circ = math.hypot(half_length, v['chassis_width'] / 2.0)

    # The threshold is DERIVED, not chosen: the fastest rotation whose field
    # still fits inside the dynamic corridor. Solved numerically so it tracks
    # the spec rather than being a number someone liked.
    # The corridor widths are VALIDATION TARGETS, published figures the model
    # is checked against, not free parameters. Reading the threshold from them
    # is deliberate: it ties the field geometry to the specification the vehicle
    # claims to meet, so widening a field silently breaks a stated capability.
    corridor_dynamic = spec['validation_targets']['corridor_width_dynamic']
    corridor_half = corridor_dynamic / 2.0
    allowed_reach = corridor_half - half_width_raw
    if allowed_reach <= 0:
        sys.exit('the field is already wider than the dynamic corridor; '
                 'the lateral supplement or the corridor requirement is wrong')
    lo, hi = 0.0, w_max
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if stopping_distance(mid * r_circ, v) <= allowed_reach:
            lo = mid
        else:
            hi = mid
    w_slow = lo

    # A LADDER OF ROTATION RATES, not one band.
    #
    # Sizing the whole near-zero-linear range by its fastest rotation was still
    # wrong, just less obviously. It gave a vehicle creeping straight ahead the
    # field of a vehicle spinning at 0.53 rad/s: a rectangle 1.09 m by 1.00 m
    # centred on the robot. Measured on the running system, one grazing return
    # from a rack corner BEHIND the vehicle landed at (-0.434, -0.499), one
    # millimetre inside the 0.500 m half width, and protective-stopped a machine
    # that was creeping forwards at 0.019 m/s away from it. Because it could not
    # accelerate, MPPI stayed acceleration-limited at one step above zero and
    # the vehicle covered 0.10 m in 45 seconds.
    #
    # The field now steps with the rotation rate, so a vehicle that is barely
    # turning gets a field barely larger than itself and only a vehicle really
    # spinning gets the big one. This is what field-set switching on a real
    # safety controller does: a finite set of field pairs selected by how the
    # vehicle is actually moving, not by the worst case it might.
    #
    # The innermost step is symmetric about zero; the rest come in mirrored
    # pairs, because a polygon carries one angular interval and left and right
    # rotation need separate entries.
    edges = sorted({0.10, 0.25, round(w_slow, 4), w_max})
    rotation_bands = {'rot_0': (-edges[0], edges[0], edges[0])}
    for k in range(1, len(edges)):
        lo, hi = edges[k - 1], edges[k]
        rotation_bands[f'rot_{k}_ccw'] = (lo, hi, hi)
        rotation_bands[f'rot_{k}_cw'] = (-hi, -lo, hi)

    for label, (tmin, tmax, w_ref) in rotation_bands.items():
        v_tip = w_ref * r_circ
        reach = stopping_distance(v_tip, v)
        stop_polys[f'stop_{label}'] = {
            'points': all_round_polygon(reach, half_width_raw, half_length),
            # Linear range is a hair either side of zero rather than exactly
            # zero, because a band of zero width can be missed by a floating
            # point comparison, which would put us back to a silent monitor.
            'linear_min': -0.05, 'linear_max': 0.05,
            'theta_min': tmin, 'theta_max': tmax,
        }
        warn_polys[f'warn_{label}'] = {
            'points': all_round_polygon(reach + 0.20, half_width_raw, half_length),
            'linear_min': -0.05, 'linear_max': 0.05,
            'theta_min': tmin, 'theta_max': tmax,
        }
        lines.append(
            f'  {label} {tmin:+.2f} to {tmax:+.2f} rad/s at |vx| <= 0.05: tip '
            f'speed {v_tip:.3f} m/s at r {r_circ:.3f} m, stop reach '
            f'{reach:.3f} m all round, half width {half_width_raw + reach:.3f} m')
    lines.append(
        f'  rotation threshold derived as {w_slow:.3f} rad/s, the fastest turn '
        f'whose field still fits the {corridor_dynamic:.2f} m '
        f'dynamic corridor')

    # ORDER MATTERS. The monitor takes the FIRST polygon whose ranges contain
    # the commanded velocity, so the most specific cases go first: spot turn,
    # then reverse, then the forward bands in increasing speed.
    def reorder(polys, special):
        return {k: polys[k] for k in special + [k for k in polys
                                                if k not in special]}

    # Rotation bands first, narrowest angular interval first, then reverse,
    # then the forward bands. The monitor takes the first match.
    rot_order = sorted((k for k in stop_polys if k.startswith('stop_rot_')),
                       key=lambda k: abs(stop_polys[k]['theta_max']
                                         - stop_polys[k]['theta_min']))
    stop_polys = reorder(stop_polys, rot_order + ['stop_reverse'])
    warn_order = [k.replace('stop_', 'warn_', 1) for k in rot_order]
    warn_polys = reorder(warn_polys, warn_order + ['warn_reverse'])

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
                    # Angular range defaults to the platform's full capability
                    # so no turn falls outside every band, but a polygon that
                    # declares its own range keeps it: that is how the rotation
                    # bands are kept apart.
                    **{k: {'points': as_param(val['points']),
                           'linear_min': val['linear_min'],
                           'linear_max': val['linear_max'],
                           'theta_min': val.get('theta_min', -w_max),
                           'theta_max': val.get('theta_max', w_max)}
                       for k, val in stop_polys.items()},
                },
                'warning': {
                    'type': 'velocity_polygon',
                    # A SPEED LIMIT, NOT A MULTIPLIER, and this is the single
                    # most important line in the file.
                    #
                    # `slowdown` scales whatever the controller asked for by a
                    # ratio. Downstream of a controller that is acceleration
                    # limited and closes its loop on MEASURED velocity, that
                    # creates a stable trap. MPPI commands v_actual + a*dt. The
                    # monitor returns ratio * that. The vehicle reaches it, so
                    # the next command is barely higher, and the loop settles
                    # at a fixed point:
                    #
                    #     v = r * (v + a*dt)   ->   v = r*a*dt / (1 - r)
                    #
                    # With r = 0.3, a = 0.3 m/s2 and dt = 0.05 s that is
                    # 0.0064 m/s. Measured on the running system: commanded
                    # 0.0128 m/s, actual 0.0026 to 0.0064 m/s, 0.10 m travelled
                    # in 45 seconds. The vehicle was not being stopped. It was
                    # being asymptotically throttled to a standstill by a
                    # warning field doing exactly what it was configured to do.
                    #
                    # `limit` caps the speed instead. The controller ramps up to
                    # the cap and holds it, which is what "slow down near
                    # people" is supposed to mean and what a real safety
                    # controller does when it selects a reduced-speed field set.
                    'action_type': 'limit',
                    # The top of the slowest forward band: the speed whose
                    # protective field fits an aisle with room to spare. Derived
                    # from the band edges rather than picked.
                    'linear_limit': bands[0],
                    # The fastest rotation whose field still fits the dynamic
                    # corridor, the same figure the rotation ladder is built on.
                    'angular_limit': round(w_slow, 4),
                    'min_points': 2,
                    'visualize': True,
                    'enabled': True,
                    'polygon_pub_topic': 'warning_field',
                    'velocity_polygons': list(warn_polys),
                    'holonomic': False,
                    **{k: {'points': as_param(val['points']),
                           'linear_min': val['linear_min'],
                           'linear_max': val['linear_max'],
                           'theta_min': val.get('theta_min', -w_max),
                           'theta_max': val.get('theta_max', w_max)}
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
