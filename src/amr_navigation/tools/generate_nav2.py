#!/usr/bin/env python3
"""Generate the Nav2 configuration from the platform spec.

WHY THIS EXISTS. `collision_monitor.yaml` has been generated from the platform
spec since the safety layer was built, so a change to the vehicle reaches the
protective fields automatically. `nav2.yaml` was not: it carried MiR250 derived
literals for the footprint, the speed limits, the inflation radius and the
local costmap size. That was invisible while there was one platform. With a
second one it means a vehicle can be given a correct body and a navigation
stack tuned for a machine 200 mm wider, which is worse than either alone
because everything looks configured.

HOW IT DIFFERS FROM generate_fields.py, AND WHY. That tool builds its output
structure in Python and dumps it, because a collision monitor configuration is
almost entirely derived geometry and its comments are a short header of derived
reaches. This file cannot work that way. Most of nav2.yaml is reasoning: which
planner, which progress checker, and the measured failure behind each choice.
Dumping a Python dict would delete all of it, and that reasoning is worth more
than the numbers it justifies.

So the source of truth is `config/nav2.yaml.in`, a full template that keeps
every comment, and this tool substitutes the values that depend on the
platform. Nothing here is hand-tuned: change the spec, regenerate.

WHAT IS DERIVED AND WHAT IS NOT. A number is derived here only if it is a
function of the vehicle. Critic weights, timeouts, plugin choices and the
sensor ranges of parts shared between platforms are NOT functions of the
vehicle, and pretending otherwise would be numerology. They stay in the
template as literals with their reasoning attached.

THE THREE COMMISSIONING CONSTANTS are declared below rather than buried. Each
is a decision rather than a measurement, each is stated once, and each is
applied to both platforms identically.
"""

import argparse
import math
import re
import sys
import textwrap
from pathlib import Path
from string import Template

import yaml

PKG = Path(__file__).resolve().parents[1]
TEMPLATE = PKG / 'config' / 'nav2.yaml.in'
SPEC_DIR = (Path(__file__).resolve().parents[2]
            / 'amr_description' / 'config' / 'platforms')

# ---- commissioning constants ---------------------------------------------
#
# COMMISSIONED SPEED. The vehicle is run at half its rated top speed. This is a
# commissioning decision and not a platform property: the protective field
# behind the rated speed reaches so far that the vehicle stops for a pedestrian
# in the next aisle and never finishes a job. Halving is the trade a real
# installation makes when it commissions a site, and it is applied as a
# fraction so it means the same thing on a 2.0 m/s vehicle and a 1.5 m/s one.
COMMISSIONED_SPEED_FRACTION = 0.5

# THE ORDINARY MOTION ENVELOPE, and it is SYMMETRIC.
#
# Braking is capped at two thirds of the emergency deceleration so the emergency
# rate stays a genuine reserve rather than a number the controller is already
# using. On the MP-400 that cap is doing real work: its unladen acceleration
# rating of 2.4 m/s2 is HIGHER than its 1.5 m/s2 emergency rate, so taken
# literally the controller would brake harder in normal driving than the
# protective fields assume it can in an emergency.
#
# ACCELERATION IS CAPPED AT THE SAME FIGURE, and that was learned the hard way.
# The first version of this file capped braking alone and left acceleration at
# the platform rating, on the reasoning that acceleration has no safety
# coupling. It does not, but it has a CONTROL coupling: a vehicle that can
# accelerate harder than it can brake cannot converge on a goal, because every
# trajectory MPPI samples towards the goal overshoots it.
#
# It was invisible on the MiR250, where min(1.0 rating, 0.667 x 1.5) is 1.0 and
# the envelope comes out symmetric BY COINCIDENCE. On the MP-400 it gave 2.4
# against 1.0, and the vehicle drove to within 0.02 m of a station at 0.7 m/s,
# failed to stop, and orbited it until the leg timed out. Measured with
# tools/track_goal.py: "CAME WITHIN 0.02 m and then moved away", commanding
# motion in 188 of 200 samples. See V-25.
ORDINARY_DECEL_FRACTION = 2.0 / 3.0

# INFLATION CLEARANCE. The inflation radius is the vehicle's circumscribed
# radius plus this band, so a planned path leaves room for a person to pass
# rather than technically fitting. The circumscribed radius is a platform
# property; the band is the decision.
INFLATION_CLEARANCE = 0.05

# ---- controller constants that other values are computed from -------------
# These live here rather than only in the template because the local costmap
# size is computed from the MPPI horizon. Two copies of the horizon would drift
# apart, and the drift would be silent: a costmap slightly too small for the
# horizon just makes the controller plan into space it cannot see.
MPPI_TIME_STEPS = 56
MPPI_MODEL_DT = 0.05
# Vertical resolution of the camera voxel layer. The voxel count is derived
# from it and the vehicle envelope height.
VOXEL_Z_RESOLUTION = 0.10
# Angular acceleration. NOT published in either platform spec, so it is a
# literal rather than a derivation dressed up as one.
AZ_MAX = 2.0


def footprint_param(half_x, half_y):
    """Nav2 wants the footprint as a STRING, like the monitor's polygons.

    Corners in the order nav2 expects, front left first, going clockwise.
    """
    pts = [(half_x, half_y), (half_x, -half_y), (-half_x, -half_y), (-half_x, half_y)]
    return '"[' + ', '.join(f'[{x:.4f}, {y:.4f}]' for x, y in pts) + ']"'


# ---- planner variants, for the comparison in docs -------------------------
#
# THREE CANDIDATES, AND ONE DELIBERATE OMISSION.
#
# Hybrid-A* is not here. It plans in x, y and heading under a minimum turning
# radius, which is what an Ackermann vehicle needs and what a differential
# drive does not have. Including it would produce a table where the project's
# own planner wins against a strawman, and a comparison you have rigged is
# worth less than no comparison.
#
# The three below all plan on a 2D costmap for a vehicle that turns on the
# spot, so the comparison is between things that could each legitimately be
# chosen for this machine.
PLANNERS = {
    'smac2d': {
        'plugin': 'nav2_smac_planner::SmacPlanner2D',
        'note': 'A* on the costmap with a cost aware travel multiplier',
        'extra': """      downsample_costmap: false
      allow_unknown: false
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10""",
    },
    'navfn': {
        'plugin': 'nav2_navfn_planner::NavfnPlanner',
        'note': 'Dijkstra on the costmap, the long standing ROS default',
        'extra': """      use_astar: false
      allow_unknown: false""",
    },
    'thetastar': {
        'plugin': 'nav2_theta_star_planner::ThetaStarPlanner',
        'note': 'any angle A*, fewer waypoints and straighter diagonals',
        'extra': """      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      allow_unknown: false""",
    },
}


def planner_block(name):
    """The GridBased body for one planner, as YAML at the right indent."""
    if name not in PLANNERS:
        raise SystemExit(f'unknown planner {name}; choose from {sorted(PLANNERS)}')
    p = PLANNERS[name]
    return (f'      # {p["note"]}\n'
            f'      plugin: "{p["plugin"]}"\n'
            f'      tolerance: 0.25\n'
            f'{p["extra"]}')


def derive(spec, platform):
    """Every platform-dependent value in the Nav2 configuration.

    Returns the substitution mapping and a list of human-readable derivation
    lines for the generated header, so the output states its own arithmetic.

    The platform NAME is passed in rather than read from the spec. The two
    specs disagree about what `platform` holds: mir250_class.yaml carries a
    mapping with an `id`, mp400_class.yaml carries a bare string. The file name
    is the one identifier that is unambiguous in both.
    """
    v = spec['values']

    # THE FOOTPRINT IS THE PODS, NOT THE CHASSIS. The scanner optical centres
    # sit at the envelope corners and 5 mm proud of them on both platforms, so
    # they, not the published chassis rectangle, are the outermost fixed
    # structure the planner has to fit through a gap.
    half_x = v['scanner_mount_x']
    half_y = v['scanner_mount_y']
    r_circ = math.hypot(half_x, half_y)
    r_inscribed = min(half_x, half_y)

    vx_max = COMMISSIONED_SPEED_FRACTION * v['max_linear_speed']
    # One figure for both directions. See the note on ORDINARY_DECEL_FRACTION:
    # an envelope that accelerates harder than it brakes cannot converge.
    ordinary_accel = min(v['max_linear_accel_unladen'],
                         ORDINARY_DECEL_FRACTION * v['emergency_decel'])
    ordinary_decel = ordinary_accel
    inflation_radius = r_circ + INFLATION_CLEARANCE

    # LOCAL COSTMAP SIZE. The controller looks one MPPI horizon ahead, so the
    # window has to hold that distance in front of the vehicle and the same
    # behind it, which is a square of twice the look-ahead. Rounded UP to a
    # whole metre, never down: a window shorter than the horizon lets the
    # controller score trajectories against cells it has no data for.
    horizon_s = MPPI_TIME_STEPS * MPPI_MODEL_DT
    lookahead = vx_max * horizon_s
    local_size = math.ceil(2.0 * lookahead)

    z_voxels = int(round(v['vehicle_envelope_height'] / VOXEL_Z_RESOLUTION))

    lines = [
        f"platform {platform}, chassis "
        f"{v['chassis_length'] * 1000:.0f} x {v['chassis_width'] * 1000:.0f} mm",
        f"footprint {half_x * 2000:.0f} x {half_y * 2000:.0f} mm, the scanner "
        f"optical centres, which stand proud of the chassis",
        f"circumscribed radius {r_circ:.4f} m, inscribed {r_inscribed:.4f} m",
        f"inflation radius {r_circ:.4f} + {INFLATION_CLEARANCE:.2f} clearance "
        f"= {inflation_radius:.4f} m",
        f"commissioned speed {COMMISSIONED_SPEED_FRACTION:.2f} x "
        f"{v['max_linear_speed']:.2f} m/s rated = {vx_max:.2f} m/s",
        f"ordinary envelope, both directions, "
        f"min({v['max_linear_accel_unladen']:.2f} unladen rating, "
        f"{ORDINARY_DECEL_FRACTION:.3f} x {v['emergency_decel']:.2f} emergency) "
        f"= {ordinary_accel:.2f} m/s2",
        f"local costmap 2 x {vx_max:.2f} m/s x {horizon_s:.2f} s horizon "
        f"= {2.0 * lookahead:.2f} m, rounded up to {local_size} m square",
        f"voxel layer {v['vehicle_envelope_height']:.2f} m envelope / "
        f"{VOXEL_Z_RESOLUTION:.2f} m = {z_voxels} voxels",
    ]

    values = {
        'platform': platform,
        'derived_header': '\n'.join(f'#   {line}' for line in lines),

        'footprint': footprint_param(half_x, half_y),
        'inflation_radius': f'{inflation_radius:.4f}',
        # THE PROXEMIC RADIUS, and it is a commissioning decision rather than a
        # platform property, which is why it is stated here and applied to both
        # platforms rather than living in a spec.
        #
        # 1.2 m is the top of the 0.45 to 1.2 m range Hall's proxemics gives for
        # personal space. Taken at the top because this vehicle is 0.6 m wide,
        # so passing at 1.2 m from a person's centre leaves roughly 0.9 m of
        # clear floor between them and the body of the robot, which is the
        # quantity a person actually experiences.
        #
        # It is NOT derived from the footprint. Inflation already covers what
        # the vehicle needs to fit; this covers what a person needs to feel
        # unthreatened, and conflating the two is how a social distance ends up
        # being justified by a chassis width.
        'proxemic_radius': '1.2000',
        'r_circ': f'{r_circ:.4f}',
        'r_circ_mm': f'{r_circ * 1000:.0f}',
        'r_inscribed': f'{r_inscribed:.4f}',
        'circ_diameter': f'{r_circ * 2.0:.4f}',
        'footprint_l_mm': f'{half_x * 2000:.0f}',
        'footprint_w_mm': f'{half_y * 2000:.0f}',
        'chassis_l_mm': f"{v['chassis_length'] * 1000:.0f}",
        'chassis_w_mm': f"{v['chassis_width'] * 1000:.0f}",
        'chassis_h_mm': f"{v['chassis_height'] * 1000:.0f}",
        'proud_mm': f"{(half_x - v['chassis_length'] / 2.0) * 1000:.1f}",
        'scan_plane_mm': f"{v['scanner_mount_height'] * 1000:.0f}",

        'vx_max': f'{vx_max:.2f}',
        'vx_min': f"{-v['max_reverse_speed']:.2f}",
        'wz_max': f"{v['max_angular_speed']:.2f}",
        'ax_max': f'{ordinary_accel:.2f}',
        'ax_min': f'{-ordinary_decel:.2f}',
        'az_max': f'{AZ_MAX:.2f}',
        'rated_speed': f"{v['max_linear_speed']:.2f}",
        'speed_fraction': f'{COMMISSIONED_SPEED_FRACTION:.2f}',
        'unladen_accel': f"{v['max_linear_accel_unladen']:.2f}",
        'emergency_decel': f"{v['emergency_decel']:.2f}",
        'ordinary_decel': f'{ordinary_decel:.2f}',

        'smoother_max_velocity': f'[{vx_max:.2f}, 0.0, '
                                 f"{v['max_angular_speed']:.2f}]",
        'smoother_min_velocity': f"[{-v['max_reverse_speed']:.2f}, 0.0, "
                                 f"{-v['max_angular_speed']:.2f}]",
        'smoother_max_accel': f'[{ordinary_accel:.2f}, 0.0, {AZ_MAX:.2f}]',
        'smoother_max_decel': f'[{-ordinary_decel:.2f}, 0.0, {-AZ_MAX:.2f}]',

        'mppi_time_steps': f'{MPPI_TIME_STEPS}',
        'mppi_model_dt': f'{MPPI_MODEL_DT}',
        'horizon_s': f'{horizon_s:.2f}',
        'lookahead': f'{lookahead:.2f}',
        'local_size': f'{local_size}',

        'z_resolution': f'{VOXEL_Z_RESOLUTION:.2f}',
        'z_voxels': f'{z_voxels}',
        'envelope_height': f"{v['vehicle_envelope_height']:.2f}",

        'corridor_90_mm': f"{spec['validation_targets']['corridor_width_90_turn'] * 1000:.0f}",
        'corridor_dynamic': f"{spec['validation_targets']['corridor_width_dynamic']:.3f}",
    }
    return values, lines


REFLOW = re.compile(r'^(\s*)#> (.*)$')
UNIT = re.compile(r'(\d) (m/s2|rad/s|m/s|mm|m|s)\b')
COMMENT_WIDTH = 79


def reflow(text):
    """Re-wrap comment paragraphs written as one long `#>` line.

    A substituted value is rarely the width of the placeholder it replaced, so
    a comment block wrapped by hand in the template comes out ragged for one
    platform or the other, with sentences broken after two words. Reflowing
    after substitution is the only way to get it right for both.

    OPT IN, NOT AUTOMATIC, and that is the whole design. Most comments in the
    template are deliberately shaped: quoted log excerpts, indented arithmetic,
    lists one item to a line. Re-wrapping those would destroy them. Only a
    paragraph marked `#>` is touched, and it must be a single line.
    """
    out = []
    for line in text.splitlines():
        m = REFLOW.match(line)
        if not m:
            out.append(line)
            continue
        indent, body = m.groups()
        # A unit must not wrap away from its number. Left to itself textwrap
        # produces "the inscribed radius 0.2845" then "m would cut it", which
        # reads as though the sentence lost a word. The separator is swapped for
        # a character textwrap will not break on, then swapped back, so the
        # output is still plain ASCII with an ordinary space.
        glued = UNIT.sub('\\1\x00\\2', body)
        wrapped = textwrap.wrap(
            glued, width=COMMENT_WIDTH,
            initial_indent=f'{indent}# ', subsequent_indent=f'{indent}# ')
        if not wrapped:
            out.append(f'{indent}#')
        else:
            out.extend(w.replace('\x00', ' ') for w in wrapped)
    return '\n'.join(out) + '\n'


def render(spec, platform, planner='smac2d', proxemic=True):
    values, lines = derive(spec, platform)
    values['planner_block'] = planner_block(planner)
    # THE CONTROL ARM, generated rather than hand edited. `off` drops the layer
    # from both plugin lists; the layer's own configuration block is left in
    # place and simply unreferenced, so the two configurations differ by
    # exactly the thing under test and diffing them shows only that.
    values['proxemic_global'] = (
        '"static_layer", "obstacle_layer", "voxel_layer", "proxemic_layer", '
        '"inflation_layer"' if proxemic else
        '"static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"')
    values['proxemic_local'] = (
        '"obstacle_layer", "voxel_layer", "proxemic_layer", "inflation_layer"'
        if proxemic else
        '"obstacle_layer", "voxel_layer", "inflation_layer"')
    body = reflow(Template(TEMPLATE.read_text()).substitute(values))
    header = (
        '# GENERATED by amr_navigation/tools/generate_nav2.py from\n'
        f'# amr_description/config/platforms/{platform}.yaml and\n'
        '# config/nav2.yaml.in. Do not hand-edit: change the spec or the\n'
        '# template and regenerate. A test asserts this file matches.\n'
        '#\n'
        '# Derived for this platform:\n'
        f"{values['derived_header']}\n"
        '\n')
    return header + body, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--platform', default='mir250_class')
    # THE PLANNER IS A COMMISSIONING CHOICE, not a platform property, so it is
    # a flag rather than a spec value. The comparison table is produced by
    # running this once per planner and measuring each.
    ap.add_argument('--planner', default='smac2d', choices=sorted(PLANNERS))
    # THE PROXEMIC LAYER IS ALSO A COMMISSIONING CHOICE, and it is a flag for
    # the same reason the planner is: the comparison table that justifies it is
    # produced by generating both ways and measuring each. Without the flag the
    # control arm would be a hand edit of a generated file, which this project
    # does not do, or a stashed template, which nobody could reproduce.
    # DEFAULT OFF, because four runs could not show that it helps. The effect
    # it was built to produce is smaller than the run to run spread of the
    # metric that would show it, 224 mm, and the best social score of the four
    # belongs to the run where the vehicle never moved. See V-59. It is kept,
    # tested and available, and it is not claimed.
    ap.add_argument('--proxemic', default='off', choices=('on', 'off'),
                    help='human aware costmap layer; unproven, see V-59')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()

    spec_file = SPEC_DIR / f'{args.platform}.yaml'
    if not spec_file.is_file():
        sys.exit(f'no platform spec at {spec_file}')
    spec = yaml.safe_load(spec_file.read_text())

    text, lines = render(spec, args.platform, args.planner,
                         proxemic=args.proxemic == 'on')

    # Parse what we produced before writing it. A template substitution that
    # produces invalid YAML would otherwise only surface when a lifecycle node
    # failed to configure, which reports as "failed to change state" and says
    # nothing about why.
    try:
        yaml.safe_load(text)
    except yaml.YAMLError as exc:
        sys.exit(f'generated configuration is not valid YAML: {exc}')

    out = args.out or (PKG / 'config' / f'nav2.{args.platform}.yaml')
    out.write_text(text)
    print(f'wrote {out}')
    for line in lines:
        print(f'  {line}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
