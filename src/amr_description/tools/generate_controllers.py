#!/usr/bin/env python3
"""Generate the ros2_control controller configuration from the platform spec.

WHY THIS EXISTS

Every other configuration in this project is generated per platform: the
protective fields, the Nav2 stack, the world, the keepout mask, the stations.
The controller configuration was not. It was one hand written file holding
MiR250 wheel geometry, loaded by whichever platform was running, because
controller_manager reads plain YAML and cannot resolve a spec.

A drift test existed for exactly this and passed throughout. It compared the
file against the spec, and its fixture loaded `mir250_class.yaml` and built the
description with `platform:=mir250_class`, so it compared the MiR250 against
itself. The gate was correct and was never pointed at the second platform.

WHAT IT COST, measured rather than argued

Running the MP-400, the controller integrated odometry with wheel_radius 0.100
against a true 0.075 and wheel_separation 0.450 against a true 0.529:

    wheels claimed   21.75 m of path
    vehicle moved    16.18 m of path
    ratio            0.744

Predicted over count from the radius alone, 0.100 / 0.075 = 1.333. Measured,
1 / 0.744 = 1.344. Those agree to within one percent, which is what makes this
the cause rather than a candidate.

An odometry scale error is the worst kind of fault to look for, because nothing
about it looks wrong. The wheels report smoothly, continuous to 7.2 mm per
sample. The scan is geometrically perfect. The vehicle drives. Only the map is
wrong, and the map being wrong reads as a SLAM tuning problem, which is where
four earlier hypotheses went.

WHY A TEMPLATE RATHER THAN A DICT DUMP

Same reason as generate_nav2.py. Most of this file is reasoning about why each
parameter is what it is, and dumping a Python dict would delete all of it. The
template carries the comments; this substitutes the values that are functions
of the vehicle, and only those.

    tools/generate_controllers.py --platform mp400_class
    tools/generate_controllers.py --all
"""

import argparse
import sys
from pathlib import Path
from string import Template

import yaml

PKG = Path(__file__).resolve().parents[1]
TEMPLATE = PKG / 'config' / 'controllers.yaml.in'
SPEC_DIR = PKG / 'config' / 'platforms'


def fields(platform, spec):
    """The values that are functions of the vehicle, and only those.

    A number belongs here if changing the vehicle changes it. Update rate,
    covariances, frame ids and the multipliers are not vehicle properties and
    stay in the template as literals with their reasoning attached.
    """
    v = spec['values']
    missing = [k for k in ('wheel_separation', 'drive_wheel_radius',
                           'max_linear_speed', 'max_linear_accel',
                           'max_angular_speed') if k not in v]
    if missing:
        raise SystemExit(
            f'{platform}: spec is missing {", ".join(missing)}, which the '
            f'controller cannot be generated without. Refusing to emit a file '
            f'with a plausible looking default in it.')
    return {
        'platform': platform,
        'wheel_separation': f'{v["wheel_separation"]:.4f}',
        'wheel_radius': f'{v["drive_wheel_radius"]:.4f}',
        'max_linear_speed': f'{v["max_linear_speed"]:.3f}',
        'max_linear_accel': f'{v["max_linear_accel"]:.3f}',
        'max_angular_speed': f'{v["max_angular_speed"]:.3f}',
    }


def generate(platform):
    spec_file = SPEC_DIR / f'{platform}.yaml'
    if not spec_file.exists():
        raise SystemExit(f'no platform spec at {spec_file}')
    spec = yaml.safe_load(spec_file.read_text())
    out = PKG / 'config' / f'controllers.{platform}.yaml'
    text = Template(TEMPLATE.read_text()).substitute(fields(platform, spec))
    out.write_text(text)
    f = fields(platform, spec)
    print(f'{out.relative_to(PKG.parents[1])}')
    print(f'  wheel_separation {f["wheel_separation"]} m   '
          f'wheel_radius {f["wheel_radius"]} m')
    print(f'  linear {f["max_linear_speed"]} m/s at {f["max_linear_accel"]} m/s2   '
          f'angular {f["max_angular_speed"]} rad/s')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--platform')
    ap.add_argument('--all', action='store_true',
                    help='every platform with a spec')
    a = ap.parse_args()
    if a.all:
        for s in sorted(SPEC_DIR.glob('*.yaml')):
            generate(s.stem)
    elif a.platform:
        generate(a.platform)
    else:
        raise SystemExit('pass --platform NAME or --all')
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
