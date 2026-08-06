#!/usr/bin/env python3
"""Measure what robots and sensors cost in simulation real-time factor.

This exists because the three-tier design in docs/adr/0003-three-tier-simulation.md
was a prediction, and a prediction with no measurement behind it is a guess. The
question it answers: how many robots, carrying which sensors, can this machine
simulate faster than real time?

Method. For each configuration the robot description is generated with the
requested sensor set, converted to SDF, and inserted into a copy of the
warehouse world with the real-time throttle DISABLED. The simulator is then run
TWICE, at two different step counts, and the per-step cost is taken from the
slope between them.

Two traps this method exists to avoid, both of which produced wrong numbers here
before it was written:

  1. The real-time throttle. With <real_time_factor> at its default the
     simulator sleeps to track real time and reports about 0.9 no matter how
     much headroom there is. Setting it to 0 is what makes any of this mean
     anything.

  2. Lazy sensor rendering. Gazebo renders a camera only while something is
     subscribed to its topic. With no consumer attached the cameras cost
     nothing at all, and an earlier version of this benchmark duly reported
     that adding two RGB-D cameras made the simulation very slightly FASTER.
     Subscribers are now attached for the duration of every measured run.

  3. Startup cost. Launching the simulator and loading the warehouse costs
     around 2 s on this machine, which is far more than a few thousand physics
     steps. A single short run therefore measures process startup, not
     simulation, and understates the real headroom by more than an order of
     magnitude. Taking the slope between two run lengths cancels the fixed cost
     out.

Both figures are reported, because both matter for different reasons. MARGINAL
real-time factor is the steady-state capability and answers "can this scenario
run live". STARTUP is a fixed per-run cost and is what dominates short CI runs.

Usage:
    python3 benchmark_sim_cost.py                  # the standard configurations
    python3 benchmark_sim_cost.py --iterations 3000
    python3 benchmark_sim_cost.py --json out.json  # for the KPI harness
"""

import argparse
import json
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve()
WS = HERE.parents[3]
XACRO = WS / 'src' / 'amr_description' / 'urdf' / 'amr.urdf.xacro'
WORLD = WS / 'src' / 'amr_sim' / 'worlds' / 'warehouse.sdf'
MODELS = WS / 'src' / 'amr_sim' / 'models'

# Robot start poses along the open aisle, spaced so they do not overlap.
POSES = [(2.0, -1.0), (2.0, -2.5), (2.0, -4.0), (2.0, -5.5), (2.0, -7.0)]

CONFIGS = [
    # (label, robot count, use_scanners, use_cameras)
    ('world only, no robot',                 0, False, False),
    ('1 robot, no sensors',                  1, False, False),
    ('1 robot, 2 safety scanners',           1, True,  False),
    ('1 robot, scanners + 2 RGB-D',          1, True,  True),
    ('3 robots, scanners only  [fleet tier]', 3, True,  False),
    ('5 robots, scanners only  [fleet tier]', 5, True,  False),
    ('3 robots, scanners + RGB-D',           3, True,  True),
]


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, check=True, **kw)


def sensor_topics(ns: str, scanners: bool, cameras: bool) -> list:
    """Topics a consumer must subscribe to for the sensors to actually render."""
    topics = []
    if scanners:
        topics += [f'/{ns}scanner_front_left/scan', f'/{ns}scanner_rear_right/scan']
    if cameras:
        for cam in ('camera_left', 'camera_right'):
            topics += [f'/{ns}{cam}/image', f'/{ns}{cam}/depth_image']
    return topics


def robot_sdf(ns: str, scanners: bool, cameras: bool) -> str:
    """Generate the description and convert it to SDF."""
    urdf = run(['xacro', str(XACRO),
                f'namespace:={ns}',
                f'use_scanners:={str(scanners).lower()}',
                f'use_cameras:={str(cameras).lower()}',
                # The ros2_control plugin needs a live ROS graph, which a bare
                # simulator run does not have. This benchmark measures physics
                # and sensor rendering, so the controller is left out rather
                # than half-loaded, and the measurement is labelled as such.
                'sim:=false']).stdout
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(urdf)
        path = f.name
    try:
        return run(['gz', 'sdf', '-p', path]).stdout
    finally:
        Path(path).unlink(missing_ok=True)


def build_world(n_robots: int, scanners: bool, cameras: bool, dest: Path) -> list:
    """Write the benchmark world. Returns the sensor topics it will publish."""
    world = WORLD.read_text()
    # Disable the real-time throttle. Without this the numbers are meaningless.
    world = re.sub(r'<real_time_factor>[^<]*</real_time_factor>',
                   '<real_time_factor>0</real_time_factor>', world)

    blocks, topics = [], []
    for i in range(n_robots):
        # Each robot gets its own namespace, so its sensors publish on their own
        # topics. Sharing a topic across robots is silent and wrong.
        ns = f'amr_{i + 1}'
        sdf = robot_sdf(ns, scanners, cameras)
        body = re.search(r'<model[^>]*>.*</model>', sdf, re.S)
        if body is None:
            sys.exit('could not find a <model> in the converted robot SDF')
        x, y = POSES[i]
        named = re.sub(r'<model\s+name=[\'"][^\'"]*[\'"]',
                       f'<model name="{ns}"', body.group(0), count=1)
        blocks.append(named.replace(
            '</model>', f'<pose>{x} {y} 0.02 0 0 0</pose></model>'))
        topics += sensor_topics(ns + '/', scanners, cameras)

    if blocks:
        world = world.replace('</world>', '\n'.join(blocks) + '\n</world>')
    dest.write_text(world)
    return topics


def _one_run(world_path: Path, iterations: int, topics: list) -> tuple:
    """Run the simulator with a subscriber on every sensor topic.

    The subscribers are the point: Gazebo renders a camera only while its topic
    has a consumer, so measuring without them measures a robot whose cameras are
    switched off.
    """
    import os
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=str(MODELS))
    subs = []
    devnull = subprocess.DEVNULL
    # The clock starts at process launch and every fixed cost (simulator
    # startup, world load, the pause below while subscribers attach) is
    # identical between the short and the long run, so it cancels in the slope.
    # Trying to exclude it by starting the clock later is what produced negative
    # startup times and an infinite real-time factor in an earlier version.
    t0 = time.perf_counter()
    try:
        proc = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--iterations', str(iterations), str(world_path)],
            stdout=devnull, stderr=subprocess.PIPE, text=True, env=env)
        if topics:
            time.sleep(3.0)  # let the simulator advertise before subscribing
            for topic in topics:
                subs.append(subprocess.Popen(['gz', 'topic', '-e', '-t', topic],
                                             stdout=devnull, stderr=devnull, env=env))
        _, stderr = proc.communicate(timeout=1800)
        wall = time.perf_counter() - t0
    finally:
        for s in subs:
            s.terminate()
    return wall, [ln for ln in (stderr or '').splitlines() if '[Err]' in ln]


def measure(world_path: Path, n_short: int, n_long: int, topics: list,
            step: float = 0.004) -> dict:
    """Two-point measurement: per-step cost from the slope, startup from the intercept."""
    w_short, errs = _one_run(world_path, n_short, topics)
    w_long, errs2 = _one_run(world_path, n_long, topics)

    per_step = (w_long - w_short) / (n_long - n_short)
    startup = w_short - n_short * per_step
    # Guard against a negative slope from timing noise on a very cheap scene.
    marginal_rtf = step / per_step if per_step > 1e-9 else float('inf')

    return {
        'startup_s': round(startup, 2),
        'per_step_ms': round(per_step * 1000.0, 4),
        'marginal_rtf': round(marginal_rtf, 1),
        'wall_short_s': round(w_short, 2),
        'wall_long_s': round(w_long, 2),
        'errors': (errs + errs2)[:3],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--short', type=int, default=1000,
                    help='step count for the short run of each pair')
    ap.add_argument('--long', type=int, default=8000,
                    help='step count for the long run of each pair')
    ap.add_argument('--json', type=Path, help='write results as JSON')
    args = ap.parse_args()

    for tool in ('xacro', 'gz'):
        if shutil.which(tool) is None:
            sys.exit(f'{tool} not found; source the workspace first')

    manifest = {
        'cpu': platform.processor() or platform.machine(),
        'python': platform.python_version(),
        'gz': run(['gz', 'sim', '--versions']).stdout.strip().splitlines()[0],
        'steps_short': args.short,
        'steps_long': args.long,
        'step_s': 0.004,
        'note': ('real-time throttle disabled; a subscriber is attached to every '
                 'sensor topic so lazy rendering is actually exercised; the '
                 'ros2_control plugin is not loaded'),
    }

    print(f'{"configuration":42s} {"us/step":>9s} {"marginal RTF":>13s} {"startup":>9s}')
    print('-' * 78)
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for label, n, scanners, cameras in CONFIGS:
            world = Path(tmp) / 'bench.sdf'
            topics = build_world(n, scanners, cameras, world)
            r = measure(world, args.short, args.long, topics)
            r['subscribed_topics'] = len(topics)
            r.update(config=label, robots=n, scanners=scanners, cameras=cameras)
            results.append(r)
            flag = '  <-- below real time' if r['marginal_rtf'] < 1.0 else ''
            print(f'{label:42s} {r["per_step_ms"] * 1000:8.1f} '
                  f'{r["marginal_rtf"]:12.1f}x {r["startup_s"]:8.2f}s{flag}')
            for e in r['errors']:
                print(f'    {e}')

    if args.json:
        args.json.write_text(json.dumps(
            {'manifest': manifest, 'results': results}, indent=2))
        print(f'\nwrote {args.json}')


if __name__ == '__main__':
    main()
