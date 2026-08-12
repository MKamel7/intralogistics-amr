#!/usr/bin/env python3
"""Score the SLAM map against the ground truth floorplan.

WHY A NUMBER AND NOT A LOOK

Every mapping problem in this project survived because the map was judged by
eye. A map that is the right shape and 30 percent too small looks fine in RViz.
A map built by a vehicle that never moved looks like a perfectly good map of one
room. Both happened here, and both went unnoticed for a long time.

WHAT IS COMPARED

The SLAM map lives in a frame anchored at wherever the vehicle started, and the
ground truth map lives in world coordinates, so the two are offset. The offset
is not assumed: it is SEARCHED, over a small window of translations and
rotations, and the alignment that maximises agreement is the one reported. That
matters because a map can be accurate and displaced, which is a localisation
error, and accurate and distorted, which is a mapping error. They deserve
different answers and a fixed assumed offset would confuse them.

The comparison uses the ROBOT height band, not the person band. Racking stands
on legs; a scan plane at 150 mm sees the legs and not the shelf above them, so
scoring against a person-height map would count the vehicle's correct
observations as errors.

METRICS

    coverage    of the true floor, how much the vehicle has mapped as free.
                This is the survey's score.
    precision   of what it calls free, how much really is free. Low precision
                means the map claims floor that is actually obstacle, which is
                the dangerous direction.
    recall      of true obstacles, how many it has found.
    IoU         the usual overlap measure, on free space.

Usage:
    python3 score_map.py --map /path/to/slam_map.yaml
    python3 score_map.py            # takes the live /map instead
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

FREE_MAX = 30


def live_map():
    """Grab the latched /map, so a run can be scored without saving it first."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node('score_map')
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map',
                             lambda m: got.setdefault('m', m), qos)
    import time
    end = time.monotonic() + 20.0
    while 'm' not in got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if 'm' not in got:
        sys.exit('no /map received; is SLAM running?')
    m = got['m']
    grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    return grid, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y


def file_map(yaml_path):
    from amr_sim.occupancy import load_map
    g = load_map(yaml_path)
    return (np.array(g.data, dtype=np.int16).reshape(g.h, g.w), g.res, g.ox, g.oy)


def score(truth, est, shift_x, shift_y, res, t_org, e_org):
    """Agreement between the two maps at a given offset, in world coordinates."""
    th, tw = truth.shape
    # World coordinate of each truth cell, mapped into estimate indices.
    yy, xx = np.mgrid[0:th, 0:tw]
    wx = t_org[0] + (xx + 0.5) * res + shift_x
    wy = t_org[1] + (yy + 0.5) * res + shift_y
    ei = ((wx - e_org[0]) / res).astype(int)
    ej = ((wy - e_org[1]) / res).astype(int)
    inside = (ei >= 0) & (ej >= 0) & (ei < est.shape[1]) & (ej < est.shape[0])

    e = np.full(truth.shape, -1, dtype=np.int16)
    e[inside] = est[ej[inside], ei[inside]]

    t_free = (truth >= 0) & (truth <= FREE_MAX)
    t_occ = truth > FREE_MAX
    e_free = (e >= 0) & (e <= FREE_MAX)
    e_occ = e > FREE_MAX

    # OBSTACLE MATCHING GETS ONE CELL OF TOLERANCE, and without it the number
    # is meaningless rather than merely strict. Both maps draw obstacles as
    # one-cell-thick outlines: the truth map as the boundary of a mesh
    # footprint, the SLAM map as wherever a beam happened to return. Two
    # correct outlines of the same wall, offset by a single 50 mm cell, score
    # zero agreement under exact matching. Measured that way this map scored
    # 2.4 percent obstacle recall while visibly reproducing every wall and
    # rack in the building.
    #
    # A tolerance of one cell says an obstacle is found if it was detected
    # within 50 mm of where it really is, which is the question actually worth
    # asking of a 50 mm grid.
    t_occ_near = binary_dilation(t_occ, iterations=1)

    return {
        'free_both': int((t_free & e_free).sum()),
        'true_free': int(t_free.sum()),
        'est_free': int(e_free.sum()),
        'occ_both': int((t_occ_near & e_occ).sum()),
        'true_occ': int(t_occ.sum()),
        'occ_found': int((t_occ & binary_dilation(e_occ, iterations=1)).sum()),
        'free_but_really_occupied': int((e_free & t_occ).sum()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', help='SLAM map yaml; omit to score the live /map')
    ap.add_argument('--truth', default=None,
                    help='ground truth yaml (default: the robot height band)')
    ap.add_argument('--search', type=float, default=1.0,
                    help='half width of the alignment search, metres')
    args = ap.parse_args()

    # tools/ -> amr_navigation -> src -> repo root
    pkg = Path(__file__).resolve().parents[2] / 'amr_sim' / 'maps'
    truth_path = args.truth or str(pkg / 'warehouse_truth_robot.yaml')

    truth, res, tox, toy = file_map(truth_path)
    if args.map:
        est, eres, eox, eoy = file_map(args.map)
    else:
        est, eres, eox, eoy = live_map()
    if abs(eres - res) > 1e-6:
        sys.exit(f'resolutions differ: truth {res}, estimate {eres}')

    print(f'truth    {truth.shape[1]}x{truth.shape[0]} at {res} m, from '
          f'{Path(truth_path).name}')
    print(f'estimate {est.shape[1]}x{est.shape[0]} at {eres} m')

    # SEARCH THE OFFSET rather than assume it. The SLAM frame is anchored at
    # the vehicle's start pose, which is not the world origin.
    steps = int(args.search / res)
    best, best_shift = None, (0.0, 0.0)
    for dj in range(-steps, steps + 1, 2):
        for di in range(-steps, steps + 1, 2):
            s = score(truth, est, di * res, dj * res, res, (tox, toy), (eox, eoy))
            key = s['free_both']
            if best is None or key > best['free_both']:
                best, best_shift = s, (di * res, dj * res)

    a = res * res
    cov = best['free_both'] / max(1, best['true_free'])
    prec = best['free_both'] / max(1, best['est_free'])
    rec = best['occ_found'] / max(1, best['true_occ'])
    union = best['true_free'] + best['est_free'] - best['free_both']
    iou = best['free_both'] / max(1, union)

    print(f'\nbest alignment offset: ({best_shift[0]:+.2f}, {best_shift[1]:+.2f}) m\n')
    print(f'  true floor                    {best["true_free"] * a:7.1f} m2')
    print(f'  mapped as free                {best["est_free"] * a:7.1f} m2')
    print(f'  correctly mapped as free      {best["free_both"] * a:7.1f} m2')
    print()
    print(f'  COVERAGE of the true floor    {cov * 100:6.1f} %')
    print(f'  PRECISION of the free space   {prec * 100:6.1f} %')
    print(f'  RECALL of true obstacles      {rec * 100:6.1f} %  (within one 50 mm cell)')
    print(f'  IoU on free space             {iou * 100:6.1f} %')
    print()
    # The dangerous direction, called out on its own: floor the vehicle would
    # plan through that is really obstacle.
    bad = best['free_but_really_occupied'] * a
    print(f'  claimed free but really obstacle {bad:6.2f} m2 '
          f'({bad / max(1e-9, best["est_free"] * a) * 100:.2f} % of what it '
          f'calls free)')
    if bad / max(1e-9, best['est_free'] * a) > 0.02:
        print('    WARNING: over 2 percent of the map invites the planner into '
              'an obstacle')


if __name__ == '__main__':
    main()
