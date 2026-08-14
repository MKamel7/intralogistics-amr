#!/usr/bin/env python3
"""Score people tracks against simulator ground truth.

Answers the question the tracker exists to answer: does classifying tracks by
their velocity estimate actually recover the precision the leg detector loses to
static warehouse structure, and what does it cost in recall?

Ground truth comes from the simulator's own pose feed, bridged to
/ground_truth/poses. That feed is a LABEL ORACLE: it is used for scoring and
never enters the control path.

Reports precision and recall twice, once over every confirmed track and once
over moving tracks only, so the improvement is measured rather than claimed.

    ros2 launch amr_bringup robot.launch.py gui:=false cameras:=false
    ros2 launch amr_sim people.launch.py scenario:=walking_people
    python3 score_tracks.py --scenario walking_people --frames 60
"""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml

from amr_sim.pedestrian_driver import BEHAVIOUR_KEYS
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

SCENARIOS = Path(__file__).resolve().parents[2] / 'amr_sim' / 'scenarios'

# The robot's pose is READ from the oracle every frame, never assumed. An
# earlier version hardcoded the spawn pose and silently produced nonsense the
# moment anything moved the robot: a pedestrian walked into it and pushed it
# 2.2 m, so every ground-truth position was wrong by that much and the scores
# collapsed in a way that looked like a tracker defect.
ROBOT_MODEL = 'amr'


# ---- occlusion ------------------------------------------------------------
#
# WHY THIS EXISTS. Recall counted a person as a miss whenever the tracker did
# not report them, including people standing behind a rack. That makes recall a
# statement about the building's geometry rather than about the tracker: move
# the racking and the number changes with nothing else different.
#
# The range gate added earlier removed people who were simply too far away. It
# did not remove the ones in range and out of sight, so the figure it produced
# is a LOWER BOUND, which is what V-36 recorded.
#
# The ray cast walks the ground truth occupancy grid from the scanner to the
# person and asks whether anything solid lies between. The truth map is a
# measurement channel and is used here for scoring only, never in the control
# path.


class TruthMap:
    """The generated ground truth floorplan, for line of sight only."""

    def __init__(self, yaml_path):
        meta = yaml.safe_load(Path(yaml_path).read_text())
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        pgm = Path(yaml_path).parent / meta['image']
        with open(pgm, 'rb') as f:
            assert f.readline().strip() == b'P5', 'expected a binary PGM'
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            self.w, self.h = (int(v) for v in line.split())
            f.readline()                      # max value
            self.data = f.read(self.w * self.h)

    def occupied(self, x, y):
        """Is this world point solid? Points off the map are treated as free,
        because absence of map is not evidence of a wall."""
        cx = int((x - self.ox) / self.res)
        cy = int((y - self.oy) / self.res)
        if not (0 <= cx < self.w and 0 <= cy < self.h):
            return False
        # PGM rows run top down; the map origin is bottom left.
        v = self.data[(self.h - 1 - cy) * self.w + cx]
        return v < 128          # dark is occupied, matching the generator

    def line_of_sight(self, ax, ay, bx, by):
        """Walk the segment and report whether it reaches b unobstructed.

        Stepping at half the cell size so a ray cannot tunnel through a wall
        one cell thick, which is exactly how racking is drawn.
        """
        d = math.hypot(bx - ax, by - ay)
        if d < 1e-6:
            return True
        steps = max(2, int(d / (self.res * 0.5)))
        for i in range(1, steps):
            f = i / steps
            if self.occupied(ax + (bx - ax) * f, ay + (by - ay) * f):
                return False
        return True


class Scorer(Node):
    def __init__(self, names):
        super().__init__('score_tracks')
        self.names = set(names)
        self.truth = {}
        self.robot = None
        self.frames = []
        self.create_subscription(TFMessage, '/ground_truth/poses', self._truth, 10)
        self.create_subscription(
            Detection3DArray, '/people_tracks', self._tracks, qos_profile_sensor_data)

    def _truth(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == ROBOT_MODEL:
                q = tf.transform.rotation
                self.robot = (tf.transform.translation.x, tf.transform.translation.y,
                              2.0 * math.atan2(q.z, q.w))
            elif tf.child_frame_id in self.names:
                self.truth[tf.child_frame_id] = (
                    tf.transform.translation.x, tf.transform.translation.y)

    def _tracks(self, msg):
        if not self.truth or self.robot is None:
            return
        entries = []
        for d in msg.detections:
            cls = d.results[0].hypothesis.class_id if d.results else ''
            entries.append((d.bbox.center.position.x, d.bbox.center.position.y,
                            cls == 'person_moving', d.id))
        # Snapshot truth AND the robot pose alongside, since both move.
        self.frames.append((entries, dict(self.truth), self.robot))


def to_base_link(x, y, robot):
    rx, ry, ryaw = robot
    dx, dy = x - rx, y - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    return c * dx - s * dy, s * dx + c * dy


def score(frames, tolerance, moving_only, max_range, truth_map=None):
    """Precision and recall over people the sensor could actually see.

    THE RANGE GATE IS NOT A CONVENIENCE. Without it every person in the
    building counts as a miss in every frame, including people behind racking
    twenty metres away, and recall becomes a measure of how far apart the
    scenario spreads people rather than of anything the tracker does.

    Measured: a window with the vehicle at (25.6, 8.9) and all six people
    between 15 and 24 m away scored precision 0.000 and recall 0.000 with
    fn 360, which is six people times sixty frames. Nothing was wrong with the
    tracker; nobody was visible. That number would have been published as a
    tracker result.

    Occlusion is NOT modelled. A person within range but behind a rack still
    counts as a miss, so recall reported here remains a lower bound. Fixing
    that needs a ray cast against the truth map and is worth doing before any
    figure from this tool goes in a results table.
    """
    tp = fp = fn = 0
    errors = []
    ids_per_truth = {}
    in_range_frames = 0
    occluded = 0
    for entries, truth, robot in frames:
        active = [e for e in entries if (e[2] if moving_only else True)]
        in_range = {n: p for n, p in truth.items()
                    if math.dist(p, robot[:2]) <= max_range}
        # OCCLUDED PEOPLE ARE EXCLUDED, NOT COUNTED AS MISSES. A person behind
        # a rack is invisible to a 2D scanner, and counting them makes recall a
        # measurement of the building rather than of the tracker.
        if truth_map is not None:
            visible = {n: p for n, p in in_range.items()
                       if truth_map.line_of_sight(robot[0], robot[1], p[0], p[1])}
            occluded += len(in_range) - len(visible)
            in_range = visible
        remaining = {n: to_base_link(p[0], p[1], robot) for n, p in in_range.items()}
        if remaining:
            in_range_frames += 1
        for tx, ty, _moving, tid in active:
            best, best_d = None, tolerance
            for name, (gx, gy) in remaining.items():
                d = math.hypot(tx - gx, ty - gy)
                if d < best_d:
                    best, best_d = name, d
            if best is None:
                fp += 1
            else:
                tp += 1
                errors.append(best_d)
                ids_per_truth.setdefault(best, set()).add(tid)
                del remaining[best]
        fn += len(remaining)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    switches = sum(max(0, len(v) - 1) for v in ids_per_truth.values())
    return {
        'tp': tp, 'fp': fp, 'fn': fn, 'in_range_frames': in_range_frames,
        'occluded': occluded,
        'precision': precision, 'recall': recall,
        'errors': sorted(errors), 'id_switches': switches,
        'ids_per_truth': {k: len(v) for k, v in ids_per_truth.items()},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='walking_people')
    ap.add_argument('--frames', type=int, default=60)
    ap.add_argument('--tolerance', type=float, default=0.50)
    # The scanner's usable range for a person. Beyond this a miss says nothing
    # about the tracker, so people further away are excluded from the score
    # rather than counted against it.
    ap.add_argument('--max-range', type=float, default=10.0,
                    help='only score people within this range of the vehicle')
    # Without a truth map the occlusion gate cannot run and recall stays a
    # lower bound. Saying so is better than quietly reporting the weaker
    # number as though it were the real one.
    ap.add_argument('--truth-map', default='',
                    help='ground truth map yaml, for the line of sight gate')
    args = ap.parse_args()

    truth_map = TruthMap(args.truth_map) if args.truth_map else None
    if truth_map is None:
        print('no --truth-map given: occluded people are counted as misses, '
              'so recall below is a LOWER BOUND\n')

    spec = yaml.safe_load((SCENARIOS / f'{args.scenario}.yaml').read_text())
    names = [p['name'] for p in spec['people']]
    # FROM THE DRIVER'S KEY LIST, not a name repeated here. This read
    # `p.get('path')` alone, which is the key from the fixed-lane era, so it
    # reported "0 walking" for every scenario written since. It printed that
    # against a scenario in which five of six people walked up to 13.6 m.
    #
    # Only the header used it, so the metrics below were never wrong, but a
    # header claiming nobody moves is how a reader concludes the tracker was
    # scored on a static world. Fourth place in this project to keep its own
    # copy of this list; see BEHAVIOUR_KEYS.
    walkers = {p['name'] for p in spec['people']
               if any(p.get(k) for k in BEHAVIOUR_KEYS)}

    rclpy.init()
    node = Scorer(names)
    deadline = time.time() + 90.0
    while rclpy.ok() and len(node.frames) < args.frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if not node.frames:
        print('no track frames received; is the stack running with people spawned?')
        return 1

    print(f'scenario   {args.scenario}: {len(names)} people '
          f'({len(walkers)} walking, {len(names) - len(walkers)} stationary), '
          f'{len(node.frames)} frames\n')

    for label, moving_only in (('all confirmed tracks', False),
                               ('moving tracks only', True)):
        r = score(node.frames, args.tolerance, moving_only, args.max_range,
                  truth_map)
        print(f'  {label}')
        print(f'    precision {r["precision"]:.3f}   recall {r["recall"]:.3f}   '
              f'(tp {r["tp"]}, fp {r["fp"]}, fn {r["fn"]})')
        if r['errors']:
            e = r['errors']
            print(f'    localisation p50 {e[len(e) // 2] * 100:.1f} cm, '
                  f'p95 {e[int(0.95 * (len(e) - 1))] * 100:.1f} cm')
        print(f'    id switches {r["id_switches"]}   '
              f'ids per person {r["ids_per_truth"]}')
        print(f'    scored on {r["in_range_frames"]} of {len(node.frames)} frames '
              f'that had somebody within {args.max_range:.0f} m')
        if truth_map is not None:
            print(f'    {r["occluded"]} person-frames excluded as occluded by '
                  f'structure')
        print()

    print('  note: the stationary worker is in the scenario on purpose. The motion')
    print('  test cannot distinguish a standing person from a post, so it is')
    print('  expected to cost recall, and that cost is the point of measuring both.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
