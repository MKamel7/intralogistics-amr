#!/usr/bin/env python3
"""Score people detections against the scenario that produced them.

The scenario file states exactly where every pedestrian is, so detections can be
matched to a known truth rather than eyeballed. This is the first use of the
simulator as a LABEL ORACLE rather than as a source of control input, which is
the inversion described in docs/validation.md.

Reports precision, recall and localisation error over a window of frames.

    ros2 launch amr_bringup robot.launch.py gui:=false cameras:=false
    ros2 launch amr_sim people.launch.py scenario:=static_people
    python3 score_detections.py --scenario static_people --frames 40
"""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray

SCENARIOS = Path(__file__).resolve().parents[2] / 'amr_sim' / 'scenarios'

# The robot's spawn pose in the world. Detections arrive in base_link and the
# robot is stationary for this scenario, so this is the only transform needed.
# A moving robot would require TF; that comes with the tracking work.
ROBOT_SPAWN = (2.0, -1.0, 0.0)


class Scorer(Node):
    def __init__(self):
        super().__init__('score_detections')
        self.frames = []
        self.create_subscription(
            Detection3DArray, '/people_detections',
            lambda m: self.frames.append(
                [(d.bbox.center.position.x, d.bbox.center.position.y,
                  d.results[0].hypothesis.score if d.results else 0.0)
                 for d in m.detections]),
            qos_profile_sensor_data)


def truth_in_base_link(people):
    rx, ry, ryaw = ROBOT_SPAWN
    out = {}
    for p in people:
        dx, dy = p['x'] - rx, p['y'] - ry
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        out[p['name']] = (c * dx - s * dy, s * dx + c * dy)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='static_people')
    ap.add_argument('--frames', type=int, default=40)
    ap.add_argument('--tolerance', type=float, default=0.40,
                    help='metres within which a detection counts as a match')
    args = ap.parse_args()

    spec = yaml.safe_load((SCENARIOS / f'{args.scenario}.yaml').read_text())
    truth = truth_in_base_link(spec['people'])

    rclpy.init()
    node = Scorer()
    deadline = time.time() + 60.0
    while rclpy.ok() and len(node.frames) < args.frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if not node.frames:
        print('no detections received; is the stack running?')
        return 1

    tp = fp = fn = 0
    errors = []
    per_person = {name: 0 for name in truth}

    for detections in node.frames:
        unmatched = dict(truth)
        for dx, dy, _score in detections:
            best, best_d = None, args.tolerance
            for name, (tx, ty) in unmatched.items():
                d = math.hypot(dx - tx, dy - ty)
                if d < best_d:
                    best, best_d = name, d
            if best is None:
                fp += 1
            else:
                tp += 1
                errors.append(best_d)
                per_person[best] += 1
                del unmatched[best]
        fn += len(unmatched)

    frames = len(node.frames)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    errors.sort()

    print(f'scenario     {args.scenario}, {len(truth)} people, {frames} frames')
    print(f'matched      {tp}   false positives {fp}   missed {fn}')
    print(f'precision    {precision:.3f}')
    print(f'recall       {recall:.3f}')
    if errors:
        print(f'localisation mean {sum(errors) / len(errors) * 100:.1f} cm, '
              f'p50 {errors[len(errors) // 2] * 100:.1f} cm, '
              f'p95 {errors[int(0.95 * (len(errors) - 1))] * 100:.1f} cm')
    print('per person detection rate:')
    for name, (tx, ty) in sorted(truth.items()):
        rng = math.hypot(tx, ty)
        print(f'  {name:16s} {rng:5.2f} m   {per_person[name]}/{frames} frames '
              f'({100 * per_person[name] / frames:.0f}%)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
