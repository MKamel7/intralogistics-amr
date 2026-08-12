#!/usr/bin/env python3
"""Author the keepout mask over the racking, from the site layout.

WHAT THIS IS, AND WHAT IT IS NOT

This is COMMISSIONING DATA. It has the same status as the building drawing an
integrator is handed on day one: the racking is bolted to the floor, its
position is known before the vehicle is switched on, and declaring it is not
cheating. Every real AMR deployment does exactly this, and the zone editor is
usually the first tool a commissioning engineer opens.

It is NOT the pose oracle and NOT the ground truth occupancy map. Those two live
under `/ground_truth/`, are used only to score results, and must never reach the
navigation stack. The distinction is worth stating precisely, because the files
look similar and confusing them would invalidate every mapping number in this
repository:

    ground truth map   what is really there, used to MARK the robot's homework.
                       Derived from every collision mesh in the world.
    keepout mask       where the operator has decided the vehicle may not go.
                       Derived from the racking only, and it would be authored
                       by hand from a floor plan on a real site.

WHY DECLARE IT AT ALL, GIVEN THE CAMERAS

The camera voxel layer already stops the vehicle driving under shelving, and it
works. But it works by rediscovering, every single run, a fact that never
changes, and it only works while the cameras do. Racking is permanent
infrastructure. Making the vehicle's ability to avoid it contingent on depth
perception functioning correctly is a fragile design: a dirty lens, a failed
camera or a sunlit aisle should degrade performance, not remove a hard
constraint.

So the two do different jobs. The keepout mask states what is permanently out of
bounds and cannot be argued with. The cameras handle everything that is not on
the drawing: a pallet left in an aisle, a person, a door left open.

FRAME

The mask is written in the SURVEYED MAP's frame, resolution and origin, because
that is the frame the vehicle will localise in once it runs on the saved map.
The survey started with the vehicle parked at a known spot, which is the other
commissioning fact this needs, and it is recorded below rather than guessed.

Usage:
    python3 build_keepout_mask.py [--margin 0.05]
"""

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np
import yaml

NAV = Path(__file__).resolve().parents[1]
SIM = NAV.parent / 'amr_sim'
SURVEYED = NAV / 'maps' / 'warehouse_surveyed.yaml'
OUT = NAV / 'maps'

# The models that are permanent racking. Everything else in the world is
# either structure the scanner sees perfectly well (walls) or clutter that may
# legitimately be moved, and neither belongs in a keepout zone.
RACK_MODELS = ('wf_shelf_d_01', 'wf_shelf_e_01', 'wf_shelf_f_01')

# WHERE THE VEHICLE WAS PARKED FOR THE SURVEY, in world coordinates. This is a
# commissioning fact: it is where the operator put the robot before pressing
# start, and it fixes the surveyed map's frame against the building. It is not
# a privileged reading of the vehicle's true pose during operation.
SURVEY_START_XY = (2.0, -1.0)


def _load_builder():
    """Reuse the ground truth builder's SDF and mesh handling."""
    path = SIM / 'tools' / 'build_ground_truth_map.py'
    spec = importlib.util.spec_from_file_location('gt_builder', path)
    mod = importlib.util.module_from_spec(spec)
    sys.argv = [str(path)]           # the module parses argv only under main
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--margin', type=float, default=0.05,
                    help='metres grown around each rack footprint')
    args = ap.parse_args()

    if not SURVEYED.exists():
        sys.exit(f'{SURVEYED} not found; run the survey and save the map first')

    meta = yaml.safe_load(SURVEYED.read_text())
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    img = SURVEYED.parent / meta['image']
    with img.open('rb') as f:
        assert f.readline().strip() == b'P5'
        dims = []
        while len(dims) < 3:
            line = f.readline()
            if not line.startswith(b'#'):
                dims += [int(t) for t in line.split()]
        w, h, _ = dims[:3]
    print(f'surveyed map {w}x{h} at {res} m, origin ({ox:.3f}, {oy:.3f})')

    gt = _load_builder()
    # The RACKING's real extent, taken over the height band a PERSON occupies,
    # not the band the scanner sees. The whole point is the shelf body above
    # the legs: at scanner height the racking is a row of posts with drivable
    # gaps, and those gaps are exactly what must be declared out of bounds.
    gt.Z_LO, gt.Z_HI = 0.06, 1.90

    root = __import__('xml.etree.ElementTree', fromlist=['ElementTree']).parse(
        gt.WORLD).getroot()
    world = root.find('world')

    grid = np.zeros((h, w), dtype=np.int8)
    cache = {}
    placed = 0
    for inc in world.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        name = uri[len('model://'):] if uri.startswith('model://') else ''
        if name not in RACK_MODELS:
            continue
        if name not in cache:
            cache[name] = gt.load_collision_meshes(gt.MODELS / name)
        pose = gt.child_pose(inc)
        for mesh in cache[name]:
            m = mesh.copy()
            m.apply_transform(pose)
            # World to surveyed-map frame: the map's origin sits at the pose
            # the vehicle was parked in when the survey began.
            m.apply_translation([-SURVEY_START_XY[0], -SURVEY_START_XY[1], 0.0])
            gt.stamp(grid, m, res, ox, oy)
        placed += 1
    print(f'{placed} rack instance(s) stamped')

    # Fill each rack's interior. Stamping meshes gives outlines, and a hollow
    # outline leaves the inside of a rack marked passable, which is the one
    # place this file exists to forbid.
    from scipy.ndimage import binary_dilation, binary_fill_holes
    keep = grid == 100
    keep = binary_fill_holes(binary_dilation(keep, iterations=1))
    if args.margin > 0:
        keep = binary_dilation(keep, iterations=max(1, int(args.margin / res)))
    print(f'keepout area {int(keep.sum()) * res * res:.1f} m2 '
          f'(margin {args.margin} m)')

    # A keepout mask is read as an occupancy grid: 100 means forbidden, 0 means
    # unrestricted. Written with the same PGM conventions as any other map so
    # nav2's map_server can serve it unchanged.
    out_img = np.full((h, w), 254, dtype=np.uint8)
    out_img[keep] = 0
    pgm = OUT / 'keepout_mask.pgm'
    with pgm.open('wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.flipud(out_img).tobytes())
    (OUT / 'keepout_mask.yaml').write_text(
        'image: keepout_mask.pgm\n'
        'mode: scale\n'
        f'resolution: {res}\n'
        f'origin: [{ox:.4f}, {oy:.4f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n')
    print(f'wrote {pgm.name} and keepout_mask.yaml to {OUT}')


if __name__ == '__main__':
    main()
