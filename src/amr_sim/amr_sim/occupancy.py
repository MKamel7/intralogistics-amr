"""Occupancy grid loading and the clearance queries a planner needs.

Shared by the pedestrian driver and the ground truth map publisher so there is
one definition of "is this point clear", rather than two that can drift apart.

The map read here is the GROUND TRUTH floorplan built by
`tools/build_ground_truth_map.py` from the world's own collision meshes. It is
not the robot's SLAM map and must never be fed to the robot: the whole point of
the mapping work is that the robot earns its map from its sensors. Scenario
figures are a different matter. A person walking through a warehouse knows the
building, so giving them the true floorplan is the accurate model, and it is
available in full at t = 0 instead of growing as the robot explores.
"""

import math
from pathlib import Path

import yaml


class Grid:
    """An occupancy grid with the queries a walker needs.

    Values follow the ROS convention: 0 to 100 is the occupancy probability and
    -1 is unknown. Unknown counts as BLOCKED everywhere here. That is the
    conservative reading and, on the ground truth map, unknown means a region
    the flood fill could not reach from the floor: the inside of a rack, or the
    world outside the walls. Neither is somewhere a person walks.
    """

    FREE_MAX = 30

    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        self.data = data
        self.w = width
        self.h = height
        self.res = resolution
        self.ox = origin_x
        self.oy = origin_y

    @property
    def bounds(self):
        return (self.ox, self.oy,
                self.ox + self.w * self.res, self.oy + self.h * self.res)

    def cell(self, x, y):
        i = int((x - self.ox) / self.res)
        j = int((y - self.oy) / self.res)
        if i < 0 or j < 0 or i >= self.w or j >= self.h:
            return -1
        return self.data[j * self.w + i]

    def free(self, x, y):
        v = self.cell(x, y)
        return 0 <= v <= self.FREE_MAX

    def clear(self, x, y, radius):
        """Free, with `radius` metres of free space all round."""
        steps = max(1, int(radius / self.res))
        for di in range(-steps, steps + 1):
            for dj in range(-steps, steps + 1):
                if di * di + dj * dj > steps * steps:
                    continue
                if not self.free(x + di * self.res, y + dj * self.res):
                    return False
        return True

    def segment_clear(self, x0, y0, x1, y1, radius, step=0.15):
        """Is the straight line from start to end clear at that radius?

        Sampled along the line rather than only at its ends, because a route
        whose endpoints are both in open aisle can still pass straight through
        a rack that sits between them.
        """
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(d / step))
        for k in range(n + 1):
            t = k / n
            if not self.clear(x0 + t * (x1 - x0), y0 + t * (y1 - y0), radius):
                return False
        return True


def load_map(yaml_path):
    """Load a map_server style PGM plus YAML pair into a Grid."""
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    image = yaml_path.parent / meta['image']
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))
    negate = int(meta.get('negate', 0))

    with image.open('rb') as f:
        if f.readline().strip() != b'P5':
            raise ValueError(f'{image} is not a binary PGM')
        dims = []
        while len(dims) < 3:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            dims += [int(t) for t in line.split()]
        w, h, maxval = dims[:3]
        pixels = f.read(w * h)

    # PGM rows run top down and a map's rows run bottom up, so the image is
    # flipped as it is read. Getting this wrong mirrors the whole warehouse,
    # which is subtle enough to survive a glance at the picture.
    data = [-1] * (w * h)
    for j in range(h):
        row = (h - 1 - j) * w
        for i in range(w):
            p = pixels[row + i]
            occ = p / maxval if negate else (maxval - p) / maxval
            if occ >= occupied_thresh:
                data[j * w + i] = 100
            elif occ <= free_thresh:
                data[j * w + i] = 0
            else:
                data[j * w + i] = -1
    return Grid(data, w, h, res, ox, oy)
