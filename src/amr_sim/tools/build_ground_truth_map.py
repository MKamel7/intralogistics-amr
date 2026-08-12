#!/usr/bin/env python3
"""Rasterise the warehouse world into a ground truth occupancy map.

WHY THIS EXISTS

Pedestrians used to plan their walks against the SLAM map on `/map`. That map is
what the ROBOT has discovered, so at startup it is almost nothing: measured, 88
m2 of which only 6.4 percent had the 0.45 m of clearance a walker needs, and all
three walkers spawned on cells that failed the test. They correctly refused to
move. Coupling the scenario to the robot's knowledge was the mistake. A person in
a real warehouse knows the building; only the robot has to discover it.

So the layout is taken from the world itself. Every collision mesh in
`warehouse.sdf` is loaded, transformed into world coordinates, sliced in a height
band and its footprint stamped into a grid. What comes out is what a surveyor
would draw: the true floorplan, complete and available at t = 0.

THE HEIGHT BAND

Only geometry between `z_lo` and `z_hi` blocks. The floor and the roof are
therefore not obstacles, which they would otherwise be, being large flat meshes
lying exactly in the plane the map is drawn on. The band's upper edge is above a
walking person, so a shelf beam at 1.6 m still blocks even though nothing at
scanner height does.

FLOOD FILL

Stamping footprints leaves everything the meshes do not cover marked free,
including the whole outdoors beyond the walls. So free space is instead grown by
flood fill from a seed inside the building. Anything the fill cannot reach stays
UNKNOWN, which is the honest label: it is not floor this scenario should use.
That also closes off the inside of a shelf, which is enclosed by its own mesh.

SECOND USE

The same file is the reference for scoring SLAM. An occupancy map with no ground
truth to compare against can only be judged by eye, and judging maps by eye is
how the previous mapping problems survived as long as they did.

Usage:
    python3 build_ground_truth_map.py [--resolution 0.05] [--show]
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

PKG = Path(__file__).resolve().parent.parent
WORLD = PKG / 'worlds' / 'warehouse.sdf'
MODELS = PKG / 'models'
OUT = PKG / 'maps'

# Height band that counts as an obstacle, in metres above the floor.
#   lower: above the floor mesh and above any threshold a wheel rolls over.
#   upper: above a standing person (1.69 m to the crown of the model), so
#          overhead structure that a person would walk into still blocks.
Z_LO, Z_HI = 0.06, 1.90

# Seed for the free-space flood fill: the robot's spawn, which is by definition
# on drivable floor inside the building.
SEED_XY = (2.0, -1.0)


def parse_pose(text):
    """SDF pose is `x y z roll pitch yaw`."""
    if not text:
        return np.eye(4)
    v = [float(t) for t in text.split()]
    v += [0.0] * (6 - len(v))
    x, y, z, r, p, yw = v[:6]
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(yw), math.sin(yw))
    m = np.eye(4)
    m[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    m[:3, 3] = (x, y, z)
    return m


def child_pose(elem):
    return parse_pose(elem.findtext('pose'))


def collada_unit(path):
    """Metres per unit, from the COLLADA `<asset><unit meter=...>` tag."""
    if path.suffix.lower() != '.dae':
        return 1.0
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return 1.0
    for unit in root.iter():
        if unit.tag.rsplit('}', 1)[-1] == 'unit':
            return float(unit.get('meter', 1.0))
    return 1.0


def load_collision_meshes(model_dir):
    """Every collision mesh of a model, in model coordinates."""
    sdf = model_dir / 'model.sdf'
    if not sdf.exists():
        return []
    root = ET.parse(sdf).getroot()
    out = []
    for model in root.iter('model'):
        m_pose = child_pose(model)
        for link in model.findall('link'):
            l_pose = m_pose @ child_pose(link)
            for col in link.findall('collision'):
                c_pose = l_pose @ child_pose(col)
                geom = col.find('geometry')
                if geom is None:
                    continue
                mesh_el = geom.find('mesh')
                if mesh_el is None:
                    # Primitives are rare in this world but cost nothing to
                    # support, and a missing one would be a silent hole.
                    prim = _primitive(geom)
                    if prim is not None:
                        prim.apply_transform(c_pose)
                        out.append(prim)
                    continue
                uri = (mesh_el.findtext('uri') or '').strip()
                if not uri.startswith('model://'):
                    continue
                rel = uri[len('model://'):]
                path = MODELS / rel
                if not path.exists():
                    print(f'    missing mesh {rel}', file=sys.stderr)
                    continue
                scene = trimesh.load(str(path), force='mesh', process=False)
                if scene is None or scene.is_empty:
                    continue
                # COLLADA carries its own unit. These meshes declare
                # `<unit meter="0.01" name="centimeter">`, Gazebo honours it and
                # trimesh 5.0 does not, so loading raw gave a warehouse 1.4 km
                # across and every model far outside the height band. Read the
                # declaration and apply it rather than hard coding a factor, so
                # a mesh authored in metres is not silently shrunk.
                scene.apply_scale(collada_unit(path))
                scale = mesh_el.findtext('scale')
                if scale:
                    s = [float(t) for t in scale.split()]
                    scene.apply_scale(s if len(s) == 3 else s[0])
                scene.apply_transform(c_pose)
                out.append(scene)
    return out


def _primitive(geom):
    box = geom.find('box')
    if box is not None:
        return trimesh.creation.box(
            extents=[float(t) for t in box.findtext('size').split()])
    cyl = geom.find('cylinder')
    if cyl is not None:
        return trimesh.creation.cylinder(radius=float(cyl.findtext('radius')),
                                         height=float(cyl.findtext('length')))
    sph = geom.find('sphere')
    if sph is not None:
        return trimesh.creation.icosphere(radius=float(sph.findtext('radius')))
    return None


def world_meshes():
    """Every collision mesh in the world, in world coordinates."""
    root = ET.parse(WORLD).getroot()
    world = root.find('world')
    cache = {}
    meshes = []
    for inc in world.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        if not uri.startswith('model://'):
            continue
        name = uri[len('model://'):]
        if name not in cache:
            cache[name] = load_collision_meshes(MODELS / name)
            print(f'  {name}: {len(cache[name])} collision mesh(es)')
        pose = child_pose(inc)
        for m in cache[name]:
            c = m.copy()
            c.apply_transform(pose)
            meshes.append((inc.findtext('name') or name, c))
    return meshes


def stamp(grid, mesh, res, ox, oy):
    """Mark every cell touched by this mesh inside the height band.

    Each triangle that spans the band is rasterised by sampling its own plane
    densely enough that no cell in it can be skipped. Sampling the triangle
    rather than its bounding box matters: a diagonal wall's bounding box is
    mostly empty floor, and filling it would wall off aisles that are open.
    """
    tris = mesh.triangles
    zmin = tris[:, :, 2].min(axis=1)
    zmax = tris[:, :, 2].max(axis=1)
    tris = tris[(zmax >= Z_LO) & (zmin <= Z_HI)]
    if len(tris) == 0:
        return 0

    h, w = grid.shape
    a = tris[:, 0, :]
    e1 = tris[:, 1, :] - a
    e2 = tris[:, 2, :] - a

    # Sample each triangle on a barycentric lattice fine enough that
    # neighbouring samples are under half a cell apart, so no cell inside a
    # triangle can be stepped over. The lattice size that needs is per triangle,
    # so triangles are bucketed by it into powers of two: a wall is a handful of
    # huge triangles and a crate is many small ones, and doing every triangle at
    # the wall's density would be an enormous amount of wasted sampling.
    longest = np.maximum(np.linalg.norm(e1, axis=1), np.linalg.norm(e2, axis=1))
    need = np.ceil(longest / (res * 0.5)).astype(int)
    need = np.clip(need, 2, 1024)
    bucket = np.power(2, np.ceil(np.log2(need)).astype(int))

    marked = 0
    for n in np.unique(bucket):
        sel = bucket == n
        u = np.linspace(0.0, 1.0, int(n))
        uu, vv = np.meshgrid(u, u)
        m = (uu + vv) <= 1.0
        bu, bv = uu[m], vv[m]
        # Chunk so the (triangles x samples x 3) block stays bounded.
        idx = np.flatnonzero(sel)
        per = max(1, int(4e6 / max(1, len(bu))))
        for s in range(0, len(idx), per):
            k = idx[s:s + per]
            pts = (a[k][:, None, :]
                   + bu[None, :, None] * e1[k][:, None, :]
                   + bv[None, :, None] * e2[k][:, None, :]).reshape(-1, 3)
            pts = pts[(pts[:, 2] >= Z_LO) & (pts[:, 2] <= Z_HI)]
            if len(pts) == 0:
                continue
            i = ((pts[:, 0] - ox) / res).astype(np.int32)
            j = ((pts[:, 1] - oy) / res).astype(np.int32)
            ok = (i >= 0) & (j >= 0) & (i < w) & (j < h)
            grid[j[ok], i[ok]] = 100
            marked += int(ok.sum())
    return marked


def flood(grid, seed_ij):
    """Free space reachable from the seed. Everything else stays unknown."""
    h, w = grid.shape
    free = np.zeros_like(grid, dtype=bool)
    si, sj = seed_ij
    if grid[sj, si] == 100:
        raise SystemExit(f'seed cell {SEED_XY} is inside an obstacle')
    stack = [(si, sj)]
    free[sj, si] = True
    while stack:
        i, j = stack.pop()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < w and 0 <= b < h and not free[b, a] and grid[b, a] != 100:
                free[b, a] = True
                stack.append((a, b))
    return free


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--resolution', type=float, default=0.05)
    # THE HEIGHT BAND IS PER AGENT, and conflating the two caused a real
    # misdiagnosis. The default band is a standing person. The VEHICLE is
    # 300 mm tall, so a shelf whose lowest shelf sits at 400 mm blocks a person
    # and not the robot, and judging the robot's position against the person
    # map reported it buried inside racking when it was in fact underneath an
    # overhang its own scanner could legitimately drive through.
    ap.add_argument('--z-lo', type=float, default=None)
    ap.add_argument('--z-hi', type=float, default=None)
    ap.add_argument('--name', default='warehouse_truth')
    ap.add_argument('--pad', type=float, default=0.5)
    args = ap.parse_args()
    res = args.resolution
    global Z_LO, Z_HI
    if args.z_lo is not None:
        Z_LO = args.z_lo
    if args.z_hi is not None:
        Z_HI = args.z_hi
    print(f'height band {Z_LO} to {Z_HI} m')

    print(f'reading {WORLD.name}')
    meshes = world_meshes()
    print(f'{len(meshes)} placed collision mesh(es)')

    band = []
    for _, m in meshes:
        lo, hi = m.bounds
        if hi[2] >= Z_LO and lo[2] <= Z_HI:
            band.append((lo, hi))
    if not band:
        raise SystemExit('nothing in the height band')
    lo = np.min([b[0] for b in band], axis=0) - args.pad
    hi = np.max([b[1] for b in band], axis=0) + args.pad

    ox, oy = float(lo[0]), float(lo[1])
    w = int(math.ceil((hi[0] - ox) / res))
    h = int(math.ceil((hi[1] - oy) / res))
    print(f'grid {w} x {h} at {res} m, origin ({ox:.2f}, {oy:.2f}), '
          f'covers x[{ox:.2f},{ox + w * res:.2f}] y[{oy:.2f},{oy + h * res:.2f}]')

    grid = np.zeros((h, w), dtype=np.int8)
    for name, m in meshes:
        n = stamp(grid, m, res, ox, oy)
        if n == 0:
            print(f'  {name}: nothing in band (floor or roof)')
    occupied = int((grid == 100).sum())
    print(f'stamped {occupied} occupied cells ({occupied * res * res:.1f} m2)')

    seed = (int((SEED_XY[0] - ox) / res), int((SEED_XY[1] - oy) / res))
    free = flood(grid, seed)
    out = np.full((h, w), -1, dtype=np.int8)
    out[free] = 0
    out[grid == 100] = 100
    n_free = int(free.sum())
    print(f'flood fill from {SEED_XY}: {n_free} free cells '
          f'({n_free * res * res:.1f} m2 of connected floor)')

    OUT.mkdir(exist_ok=True)
    # PGM: 254 free, 0 occupied, 205 unknown, rows written top down.
    img = np.full((h, w), 205, dtype=np.uint8)
    img[out == 0] = 254
    img[out == 100] = 0
    pgm = OUT / f'{args.name}.pgm'
    with pgm.open('wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.flipud(img).tobytes())
    (OUT / f'{args.name}.yaml').write_text(
        f'image: {args.name}.pgm\n'
        f'resolution: {res}\n'
        f'origin: [{ox:.4f}, {oy:.4f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        # 0.196, not 0.25, and the difference is not cosmetic. The PGM writes
        # unknown as 205, which decodes to an occupancy of (255-205)/255 =
        # 0.196 exactly. The ROS convention sets free_thresh to that same value
        # so unknown lands on the boundary and stays unknown. At 0.25 every
        # unknown cell decodes as FREE, which inflated this map's floor from
        # 236.2 m2 to 312.8 m2 and would have quietly flattered every coverage
        # number measured against it.
        'free_thresh: 0.196\n'
        'mode: trinary\n')
    print(f'wrote {pgm.name} and {args.name}.yaml to {OUT}')

    # The number that actually decides whether walkers can wander: how much of
    # the floor has a walker's clearance around it.
    for radius in (0.35, 0.45, 0.60):
        s = int(radius / res)
        k = np.ones((2 * s + 1, 2 * s + 1), dtype=bool)
        yy, xx = np.mgrid[-s:s + 1, -s:s + 1]
        k &= (xx * xx + yy * yy) <= s * s
        pad = np.pad(free, s, constant_values=False)
        clear = np.ones_like(free)
        for dj in range(-s, s + 1):
            for di in range(-s, s + 1):
                if not k[dj + s, di + s]:
                    continue
                clear &= pad[s + dj:s + dj + h, s + di:s + di + w]
        c = int(clear.sum())
        print(f'  floor with {radius:.2f} m clearance: {c * res * res:7.1f} m2 '
              f'({c / max(1, n_free) * 100:.1f} percent of floor)')


if __name__ == '__main__':
    main()
