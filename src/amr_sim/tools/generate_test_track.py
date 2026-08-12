#!/usr/bin/env python3
"""Generate a warehouse test track whose dimensions come from the platform spec.

WHY THIS WORLD EXISTS

The other world, `warehouse.sdf`, is the AWS RoboMaker small warehouse imported
from Gazebo Classic. It is a FOUND asset: nobody sized it for this vehicle.
Measured at robot height its corridors have a median width of 1.34 m and a 25th
percentile of 0.64 m, which is narrower than the robot, so about a quarter of
that building is impassable by construction. See V-22.

That makes results ambiguous in a way no amount of care fixes. When a cycle
fails you cannot separate "the navigation is wrong" from "this aisle is 0.64 m",
because the floor plan is itself an uncontrolled variable. A full day of
debugging was spent testing hypotheses against a building that was part of the
question.

So this world is an INSTRUMENT rather than scenery. Every dimension the vehicle
is scored on is a figure the platform's own datasheet publishes, taken from
`validation_targets` in the platform spec, so the track cannot drift from the
claims it is testing:

    aisle 1     corridor_width_default    the aisle the sheet quotes
    aisle 2     corridor_width_dynamic    the headline claim
    corner      corridor_width_90_turn    a 90 degree turn, muted fields
    doorway     doorway_width_default     the one gap with a hard frame

One transport cycle traverses aisle 2, the corner and the doorway in series, so
the headline result stops being "4 of 5 cycles", which nobody can judge, and
becomes "which of the four published corridor figures the vehicle achieves, and
why it misses the ones it misses".

WHAT IS NOT DERIVED FROM THE SHEET, and is labelled as such below: the pinch
aisle and the open bay, which are the MEASURED median and 75th percentile of the
AWS building. They are here so the designed world keeps the real one's
difficulty rather than replacing it with something comfortable.

THE TRAP THIS MUST NOT FALL INTO. A world built until the robot passes is the
same failure as pedestrians that avoid the robot: it manufactures a result. V-22
already forbids widening the aisles to make a claim work. Designing a new
instrument is legitimate; quietly loosening the old one is not. Both worlds stay
in the repository and both get run.

BOTH OUTCOMES ARE DESIGNED IN. The open bay is wide enough that a vehicle can
route around a person standing in it; aisle 2 is not. The same pedestrian
behaviour therefore produces a re-route in one place and a correct wait in the
other, which is a stronger result than a demo tuned so it always succeeds.
"""

import argparse
import math
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
SPEC_DIR = (Path(__file__).resolve().parents[2]
            / 'amr_description' / 'config' / 'platforms')

# ---- fixed building geometry ---------------------------------------------
# The shell is a plain rectangle. Its size is a consequence of the zones that
# have to fit inside it, not a number with meaning of its own.
INTERIOR_X = 24.0          # m
INTERIOR_Y = 12.0          # m
WALL_T = 0.20              # m
WALL_H = 3.00              # m

RACK_DEPTH = 1.20          # m, standard pallet depth
RACK_HEIGHT = 2.20         # m
RACK_X0 = 7.0              # m, racking field starts here
RACK_X1 = 17.0             # m, and ends here

# The measured figures from V-22, carried so the designed world keeps the real
# building's difficulty. NOT from any datasheet, and labelled accordingly.
PINCH_WIDTH = 1.340        # m, median corridor of the AWS warehouse
OPEN_BAY_MIN = 2.300       # m, 75th percentile of the same


def zones(spec):
    """The zone table. Every width is a figure with a stated origin."""
    t = spec['validation_targets']
    return [
        # name        width                        origin
        ('aisle_1',   t['corridor_width_default'], 'published: corridor_width_default'),
        ('aisle_2',   t['corridor_width_dynamic'], 'published: corridor_width_dynamic'),
        ('pinch',     PINCH_WIDTH,                 'measured: AWS median, V-22'),
        ('corner',    t['corridor_width_90_turn'], 'published: corridor_width_90_turn'),
        ('doorway',   t['doorway_width_default'],  'published: doorway_width_default'),
        ('open_bay',  OPEN_BAY_MIN,                'measured: AWS p75, V-22'),
    ]


def layout(spec):
    """Solve the y positions of the racking rows from the aisle widths.

    Built from the TOP of the building downward, so an aisle width change moves
    the racking rather than silently eating into the next aisle. The three
    aisles are stacked with a rack between each pair.
    """
    t = spec['validation_targets']
    a1 = t['corridor_width_default']
    a2 = t['corridor_width_dynamic']
    a3 = PINCH_WIDTH

    top = INTERIOR_Y - 1.75          # leave a cross-passage along the north wall
    rows = []
    y = top
    for name, w in (('rack_a', RACK_DEPTH), ('aisle_1', a1),
                    ('rack_b', RACK_DEPTH), ('aisle_2', a2),
                    ('rack_c', RACK_DEPTH), ('pinch', a3),
                    ('rack_d', RACK_DEPTH)):
        rows.append((name, y - w, y))       # (name, y_lo, y_hi)
        y -= w
    return {name: (lo, hi) for name, lo, hi in rows}


def box(name, x, y, z, sx, sy, sz, colour):
    """A static collision-and-visual box. Everything here is a box on purpose.

    The AWS import exists for scenery that looks like a warehouse. This world is
    an instrument, and a mesh that is 3 cm different from its collision shape
    would put that difference straight into the measurement.
    """
    r, g, b = colour
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world(spec, platform):
    lay = layout(spec)
    t = spec['validation_targets']
    corner_w = t['corridor_width_90_turn']
    door_w = t['doorway_width_default']

    models = []
    grey = (0.55, 0.56, 0.54)
    steel = (0.42, 0.45, 0.50)

    # ---- shell ------------------------------------------------------------
    hx, hy = INTERIOR_X / 2.0, INTERIOR_Y / 2.0
    models.append(box('wall_south', hx, -WALL_T / 2, WALL_H / 2,
                      INTERIOR_X + 2 * WALL_T, WALL_T, WALL_H, grey))
    models.append(box('wall_north', hx, INTERIOR_Y + WALL_T / 2, WALL_H / 2,
                      INTERIOR_X + 2 * WALL_T, WALL_T, WALL_H, grey))
    models.append(box('wall_west', -WALL_T / 2, hy, WALL_H / 2,
                      WALL_T, INTERIOR_Y, WALL_H, grey))
    models.append(box('wall_east', INTERIOR_X + WALL_T / 2, hy, WALL_H / 2,
                      WALL_T, INTERIOR_Y, WALL_H, grey))

    # ---- racking rows -----------------------------------------------------
    rack_len = RACK_X1 - RACK_X0
    rack_cx = (RACK_X0 + RACK_X1) / 2.0
    for name in ('rack_a', 'rack_b', 'rack_c', 'rack_d'):
        lo, hi = lay[name]
        models.append(box(name, rack_cx, (lo + hi) / 2.0, RACK_HEIGHT / 2.0,
                          rack_len, hi - lo, RACK_HEIGHT, steel))

    # ---- the 90 degree corner --------------------------------------------
    # The cross aisle runs north-south at the east end of the racking, and its
    # WIDTH is the published corner figure. A vehicle leaving aisle 2 has to
    # turn through 90 degrees inside exactly that much space, which is what the
    # datasheet row actually claims.
    cross_x0 = RACK_X1
    cross_x1 = RACK_X1 + corner_w
    a2_lo, a2_hi = lay['aisle_2']
    a1_lo, a1_hi = lay['aisle_1']

    # ---- doorway ----------------------------------------------------------
    # Two blocks with the published gap between them, east of the cross aisle,
    # at the height of aisle 1 so the route must turn north through the corner
    # before it can reach the door.
    door_lo = a1_lo + (a1_hi - a1_lo - door_w) / 2.0
    door_hi = door_lo + door_w
    blk_x0, blk_x1 = cross_x1, cross_x1 + 1.0
    models.append(box('door_block_north', (blk_x0 + blk_x1) / 2.0,
                      (door_hi + INTERIOR_Y) / 2.0, WALL_H / 2.0,
                      blk_x1 - blk_x0, INTERIOR_Y - door_hi, WALL_H, grey))
    models.append(box('door_block_south', (blk_x0 + blk_x1) / 2.0,
                      door_lo / 2.0, WALL_H / 2.0,
                      blk_x1 - blk_x0, door_lo, WALL_H, grey))

    stations = {
        'goods_in': (2.5, (a2_lo + a2_hi) / 2.0, 0.0),
        'dispatch': (INTERIOR_X - 2.5, (door_lo + door_hi) / 2.0, 0.0),
    }

    # Every solid rectangle, in world coordinates, for the ground truth map and
    # the pedestrian scenario. Collected here rather than re-parsed from the SDF
    # so the map cannot disagree with the world it describes.
    solids = [
        (RACK_X0, RACK_X1, lay[n][0], lay[n][1])
        for n in ('rack_a', 'rack_b', 'rack_c', 'rack_d')
    ] + [
        (blk_x0, blk_x1, door_hi, INTERIOR_Y),
        (blk_x0, blk_x1, 0.0, door_lo),
    ]

    derived = {
        'solids': solids,
        'aisle_1_y': lay['aisle_1'],
        'aisle_2_y': lay['aisle_2'],
        'pinch_y': lay['pinch'],
        'corner_x': (cross_x0, cross_x1),
        'doorway_y': (door_lo, door_hi),
        'stations': stations,
    }

    header = ['Generated by amr_sim/tools/generate_test_track.py from',
              f'amr_description/config/platforms/{platform}.yaml.',
              'Do not hand-edit: change the spec or the generator.',
              '',
              'Zone widths and where each number comes from:']
    for name, width, origin in zones(spec):
        header.append(f'  {name:<10} {width:.3f} m   {origin}')

    comment = '\n'.join(f'     {line}' for line in header)
    body = ''.join(models)

    return f"""<?xml version="1.0"?>
<!-- {comment.lstrip()}
-->
<sdf version="1.10">
  <world name="test_track">

    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
      <cast_shadows>false</cast_shadows>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.32 0.33 0.32 1</ambient>
            <diffuse>0.36 0.37 0.36 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{body}  </world>
</sdf>
""", derived


def build_truth_map(derived):
    """Rasterise the track into a ground truth occupancy map.

    NOT reconstructed from the SDF. `build_ground_truth_map.py` exists for the
    AWS world and rasterises collision meshes, which is the only option when the
    layout came from an asset pack. Here the layout is generated, so the map is
    emitted from the SAME zone table that emitted the world and cannot disagree
    with it.

    The ground truth map is measurement only. It scores results and must never
    reach the control path, which is why it lives in amr_sim rather than in
    amr_navigation.
    """
    res = MASK_RES
    w = int(round(INTERIOR_X / res))
    h = int(round(INTERIOR_Y / res))
    grid = bytearray([254]) * (w * h)        # 254 = free in the map_server sense

    for x0, x1, y0, y1 in derived['solids']:
        i0, i1 = max(0, int(x0 / res)), min(w, int(math.ceil(x1 / res)))
        j0, j1 = max(0, int(y0 / res)), min(h, int(math.ceil(y1 / res)))
        for j in range(j0, j1):
            row = (h - 1 - j) * w            # PGM rows run top down
            for i in range(i0, i1):
                grid[row + i] = 0            # 0 = occupied

    pgm = b'P5\n' + f'{w} {h}\n255\n'.encode() + bytes(grid)
    meta = {
        'image': 'test_track_truth.pgm',
        'resolution': res,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    return pgm, meta, (w, h)


def build_scenario(derived):
    """Pedestrians placed where the geometry decides the outcome.

    The two zones exist so the SAME behaviour produces different results:

        P1, open bay      wide enough for the vehicle to route around a person
        P2, scored aisle  not wide enough, so the correct outcome is to wait

    A scenario whose people are all in the open bay would only ever demonstrate
    re-routing, which is the flattering half of the story. One of each is what
    makes the pair a measurement instead of a demo.
    """
    a2_lo, a2_hi = derived['aisle_2_y']
    a2_mid = (a2_lo + a2_hi) / 2.0
    a1_lo, a1_hi = derived['aisle_1_y']

    return {
        'name': 'track_people',
        'description': ('Pedestrians on the generated test track: one in the '
                        'open bay where a re-route is possible, one in the '
                        'scored aisle where it is not'),
        'people': [
            # P1. In the open bay, on the vehicle's line out of goods_in, with
            # room either side for it to pass once this person stops.
            {'name': 'walker_bay', 'x': 4.6, 'y': a2_mid, 'yaw': 3.14159,
             'wander': {'speed': 0.9, 'range': 3.0}},
            # P2. Inside the 1.000 m aisle. There is no gap here for a vehicle
            # of this size, so a vehicle that tries to squeeze past is wrong and
            # one that waits is right.
            {'name': 'walker_aisle', 'x': 12.0, 'y': a2_mid, 'yaw': 0.0,
             'wander': {'speed': 0.7, 'range': 2.5}},
            # A third on the wide aisle, so the run is not two set pieces with
            # nothing else moving.
            {'name': 'walker_wide', 'x': 10.0, 'y': (a1_lo + a1_hi) / 2.0,
             'yaw': 3.14159, 'wander': {'speed': 1.1, 'range': 4.0}},
            # Stationary, for the same reason the AWS scenario has one: a person
            # who does not move is the case the motion test cannot tell from
            # structure, and leaving it out flatters the tracker.
            {'name': 'worker_standing', 'x': 3.0, 'y': 9.5, 'yaw': 3.14159},
        ],
    }


MASK_RES = 0.05            # m/cell, matches the SLAM map
MASK_PAD = 6.0             # m of margin around the building on every side


def build_keepout_mask():
    """An ALL-FREE keepout mask, and the emptiness is the point.

    The keepout filter exists because the AWS warehouse's racking is see-through
    at scan height: a 2D scanner at 150 mm reads a rack as a row of thin legs
    with drivable gaps between them, and the planner believed those gaps and
    wedged the vehicle under the shelving. The mask states what is permanently
    forbidden regardless of what the sensors think.

    None of that applies here. Every obstacle on this track is a solid box 2.2 m
    tall, which the scanner sees as a wall because it is one. There is nothing a
    keepout zone would add that the scan does not already provide, so declaring
    forbidden areas would be inventing a constraint to look thorough.

    The mask is still EMITTED, all free, rather than the filter being switched
    off. A costmap that declares a filter and never receives its mask is the
    failure that ran a full five-cycle mission with no keepout zones at all
    while preflight reported every check passing, see V-25. An always-free mask
    is honest about the track and keeps the plumbing identical between worlds.

    Generously padded, so it covers the map frame wherever SLAM happens to put
    the origin. Outside the mask the filter treats space as free anyway; the
    padding means the vehicle is never near that edge.
    """
    w = int(round((INTERIOR_X + 2 * MASK_PAD) / MASK_RES))
    h = int(round((INTERIOR_Y + 2 * MASK_PAD) / MASK_RES))
    # PGM P5, one byte per cell, 255 = free under the `scale` mode below.
    pgm = b'P5\n' + f'{w} {h}\n255\n'.encode() + bytes([255]) * (w * h)
    meta = {
        'image': 'keepout_mask_test_track.pgm',
        'resolution': MASK_RES,
        'origin': [-MASK_PAD, -MASK_PAD, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'mode': 'scale',
    }
    return pgm, meta, (w, h)


def build_stations(derived, spec):
    """Station poses, with approach headings DERIVED rather than hand-authored.

    The approach heading is the direction the vehicle must be facing on arrival.
    Both stations here are approached along the aisle they sit on, so the
    heading follows from the geometry instead of being typed in. On the AWS
    world these were hand-authored against one platform, and the leg that kept
    failing was the one whose approach pose nobody had re-derived.
    """
    gi = derived['stations']['goods_in']
    dp = derived['stations']['dispatch']
    return {
        'stations': [
            {'name': 'goods_in', 'x': round(gi[0], 3), 'y': round(gi[1], 3),
             'yaw': 0.0,
             'note': 'west end of aisle 2, approached heading east'},
            {'name': 'dispatch', 'x': round(dp[0], 3), 'y': round(dp[1], 3),
             'yaw': 0.0,
             'note': 'east of the doorway, approached heading east'},
        ],
        'route': ['goods_in', 'dispatch'],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--platform', default='mir250_class')
    ap.add_argument('--out', type=Path)
    ap.add_argument('--stations-out', type=Path)
    args = ap.parse_args()

    spec_file = SPEC_DIR / f'{args.platform}.yaml'
    if not spec_file.is_file():
        sys.exit(f'no platform spec at {spec_file}')
    spec = yaml.safe_load(spec_file.read_text())

    text, derived = build_world(spec, args.platform)

    # FAIL IF THE TRACK CANNOT HOLD THE VEHICLE IT IS FOR. A zone narrower than
    # the vehicle is not a hard test, it is an impossible one, and it would be
    # measured as a navigation failure.
    v = spec['values']
    width = 2.0 * v['scanner_mount_y']
    for name, w, origin in zones(spec):
        if name in ('corner', 'open_bay'):
            continue
        if w <= width:
            sys.exit(f'zone {name} is {w:.3f} m against a vehicle {width:.3f} m '
                     f'wide; the track would be testing nothing')

    out = args.out or (PKG / 'worlds' / f'test_track.{args.platform}.sdf')
    out.write_text(text)
    print(f'wrote {out}')

    st = args.stations_out or (
        Path(__file__).resolve().parents[2] / 'amr_mission' / 'config'
        / f'stations.test_track.{args.platform}.yaml')
    header = ('# GENERATED by amr_sim/tools/generate_test_track.py alongside the\n'
              '# world. Approach poses are derived from the track geometry, not\n'
              '# hand-authored. Do not edit: change the generator.\n')
    st.write_text(header + yaml.safe_dump(build_stations(derived, spec),
                                          sort_keys=False))
    print(f'wrote {st}')

    maps = Path(__file__).resolve().parents[2] / 'amr_navigation' / 'maps'
    pgm, meta, (mw, mh) = build_keepout_mask()
    (maps / 'keepout_mask_test_track.pgm').write_bytes(pgm)
    (maps / 'keepout_mask_test_track.yaml').write_text(
        '# GENERATED by amr_sim/tools/generate_test_track.py.\n'
        '# ALL FREE on purpose: every obstacle on this track is a solid box the\n'
        '# scanner sees directly, so a keepout zone would add nothing. Emitted\n'
        '# rather than the filter being disabled, because a costmap that\n'
        '# declares a filter and never receives its mask is the silent failure\n'
        '# recorded in V-25.\n'
        + yaml.safe_dump(meta, sort_keys=False))
    print(f'wrote {maps / "keepout_mask_test_track.yaml"}  ({mw} x {mh} cells, all free)')

    tpgm, tmeta, (tw, th) = build_truth_map(derived)
    (PKG / 'maps' / 'test_track_truth.pgm').write_bytes(tpgm)
    (PKG / 'maps' / 'test_track_truth.yaml').write_text(
        '# GENERATED by amr_sim/tools/generate_test_track.py from the same zone\n'
        '# table as the world, so it cannot disagree with the geometry it\n'
        '# describes. MEASUREMENT ONLY: the ground truth map scores results and\n'
        '# must never reach the control path.\n'
        + yaml.safe_dump(tmeta, sort_keys=False))
    print(f'wrote {PKG / "maps" / "test_track_truth.yaml"}  ({tw} x {th} cells)')

    scen = PKG / 'scenarios' / 'track_people.yaml'
    scen.write_text(
        '# GENERATED by amr_sim/tools/generate_test_track.py alongside the\n'
        '# world. Spawn points are placed against the zones, so P1 lands where a\n'
        '# re-route is possible and P2 where it is not. Do not hand-edit.\n'
        + yaml.safe_dump(build_scenario(derived), sort_keys=False))
    print(f'wrote {scen}')

    for name, w, origin in zones(spec):
        print(f'  {name:<10} {w:.3f} m   {origin}')
    for key in ('aisle_1_y', 'aisle_2_y', 'pinch_y', 'doorway_y'):
        lo, hi = derived[key]
        print(f'  {key:<12} {lo:.3f} to {hi:.3f}  ({hi - lo:.3f} m)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
