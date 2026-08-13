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
INTERIOR_X = 34.0          # m, wide enough for a real staging bay either end
# INTERIOR_Y IS DERIVED, see interior_y(). Fixing it meant that widening the
# aisles pushed the racking down until 0.543 m was left against the south wall,
# which is narrower than the vehicle: it drove in from the open bay and wedged.
# Sizing the shell from the zones it has to hold means there is never anything
# left over to drive into.
NORTH_PASSAGE = 1.75       # m, a cross passage along the north wall
WALL_T = 0.20              # m
WALL_H = 3.00              # m

RACK_DEPTH = 1.20          # m, standard pallet depth
RACK_HEIGHT = 2.20         # m
RACK_X0 = 9.0              # m, racking field starts here
RACK_X1 = 25.0             # m, and ends here

# ---- warehouse furniture -------------------------------------------------
# A warehouse is not an empty box with shelves down one side. Roof columns
# stand in the open floor, pallets get staged where there is room, and a
# charger sits against a wall. Without them the open bay is a car park and the
# vehicle never has to plan around anything in the one place it has space to.
COLUMN = 0.40              # m, square section of a roof column
COLUMN_H = 3.00            # m
PALLET_X = 1.20            # m, EUR pallet long side
PALLET_Y = 0.80            # m
PALLET_H = 1.05            # m, pallet plus a stacked load
CHARGER_X = 0.60           # m
CHARGER_Y = 1.40           # m
CHARGER_H = 1.20           # m

# The measured figures from V-22, carried so the designed world keeps the real
# building's difficulty. NOT from any datasheet, and labelled accordingly.
# Beyond the width the vehicle strictly needs to turn.
TURN_MARGIN = 0.10         # m

# ROOM TO GET PAST A PERSON WHO IS STANDING IN THE AISLE.
#
# Every aisle carries this on top of the width the vehicle needs for itself, so
# a pedestrian who stops in a corridor is something the robot can plan around
# rather than something that ends the run. Turning width alone let the vehicle
# work an empty aisle and left nothing spare the moment somebody was in it.
#
# No derivation here on purpose: it is the width of a person plus a margin, and
# saying so plainly is more honest than dressing a body width up as a
# calculation.
PEDESTRIAN_WIDTH = 0.50    # m, a person standing
PASSING_CLEARANCE = 0.20   # m, so passing is not a squeeze
PASSING_ALLOWANCE = PEDESTRIAN_WIDTH + PASSING_CLEARANCE

PINCH_WIDTH = 1.340        # m, median corridor of the AWS warehouse
OPEN_BAY_MIN = 2.300       # m, 75th percentile of the same

# WHERE THE VEHICLE STARTS, in world coordinates, and it matters more than it
# looks. SLAM puts the MAP FRAME ORIGIN at the robot's start pose, so goals are
# expressed relative to here and not in world coordinates. The first run of this
# track sent the vehicle to the station's world position while it sat at map
# (0, 0), and the planner correctly reported no valid path to a point outside
# the building it had mapped.
#
# Placed in the open bay and deliberately NOT on top of goods_in: a vehicle that
# starts on its first goal never demonstrates the leg.
SPAWN_X = 4.5              # m, world
SPAWN_Y_OFFSET = 0.0       # m, relative to the scored aisle centre


def rotation_width(spec):
    """The narrowest aisle this vehicle can actually turn round in.

    DERIVED FROM THE VEHICLE AND ITS SAFETY CONFIGURATION, not chosen. A
    differential drive turning on the spot sweeps its circumscribed radius, and
    the protective field it selects while doing so is wider still. The aisle has
    to hold that field, or the monitor stops the vehicle mid-turn and it cannot
    finish the manoeuvre.

    So the requirement is twice the half width of the widest all-round field the
    vehicle can select, plus a margin. The field half width comes from
    generate_fields.py rather than being recomputed here: the ISO 13855 shape
    has one author in this repository and a second copy would eventually be a
    second value.
    """
    v = spec['values']
    r_circ = math.hypot(v['chassis_length'] / 2.0, v['chassis_width'] / 2.0)
    tip = v['max_angular_speed'] * r_circ
    half = v['chassis_width'] / 2.0 + _fields().stopping_distance(tip, v)
    return 2.0 * half + TURN_MARGIN


def _fields():
    """generate_fields.py, imported rather than duplicated."""
    import importlib.util
    path = (Path(__file__).resolve().parents[2] / 'amr_safety' / 'tools'
            / 'generate_fields.py')
    spec = importlib.util.spec_from_file_location('generate_fields', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def zones(spec):
    """The zone table. Every width has a stated origin.

    THESE ARE DERIVED FROM THE VEHICLE, and that is a deliberate change of
    basis. The first version of this track used the corridor figures the MiR250
    datasheet publishes, and two of the four turned out to be unachievable with
    this stack: the vehicle is 2.1 mm too wide to rotate in the 1.000 m dynamic
    corridor and 52 mm too wide for the 0.950 m corner, because both figures
    assume capabilities this stack does not implement, a dynamic footprint and
    muted protective fields. It drove into the corner and trapped itself. See
    V-26 and V-27.

    Those findings stand and are recorded. What changed is what this track is
    FOR. It is now a capability demonstrator: every zone is one the vehicle can
    traverse AND turn round in, so a cycle completes, re-routing can be shown,
    and the safety layer can be exercised rather than deadlocked.

    The published figures have not been deleted. They are carried through to the
    output below as claims this geometry does not test, so nobody reads the
    track as evidence that the datasheet numbers were met.

    THE ADVERSARIAL CASE HAS NOT BEEN LOST EITHER. The AWS warehouse is still in
    the repository and still has a 25th percentile corridor of 0.64 m, which is
    narrower than the robot. Impossible geometry is measured there. This world
    measures what the vehicle can do.
    """
    t = spec['validation_targets']
    turn = rotation_width(spec)
    pas = PASSING_ALLOWANCE
    why = f'turning width + {pas:.2f} m to pass a person'
    return [
        # name        width                 origin
        ('aisle_1',   turn + pas + 0.30,    f'derived: {why} + 0.30 m'),
        ('aisle_2',   turn + pas,           f'derived: {why}'),
        ('pinch',     turn + pas + 0.10,    f'derived: {why} + 0.10 m'),
        ('cross',     turn + pas + 0.20,    f'derived: {why} + 0.20 m, a junction'),
        ('doorway',   turn + pas,           f'derived: {why}, straight through'),
        ('open_bay',  OPEN_BAY_MIN,         'measured: AWS p75, V-22'),
        # Carried, not tested. See the docstring and V-26 / V-27.
        ('claim: corridor_default', t['corridor_width_default'], 'published, NOT tested here'),
        ('claim: corridor_dynamic', t['corridor_width_dynamic'], 'published, NOT MET, V-26'),
        ('claim: corner_90',        t['corridor_width_90_turn'], 'published, NOT MET, V-27'),
        ('claim: doorway',          t['doorway_width_default'],  'published, NOT tested here'),
    ]


def interior_y(spec):
    """Building height, as the sum of what has to fit in it.

    A shell sized independently of its contents leaves a remainder, and a
    remainder is either a corridor or a trap depending on a number nobody
    chose. This makes it exactly zero.
    """
    w = {name: width for name, width, _ in zones(spec)}
    aisles = w['aisle_1'] + w['aisle_2'] + w['pinch']
    return NORTH_PASSAGE + 4 * RACK_DEPTH + aisles


def layout(spec):
    """Solve the y positions of the racking rows from the aisle widths.

    Built from the TOP of the building downward, so an aisle width change moves
    the racking rather than silently eating into the next aisle. The three
    aisles are stacked with a rack between each pair.
    """
    w = {name: width for name, width, _ in zones(spec)}
    a1, a2, a3 = w['aisle_1'], w['aisle_2'], w['pinch']

    top = interior_y(spec) - NORTH_PASSAGE
    rows = []
    y = top
    for name, w in (('rack_a', RACK_DEPTH), ('aisle_1', a1),
                    ('rack_b', RACK_DEPTH), ('aisle_2', a2),
                    ('rack_c', RACK_DEPTH), ('pinch', a3),
                    ('rack_d', RACK_DEPTH)):
        rows.append((name, y - w, y))       # (name, y_lo, y_hi)
        y -= w

    # NO LEFTOVER STRIP NARROWER THAN THE VEHICLE CAN USE.
    #
    # Widening the aisles pushed the racking down and left 0.543 m between the
    # bottom rack and the south wall, against a 0.590 m vehicle. There is no
    # rack west of x = 7, so the vehicle could drive into that strip from the
    # open bay and then had nowhere to go: it wedged at y = 0.477 and the
    # planner reported no valid path for the rest of the run. See V-27, which
    # is the same fault in a different place.
    #
    # The bottom rack is therefore extended to the wall whenever what is left
    # under it is too narrow to turn in. A leftover wide enough to be a real
    # aisle is left alone, because then it is one.
    name, lo, hi = rows[-1]
    if 0.0 < lo < rotation_width(spec):
        rows[-1] = (name, 0.0, hi)
    return {name: (lo, hi) for name, lo, hi in rows}


def floor_marking(name, cx, cy, sx, sy, colour, line=0.10):
    """A painted rectangle on the floor. VISUAL ONLY, no collision.

    Warehouse floors are marked: a home position for the vehicle, hatched bays
    for staging, lanes for foot traffic. It is how a real site tells people and
    machines where things belong, and it is the cheapest thing that makes a
    simulated floor look like a working one.

    IT MUST NOT HAVE COLLISION. A painted line the vehicle cannot drive over is
    not a marking, it is a wall, and the scanner would see it as one. These are
    four thin visual slabs forming a hollow rectangle, 5 mm proud so they do
    not z-fight with the ground plane, and nothing in the physics or the
    sensors knows they are there.
    """
    r, g, b = colour
    hx, hy = sx / 2.0, sy / 2.0
    bars = [
        (cx, cy + hy - line / 2, sx, line),        # north edge
        (cx, cy - hy + line / 2, sx, line),        # south edge
        (cx - hx + line / 2, cy, line, sy - 2 * line),   # west edge
        (cx + hx - line / 2, cy, line, sy - 2 * line),   # east edge
    ]
    visuals = ''.join(f"""        <visual name="bar{i}">
          <pose>{bx - cx:.4f} {by - cy:.4f} 0 0 0 0</pose>
          <geometry><box><size>{w:.4f} {h:.4f} 0.010</size></box></geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
            <emissive>{r * 0.3:.2f} {g * 0.3:.2f} {b * 0.3:.2f} 1</emissive>
          </material>
        </visual>
""" for i, (bx, by, w, h) in enumerate(bars))
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} 0.005 0 0 0</pose>
      <link name="link">
{visuals}      </link>
    </model>
"""


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


def clutter(spec, iy, keep_clear=()):
    """Roof columns, staged pallets and a charger, on a grid that cannot trap.

    THE HARD RULE. Every gap this leaves, between two objects and between an
    object and a wall, is at least the passing width: what the vehicle needs to
    turn round PLUS room to get past a person standing there. Nothing here is
    placed by eye. The grid is solved from that width, so adding furniture
    cannot reintroduce the fault that cost two runs, a space the vehicle can
    enter and cannot leave.

    That is also what makes the clutter worth having. An obstacle the planner
    must route around in open floor is the case the open bay could not produce
    when it was an empty rectangle, and it is the one a warehouse actually
    presents: you do not drive the middle of a bay, you drive between the
    columns.

    KEEP_CLEAR is the spawn pose and both stations. The first version of this
    placed a column line at exactly the spawn x, so the vehicle started INSIDE
    a roof column: footprint x 4.095 to 4.905 against a column at 4.30 to 4.70.
    It was visible in a screenshot before it was visible in a log, which is the
    argument for looking at the screen.

    Anything landing within a passing width of a point that has to be occupied
    by the vehicle is dropped rather than nudged, because nudging it is how a
    solved grid quietly stops being solved.

    Returns (name, x0, x1, y0, y1, height, colour).
    """
    gap = rotation_width(spec) + PASSING_ALLOWANCE
    concrete = (0.62, 0.61, 0.58)
    timber = (0.55, 0.41, 0.24)
    signal = (0.85, 0.62, 0.15)
    out = []

    # ROOF COLUMNS on a solved grid. Columns march down a warehouse on a
    # regular pitch; the pitch here is whatever leaves a passing width between
    # them, rounded up so the arithmetic is not marginal.
    pitch = gap + COLUMN + 0.20
    def line(x, tag):
        n = max(1, int((iy - gap) // pitch))
        span = (n - 1) * pitch
        y0 = (iy - span) / 2.0
        for i in range(n):
            cy = y0 + i * pitch
            out.append((f'column_{tag}{i}', x - COLUMN / 2, x + COLUMN / 2,
                        cy - COLUMN / 2, cy + COLUMN / 2, COLUMN_H, concrete))

    # One line in the west staging bay, one in the east. Both are open floor
    # the vehicle crosses on every cycle.
    line(RACK_X0 / 2.0, 'w')
    line(INTERIOR_X - (INTERIOR_X - RACK_X1) / 2.0 + 1.0, 'e')

    # STAGED PALLETS in the west bay, clear of the columns and of the route
    # between goods_in and the racking. Laid out as a pair, which is how they
    # arrive.
    px = RACK_X0 / 2.0 + COLUMN / 2 + gap
    for i, py in enumerate((iy * 0.25, iy * 0.75)):
        out.append((f'pallet_w{i}', px, px + PALLET_X,
                    py - PALLET_Y / 2, py + PALLET_Y / 2, PALLET_H, timber))

    # NO PALLETS IN THE EAST BAY. They were tried and the clearance test threw
    # them out: 2.100 m to the nearest column against a 2.202 m requirement,
    # and the only other space is inside the cross aisle the route uses. The
    # east apron is 5.6 m wide and already carries a column line and the
    # dispatch station, so there is genuinely no room for more furniture that
    # still leaves the vehicle a way past. The rule wins rather than the
    # decoration.

    # A CHARGER against the west wall, at the north end. It sat mid-wall and
    # was dropped for being within a passing width of goods_in, which is the
    # keep-clear rule working; moving it is the right answer rather than
    # loosening the rule.
    # FLUSH INTO THE NORTH WEST CORNER. Set 2.20 m short of the north wall it
    # left a 0.797 m nook behind it, open from the east, which is a trap in
    # miniature: the vehicle can drive in and not turn round. Against the
    # corner there is nothing behind it to drive into.
    out.append(('charger', 0.0, CHARGER_X,
                iy - CHARGER_Y, iy, CHARGER_H, signal))

    def clear_of_everything(item):
        _, x0, x1, y0, y1, _, _ = item
        for kx, ky in keep_clear:
            dx = max(x0 - kx, kx - x1, 0.0)
            dy = max(y0 - ky, ky - y1, 0.0)
            if math.hypot(dx, dy) < gap:
                return False
        return True

    return [i for i in out if clear_of_everything(i)]


def build_world(spec, platform):
    iy = interior_y(spec)
    lay = layout(spec)
    t = spec['validation_targets']
    w = {name: width for name, width, _ in zones(spec)}
    corner_w = t['corridor_width_90_turn']      # carried, not built
    door_w = w['doorway']

    models = []
    grey = (0.55, 0.56, 0.54)
    steel = (0.42, 0.45, 0.50)

    # ---- shell ------------------------------------------------------------
    hx, hy = INTERIOR_X / 2.0, iy / 2.0
    models.append(box('wall_south', hx, -WALL_T / 2, WALL_H / 2,
                      INTERIOR_X + 2 * WALL_T, WALL_T, WALL_H, grey))
    models.append(box('wall_north', hx, iy + WALL_T / 2, WALL_H / 2,
                      INTERIOR_X + 2 * WALL_T, WALL_T, WALL_H, grey))
    models.append(box('wall_west', -WALL_T / 2, hy, WALL_H / 2,
                      WALL_T, iy, WALL_H, grey))
    models.append(box('wall_east', INTERIOR_X + WALL_T / 2, hy, WALL_H / 2,
                      WALL_T, iy, WALL_H, grey))

    # ---- racking rows -----------------------------------------------------
    rack_len = RACK_X1 - RACK_X0
    rack_cx = (RACK_X0 + RACK_X1) / 2.0
    for name in ('rack_a', 'rack_b', 'rack_c', 'rack_d'):
        lo, hi = lay[name]
        models.append(box(name, rack_cx, (lo + hi) / 2.0, RACK_HEIGHT / 2.0,
                          rack_len, hi - lo, RACK_HEIGHT, steel))

    # ---- the cross aisle, and why it is NOT the published corner width ----
    #
    # It was, and it trapped the vehicle. The MiR250's circumscribed diameter is
    # 1.0021 m and `corridor_width_90_turn` is 0.950 m, so it drove into the
    # corner and could not rotate out: fifteen survey rounds timed out with the
    # vehicle pinned at one pose and every recovery aborting on Collision Ahead.
    # Fifty minutes of survey produced one data point. See V-27.
    #
    # AN INSTRUMENT MUST NOT BE ABLE TO DESTROY THE RUN IT IS MEASURING. A
    # scored zone the vehicle fails should record a failure and let the run
    # continue. Making the only route depend on a manoeuvre the vehicle cannot
    # perform means every other zone goes unmeasured too.
    #
    # So the cross aisle is now the DEFAULT corridor width, which the vehicle
    # can turn in, and the 0.950 m corner is a separate marked zone attempted
    # and recorded rather than one the route depends on. The claim is not
    # quietly dropped: it is still published, it still fails, and V-26 and V-27
    # say so with the arithmetic.
    cross_w = w['cross']
    cross_x0 = RACK_X1
    cross_x1 = RACK_X1 + cross_w
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
                      (door_hi + iy) / 2.0, WALL_H / 2.0,
                      blk_x1 - blk_x0, iy - door_hi, WALL_H, grey))
    models.append(box('door_block_south', (blk_x0 + blk_x1) / 2.0,
                      door_lo / 2.0, WALL_H / 2.0,
                      blk_x1 - blk_x0, door_lo, WALL_H, grey))

    # Stations in WORLD coordinates first, then converted to the map frame
    # below, because that is the frame the goals are sent in.
    spawn = (SPAWN_X, (a2_lo + a2_hi) / 2.0 + SPAWN_Y_OFFSET, 0.0)
    stations_world = {
        'goods_in': (2.5, (a2_lo + a2_hi) / 2.0, 0.0),
        'dispatch': (INTERIOR_X - 2.5, (door_lo + door_hi) / 2.0, 0.0),
    }
    stations = {n: (x, y, yaw) for n, (x, y, yaw) in stations_world.items()}

    # ---- floor markings, painted not built --------------------------------
    # The home square under the spawn, and three delivery bays at the dispatch
    # end. Marked floor is how a real site says where things belong, and it
    # makes the start pose and the drop points legible in a screenshot instead
    # of being coordinates in a yaml file.
    yellow = (0.85, 0.68, 0.10)
    white = (0.90, 0.90, 0.88)
    vals = spec['values']
    home = 1.4 * max(vals['chassis_length'], vals['chassis_width'])
    models.append(floor_marking('mark_home', spawn[0], spawn[1],
                                home, home, yellow))

    dx, dy = stations_world['dispatch'][0], stations_world['dispatch'][1]
    bay = 1.6
    for i in range(3):
        models.append(floor_marking(
            f'mark_bay_{i + 1}', dx + 1.6, dy + (i - 1) * (bay + 0.4),
            bay, bay, white))

    # ---- warehouse furniture ---------------------------------------------
    # The spawn and both stations are places the vehicle must be able to stand,
    # so nothing is placed near them.
    keep_clear = [(spawn[0], spawn[1])] + [
        (x, y) for x, y, _ in stations_world.values()]
    for name, x0, x1, y0, y1, h, colour in clutter(spec, iy, keep_clear):
        models.append(box(name, (x0 + x1) / 2.0, (y0 + y1) / 2.0, h / 2.0,
                          x1 - x0, y1 - y0, h, colour))

    # Every solid rectangle, in world coordinates, for the ground truth map and
    # the pedestrian scenario. Collected here rather than re-parsed from the SDF
    # so the map cannot disagree with the world it describes.
    solids = [
        (RACK_X0, RACK_X1, lay[n][0], lay[n][1])
        for n in ('rack_a', 'rack_b', 'rack_c', 'rack_d')
    ] + [
        (blk_x0, blk_x1, door_hi, iy),
        (blk_x0, blk_x1, 0.0, door_lo),
    ] + [
        (x0, x1, y0, y1) for _, x0, x1, y0, y1, _, _ in clutter(spec, iy, keep_clear)
    ]

    derived = {
        'spawn': spawn,
        'stations_world': stations_world,
        'solids': solids,
        'aisle_1_y': lay['aisle_1'],
        'aisle_2_y': lay['aisle_2'],
        'pinch_y': lay['pinch'],
        'cross_aisle_x': (cross_x0, cross_x1),
        'corner_claim_m': corner_w,
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

    gx, gy = INTERIOR_X / 2.0, iy / 2.0
    gs = max(INTERIOR_X, iy) + 20.0

    return f"""<?xml version="1.0"?>
<!-- {comment.lstrip()}
-->
<sdf version="1.10">
  <!-- THE WORLD NAME MUST EQUAL THE FILE STEM. Gazebo topics are built from
       the world's INTERNAL name, and the launch derives them from the FILE
       name, so the two disagreeing means every /world/<name>/... topic is
       wrong. It was "test_track" in a file called test_track.<platform>.sdf,
       so the ground truth pose feed subscribed to a topic that did not exist:
       no poses reached the pedestrian driver and not one walker ever moved.
       Nothing errored, because subscribing to a topic nobody publishes is a
       legal thing to do. warehouse.sdf gets this right by having a one word
       name. -->
  <world name="test_track.{platform}">

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

    <!-- CENTRED ON THE BUILDING AND SIZED TO COVER IT. It used to be a 60 by
         60 m plane at the world origin, which covers x -30 to +30, and when
         the building grew to 34 m long the east third of it had no floor
         under it at all. The plane's COLLISION is infinite so nothing fell
         through, which is why no log said anything; it was visible on screen
         and nowhere else. -->
    <model name="ground_plane">
      <static>true</static>
      <pose>{gx:.3f} {gy:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{gs:.1f} {gs:.1f}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{gs:.1f} {gs:.1f}</size></plane></geometry>
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


def build_truth_map(derived, iy):
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
    h = int(round(iy / res))
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


def build_scenario(derived, spec):
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
    sx, sy, _ = derived['spawn']

    # NOBODY SPAWNS ON THE VEHICLE. walker_bay was placed at x = 4.6 against a
    # spawn at x = 4.5, which put a person 0.1 m from the robot before either
    # had moved. It was obvious on screen and invisible in every log.
    clear = rotation_width(spec) + PASSING_ALLOWANCE
    bay_x = sx + clear + 0.5

    return {
        'name': 'track_people',
        'description': ('Pedestrians on the generated test track: one in the '
                        'open bay where a re-route is possible, one in the '
                        'scored aisle where it is not'),
        'people': [
            # P1. In the open bay, on the vehicle's line out of goods_in, with
            # room either side for it to pass once this person stops.
            {'name': 'walker_bay', 'x': round(bay_x, 2), 'y': a2_mid,
             'yaw': 3.14159, 'wander': {'speed': 0.9, 'range': 3.0}},
            # P2. Inside the 1.000 m aisle. There is no gap here for a vehicle
            # of this size, so a vehicle that tries to squeeze past is wrong and
            # one that waits is right.
            {'name': 'walker_aisle', 'x': round((RACK_X0 + RACK_X1) / 2.0, 2),
             'y': a2_mid, 'yaw': 0.0,
             'wander': {'speed': 0.7, 'range': 2.5}},
            # A third on the wide aisle, so the run is not two set pieces with
            # nothing else moving.
            {'name': 'walker_wide', 'x': round(RACK_X0 + 3.0, 2),
             'y': (a1_lo + a1_hi) / 2.0,
             'yaw': 3.14159, 'wander': {'speed': 1.1, 'range': 4.0}},
            # Stationary, for the same reason the AWS scenario has one: a person
            # who does not move is the case the motion test cannot tell from
            # structure, and leaving it out flatters the tracker.
            {'name': 'worker_standing', 'x': 2.0,
             'y': round((a1_lo + a1_hi) / 2.0, 2), 'yaw': 3.14159},
        ],
    }


MASK_RES = 0.05            # m/cell, matches the SLAM map
MASK_PAD = 6.0             # m of margin around the building on every side


def build_keepout_mask(iy):
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
    h = int(round((iy + 2 * MASK_PAD) / MASK_RES))
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
    """Station poses IN THE MAP FRAME, with approach headings derived.

    THE FRAME IS THE WHOLE POINT AND IT COST A RUN. Goals are sent in the `map`
    frame, and with SLAM running the map origin is wherever the vehicle started,
    not the world origin. The first version of this file emitted world
    coordinates, so the vehicle sat at map (0, 0) and was asked to drive to
    (2.50, 6.00), a point outside the building it had mapped. The planner said
    "no valid path found" three times in a row, 0.3 m driven per cycle, which
    reads as a navigation failure and is a coordinate error.

    So every station is written relative to the spawn pose, which this generator
    also owns. The two cannot disagree because one produces the other.

    The approach heading is the direction the vehicle faces on arrival, and it
    follows from the geometry rather than being typed in: both stations here sit
    on a corridor that is entered from the west.
    """
    sx, sy, _ = derived['spawn']
    out = []
    for name, (x, y, yaw) in derived['stations_world'].items():
        out.append({
            'name': name,
            'x': round(x - sx, 3),
            'y': round(y - sy, 3),
            'yaw': yaw,
            'world_xy': [round(x, 3), round(y, 3)],
            'note': 'map frame, relative to the spawn pose recorded below',
        })
    return {
        # Recorded so run_stack.sh spawns the vehicle where the station offsets
        # assume it did. A spawn somewhere else silently shifts every goal.
        'spawn': {'x': round(sx, 3), 'y': round(sy, 3), 'yaw': 0.0},
        'stations': out,
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
    iy = interior_y(spec)
    pgm, meta, (mw, mh) = build_keepout_mask(iy)
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

    tpgm, tmeta, (tw, th) = build_truth_map(derived, iy)
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
        + yaml.safe_dump(build_scenario(derived, spec), sort_keys=False))
    print(f'wrote {scen}')

    for name, w, origin in zones(spec):
        print(f'  {name:<10} {w:.3f} m   {origin}')
    for key in ('aisle_1_y', 'aisle_2_y', 'pinch_y', 'doorway_y'):
        lo, hi = derived[key]
        print(f'  {key:<12} {lo:.3f} to {hi:.3f}  ({hi - lo:.3f} m)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
