#!/usr/bin/env python3
"""Film the vehicle with cameras that live in the scene.

WHY THIS EXISTS, AND WHAT IT REPLACES

The first demo video was made by screen-grabbing the Gazebo window with
import(1) in a loop. That produced 4.3 frames per second, which was then
encoded at 10, so the footage was both jerky and 2.3 times real time. The
vehicle was a few dozen pixels wide because the window also contained an entity
tree, a physics panel and a toolbar. And the shot could not be reproduced: it
depended on where the window happened to be and where the orbit camera happened
to be left.

A camera sensor inside the simulation fixes all of that at once, and one of the
fixes is worth stating plainly because it was the worst of the four faults:

  THE FRAMES CARRY SIMULATION TIMESTAMPS. A screen grab samples the wall
  clock, so its playback speed is a function of machine load and there is no
  record of what the load was; the old video ran 2.3 times fast and no amount
  of careful editing could have recovered the true speed. A camera sensor
  stamps every frame with the simulated time it was rendered at, so the true
  speed is knowable from the file itself.

Knowable, note, and not automatic. A camera ASKED for 30 Hz does not deliver
it: measured over a real recording the median gap was 36 ms, the tenth
percentile 0 ms and the ninetieth 132 ms, because the renderer keeps up when
the scene is cheap and falls behind when it is not. Encoding those frames at 30
fps would run about 1.75 times fast, which is the same error as before wearing
different clothes.

`conform` is what makes the claim true: it resamples the frames onto a uniform
grid using their own stamps, holding the newest frame at or before each slot.
Slow is then genuinely fine. Four 720p cameras drag the real time factor to
about 0.05 on this laptop, which costs wall-clock patience and changes nothing
about the output.

HOW SHOTS ARE CHOSEN

The other half of this file exists because of how the first cut was edited. Its
"transport cycle" segment showed a vehicle that was nearly stationary, because
it came from a cycle that failed to reach goods_in and spent the segment held by
protective stops. Its delivery segment began after the box had already been
placed. Both were timestamps picked without looking at the frames.

So the recorder writes a per-frame log: for every camera, every frame, the
vehicle's distance, whether it is inside that camera's frustum, how many pixels
across it is, and how fast it is moving. Choosing a shot is then a query over
that log rather than a guess, and `--report` prints the ranked candidates. A
segment in which the vehicle is stationary or out of frame cannot be selected by
accident, because the selector can see both.

The cameras are static, have no collision geometry, and sit at heights well
above the 0.110 m scan plane. They are tripods in the building: they are not
part of the vehicle, not part of the safety concept, and cannot appear in a
scan. `/ground_truth/poses` is used here for CAMERA AIMING AND SHOT SELECTION
only, which is measurement, never control. See ADR 0004.
"""

import argparse
import csv
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

VEHICLE = 'amr'

# nav2_msgs/CollisionMonitorState action_type. Same mapping as
# tools/classify_stops.py, which is the other reader of this topic.
ACTION = {0: 'clear', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}

# Worst first. Used to name an episode after the strongest thing the monitor
# did in it, and to rank the safety beat.
SEVERITY = {'stop': 0, 'slowdown': 1, 'approach': 2, 'limit': 3, 'clear': 4}

# --- the shot list ---------------------------------------------------------
#
# Positions are WORLD frame, in metres. They were not found by flying the GUI
# camera around and writing down where it ended up. They came out of a search
# over the TRUTH OCCUPANCY GRID (src/amr_sim/maps/*_truth.pgm), which is the
# only artefact in the project that knows where the building's obstacles are:
#
#   1. reject any position whose 0.45 m surround is not free, so the tripod is
#      not standing inside a shelf
#   2. reject any position closer than 1.2 m to the route, so the vehicle does
#      not pass through the lens
#   3. for each candidate, march a ray across the grid to every point on the
#      route and count the points that are BOTH inside the frustum and not
#      occluded by racking
#   4. rank by unoccluded points times the length of route covered
#
# Step 3 is the one that a frustum check alone does not give. A camera can have
# the vehicle perfectly framed and still be looking at the back of a shelf, and
# that is not visible in a shot list, only in the footage.
#
# The warehouse route is goods_in (0.42, -6.45) to dispatch (1.17, 1.65) in the
# world frame, an 8.1 m north-south run. `look` is a point the camera is aimed
# at, so a shot is described by where the operator stands and what they point
# at; yaw and pitch follow. Roll is always zero, because a level horizon is not
# a stylistic choice.
#
# THE HEIGHTS AND AIM POINTS WERE SET FROM THE STILLS, not from the search. The
# search happily returned cameras at 1.70 m aimed at a route point two metres
# away, which is a 28 degree downward pitch: the frames were two thirds bare
# concrete, and worse, a vehicle passing 1.2 m from the lens sits 48 degrees
# below the axis and falls out of the bottom of a 42 degree vertical frame. A
# camera can score perfectly on coverage and still be a shot of a floor.
#
# So every camera now sits near the vehicle's own height and aims at a FAR
# route point, which flattens the pitch to a few degrees, keeps the vehicle
# near the middle of the frame for the whole pass, and leaves the racking and
# the building visible behind it.
WAREHOUSE = [
    # Down the aisle, head on. The vehicle drives at the lens for the length of
    # the route.
    dict(name='aisle', pos=(1.00, 4.50, 0.90), look=(0.42, -6.45, 0.35),
         fov=1.05),

    # A close pass, 1.2 m off the route.
    dict(name='pass', pos=(-0.25, 0.75, 0.65), look=(0.42, -6.45, 0.40),
         fov=1.20),

    # The dispatch end, looking back down the route it arrives along.
    dict(name='bay', pos=(3.00, 2.25, 0.85), look=(0.42, -6.45, 0.40),
         fov=1.20),

    # The goods_in end, looking back up the route it departs along.
    #
    # COMPROMISED, and kept as the example of what the search cannot see. A
    # rack upright stands just off the lens and blocks the right third of the
    # frame. The occupancy raycast passed it: the grid is 5 cm and a rack post
    # is thinner than that in places, so the ray slipped by, and a post one
    # metre from the camera hides an angular slice out of all proportion to
    # the number of cells it occupies.
    #
    # The route is genuinely unoccluded from here, exactly as scored. The shot
    # is still unusable. That is the difference between an occupancy grid and a
    # frame, and the only instrument that tells them apart is looking.
    dict(name='south', pos=(2.75, -7.75, 0.80), look=(1.17, 1.65, 0.40),
         fov=1.20),
]

# The test track is a different building: 42 by 13, four 16 m racks centred
# x=17, a 2.01 m aisle at y=5.51 running x=9 to 25, and a 37 m route from
# goods_in (2.5, 5.507) to dispatch (39.5, 6.482).
TEST_TRACK = [
    dict(name='wide', pos=(1.5, -3.0, 9.5), look=(19.0, 6.0, 0.0), fov=1.20),
    dict(name='aisle', pos=(26.2, 5.51, 1.15), look=(10.0, 5.51, 0.35),
         fov=1.05),
    dict(name='pass', pos=(17.0, 6.35, 0.75), look=(9.5, 5.30, 0.30),
         fov=1.20),
    dict(name='bay', pos=(35.2, 2.60, 1.70), look=(39.4, 7.60, 0.40),
         fov=1.15),
]

SHOTS_BY_WORLD = {'warehouse': WAREHOUSE}

# CAMERA MODELS ARE NAMESPACED, because a shot name is not unique in a world.
#
# The test track contains a zone marker model called `aisle` and a shot called
# `aisle`, so spawning the camera collided with the building. `create` exits 0
# when the name is taken, so the only reason this surfaced at all is the pose
# readback below, which reported the simulator holding a yaw of -180 degrees
# for a camera that had been asked for -0.0. Without that check the shot would
# have filmed from wherever the marker happens to sit.
#
# The TOPIC keeps the bare name, so `/film/aisle` still identifies the shot.
MODEL_PREFIX = 'film_'

SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{model}">
    <static>true</static>
    <link name="link">
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>{fov:.4f}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.10</near><far>80.0</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>{rate}</update_rate>
        <visualize>false</visualize>
        <topic>film/{name}</topic>
      </sensor>
    </link>
  </model>
</sdf>
"""


def angle_error(got, want):
    """Smallest angle between two headings, in radians.

    THE POSE CHECK COMPARED ANGLES BY SUBTRACTION, and a camera aimed straight
    down an east-west aisle is at exactly pi. `atan2` returns +pi, the
    simulator reports -pi, the difference is 6.28 rad against a 1e-3 tolerance,
    and the best shot in the test track was rejected every time as though the
    spawn had silently failed. The pose was correct in every digit.

    That is the same class of error the check exists to catch, pointed the
    other way: there a wrong pose was reported as success, here a right one is
    reported as failure. A tolerance on a circle has to be measured on the
    circle.
    """
    return abs((got - want + math.pi) % (2.0 * math.pi) - math.pi)


def aim(pos, look):
    """Yaw and pitch that point a camera's +X axis from `pos` at `look`.

    Positive pitch about +Y takes +X toward -Z under the right hand rule, so
    a camera above its target has POSITIVE pitch. Getting this backwards aims
    at the ceiling, which is obvious in one frame and so is not worth a test.
    """
    dx, dy, dz = look[0] - pos[0], look[1] - pos[1], look[2] - pos[2]
    return math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))


def frustum(pos, yaw, pitch, fov, aspect, target, radius):
    """Is a sphere at `target` inside this camera's view, and how big?

    Returns (visible, distance, pixels_across). `pixels_across` is what makes
    the difference between a shot of the vehicle and a shot of a building that
    happens to contain one: a 40 pixel vehicle is a speck, which is exactly
    what the first cut of the video shipped.
    """
    dx, dy, dz = target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-6:
        return False, dist, 0.0
    # Rotate the target into the camera frame. The camera's orientation is
    # Rz(yaw) . Ry(pitch), so a world vector becomes Ry(-pitch) . Rz(-yaw) . v.
    #
    # The pitch half of this was inverted on the first attempt, which is
    # equivalent to using -pitch and agrees with `aim` only when pitch is zero.
    # Every level shot therefore looked correct while the one high shot saw
    # nothing at all. Written out rather than folded into cos(-pitch) so the
    # sign is visible.
    cy, sy = math.cos(yaw), math.sin(yaw)
    x1, y1 = dx * cy + dy * sy, -dx * sy + dy * cy
    cp, sp = math.cos(pitch), math.sin(pitch)
    fwd, up = x1 * cp - dz * sp, x1 * sp + dz * cp
    if fwd <= 0.10:
        return False, dist, 0.0            # behind the lens or inside near clip
    vfov = 2.0 * math.atan(math.tan(fov / 2.0) / aspect)
    # Allow the sphere's radius to hang over the edge: a vehicle half in frame
    # is still a usable shot, a vehicle fully outside it is not.
    slack_h = math.atan2(radius, fwd)
    ok = (abs(math.atan2(y1, fwd)) <= fov / 2.0 + slack_h and
          abs(math.atan2(up, fwd)) <= vfov / 2.0 + slack_h)
    px = 2.0 * radius / (2.0 * fwd * math.tan(fov / 2.0))
    return ok, dist, px


def gz_call(args, timeout=10.0):
    """Run a gz or ros2 command, and DEGRADE TO A WARNING rather than dying.

    `ros_gz_sim set_entity_pose` once hung and killed a mission at its first
    delivery because a helper raised TimeoutExpired. Filming is not the system
    under test, so nothing here is allowed to take the stack down with it.
    """
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return False, 'timed out'
    except OSError as exc:
        return False, str(exc)


def drop(name, world):
    """Remove a model, via the SERVICE rather than the ros_gz_sim wrapper.

    `ros2 run ros_gz_sim remove -world W -entity N` exits 0 and leaves the
    model exactly where it was. The service below actually removes it and
    answers `data: true`. Two tools in this sequence report success without
    doing anything, which is why every pose is checked afterwards.
    """
    return gz_call(['gz', 'service', '-s', f'/world/{world}/remove',
                    '--reqtype', 'gz.msgs.Entity',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                    '--req', f'name: "{name}", type: MODEL'], timeout=15.0)[0]


def models(world):
    """Every model name the simulator currently holds."""
    ok, out = gz_call(['gz', 'model', '--list'], timeout=15.0)
    if not ok:
        return set()
    return {m.group(1) for m in re.finditer(r'^\s*-\s+(\S+)\s*$', out,
                                            re.MULTILINE)}


def pose_of(name, world):
    """The pose the simulator actually holds for a model, or None.

    This exists because `ros_gz_sim create` EXITS 0 WHEN THE NAME IS ALREADY
    TAKEN. Re-running the tool after changing a camera's height therefore
    printed the new pose, reported success, and filmed from the old one: the
    stills came back byte for byte identical and the only reason it was caught
    is that a 29 degree pitch change has to be visible and was not.
    """
    ok, out = gz_call(['gz', 'model', '-m', name, '-p'], timeout=10.0)
    if not ok:
        return None
    nums = re.findall(r'\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\]', out)
    if len(nums) < 2:
        return None
    xyz = tuple(float(v) for v in nums[0])
    rpy = tuple(float(v) for v in nums[1])
    return xyz, rpy


def spawn(shots, world, size, rate, workdir):
    """Write one SDF per shot and spawn it. Returns the shots that took.

    Every camera is removed first and its pose is CHECKED AFTER SPAWNING,
    because neither of those steps can be trusted to its exit code.
    """
    w, h = size
    live = []

    # One listing, one removal pass, one confirmation. Polling `gz model -p`
    # per camera took two seconds a call and turned a spawn into four minutes.
    present = models(world)
    stale = [MODEL_PREFIX + s['name'] for s in shots
             if MODEL_PREFIX + s['name'] in present]
    for name in stale:
        drop(name, world)
    if stale:
        for _ in range(10):
            time.sleep(1.0)
            if not (set(stale) & models(world)):
                break
        else:
            print(f'  WARNING: {sorted(set(stale) & models(world))} would not '
                  f'go away; their poses will fail the check below')

    for s in shots:
        yaw, pitch = aim(s['pos'], s['look'])
        path = os.path.join(workdir, s['name'] + '.sdf')
        with open(path, 'w') as fh:
            fh.write(SDF.format(model=MODEL_PREFIX + s['name'],
                                name=s['name'], fov=s['fov'], w=w, h=h,
                                rate=rate))
        gz_call([
            'ros2', 'run', 'ros_gz_sim', 'create', '-world', world,
            '-file', path, '-name', MODEL_PREFIX + s['name'],
            '-x', f"{s['pos'][0]:.4f}", '-y', f"{s['pos'][1]:.4f}",
            '-z', f"{s['pos'][2]:.4f}",
            '-R', '0', '-P', f'{pitch:.6f}', '-Y', f'{yaw:.6f}'], timeout=30.0)

        got = pose_of(MODEL_PREFIX + s['name'], world)
        if got is None:
            print(f"  {s['name']:6s} FAILED: no such model after create")
            continue
        (gx, gy, gz_), (_, gp, gyw) = got
        off = math.dist((gx, gy, gz_), s['pos'])
        # 1 mm and 1 mrad. The failure this catches is not a small error, it is
        # the whole previous pose, so the tolerance only has to exclude float
        # formatting.
        if (off > 1e-3 or angle_error(gp, pitch) > 1e-3
                or angle_error(gyw, yaw) > 1e-3):
            print(f"  {s['name']:6s} FAILED: simulator holds "
                  f"({gx:.2f}, {gy:.2f}, {gz_:.2f}) pitch "
                  f"{math.degrees(gp):.1f} yaw {math.degrees(gyw):.1f}, "
                  f"not what was asked for. A camera of this name probably "
                  f"already existed; create exits 0 either way.")
            continue
        s = dict(s, yaw=yaw, pitch=pitch)
        live.append(s)
        print(f"  {s['name']:6s} at {s['pos']} yaw {math.degrees(yaw):7.1f} "
              f"pitch {math.degrees(pitch):6.1f}  (pose confirmed)")
    return live


def stills(shots, size, outdir):
    """One frame per camera, then stop.

    The cheapest possible version of the check that both cuts of the demo
    skipped: LOOK AT THE FRAME. A camera aimed into the back of a shelf, a
    pitch sign that points at the ceiling, and a colour channel swap are all
    obvious here and all invisible in a shot list.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from sensor_msgs.msg import Image
    import cv2
    import numpy as np

    w, h = size
    got = {}

    class Grab(Node):
        def __init__(self):
            super().__init__('film_stills', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.VOLATILE, depth=2)
            for s in shots:
                n = s['name']
                self.create_subscription(Image, f'/film/{n}',
                                         lambda m, n=n: self._one(m, n), qos)

        def _one(self, msg, name):
            if name in got or msg.width != w or msg.height != h:
                return
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            path = os.path.join(outdir, f'still_{name}.png')
            cv2.imwrite(path, img[:, :, ::-1])
            # Report the mean level too. A frame that is 3 percent grey is a
            # camera inside a wall, and it reads as "captured" to anything that
            # only checks the file exists. That is exactly how a 5.5 KB file of
            # solid black was once reported as a working recording.
            got[name] = (path, float(img.mean()))

    rclpy.init()
    node = Grab()
    stop = time.time() + 30.0
    while time.time() < stop and len(got) < len(shots):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    for s in shots:
        n = s['name']
        if n in got:
            path, mean = got[n]
            flag = '  <-- nearly black, check this camera' if mean < 12 else ''
            print(f'  {n:6s} {path}  mean level {mean:5.1f}{flag}')
        else:
            print(f'  {n:6s} NO FRAME arrived')
    return 0 if len(got) == len(shots) else 1


def record(shots, size, rate, duration, outdir):
    """Subscribe to every camera, write a video and a per-frame log each."""
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage
    from nav2_msgs.msg import CollisionMonitorState
    import cv2
    import numpy as np

    IMG_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=2)

    w, h = size
    aspect = float(w) / float(h)

    class Camera(Node):
        def __init__(self):
            # Every probe in this project sets this. Without it the node runs
            # on wall time while the frames carry sim stamps, and the log
            # becomes a comparison between two different clocks.
            super().__init__('film_recorder',
                             parameter_overrides=[
                                 Parameter('use_sim_time', value=True)])
            self.writers = {}
            self.logs = {}
            self.rows = {}
            self.counts = {s['name']: 0 for s in shots}
            self.stamps = {s['name']: [] for s in shots}
            self.shots = {s['name']: s for s in shots}
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            for s in shots:
                n = s['name']
                self.writers[n] = cv2.VideoWriter(
                    os.path.join(outdir, f'{n}.mp4'), fourcc, float(rate),
                    (w, h))
                fh = open(os.path.join(outdir, f'{n}.csv'), 'w', newline='')
                self.logs[n] = fh
                self.rows[n] = csv.writer(fh)
                self.rows[n].writerow(['frame', 'sim_t', 'visible', 'dist_m',
                                       'px_across', 'speed_mps', 'veh_x',
                                       'veh_y', 'action', 'polygon',
                                       'person_m'])
                self.create_subscription(
                    Image, f'/film/{n}',
                    lambda m, n=n: self._frame(m, n), IMG_QOS)

            # WHAT THE SAFETY MONITOR WAS DOING, frame by frame.
            #
            # A caption saying "the protective field stops it" is a claim about
            # the monitor, and until now the only evidence available at cut
            # time was the vehicle's speed. Speed cannot tell a protective stop
            # from a goal arrival, a planner pause or a spot turn between
            # waypoints, so a beat labelled as safety could easily have been
            # none of it. classify_stops.py sees the real thing but only
            # aggregates it, and joining its output to the frames would mean
            # reconciling two processes' clocks.
            #
            # So the recorder subscribes to the monitor itself and stamps every
            # frame with the action in force and the polygon that selected it.
            # The claim and the picture then come out of one file, on one
            # clock, and `--events` can only select a stop that actually was
            # one.
            self.action = 'clear'
            self.polygon = ''
            self.create_subscription(CollisionMonitorState,
                                     '/collision_monitor_state',
                                     self._monitor, 20)

            # NEAREST PERSON, so that avoiding one can be selected rather
            # than asserted. A vehicle driving past somebody and a vehicle
            # driving down an empty aisle look identical in the speed log, and
            # the difference is the whole claim.
            self.person = None
            self.people = {}

            self.veh = None          # (x, y, z)
            self.speed = 0.0
            self.prev = None         # (t, x, y)
            self.create_subscription(
                TFMessage, '/ground_truth/poses', self._truth,
                QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           history=QoSHistoryPolicy.KEEP_LAST,
                           durability=QoSDurabilityPolicy.VOLATILE, depth=10))
            self.t_first = None
            self.t_last = None

        def _nearest_person(self):
            if self.veh is None or not self.people:
                return None
            return min(math.dist((x, y), (self.veh[0], self.veh[1]))
                       for x, y in self.people.values())

        def _monitor(self, msg):
            self.action = ACTION.get(msg.action_type, 'unknown')
            self.polygon = msg.polygon_name or ''

        def _truth(self, msg):
            for tf in msg.transforms:
                cid = tf.child_frame_id
                if cid.startswith('walker') or 'worker' in cid:
                    self.people[cid] = (tf.transform.translation.x,
                                        tf.transform.translation.y)
                if cid != VEHICLE:
                    continue
                p = tf.transform.translation
                t = (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
                if self.prev is not None:
                    dt = t - self.prev[0]
                    if dt > 1e-3:
                        self.speed = math.hypot(p.x - self.prev[1],
                                                p.y - self.prev[2]) / dt
                        self.prev = (t, p.x, p.y)
                else:
                    self.prev = (t, p.x, p.y)
                self.veh = (p.x, p.y, p.z)

        def _frame(self, msg, name):
            if msg.height != h or msg.width != w:
                return
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                img = buf.reshape((msg.height, msg.width, 3))
            except ValueError:
                return
            # Gazebo publishes rgb8; OpenCV writes bgr8. Getting this wrong
            # turns the yellow vehicle blue, which is subtle enough on a grey
            # floor to survive a careless look.
            self.writers[name].write(img[:, :, ::-1])
            i = self.counts[name]
            self.counts[name] = i + 1
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.stamps[name].append(t)
            if self.t_first is None:
                self.t_first = t
            self.t_last = t
            s = self.shots[name]
            if self.veh is None:
                self.rows[name].writerow([i, f'{t:.3f}', 0, '', '', '', '', '',
                                          self.action, self.polygon, ''])
                return
            # 0.55 m: the vehicle's own diagonal half extent, so `px_across`
            # is roughly how many pixels of frame the vehicle occupies.
            vis, dist, px = frustum(s['pos'], s['yaw'], s['pitch'], s['fov'],
                                    aspect, self.veh, 0.55)
            pd = self._nearest_person()
            self.rows[name].writerow([
                i, f'{t:.3f}', int(vis), f'{dist:.3f}', f'{px * w:.1f}',
                f'{self.speed:.3f}', f'{self.veh[0]:.3f}', f'{self.veh[1]:.3f}',
                self.action, self.polygon,
                '' if pd is None else f'{pd:.3f}'])

        def close(self):
            for n, wr in self.writers.items():
                wr.release()
            for fh in self.logs.values():
                fh.close()

    rclpy.init()
    node = Camera()
    stop = time.time() + duration
    try:
        while time.time() < stop:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.close()
    span = (node.t_last - node.t_first) if node.t_first is not None else 0.0
    print(f'\nrecorded {span:.1f} s of SIMULATED time '
          f'in {duration:.0f} s of wall clock '
          f'(real time factor about {span / max(duration, 1e-6):.2f})')
    for s in shots:
        n = s['name']
        c = node.counts[n]
        st = node.stamps[n]
        # A uniform cadence is what makes the playback real time. If frames
        # were dropped, say so here rather than claiming real time later.
        gaps = [b - a for a, b in zip(st, st[1:])]
        worst = max(gaps) if gaps else 0.0
        nominal = 1.0 / rate
        drops = sum(1 for g in gaps if g > nominal * 1.5)
        print(f'  {n:6s} {c:5d} frames, worst gap {worst * 1000:6.1f} ms '
              f'(nominal {nominal * 1000:.1f}), {drops} gap(s) over 1.5x')
    node.destroy_node()
    rclpy.shutdown()
    return 0


def count_frames(path):
    """How many frames a file holds, without decoding any of them."""
    try:
        out = subprocess.run(
            ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
             '-count_packets', '-show_entries', 'stream=nb_read_packets',
             '-of', 'csv=p=0', path],
            capture_output=True, text=True, timeout=120).stdout.strip()
        return int(out.split(',')[0])
    except (OSError, ValueError, subprocess.TimeoutExpired):
        import cv2
        cap = cv2.VideoCapture(path)
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        return max(n, 0)


def logs(outdir):
    """Every camera's per-frame log in a recording, by camera name."""
    out = {}
    for path in sorted(os.listdir(outdir)):
        if path.endswith('.csv'):
            with open(os.path.join(outdir, path)) as fh:
                # A row without a stamp is the half written last line of a
                # recording still in progress, which is exactly when this gets
                # read to see whether the shoot is worth continuing.
                out[path[:-4]] = [r for r in csv.DictReader(fh)
                                  if (r.get('sim_t') or '').strip()]
    return out


def slices(ts, want, step):
    """Index ranges covering `want` SIMULATED seconds, every `step` seconds.

    THE UNIT THAT SELECTION AND CUTTING HAVE TO SHARE, and for a while they did
    not. `--report` counted frames and divided by the nominal rate, so it
    offered a start of "90.0"; `conform` reads that number as simulated seconds
    since the camera's first frame. Those are the same number only if the
    camera delivers exactly `rate` frames per simulated second, which is the
    one thing this file's docstring says it never does. Measured on the
    recording this was found in: 610 frames over 43.0 s of simulated time on
    one camera and 305 over 44.5 s on another, so the printed offsets were
    short by a factor of 2.1 and 4.4.

    The selector was therefore scoring one moment and the cut was taking a
    different one, with nothing in the output to show it: the clip still ran at
    true speed, still held the vehicle in frame often enough to look deliberate,
    and simply was not the window that had been ranked. Both ends now work in
    simulated seconds, which is the clock the frames carry.
    """
    import bisect
    out = []
    i = 0
    n = len(ts)
    while i < n:
        if ts[i] + want > ts[-1]:
            break
        stop = bisect.bisect_right(ts, ts[i] + want)
        out.append((i, stop))
        nxt = bisect.bisect_left(ts, ts[i] + step)
        i = max(i + 1, nxt)
    return out


def hold_indices(stamps, start, length, rate):
    """Source frame index for each slot of a uniform output grid.

    Zero order hold: slot k at time start + k/rate takes the newest frame whose
    stamp is at or before it. Separated from the video handling so it can be
    tested on synthetic stamps, which is the only way to check the timing
    against a case whose answer is known by hand.
    """
    out = []
    i = 0
    n = len(stamps)
    if not n:
        return out
    t0 = stamps[0] + start
    for k in range(int(round(length * rate))):
        t = t0 + k / float(rate)
        while i + 1 < n and stamps[i + 1] <= t:
            i += 1
        out.append(i)
    return out


def conform(outdir, name, start, length, rate, dest):
    """Cut a clip and put it on a UNIFORM timeline using the sim stamps.

    The camera is asked for 30 Hz of simulated time and does not deliver it.
    Measured over a real recording: median gap 36 ms, tenth percentile 0 ms
    (frames sharing a stamp), ninetieth 132 ms, worst 396 ms, and only about a
    third of gaps within 5 ms of the median. The renderer keeps up when the
    scene is cheap and falls behind when it is not.

    So "played back at 30 fps it is real time by construction" is true of the
    REQUEST and not of the FILE. Encoding these frames at 30 fps would run
    about 1.75 times fast, which is the same class of error as the screen grab
    it replaced, arrived at from the opposite direction: there I sampled wall
    clock and pretended it was uniform, here I would take an irregular sim
    clock and pretend the same.

    The frames carry their own timestamps, so the fix is to resample rather
    than to assume. For each slot on a uniform grid take the newest frame at or
    before it, a zero order hold. Slow stretches repeat a frame and fast ones
    drop one, and the clip runs at true speed either way.
    """
    import cv2

    with open(os.path.join(outdir, f'{name}.csv')) as fh:
        stamps = [float(r['sim_t']) for r in csv.DictReader(fh)]
    # ONLY THE FRAMES THE CLIP NEEDS ARE KEPT. Decoding the whole file into a
    # list costs 2.7 MB a frame at 720p, so a twenty five minute recording is
    # tens of gigabytes and the machine starts swapping while ffmpeg is still
    # waiting. The output indices are known before any pixels are read, so the
    # decoder walks the file once and keeps only the span between the first and
    # last of them.
    total = count_frames(os.path.join(outdir, f'{name}.mp4'))
    n = min(total, len(stamps))
    if n:
        idx = hold_indices(stamps[:n], start, length, rate)
        lo, hi = (min(idx), max(idx)) if idx else (0, -1)
        cap = cv2.VideoCapture(os.path.join(outdir, f'{name}.mp4'))
        frames = {}
        k = 0
        while k <= hi:
            ok, img = cap.read()
            if not ok:
                break
            if k >= lo:
                frames[k] = img
            k += 1
        cap.release()
    else:
        frames, idx = {}, []
    if not n:
        print(f'{name}: nothing to cut')
        return None
    # The recorder writes the CSV row and the video frame together, so a length
    # mismatch means one of them was truncated, not that they are misaligned.
    if abs(total - len(stamps)) > 2:
        print(f'  {name}: {total} frames but {len(stamps)} log rows, '
              f'using {n}')
    if not idx or min(idx) not in frames:
        print(f'  {name}: +{start:.1f}s is past the end of the recording')
        return None
    first = frames[min(idx)]
    out = cv2.VideoWriter(dest, cv2.VideoWriter_fourcc(*'mp4v'), float(rate),
                          (first.shape[1], first.shape[0]))
    for i in idx:
        out.write(frames.get(i, first))
    out.release()
    used = set(idx)
    print(f'  {name} +{start:5.1f}s for {length:4.1f}s -> {len(used):4d} '
          f'distinct source frames over {int(length * rate)} slots')
    return dest


def report(outdir, rate, want, min_px, min_speed, max_px):
    """Rank the windows in which the vehicle is actually visible and moving.

    This is the part that the first cut of the video did by eye, and got wrong
    twice in four segments. A window scores on how much of it has the vehicle
    in frame, how big it is, and whether it is MOVING; a stationary vehicle
    scores zero however well framed it is.
    """
    best = []
    for name, rows in sorted(logs(outdir).items()):
        ts = [float(r['sim_t']) for r in rows]
        if len(ts) < 2 or ts[-1] - ts[0] < want:
            print(f'{name}: only {ts[-1] - ts[0] if ts else 0:.1f} s of '
                  f'simulated time, shorter than {want:.0f} s')
            continue
        for start, stop in slices(ts, want, 0.5):
            win = rows[start:stop]
            vis = [r for r in win if r['visible'] == '1' and r['px_across']]
            # A WINDOW WITH ALMOST NO FRAMES IN IT is not a good shot however
            # well it scores. The frame rate collapses to a few Hz when the
            # scene is expensive, and a window holding four frames would be
            # four stills held for two seconds each.
            if len(win) < max(4, want * 3) or len(vis) < 0.75 * len(win):
                continue
            px = sum(float(r['px_across']) for r in vis) / len(vis)
            sp = sum(float(r['speed_mps']) for r in vis if r['speed_mps']) / \
                max(1, len([r for r in vis if r['speed_mps']]))
            # TOO CLOSE IS ALSO A BAD SHOT, and it outranks everything else
            # if the score is size times speed: the first ranking was topped
            # by windows where the vehicle was 2827 px across on a 1280 px
            # frame, which is it driving over the lens.
            if px < min_px or px > max_px or sp < min_speed:
                continue
            best.append((px * sp, name, ts[start] - ts[0], px, sp,
                         len(vis) / float(len(win))))
    best.sort(reverse=True)
    print(f'\n{"cam":8s} {"ss":>7s} {"px":>7s} {"m/s":>6s} {"inframe":>8s}')
    for score, name, ss, px, sp, frac in best[:18]:
        print(f'{name:8s} {ss:7.1f} {px:7.0f} {sp:6.2f} {frac * 100:7.0f}%')
    if not best:
        print('nothing passed the gates; loosen --min-px or --min-speed')
    return best


def events(outdir, want, min_px, max_px, lead):
    """Windows where the SAFETY MONITOR FIRED and the vehicle was in frame.

    `--report` ranks on size and motion, which is what a establishing shot
    needs and is the wrong instrument for the one beat that makes a claim. A
    caption reading "a person steps out and the protective field stops it" is a
    statement about the collision monitor, and motion alone cannot support it:
    the vehicle also stops when it reaches a goal, when the planner is thinking
    and between waypoints, and all four look identical from a camera on a wall.

    The recorder stamps every frame with the action the monitor had in force
    and the polygon that selected it, so a stop can be found rather than
    inferred, and the polygon name says which field did it. A window is offered
    only if the stop happened inside it while the vehicle was framed.

    The window opens `lead` seconds BEFORE the monitor fires, because the beat
    is the approach and the stop together. Starting it on the stop itself gives
    a clip of a vehicle that is already stationary, which is the mistake the
    first cut of this video made in a different form.

    AND THE VEHICLE HAS TO ACTUALLY COME TO REST. This gate was missing, and
    the video shipped without it: the chosen window had thirteen frames the
    monitor called `stop` and a ground truth speed that never fell below
    0.44 m/s, so a badge reading PROTECTIVE STOP sat over a vehicle driving
    past at three quarters of a metre a second. The monitor asserting stop and
    the vehicle stopping are different events. Most assertions are momentary,
    which is what `103 protective stops per cycle, 7 % of cycle time` in the
    README describes, and a viewer cannot see one.

    So an episode qualifies only if the speed drops below HALT_SPEED for at
    least HALT_MIN seconds inside the window, and episodes rank by how long the
    vehicle is genuinely held. Selecting on the log alone was not enough; the
    log was right and the claim was still false.
    """
    # AND IT HAS TO BE DRIVING FIRST. Requiring a halt alone swung the
    # selection to the other extreme: the best scoring window had the vehicle
    # at rest for all eight seconds, which is a photograph of a parked robot,
    # not a vehicle being stopped. The beat is approach then stop, so a window
    # must contain both.
    HALT_SPEED, HALT_MIN, MOVE_SPEED, MOVE_MIN = 0.05, 0.5, 0.20, 1.5
    found = []
    for name, rows in sorted(logs(outdir).items()):
        ts = [float(r['sim_t']) for r in rows]
        if len(ts) < 2:
            continue
        t0 = ts[0]
        i = 0
        while i < len(rows):
            if (rows[i].get('action') or 'clear') == 'clear':
                i += 1
                continue
            # One episode: consecutive non-clear frames, allowing a short
            # flicker back to clear without splitting the beat in two.
            j = i
            last = i
            while j < len(rows) and ts[j] - ts[last] < 1.0:
                if (rows[j].get('action') or 'clear') != 'clear':
                    last = j
                j += 1
            ep = rows[i:last + 1]
            # THE MOST SEVERE ACTION IN THE EPISODE, not the most common one.
            # A majority vote labelled an episode `clear`, because the flicker
            # tolerance above deliberately spans frames where the monitor had
            # already released and the beat is named for what it did at its
            # worst, not for what it spent most of its frames doing.
            act = min((r['action'] for r in ep if r.get('action')),
                      key=lambda a: SEVERITY.get(a, 99), default='clear')
            poly = next((r['polygon'] for r in ep
                         if r['polygon'] and r['action'] == act), '')
            # HOW LONG IT WAS ACTUALLY HELD, which decides whether the beat can
            # be seen at all. A stop that lasts one frame is a real monitor
            # event and a twitch on camera; captioning it as the field stopping
            # the vehicle would be true and unsupported by the picture.
            firing = [k for k in range(i, last + 1)
                      if (rows[k].get('action') or 'clear') != 'clear']
            held = ts[firing[-1]] - ts[firing[0]] if firing else 0.0
            # The window around it, in simulated seconds from this camera's
            # first frame, which is the unit --cut takes.
            ss = max(0.0, (ts[i] - t0) - lead)
            lo = next((k for k in range(len(ts)) if ts[k] - t0 >= ss), 0)
            hi = next((k for k in range(lo, len(ts))
                       if ts[k] - t0 >= ss + want), len(ts) - 1)
            win = rows[lo:hi + 1]
            wts = ts[lo:hi + 1]
            # The longest run inside the window during which the vehicle is
            # genuinely at rest. This is what the viewer can see; `held` is
            # only what the monitor said.
            halt, run_from, moving = 0.0, None, 0.0
            for k, (r, tt) in enumerate(zip(win, wts)):
                s = float(r['speed_mps'] or 9.0)
                if s < HALT_SPEED:
                    run_from = tt if run_from is None else run_from
                    halt = max(halt, tt - run_from)
                else:
                    run_from = None
                if s > MOVE_SPEED and k:
                    moving += tt - wts[k - 1]
            vis = [r for r in win if r['visible'] == '1' and r['px_across']]
            if (len(win) >= max(4, want * 3) and len(vis) >= 0.6 * len(win)
                    and halt >= HALT_MIN and moving >= MOVE_MIN):
                px = sum(float(r['px_across']) for r in vis) / len(vis)
                if min_px <= px <= max_px:
                    found.append((-SEVERITY.get(act, 99), halt, held,
                                  len(vis) / float(len(win)), name, ss, act,
                                  poly, px))
            i = j
    # Severest first, then longest held, then best framed. Ranking on framing
    # alone put a one frame stop above a stop that held the vehicle for a
    # second and a half, and only the second one can be seen.
    found.sort(reverse=True)
    print(f'\n{"cam":8s} {"ss":>7s} {"action":>9s} {"halt":>6s} {"held":>6s} '
          f'{"px":>6s} {"inframe":>8s}  polygon')
    for _, halt, held, frac, name, ss, act, poly, px in found[:18]:
        print(f'{name:8s} {ss:7.1f} {act:>9s} {halt:5.1f}s {held:5.1f}s '
              f'{px:6.0f} {frac * 100:7.0f}%  {poly}')
    if not found:
        print('no window where the monitor fired AND the vehicle came to '
              'rest in frame')
    return found


def avoids(outdir, want, min_px, max_px, near, keep_moving):
    """Windows where the vehicle passes CLOSE TO A PERSON without stopping.

    The protective stop beat was cut because it showed the system at its
    least impressive: a robot halted by somebody walking in front of it, which
    is correct behaviour and reads on screen as a robot that cannot cope. What
    the human aware layer actually does is keep going and leave room, and that
    is the harder thing to show because a vehicle driving past a person and a
    vehicle driving down an empty aisle look the same in a speed log.

    So the recorder logs the distance to the nearest person with every frame,
    and this asks for the case that makes the claim: a person within `near`
    metres at some point in the window, the vehicle in frame, and the vehicle
    NEVER dropping below `keep_moving` while they are near each other. A window
    containing a halt is rejected, because that is the other beat, the one that
    was cut.

    Ranked by how close the pass was: the closest genuine pass is the one worth
    showing, and the distance is printed so the caption can quote it.
    """
    found = []
    for name, rows in sorted(logs(outdir).items()):
        ts = [float(r['sim_t']) for r in rows]
        if len(ts) < 2 or ts[-1] - ts[0] < want:
            continue
        for start, stop in slices(ts, want, 0.5):
            win = rows[start:stop]
            if len(win) < max(4, want * 3):
                continue
            vis = [r for r in win if r['visible'] == '1' and r['px_across']]
            if len(vis) < 0.75 * len(win):
                continue
            ds = [float(r['person_m']) for r in win if r.get('person_m')]
            if len(ds) < 0.5 * len(win):
                continue
            closest = min(ds)
            if closest > near:
                continue
            # Never stops WHILE the person is close. A halt somewhere else in
            # the window is a different story and belongs to a different beat.
            stalled = any(float(r['speed_mps'] or 9.0) < keep_moving
                          for r in win
                          if r.get('person_m')
                          and float(r['person_m']) <= near)
            if stalled:
                continue
            px = sum(float(r['px_across']) for r in vis) / len(vis)
            sp = sum(float(r['speed_mps'] or 0) for r in win) / len(win)
            if not min_px <= px <= max_px:
                continue
            found.append((-closest, name, ts[start] - ts[0], px, sp, closest))
    found.sort(reverse=True)
    print(f'\n{"cam":8s} {"ss":>7s} {"closest":>8s} {"m/s":>6s} {"px":>6s}')
    for _, name, ss, px, sp, closest in found[:15]:
        print(f'{name:8s} {ss:7.1f} {closest:7.2f}m {sp:6.2f} {px:6.0f}')
    if not found:
        print('no window with a close pass that kept moving')
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--platform', default='mp400_class')
    ap.add_argument('--world', default='warehouse',
                    help='the gz world name, which also selects the shot list')
    ap.add_argument('--duration', type=float, default=180.0,
                    help='wall clock seconds to record')
    ap.add_argument('--rate', type=int, default=30, help='camera Hz, sim time')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--out', default=None)
    ap.add_argument('--shots', default=None,
                    help='comma separated subset of the shots for the world')
    ap.add_argument('--stills', action='store_true',
                    help='grab one frame per camera and stop. LOOK AT THESE '
                         'before committing to a long recording: it is the '
                         'contact sheet, and it is how a camera aimed at the '
                         'back of a shelf gets caught in 30 seconds instead '
                         'of after a run.')
    ap.add_argument('--cut', default=None,
                    help='conform clips onto a uniform timeline: '
                         '"aisle:12.5:8,pass:40:7" as camera:start_s:length_s. '
                         'Start is measured from the first frame of that '
                         'camera, which is what --report prints.')
    ap.add_argument('--avoids', action='store_true',
                    help='list windows where the vehicle passes close to a '
                         'person without stopping')
    ap.add_argument('--near', type=float, default=2.0,
                    help='--avoids: how close counts as a close pass, metres')
    ap.add_argument('--events', action='store_true',
                    help='list windows where the collision monitor fired and '
                         'the vehicle was in frame, for the safety beat')
    ap.add_argument('--lead', type=float, default=4.0,
                    help='--events: seconds of approach to include before the '
                         'monitor fires')
    ap.add_argument('--report', action='store_true',
                    help='rank shot candidates from an existing recording')
    ap.add_argument('--want', type=float, default=7.0,
                    help='--report: candidate window length in seconds')
    ap.add_argument('--min-px', type=float, default=90.0,
                    help='--report: reject windows where the vehicle is '
                         'smaller than this many pixels across')
    ap.add_argument('--max-px', type=float, default=700.0,
                    help='--report: reject windows where it is so close it '
                         'fills the frame')
    ap.add_argument('--min-speed', type=float, default=0.15,
                    help='--report: reject windows where it is barely moving')
    args = ap.parse_args()

    outdir = args.out or os.path.join(
        os.environ.get('SCRATCH', '/tmp'), 'film')
    os.makedirs(outdir, exist_ok=True)

    if args.cut:
        made = []
        for spec in args.cut.split(','):
            name, start, length = spec.split(':')
            dest = os.path.join(outdir, f'clip_{name}_{start}.mp4')
            if conform(outdir, name, float(start), float(length), args.rate,
                       dest):
                made.append(dest)
        print('\n'.join(made))
        return 0 if made else 1

    if args.avoids:
        return 0 if avoids(outdir, args.want, args.min_px, args.max_px,
                           args.near, 0.05) else 1

    if args.events:
        return 0 if events(outdir, args.want, args.min_px, args.max_px,
                           args.lead) else 1

    if args.report:
        return 0 if report(outdir, args.rate, args.want, args.min_px,
                           args.min_speed, args.max_px) else 1

    world = args.world.format(platform=args.platform)
    shots = SHOTS_BY_WORLD.get(world)
    if shots is None:
        shots = TEST_TRACK if world.startswith('test_track') else None
    if shots is None:
        print(f'no shot list for world {world}; known: '
              + ', '.join(sorted(SHOTS_BY_WORLD)) + ', test_track.*')
        return 2
    if args.shots:
        keep = set(args.shots.split(','))
        shots = [s for s in shots if s['name'] in keep]
        if not shots:
            print('no shot matched', args.shots)
            return 2

    print(f'world {world}, {len(shots)} camera(s) at {args.width}x'
          f'{args.height} {args.rate} Hz into {outdir}')
    live = spawn(shots, world, (args.width, args.height), args.rate, outdir)
    if not live:
        print('no camera spawned; is the simulator up?')
        return 1

    names = ' '.join(f'/film/{s["name"]}@sensor_msgs/msg/Image[gz.msgs.Image'
                     for s in live)
    print(f'bridging {len(live)} image topic(s)')
    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + names.split(),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(4.0)
        if args.stills:
            rc = stills(live, (args.width, args.height), outdir)
        else:
            rc = record(live, (args.width, args.height), args.rate,
                        args.duration, outdir)
    finally:
        bridge.send_signal(signal.SIGINT)
        try:
            bridge.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            bridge.kill()
        # Take the tripods back out. A camera left in the world would show up
        # in the next run's entity list and confuse anyone reading it.
        for s in live:
            drop(MODEL_PREFIX + s['name'], world)
    return rc


if __name__ == '__main__':
    sys.exit(main())
