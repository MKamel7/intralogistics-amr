# Intralogistics AMR Fleet

A multi-robot autonomous mobile robot fleet for indoor intralogistics, on ROS 2 Jazzy, Gazebo
Harmonic and Nav2. Robots move VDA KLT load carriers between conveyor stations in a warehouse
shared with people on foot, under a central traffic controller.

**Status: early. Phase 0 and part of Phase 1 are done.** This README documents what exists and what
is measured, not what is planned. Anything not yet built is listed under Roadmap and is not claimed
anywhere else.

## Why this exists

Most student ROS 2 fleet projects stop at "several robots navigate to goals". The things that
decide whether an AMR is usable in a real plant are the ones that get skipped: whether it localises
without cheating, whether it sees a person in time to stop, whether the fleet deadlocks in a
two-way aisle, whether anything is actually carried, and whether any of it is measured. This project
is aimed squarely at those.

## Current state

### Done

**Gazebo Harmonic warehouse world.** Imported from the AWS RoboMaker small warehouse assets by
`src/amr_sim/tools/import_aws_warehouse.py`, which rewrites Gazebo Classic mesh URIs, renames the
models to prevent shadowing, and repairs two inertia tensors that violate the triangle inequality
and that Harmonic correctly rejects. The import is a script, so it is repeatable.

**Robot description.** A MiR250-class AMR generated entirely from the platform spec: two drive
wheels on `ros2_control`, four real two-degree-of-freedom casters, two 275 degree safety scanners at
diagonally opposite corners, two RGB-D cameras and an IMU. Sensor sets are switchable so the
simulation tiers below are real rather than aspirational. 26 tests cover it.

**Measured simulation cost**, by `src/amr_evaluation/tools/benchmark_sim_cost.py` on an i5-1235U,
with the real-time throttle disabled and a subscriber attached to every sensor topic:

| configuration | us/step | marginal RTF | verdict |
|---|---|---|---|
| world only, no robot | 57 | ~70x | scenery is nearly free |
| 1 robot, no sensors | 290 | ~14x | |
| 1 robot, 2 safety scanners | 643 | 6.2x | |
| 1 robot, scanners + 2 RGB-D | 1977 | 2.0x | **perception tier** |
| 3 robots, scanners only | 2001 | 2.0x | **fleet tier** |
| 5 robots, scanners only | 3492 | 1.1x | fleet tier ceiling |
| 3 robots, scanners + RGB-D | 6468 | 0.6x | **below real time** |

Fixed startup is about 2.2 s per run regardless of configuration. The cheap configurations vary by
roughly 8 percent between runs, because their per-step cost is a small difference between two larger
wall times, so the leading figures there are quoted to two significant digits rather than implying
precision the method does not have. The expensive rows, which are the ones the tier split turns on,
repeat to within a few percent.

That table is the evidence for the three-tier design in
[ADR 0003](docs/adr/0003-three-tier-simulation.md): a fleet tier of 3 to 5 robots with scanners
only, and a perception tier of one fully equipped robot. Three robots carrying cameras runs below
real time, which is the configuration the tiering exists to avoid.

**Bringup that drives.** `ros2 launch amr_bringup robot.launch.py` puts the robot in the warehouse
with `ros2_control`, the bridge and both controllers active. Controller spawning is chained to the
spawn process exiting rather than to a timer, because `gz_ros2_control` only starts its
controller_manager once the model is in the world.

Verified against the platform sheet by `src/amr_evaluation/tools/drive_check.py`: 0.5 m/s command
gives 2.59 m in 6.0 s (the deficit against 3.0 m is exactly the acceleration ramp), a 0.6 rad/s spot
turn reaches 0.6 rad/s, and a full-speed ramp reaches 2.0 m/s in 6.55 m at an effective 0.30 m/s2.

That last figure exposed an inconsistency in the reference itself: the sheet publishes both a
0.3 m/s2 acceleration limit and a 9.5 m distance to reach 2.0 m/s, and under constant acceleration
those imply 6.67 m and 0.21 m/s2 respectively. They cannot both hold. The model follows the
acceleration figure and therefore accelerates more aggressively than the real machine, which makes
any future throughput number optimistic. Recorded in [docs/validation.md](docs/validation.md)
rather than tuned away.

**The vehicle occludes its own scanners**, which took a fix. The scanners were originally mounted
inside a single solid chassis box, and because the renderer culls backfaces the vehicle was
invisible to its own lidar: rays aimed into the robot returned a median of 9.34 m, seeing through
the chassis to the far wall, while the scans otherwise looked entirely healthy. The chassis is now
two overlapping boxes leaving open corner recesses for the sensors, inside the published envelope,
and the same arc now returns 0.06 m. See V-05 in [docs/validation.md](docs/validation.md).

**Merged 360 degree scan** (`amr_perception`, C++). The two 275 degree scanners are merged through
TF into a single 2118-bin scan in `base_link` at the sensors' own 0.17 degree resolution. Each scan
is transformed using the transform that held at its own timestamp, routed through the odometry
frame, because the scanners are not phase locked and up to 35 ms of vehicle motion at 2 m/s is 70 mm,
larger than the 20 mm object the scanner is specified to detect. Bins take the nearest return, never
an average. Self-returns are removed by a footprint filter, which is not hypothetical: the corner
scanners see their own chassis at about 0.06 m.

Coverage is measured, not assumed: largest contiguous gap **4.25 degrees**, improved from 11.56
degrees by moving the optics flush to the envelope corner. That is short of the sheet's "360 degree"
claim and V-06 in [docs/validation.md](docs/validation.md) says so, with the reason and the
consequence for the later protective-field work.

**Geometric people detection** (`amr_perception`, C++). Clusters the merged scan with an adaptive
break threshold, keeps clusters that are leg sized and round rather than flat, and pairs them at a
plausible stance width. 16 unit tests on synthesised scans with exact known truth.

Scored against a scenario whose pedestrian positions are known: **recall 0.875, localisation p50
5.4 cm, precision 0.168**. The precision figure is real and is not tuned away. A rack upright is a
round leg-sized cylinder, so on a single 150 mm plane it is indistinguishable from a calf; 56 percent
of the false positives come from nine fixed positions that are present in almost every frame. Height
(from RGB-D) and motion (from tracking) are what separate the two, and both are next. V-07 in
[docs/validation.md](docs/validation.md) has the analysis.

**Pedestrian tracking** (`amr_perception`, C++). Constant-velocity Kalman filter per track, global
nearest neighbour association with a Mahalanobis gate so a track that has been coasting through an
occlusion is allowed to search wider, M-of-N confirmation, and coasting through about a second of
full occlusion. 14 unit tests including two targets crossing without exchanging identities.

Measured against the `walking_people` scenario: classifying tracks by velocity lifts precision from
0.071 to **0.312** and cuts ID switches from 3 to 1, while halving recall from 0.575 to 0.242. The
recall cost is real and expected: the scenario contains a worker standing still, and a stationary
person has no more velocity than a rack upright. V-08 in
[docs/validation.md](docs/validation.md) has the numbers and the measurement bug that nearly
disguised itself as one.

**Depth-based people detection** (`amr_perception`, C++). Unprojects the depth image, removes floor
and ceiling, clusters into vertical columns and classifies by height profile: a person is narrow at
the calves, wider at the torso and stops around 1.75 m, a rack upright is uniform and keeps going.
12 unit tests on depth images synthesised by ray-marching known solids, so every case has exact
truth.

Building it exposed a real constraint. The camera's 58 degree vertical field of view at 0.27 m sees
no higher than 1.66 m at 2.5 m range, so **closer than about 2.7 m every tall object is truncated to
the same apparent height** and the "tops out at person height" test is meaningless. Clusters now
carry a truncation flag and the detector declines to conclude anything from a height it never
observed. The width profile is what actually carries the discrimination. V-10 in
[docs/validation.md](docs/validation.md).

**Near-field diagnosis, and where safety actually comes from.** Detection at 1.28 m was 55 percent.
The cause recorded for it turned out to be wrong: the legs do not merge, the returns are *fragmented*
by the re-binning in the merge, into runs of one to four points separated by two and three bin holes.
Bridging those holes, only where the points either side agree in space, lifted precision to 0.218,
recall to 0.900 and the near-field rate to 60 percent.

That was treating a symptom. The representation was the cause: a LaserScan is a lossy container for
two sensors at different origins. The merger now also publishes the returns un-binned as a point
cloud and the detector clusters those spatially, which took **recall to 1.000** and the near-field
rate to **100 percent** with localisation p50 of 4.5 cm. The binned scan stays for the costmap and
the collision monitor, where holes are harmless.

Precision is still around 0.18 and that is not this fix's job: it is structural, a rack upright
really is a leg-shaped object. The important measurement is what that does and does not mean: on the
same 40 frames, **returns from that pedestrian are present in 100 percent of them**. What fails is
naming them, not seeing them. So the protective stop is built on the merged scan directly and never
consumes the classifier, which is both how ISO 3691-4 protective fields work and the only
architecture that does not make safety depend on a component measured at 0.218 precision. V-11 in
[docs/validation.md](docs/validation.md).

**Protective and warning fields** (`amr_safety`). Speed-switched fields on `nav2_collision_monitor`,
sitting between the velocity command and the wheels. The geometry is not chosen: it is generated
from a stopping-distance calculation over the platform spec, rounded outward only, and 11 tests
assert each field covers its own stopping distance, that the bands leave no uncovered speed, and
that the observation source is the merged scan rather than any classifier output.

Measured with an obstacle 0.778 m ahead: at a 0.25 m/s command the active field reaches 0.561 m, the
obstacle is outside it and the vehicle moves at 0.081 m/s under warning-field slowdown; at 0.80 m/s
the field reaches 0.854 m, the obstacle is inside and the vehicle **stops at 0.000 m/s**. Same
obstacle, different speed, different outcome, which is the whole point of speed-dependent fields.

The reasoning, the two estimated inputs it rests on, and what it does not prove are in
[docs/safety_concept.md](docs/safety_concept.md).

**Mapping.** `slam_toolbox` on the merged 360 degree scan rather than either raw scanner, since a
single 275 degree corner-mounted sensor sees less than half the surroundings and loop closure on a
partial view is much weaker. Produces a 5 cm occupancy grid and `map -> odom`. Three separate silent
faults had to be fixed to get there, none of which logged an error; V-14 in
[docs/validation.md](docs/validation.md) has them.

**Battery and state of charge.** A power model fitted to the three runtimes the platform sheet
publishes, exposed as `sensor_msgs/BatteryState`. Deliberately labelled as calibration rather than
validation in [docs/validation.md](docs/validation.md): three published figures and three model
terms means reproducing them is arithmetic, not evidence. The sheet's undefined duty cycle for
"active operation" is made an explicit reference-speed parameter, and a test asserts the published
runtimes survive any choice of it.

**Platform specification with an enforced provenance gate.** Every physical constant of the robot
lives in `src/amr_description/config/platforms/mir250_class.yaml`, tagged `datasheet`, `derived`,
`estimated` or `tuned`, with its source. Currently 54 constants: 29 from a data sheet, 5 derived from data sheet values, 20 estimated with the reasoning recorded.
`test_platform_spec.py` fails the build if a value has no source, a source has no value, a
non-datasheet value gives no reasoning, or the geometry contradicts itself.

The spec also holds `validation_targets`: published figures the project measures itself **against**
rather than tunes to. The 1000 mm dynamic-footprint corridor, the 950 mm corridor for a 90 degree
turn, the 20 mm object detectable at 1000 mm, 9.5 m to reach full speed, and the 13 h and 17.4 h
runtimes.

### Not done yet

Everything else: bringup and control, perception, mapping, navigation, the load transfer, the
people, the safety layer and the fleet interface. See Roadmap.

One open item is tracked in the code rather than hidden: the platform sheet gives a 114 degree
camera field of view under a heading covering two cameras, without saying whether the figure is per
camera or combined. The model currently uses the optimistic reading, and no detection or coverage
number may be published from it until the Intel data sheet is archived and the figure resolved.

## The robot

A **MiR250-class** AMR: a 250 kg payload differential-drive platform with two drive wheels, four
casters, two safety laser scanners at diagonally opposite corners and two 3D cameras.

It is a class of machine derived from a published specification, with its own livery. It is not a
model of any vendor's product, carries no vendor branding, and is not presented as equivalent to
one. The reasoning behind choosing this reference over a specific target employer's platform is in
[ADR 0002](docs/adr/0002-mir250-class-reference-platform.md); it came down to provenance, since the
MiR specification publishes around 40 usable constants against about 8 for the alternative.

Data sheets are archived in [`docs/datasheets/`](docs/datasheets/) with text extractions beside
them, so every constant cites a line rather than a memory.

## Layout

```
src/amr_description   platform specs, xacro description, meshes
    amr_msgs          typed interfaces
    amr_perception    detection, tracking, prediction (C++)
    amr_navigation    costmap layers, behaviour tree nodes, Nav2 params
    amr_safety        safety supervisor, protective field configuration
    amr_fleet         task allocation, traffic control, VDA 5050 client
    amr_sim           world, scenery models, importer, scenarios
    amr_bringup       launch
    amr_evaluation    KPI harness and analysis
docs/adr              architecture decision records
docs/architecture     arc42 architecture documentation
docs/datasheets       archived source documents for every physical constant
requirements/         requirements with IDs, traced to tests
```

Packages appear as they are built. An empty directory here means the package is planned, not
missing.

## Build

```bash
git clone https://github.com/MKamel7/intralogistics-amr-fleet.git
cd intralogistics-amr-fleet
colcon build --symlink-install
source install/setup.bash
```

Run the world on its own:

```bash
gz sim -r src/amr_sim/worlds/warehouse.sdf
```

Tests, no ROS needed:

```bash
python3 -m pytest src/amr_description/test -q
```

## How this is built

Decisions that were expensive to make are recorded as [ADRs](docs/adr/). Four exist so far: the
Jazzy and Harmonic target, the platform choice, the three-tier simulation, and the C++ and Python
split. Each states the context and the measurement that forced it.

Physical constants are never hardcoded. They live in a platform spec, carry a recorded source, and
a test fails the build if that decays.

Results are measured, not asserted. Any performance number in this README names the machine it was
measured on and the conditions it was measured under.

Published figures from the reference platform are validated **against**, never tuned to. Where the
model and the reference disagree, [docs/validation.md](docs/validation.md) says so and says which
way the error points.

## Known not working

Two things are configured but do not work, recorded here rather than left to be discovered:

- **Walking pedestrians.** Static ones are correct and are what the demo uses. Walking ones cover
  only part of their lane or do not move; five approaches are documented in OPEN-1 of
  [docs/validation.md](docs/validation.md), along with the fix that should be done instead.

## Roadmap

In order, each independently demonstrable:

1. Perception: human detection, tracking, prediction (scan conditioning and merge done)
3. Mapping and localisation, including the aisle degeneracy benchmark
4. KLT load transfer and precision docking
5. Navigation: footprint-aware planning, MPPI, human-aware costs
6. People in the world, seeded scenarios
7. Safety layer: protective and warning fields, ISO 3691-4 state machine
8. Fleet layer: VDA 5050 interface, lane reservation, task allocation
9. Evaluation: KPI harness, before and after comparisons

## Predecessor

This project began as a rebuild of a university group project
([`warehouse-fleet`](https://github.com/MKamel7/warehouse-fleet), SoSe 2026), which is preserved as
submitted. Almost nothing carries over: different simulator, different ROS distribution, different
robot, different architecture. This repository is solo work from its first commit.

## Licence

Apache-2.0. The imported warehouse scenery is derived from the AWS RoboMaker small warehouse world,
copyright Amazon.com, Inc., licensed MIT-0; see `src/amr_sim/models/README.md`.
