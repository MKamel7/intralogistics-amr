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
simulation tiers below are real rather than aspirational. 23 tests cover it.

**Measured simulation cost**, by `src/amr_evaluation/tools/benchmark_sim_cost.py` on an i5-1235U,
with the real-time throttle disabled and a subscriber attached to every sensor topic:

| configuration | us/step | marginal RTF | verdict |
|---|---|---|---|
| world only, no robot | 52.8 | 75.7x | scenery is nearly free |
| 1 robot, no sensors | 302.6 | 13.2x | |
| 1 robot, 2 safety scanners | 668.5 | 6.0x | |
| 1 robot, scanners + 2 RGB-D | 1977.6 | 2.0x | **perception tier** |
| 3 robots, scanners only | 2016.4 | 2.0x | **fleet tier** |
| 5 robots, scanners only | 3509.5 | 1.1x | fleet tier ceiling |
| 3 robots, scanners + RGB-D | 6330.7 | 0.6x | **below real time** |

Fixed startup is about 2.2 s per run regardless of configuration.

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

**Platform specification with an enforced provenance gate.** Every physical constant of the robot
lives in `src/amr_description/config/platforms/mir250_class.yaml`, tagged `datasheet`, `derived`,
`estimated` or `tuned`, with its source. Currently 20 datasheet values, 4 derived, 10 estimated.
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

## Roadmap

In order, each independently demonstrable:

1. Battery and state-of-charge model, RViz configuration
2. Perception: scan conditioning, human detection, tracking, prediction
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
