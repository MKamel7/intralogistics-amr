# Intralogistics AMR

An autonomous mobile robot for indoor intralogistics, on ROS 2 Jazzy, Gazebo Harmonic and Nav2.
It moves load carriers between stations in a warehouse shared with people on foot, with a safety
layer that sits after the planner and can override it.

**One robot, not a fleet.** The repository was originally named for a multi-robot system and the
name outran the code; there is no traffic controller and no task allocation. A fleet layer is
listed under Roadmap and is claimed nowhere else.

**Status: one platform validated end to end, with 62 recorded findings.** This README documents
what exists and what is measured, not what is planned. Every figure below is traceable to an entry
in `docs/validation.md`; anything not built is under Roadmap and is claimed nowhere else.

### Measured, on the generated track, MP-400 class

| | result | where |
|---|---|---|
| transport cycles | **12 of 12** across five runs | V-44 |
| cycle time | 223 s [175 to 272], sd 24, n=12 | V-44 |
| distance per cycle | 78.1 m [65.4 to 82.6] | V-44 |
| contacts the vehicle drove into | **0** in 248 000 samples across two arms | V-51 |
| deepest a person reached inside the footprint | **-0.100 m**, against -0.466 m before V-39 was closed | V-49 |
| cost of that safety | 103 protective stops per cycle, **7 % of cycle time**, against 38 and 2 % | V-49 |
| localisation error | p50 0.027 m driving, 0.055 m parked | V-37 |
| odometry against ground truth | ratio 1.025 | V-33 |
| people tracking | precision 0.615, recall 0.988, 0 ID switches | V-36 |
| sensor to command latency | p50 84 ms, **p95 124 ms**, n=397, against a 0.10 s estimate | V-56 |
| the latency tail | **retracted**: it was a probe pairing artifact, not the stack | V-56 |
| protective field coverage outside the sensor's blind zone | **55.1 mm** against the 50 mm needed | V-49 |
| braking distance, laden and unladen | **9 mm** median both, 85 to 97 mm worst, n=859 | V-60 |
| deceleration in a protective stop | **3.5 to 4.1 m/s2**, against the 2.4 the fields assume | V-60 |
| an unsecured 100 kg load, over a duty cycle | **0.0 mm** of slide, **3.8 deg** of rotation, none lost | V-61 |
| why it stays on | the stop peaks at **14.9 m/s2** but only for a **4 ms** step, and slip goes as time squared | V-61 |
| parked accuracy at a station | median **117 mm**, worst 212 mm against a 200 mm tolerance | V-62 |

The latency row was wrong for most of this project's life and the correction is worth reading.
It said p95 796 ms and called the spec estimate refuted, on the strength of 43 samples pooled from
five runs of 17, 6, 7, 7 and 6, containing an artifact: the probe armed a sample on a protective
stop even when the vehicle was already stationary, so nothing closed it until an unrelated later
stop and the interval spanned both. Guarded, over 397 samples, the p95 is 124 ms and the maximum is
144 ms against a previous 980. See V-56.

`control_latency: 0.10 # NOT YET MEASURED` is therefore **not refuted**. At the commissioned
0.75 m/s the gap between the estimate and the measured p95 is 18 mm, not the 522 mm previously
claimed. Whether to write the measured figure into the spec is a decision, not a correction, and
it is recorded as one.

### Planner comparison

Nine runs, three per planner, same track and scenario (V-47):

| planner | cycles | cycle time (s) | distance (m) |
|---|---|---|---|
| **SmacPlanner2D** | **9 of 9** | 192.8 [123 to 231] | 70.2 [50.3 to 78.6] |
| NavFn | 8 of 9 | 219.4 [86 to 289] | 70.8 [38.5 to 80.9] |
| ThetaStar | 6 of 9 | 209.5 [142 to 236] | 75.0 [62.0 to 81.3] |

Every range overlaps, so no claim is made that any planner produces shorter or faster paths. What
separates them is what they refuse: ThetaStar rejected a start pose in an inflated cell 98 times,
where SmacPlanner2D tolerates it and re-plans. This vehicle demonstrably comes to rest in inscribed
cells, so the planner is kept **on tolerance, not on path quality**. Hybrid-A* is excluded because
it plans under a turning radius a differential drive does not have, and a table where the project's
own planner beats a strawman is worth less than no table.

## Why this exists

Most ROS 2 navigation projects stop at "the robot reaches the goal". The things that decide whether
an AMR is usable in a real plant are the ones that get skipped: whether it localises without
cheating, whether it sees a person in time to stop, whether anything is actually carried, and
whether any of it is measured against a source rather than asserted.

The claim this repository is built around:

> Every physical constant is traceable to a source, the protective envelope is generated from the
> vehicle specification rather than tuned, and the build fails when either stops being true.

That is enforced, not aspirational. `test_platform_spec.py` fails the build when a value loses its
provenance, and it has caught a real fault: a scanner mounting position quoted to the millimetre
from an operating manual that described a different sensor than the one fitted. See V-23.

## Current state

### Done

**Gazebo Harmonic warehouse world.** Imported from the AWS RoboMaker small warehouse assets by
`src/amr_sim/tools/import_aws_warehouse.py`, which rewrites Gazebo Classic mesh URIs, renames the
models to prevent shadowing, and repairs two inertia tensors that violate the triangle inequality
and that Harmonic correctly rejects. The import is a script, so it is repeatable.

**Two worlds, and they do different jobs.** The AWS import above is a *found* building: nobody
sized it for this vehicle, and measured at robot height its corridors have a median width of 1.34 m
and a 25th percentile of 0.64 m, narrower than the robot. That is the honest robustness case, and
it is also an uncontrolled variable: when a cycle fails there, you cannot separate bad navigation
from a 0.64 m aisle.

So there is a second world, generated by `src/amr_sim/tools/generate_test_track.py`, whose aisle
widths **are the corridor figures the platform datasheet publishes**, read from `validation_targets`
so the track cannot drift from the claims it tests:

The track was originally built to the datasheet corridor figures, and measuring the vehicle in it
**refuted them**: the MiR250 is 2.1 mm too wide for its own published 1.000 m dynamic corridor and
52 mm too wide for its 0.950 m corner, so it trapped itself and killed a run (V-26, V-27). The
widths are now derived from what the vehicle physically needs, and the published claims are printed
alongside as claims:

| zone | width | origin |
|---|---|---|
| aisle 1 | 2.305 m | derived: turning width + 0.70 m to pass a person + 0.30 m |
| aisle 2 | 2.005 m | derived: turning width + 0.70 m to pass a person |
| pinch | 2.105 m | derived: turning width + 0.70 m + 0.10 m |
| crossing | 2.205 m | derived: a junction, + 0.20 m |
| doorway | 2.005 m | derived: straight through |
| open bay | 2.300 m | measured, AWS p75 (V-22) |

Turning width is twice the widest all-round protective field the vehicle can select, so an aisle is
wide enough for it to turn round in rather than merely to fit. The 0.70 m is a pedestrian plus
clearance, so a person standing in a corridor is a detour rather than the end of the run. The
building's depth is derived from the zones it must hold, because a shell sized independently of its
contents leaves a remainder, and a remainder is either a corridor or a trap.

Run it with `tools/run_stack.sh --test-track`.

The open bay is wide enough for the vehicle to route around a person standing in it and the scored
aisle is not, so the same pedestrian behaviour produces a re-route in one place and a correct wait
in the other. Both are asserted by tests, so neither can be lost by adjusting a width.

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

**Walking pedestrians.** Three walkers pace their lanes back and forth and one worker stands still,
all on the floor. Getting there was mostly not a physics problem: sixteen ground-truth publishers and
eleven driver nodes had accumulated from repeated launches and were issuing conflicting commands to
the same pedestrians, which made every measurement contradict the last. V-15 in
[docs/validation.md](docs/validation.md).

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

**Physical load transfer.** A cycle loads and unloads as a mission state with a dwell, not as a
lift or a fork engaging a pallet. Nothing is carried in the physics.

**Precision docking.** The vehicle parks by navigation goal, not by aligning to a marker. The
localisation figures above are what a docking claim would have to be built on, and they are not
sufficient for one on their own.

**The fleet layer, deliberately.** There is no dispatcher, no lane reservation and no task
allocation. The VDA 5050 interface is the *vehicle* half, which is what an integrator connects to,
and it is tested end to end against a broker. A dispatcher with one robot behind it would be a
claim without a measurement.

**A human-aware costmap layer.** People are detected, tracked and scored, and the planner routes
around them as ordinary obstacles. It does not yet pay a cost for passing close to a person, which
is what the social metrics in V-43 exist to make measurable.

**The MiR250 as a running vehicle.** Its specification and generated configuration are kept and the
tests run over both platforms, which is what caught V-33. It is not validated on the track and no
cycle count is claimed for it.

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
src/amr_description   platform specs, xacro description, generated controllers
    amr_perception    scan merge, leg and height detection, tracking (C++)
    amr_navigation    Nav2 configuration generator, survey runner, keepout masks
    amr_safety        protective field generator, collision monitor configuration
    amr_sim           world generator, pedestrian driver, ground truth oracle
    amr_mission       transport task, station definitions
    amr_bringup       launch
    amr_evaluation    scoring tools and the experiment runner
    amr_vda5050       VDA 5050 vehicle interface over MQTT
tools/                the instruments: seven probes, the stack runner, teardown
docs/validation.md    the laboratory notebook, 46 numbered findings
docs/findings.md      four of them worth reading first, the short version
docs/adr              architecture decision records
docs/architecture     arc42 architecture documentation
docs/datasheets       archived source documents for every physical constant
requirements/         requirements with IDs, traced to tests
Dockerfile            builds from a clean base and runs the suite
```

**The instruments in `tools/` are half the project.** Each one exists because a measurement was
wrong in a way that looked ordinary:

| tool | what it measures | why it exists |
|---|---|---|
| `measure_slip.py` | wheel odometry against ground truth | found a 33 % scale error, V-33 |
| `measure_localisation.py` | believed pose against true pose | the vehicle parked over the paint, V-34 |
| `measure_contacts.py` | how close the vehicle came to people | a pedestrian walked through the robot |
| `measure_social.py` | proxemic zones, time to collision | V-43 |
| `measure_path_efficiency.py` | planner overhead vs controller overhead | they imply opposite fixes, V-38 |
| `measure_control_latency.py` | sensor to command | p95 124 ms over 397 samples, V-56 |
| `experiment.py` | N runs, reported as a distribution | one run cost a retraction |

## Try it

```
./demo.sh
```

Builds if needed, brings the stack up, runs two transport cycles on the MiR250
with the cameras off, and prints the result table the transport task produces.
About four minutes on a laptop. Nothing needs clicking.

`tools/run_stack.sh --help` is the real instrument behind it: platform
selection, the two worlds, survey and mission tasks, and the preflight gate that
refuses to measure an unhealthy stack.

## Build

```bash
git clone https://github.com/MKamel7/intralogistics-amr.git
cd intralogistics-amr
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

The whole suite, the way the build runs it:

```bash
colcon test && colcon test-result
```

`colcon test` reports more than `pytest src` because it also runs each
package's lint tests. What it must never report is FEWER pytest cases, and for
most of this project's life it did: 289 against 337, because eleven test files
were never registered with `ament_add_pytest_test` and one package was the
wrong build type. All eleven were passing, which is why it went unnoticed for
the whole project. `test_registration.py` now walks the source tree and fails
if a test file exists that the build does not run. See V-50.

## How this is built

Decisions that were expensive to make are recorded as [ADRs](docs/adr/). Ten exist so far, one of them still Proposed. Each
states the context and the measurement that forced it.

| # | Decision |
|---|---|
| [0001](docs/adr/0001-ros2-jazzy-and-gazebo-harmonic.md) | ROS 2 Jazzy and Gazebo Harmonic as the target |
| [0002](docs/adr/0002-mir250-class-reference-platform.md) | A MiR250-class reference platform, chosen on provenance |
| [0003](docs/adr/0003-three-tier-simulation.md) | Three simulation tiers, so the CPU budget is explicit |
| [0004](docs/adr/0004-cpp-and-python-split.md) | C++ for hot paths and plugins, Python for tooling |
| [0005](docs/adr/0005-scanner-for-safety-cameras-for-navigation.md) | The scanner is the safety sensor, the cameras are navigation sensors |
| [0006](docs/adr/0006-ground-truth-is-an-oracle-not-an-input.md) | Ground truth is an evaluation oracle and never an input |
| [0007](docs/adr/0007-keepout-zones-are-commissioning-data.md) | Keepout zones are commissioning data, authored from the site layout |
| [0008](docs/adr/0008-margin-belongs-to-the-mission-layer.md) | Navigation margin belongs to the mission layer, not the planner |
| [0009](docs/adr/0009-the-monitor-limits-speed-and-covers-every-velocity.md) | The monitor caps speed, and its bands must cover every velocity |
| [0010](docs/adr/0010-localisation-mode-for-mission-runs.md) | **Proposed.** Bound SLAM, or localise on a saved map, for mission runs |

Physical constants are never hardcoded. They live in a platform spec, carry a recorded source, and
a test fails the build if that decays.

Results are measured, not asserted. Any performance number in this README names the machine it was
measured on and the conditions it was measured under.

Published figures from the reference platform are validated **against**, never tuned to. Where the
model and the reference disagree, [docs/validation.md](docs/validation.md) says so and says which
way the error points.

## Roadmap

What is left, in the order it would be worth doing:

1. **Decide whether to write the measured latency into the spec.** `control_latency: 0.10` is
   marked NOT YET MEASURED and is now measured: p95 124 ms over 397 samples, sd 24 ms. The estimate
   is short by 24 ms, which is 18 mm of travel at 0.75 m/s. That is a decision about a safety
   constant rather than a correction, so nothing has written it, and whoever does should run a cycle
   afterwards: V-42 and V-45 are what enlarging a protective field costs and neither was predicted
   from arithmetic.
2. **Resolve whether the proxemic layer helps.** It is built, tested and shipped DISABLED, because
   four runs could not show an effect: the within-arm spread of the metric is 224 mm against an
   effect of a few tens of millimetres, and the best social score of the four belongs to the run
   where the vehicle never moved. Settling it needs a metric normalised by exposure and three runs
   per configuration, as V-47 did for the planners. See V-59.
3. **Precision docking**, once localisation is good enough to support the claim.
4. **Precision docking, and it is NOT reachable from the map frame.** Measured: the vehicle parks
   a median 117 mm from the station, worst 212 mm against a 200 mm goal tolerance, because a
   tolerance governs where the vehicle BELIEVES it is and the localisation error adds on top.
   Docking needs about 10 mm; the localisation floor is 55 mm, so a perfect controller would still
   be five times too coarse. It needs a dock the vehicle can SEE, which makes the error a sensor
   error rather than a localisation one. See V-62.
5. **Tighten `xy_goal_tolerance` toward the localisation floor**, which should roughly halve the
   parked error for the cost of one number, and must be measured rather than assumed: a tolerance
   below what the controller can achieve buys goal-reached timeouts instead of accuracy.
6. **Explain the heading asymmetry at `goods_in`**, 26.9 degrees worst against 2.9 at `dispatch`,
   with all three samples negative and growing across the run. A pattern, not noise, and unexplained.

Not on this list: a fleet layer. The project runs one vehicle and says so.

Also not on this list: closing V-39 on the MiR250. It stays deliberately open there, recorded in
`UNSHAPED_SELF_FILTER` in two test modules with the reasoning attached. That platform is kept as a
second specification because generating two vehicles from one set of tools is what caught V-33, and
it is not a runtime target, so measuring its self filter would be work in service of nothing.

## Predecessor

This project began as a rebuild of a university group project
([`warehouse-fleet`](https://github.com/MKamel7/warehouse-fleet), SoSe 2026), which is preserved as
submitted. Almost nothing carries over: different simulator, different ROS distribution, different
robot, different architecture. This repository is solo work from its first commit.

## Licence

Apache-2.0. The imported warehouse scenery is derived from the AWS RoboMaker small warehouse world,
copyright Amazon.com, Inc., licensed MIT-0; see `src/amr_sim/models/README.md`.
