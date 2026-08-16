# Handover

## Read this first

**471 tests pass under `colcon test` with 5 documented skips, ruff is clean,
`gz sdf` validates both worlds, and the container builds from a clean base and
runs the same suite to the same result.** 51 numbered findings in
`docs/validation.md`.

That test figure was 289 two days ago and the suite has not grown by 182
tests. Eleven files across three packages were never registered with the
build, and one package was the wrong build type and reported FAILED on every
build while its tests passed under a direct run. See V-50.

The project is **one vehicle**, the MP-400 class, and it says so everywhere.
The repository was renamed from `intralogistics-amr-fleet` to
`intralogistics-amr` because the old name claimed a fleet that does not exist
and is not coming.

### What is measured

| | result | where |
|---|---|---|
| transport cycles | 12 of 12 across five runs | V-44 |
| contacts the vehicle drove into | 0 in 248 000 samples across two arms | V-51 |
| deepest a person reached inside the footprint | -0.100 m, against -0.466 m before V-39 | V-49 |
| cost of that safety | 103 protective stops per cycle, 7 % of cycle time, against 38 and 2 % | V-49 |
| localisation | p50 0.027 m driving, 0.055 m parked | V-37 |
| odometry against truth | ratio 1.025 | V-33 |
| people tracking | precision 0.615, recall 0.988, 0 ID switches | V-36 |
| sensor to command latency | p50 68 ms, p95 796 ms against a 0.10 s estimate | V-44 |
| planner choice | SmacPlanner2D 9 of 9, NavFn 8 of 9, ThetaStar 6 of 9 | V-47 |

### The three things most worth knowing

**1. The latency estimate is refuted and deliberately not fixed.** Every
protective field is sized through ISO 13855 by `control_latency: 0.10`, marked
NOT YET MEASURED. Measured, it is p50 68 ms and p95 796 ms, which at the
commissioned 0.75 m/s is 522 mm of travel the fields do not carry. The p95 is
NOT written into the spec, because a p50 of 68 against a p99 of 1260 is a
control path that is occasionally starved rather than one that is slow, and
sizing a field on that tail would add roughly 0.6 m to it. V-42 and V-45 are
what happens when a field is enlarged without measuring the cost.

**2. V-39 is closed, and two attempts to close it first made things worse.**
The scan merger's self filter used to blank a region larger than the vehicle,
which left the forward protective fields 5.1 mm of lateral coverage.
Enlarging the fields trapped the MP-400 against a rack (V-42) and dropped the
MiR250 from 3 of 3 to 2 of 9 (V-45); both were reverted. Measuring the filter
margin instead got it from 5.1 mm to 33.1 mm and took contacts from six to
zero (V-46). Shaping the filter
to the vehicle rather than to its bounding box closed it at 55.1 mm, with no
field resized (V-49). It had been called a hardware limit for three findings,
because every attempt asked how LARGE the margin should be and none asked what
SHAPE the filter was. Measured against a control arm on the same protocol,
the deepest a person reached inside the footprint went from -0.466 m to
-0.100 m, at a cost of about five percent of cycle time.

**3. The vehicle has never driven into anybody, and that claim is weaker than
it sounds.** The pedestrians carry no collision geometry, deliberately, so a
person cannot be struck in this simulation. Contact is measured geometrically
by `tools/measure_contacts.py` against the ground truth oracle, and zero is
evidence the stack kept clear rather than evidence that anything would have
stopped it.

The wording matters and used to be wrong. The probe called any contact above
0.02 m/s one the vehicle drove into, which fired on a vehicle creeping at 0.03
while a person walked into it at 0.91. Both velocities are now projected onto
the line between the two bodies, so the closing rate splits into a share each.
Every contact across both V-49 arms, four in 248 000 samples, was a person
walking into a stationary vehicle. See V-51.

### The MiR250

Its spec and generated configuration are kept, and the tests run over both
platforms, which is what caught V-33. **It is not a validated runtime target
and no cycle count is claimed for it.** Its last measured state on the track is
0 of 3, with the planner refusing to plan from an inscribed start caused by two
cells baked into the SLAM map 0.14 m from the vehicle centre, inside its own
footprint, with the nearest live scan return 1.182 m away and localisation at
0.032 m. That evidence is recorded so it survives the decision to stop chasing
it.

### The four faults that were all the same shape

Worth naming together, because the next one will look like these:

* `config/controllers.yaml`, one file of wheel geometry for every platform.
* `scenarios/track_people.yaml`, one pedestrian scenario written by whichever
  platform was generated last, while every coordinate in it derives from that
  platform's aisle positions. People moved by up to 0.50 m between them.
* the pedestrian behaviour key list, kept by hand in `people.launch.py` and
  again in `score_tracks.py`, so adding a behaviour to the driver produced
  models that spawn, a driver that runs, no command bridge, and a crowd that
  stands still with no error anywhere.

Each was silent. Each looked fully configured. Each was found by measuring a
physical outcome, never by reading the code. Anything derived per platform must
be generated per platform, and any list two programs share must have one owner.

### What the instruments now measure

Five probes, all reading `/ground_truth/poses` as a label oracle that never
touches the control path:

    tools/measure_slip.py             wheel odometry against ground truth
    tools/measure_localisation.py     believed pose against true pose
    tools/measure_contacts.py         how close the vehicle came to people
    tools/measure_path_efficiency.py  planner overhead against controller overhead
    tools/measure_control_latency.py  sensor to command, still unresolved

`measure_contacts.py` exists because a pedestrian was seen walking through the
robot. The person model carries no collision geometry on purpose, which means
**a person cannot be hit in this simulation**, so every safety claim about the
collision monitor was unfalsifiable and nothing anywhere counted contacts. It
measured a vehicle passing within **89 mm** of a person with no protective stop,
which is the largest open question in the project.

Two of these probes measured correctly and then printed the wrong verdict on
their own numbers: `measure_slip` accepted a 34 percent odometry error as
consistent, and `measure_localisation` compared a bare error against a bay edge
without the vehicle that has to fit inside it. Writing the measurement is the
easy half.

### The test track is now a warehouse, and it is generated

`tools/run_stack.sh --test-track --run survey_mission --cycles 5`

BUDGET AN HOUR. Surveying scales with floor area and this building has about
480 m2 of it, roughly double the first version. A 2400 s cap killed a run at
round 17 with 476.9 m2 mapped, which is all but finished, and the truncation
reads as a failed survey rather than as a clock running out. The AWS warehouse
needs a fraction of this.

The right fix is not a longer timeout. It is to survey ONCE, save the map, and
run missions against it with AMCL, which is what a commissioned vehicle does and
what the amcl block in the Nav2 configuration is already there for. That is a
real piece of work and it is not done.

42.0 by 13.56 m, and the height is DERIVED from the zones it must hold rather
than fixed, because a shell sized independently of its contents leaves a
remainder and a remainder is either a corridor or a trap. It carries four
racking rows, four equidistant roof columns, staged pallets, a corner charger, a
green home square around the spawn and three white delivery bays spread down the
back with a pallet between each pair.

Every width is derived. Each aisle is what the vehicle needs to turn round,
which is twice the widest all-round protective field it can select, PLUS a
pedestrian plus clearance so a person standing in a corridor is a detour rather
than the end of the run. Floor markings are visual only: a marking with
collision is a kerb the vehicle refuses to cross.

Six invariants are asserted rather than eyeballed, and each exists because it
was violated at least once: every aisle turnable, no leftover strip, furniture
clear of every wall, object and required pose, the floor covering the building,
nobody spawning on the vehicle, and the world name matching the file stem.

### What the two probes measured, and why it matters

Both attach to any run: `--latency` and `--classify`.

    157 protective stops:  155 structure, 0 pedestrians
    control_latency:       p50 76 ms, p95 328 ms, sd 82.8 ms

Neither was the expected answer and both are in V-28. The stops are not the
price of sharing a floor with people, they are the vehicle stopping for
structure at half a metre that the costmap does not know about, 43 of them
BEHIND it while driving forward. The latency estimate of 0.10 s is conservative
at the median and out by 3.3 times at the p95, which at 1.0 m/s is 228 mm of
protective field missing in one stop in twenty.

NOTHING HAS BEEN WRITTEN TO ANY SPEC on the strength of either. Both change the
safety case and want a decision.

### The next measurement, and it is narrow

The stop classifier has cut the structure stops down to two candidates:

  1. the self filter leaks at its corners. It rejects a 0.460 by 0.350 m
     RECTANGLE, which is the wrong shape for scanners mounted at 45 degrees on
     a vehicle whose circumscribed radius is 0.501 m. V-23 records this vehicle
     observing its own structure at 0.440 m.
  2. the rear field fires during forward motion.

Both are checkable and neither is tested. Start there rather than anywhere else.

### Still open

The MP-400 sits at 0 of 5 with a correct configuration. V-25 records five
refuted hypotheses, including one refuted before it was implemented because the
working platform disproved it.

---

The rest of this file is the previous handover, written 2026-08-12, kept
because its plan and its warnings are still current where this session has not
overwritten them.

## What works

An AMR surveys a warehouse it has never seen, builds a map, then repeatedly
collects a load at one station and delivers it to another while people walk
around it. A safety layer sits after the planner and can override it.

    4 of 5 transport cycles complete, about 70 s for a 17 m cycle
    safety costs 4 percent of cycle time
    SLAM maps 96 percent of the building unattended

This is the deliverable. Everything below is improvement, not repair.

## The test suite

    337 passed, 5 skipped

**There are no xfails left, and the way the last two went is the point.** Both
were `strict` and both marked V-39, the protective fields lying inside the
sensor's blind zone. Turning them into real assertions in V-49 immediately
found a bug in one of the tests: it read `points[0]` of each polygon, which for
a REVERSE field is the front corner sitting on the chassis by construction, so
it had been reporting a 151 mm reach as a negative one. A test that is expected
to fail is not being read.

**The five skips are load bearing and must not be deleted.** Three are the
MiR250's self filter, which is still a bounding box on purpose: 11 mm of the
40 mm measured on that vehicle is not explained by its pod geometry and nobody
has identified it, so V-39 stays open there. `UNSHAPED_SELF_FILTER` in
`amr_description/test/test_platform_spec.py` carries the reasoning, and each
entry asserts itself stale the moment that platform declares pods.

Run everything with ROS and the workspace sourced, or 14 description tests
error on a missing `xacro` rather than failing. Use the shared helper rather
than sourcing by hand, because `set -u` across the ROS setup scripts aborts on
`AMENT_TRACE_SETUP_FILES` and that trap has caught three scripts here:

    . tools/ros_env.sh
    python3 -m pytest src -q

Or in the container, from a clean base, to the same result:

    docker build -t amr . && docker run --rm amr

**Before starting anything that launches a simulator, check what is running.**
`pgrep -f run_stack` matches the shell asking the question, which has produced
a wrong answer three times, once costing a whole measurement:

    tools/whats_running.sh

Run everything with ROS and the workspace sourced, or 14 description tests
error on a missing `xacro` rather than failing:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src -q

## What the previous handover got wrong about those five failures

It said they were "all in `estimated` values, all mechanical". Three were not,
and the difference is recorded in full as V-23 in `docs/validation.md`.

The important one: `scanner_mount_x` and `scanner_mount_y` were labelled
`datasheet` against MP-400 manual section 1.3.3 and correctly transcribed from
it. That section locates the MP-400's OWN laser scanners, the unrated ones the
spec says at length this project does not fit. The strongest looking provenance
in the file was describing a sensor that is not on the vehicle. Three more
values in the same block contradicted the archived SICK sheet while labelled
`datasheet`, and only one of them failed a test. The corridor targets were not
published anywhere and had been invented.

Read V-23 before trusting any other figure in that spec.

## Running it

    tools/run_stack.sh --cameras off --run mission --cycles 5
    tools/run_stack.sh --platform mir250_class --cameras off  # the second platform
    tools/run_stack.sh --test-track            # the datasheet-sized track
    tools/run_stack.sh --run survey            # map the building
    tools/preflight.py                          # 21 health checks, ~15 s
    tools/stop_all.sh                           # always use this, never pkill

`--platform` reaches the robot description, the protective fields and the whole
Nav2 configuration, because all three are now generated per platform. There is
no fallback file on purpose: an unknown platform fails the launch instead of
quietly driving on another vehicle's tuning.

Logs go to a timestamped directory under `/tmp/amr-logs/`, with `latest` as a
symlink. Never read a fixed filename: three times in one session a stale log was
read as a fresh result, and once a previous run's numbers were nearly reported
as new.

`--cameras off` is the fleet tier and roughly halves CPU load. The keepout zones
cover the racking, so the cameras are not load-bearing for that case.

## What is left

Every item below is something this project can state a reason for, not a wish
list. Nothing here is claimed anywhere else in the repository.

**1. Attribute the latency tail.** p50 68 ms against p99 1260 ms. Until it is
attributed no protective field can honestly be sized on it. The candidates are
the executor contention behind the 380 ms MPPI iterations in V-37 and the scan
merger lag behind the transient source rejections in V-41.

**2. A human-aware costmap layer.** People are detected, tracked and scored,
and the planner routes around them as ordinary obstacles. It does not yet pay a
cost for passing close to one, which is exactly what the proxemic figures in
V-43 exist to make measurable.

**3. Precision docking**, once localisation supports the claim, and **physical
load transfer**, so that something is actually carried. A cycle currently loads
and unloads as a mission state with a dwell.

**4. Saved map plus AMCL.** Every mission still pays for a survey first. A
commissioned vehicle surveys once, saves, and localises. The `amcl` block is
already in the Nav2 configuration and unused.

### Deliberately not on this list

**A fleet layer.** Cancelled. The VDA 5050 vehicle interface stays because it
is the half an integrator connects to and it is tested end to end against a
broker; a dispatcher with one robot behind it would be a claim without a
measurement.

**HuNavSim.** Aborted on evidence in V-48: its Gazebo wrapper depends on
`gazebo_ros`, which is Classic, which is end of life and not packaged for
Jazzy. Adopting it means writing a new Harmonic system plugin to replace a
pedestrian system that already works and is measured.

**The MiR250 as a running vehicle.** See above.

## Things that are NOT true, despite having been said

**The warehouse does not have 1.35 m aisles.** That figure is from the MiR250
datasheet and was repeated as though measured. Real median corridor is 1.34 m,
p75 is 2.30 m. See V-22.

**The MP-400 is not the fix for the clearance failures.** It reaches 67.8 percent
of the floor against the MiR250's 63.2 percent. Five points, not a
transformation. Its value is architectural.

**The cause of the marginal clearance failures is unknown.** No investigation has
started. Do not assume a platform change addresses it.

**Do not widen the aisles.** The measurement does not support it.

## Method that worked, and is worth keeping

Three of the hardest faults hid behind explanations that were coherent and
wrong. What separated them, every time, was measuring the discriminating
quantity BEFORE building anything.

ADR 0010 is the clearest case: it proposed two fixes for a transform failure,
was written up as Proposed, the deciding measurement was taken, and it was
Rejected without a line of its implementation being written. That saved building
AMCL localisation to fix something it does not touch.

The six tools in `tools/` each exist because of a specific false conclusion, and
each documents it. Two of them produced false alarms of their own before being
fixed. A diagnostic that lies is worse than no diagnostic.

## Repository conventions

    physical constants live in config/platforms/*.yaml with recorded provenance
    a test fails the build if a value loses its source
    generated configs are never hand edited; a test asserts they match the generator
    a provenance label is a claim about a PART, not just about a document: check
      that the row you are citing describes the component actually fitted
    per-platform configs are generated for every platform, never defaulted
    /ground_truth/ is measurement only and must never reach the control path
    git authorship is Mohamed Kamel only, no AI attribution anywhere
    no em dashes, en dashes or double hyphens in any written document
