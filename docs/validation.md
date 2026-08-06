# Validation record

Verification asks "does the model do what I told it to". Validation asks "does the model behave
like the thing it represents". This file records the second one, against the archived data sheets in
`docs/datasheets/`.

A validation target is a published figure the model is checked **against**. It is never a tuning
target: where the model and the reference disagree, the disagreement is recorded here and the model
is left alone unless there is an independent reason to change it. Targets live in the
`validation_targets` block of each platform spec so they cannot quietly drift.

---

## V-01. Acceleration and the distance to reach top speed

**Status: reference is internally inconsistent. Model follows the acceleration figure.**
Measured 2026-08-06 by `src/amr_evaluation/tools/drive_check.py` against a live bringup.

The platform sheet publishes two figures that constrain the same behaviour:

- acceleration limit at maximum payload: **0.3 m/s2**
- minimum distance to achieve maximum speed: **9.5 m**
- maximum speed: **2.0 m/s**

Under constant acceleration these over-determine each other, and they do not agree:

| from | implies |
|---|---|
| 9.5 m to reach 2.0 m/s | a = v2 / 2d = **0.21 m/s2** |
| a = 0.3 m/s2 to reach 2.0 m/s | d = v2 / 2a = **6.67 m** |

So the sheet's own numbers differ by about 40 percent depending which you start from.

**Measured from the model:** reached 2.0 m/s after **6.55 m in 6.60 s**, effective acceleration
**0.30 m/s2**. That matches the configured limit and the 6.67 m it implies, to within the 0.12 m
attributable to sampling at the odometry rate.

**Interpretation.** The most likely explanation is that the real platform ramps with a jerk limit,
an S-curve rather than a step in acceleration. A jerk-limited ramp stretches the distance without
raising peak acceleration, which would reconcile 0.3 m/s2 with 9.5 m. This is a hypothesis, not
something the sheet states.

**Decision.** The model follows the acceleration figure, because that is the one the controller can
enforce directly and the one that governs stopping distance in the safety work later. The
consequence is that the modelled robot reaches top speed in about 6.6 m where the real machine takes
9.5 m, so **it accelerates more aggressively than the reference**. Anywhere that matters (throughput
figures, aisle transit times) this makes the model optimistic, and that is stated rather than
buried.

**Open.** Adding a jerk limit to the velocity smoother would let the model match both figures at
once. Worth doing before any throughput number is published.

---

## V-02. Corridor widths

**Status: not yet measured.** Targets recorded, planner not built.

The sheet publishes an operational corridor width of 1350 mm with the default footprint and
1000 mm with a dynamic footprint, and 950 mm for a 90 degree turn with muted protective fields. The
robot is 800 x 580 mm, so none of these are trivially satisfied and all three are meaningful tests
of footprint-aware planning.

## V-03. Minimum detectable object

**Status: not yet measured.** Targets recorded, perception not built.

20 mm at 1000 mm and 70 mm at 2500 mm, from the platform sheet, cross-checked against the scanner
sheet which offers a configurable 20 mm resolution. The scanner model is configured at that setting
on the strength of the two sheets agreeing, rather than to make a result come out.

## V-04. Runtime and state of charge

**Status: CALIBRATED, not validated.** This distinction is the whole point of the entry.

The sheet publishes three runtimes from a 1.63 kWh pack: 22 h standby, 17.4 h active unloaded, 13 h
active at maximum payload. The power model has three terms, so fitting it to those three figures
reproduces all of them exactly, by construction:

| term | fitted value |
|---|---|
| standby | 74.09 W |
| driving, at the reference speed | +39.17 W |
| full payload, at the reference speed | +63.41 W |

**This is a fit, not evidence.** Three equations, three unknowns. It cannot be cited as validation
of the energy behaviour, and `test_battery_model.py` says so in its own docstring. What it buys is
that state of charge falls at a rate anchored to a real pack, so an energy-per-task figure derived
later is traceable to a published source rather than invented.

**An undefined quantity in the source, made explicit rather than guessed silently.** The sheet never
says what duty cycle "active operation time" assumes. It cannot be continuous driving at top speed:
13 h at 2.0 m/s is 93.6 km on 1.63 kWh, about 17 Wh/km, which is implausibly efficient for a machine
of this class. The model therefore names a **reference speed** at which the published active figures
are taken to hold, defaulting to half of top speed, and a test asserts that the published runtimes
are still reproduced whatever reference speed is chosen. The assumption changes the coefficients; it
must not change the answer.

**Not modelled, because the sheet gives nothing to model them from:** voltage sag, temperature
dependence, and capacity fade over the rated 3000 cycles. Discharge is linear in energy.

---

## V-05. The vehicle must occlude its own scanners

**Status: defect found and fixed.** Measured 2026-08-06 on the running system.

The scanners were mounted at (0.34, 0.245) relative to the chassis centre, inside a chassis modelled
as a single 800 x 580 mm box. That put each sensor **inside** solid geometry, with the nearest wall
40 mm away in x and 30 mm in y, both below the 50 mm minimum range.

The scans looked perfectly healthy: 1618 returns, no infinities, a 0.27 m minimum and a 5 m median.
Nothing indicated a problem. The reason is that the renderer culls backfaces, so a sensor sitting
inside a box sees no box at all, and the vehicle was simply invisible to its own lidar.

Probing the arc that points into the vehicle made it obvious:

| front-left scanner, arc | before | after |
|---|---|---|
| outward, towards the world | 470 returns, median 4.55 m | 470 returns, median 4.57 m |
| inward, into the vehicle | 159 returns, **median 9.34 m** | 159 returns, **median 0.06 m** |

A 9.34 m median on rays aimed at the robot's own centre means the scanner was seeing straight
through the chassis to the far wall of the warehouse. Any later claim about 360 degree coverage,
protective field geometry or blind sectors would have rested on that artifact.

**Fix.** The chassis is now two overlapping boxes forming a plus shape, leaving the four corners
open as recesses for the scanners. Both boxes stay inside the published 800 x 580 mm envelope, the
sensors sit in the open corners rather than inside solid geometry, and the remaining body is a real
occluder. After the change the inward arc returns 0.06 m, which is the chassis wall where it should
be.

**Consequence for the perception work.** Those self-returns are real and must be removed by a
footprint filter in the scan merge. That was already planned; it is now a demonstrated requirement
with a measurement behind it rather than an anticipated one.

**Generalisation worth remembering.** A sensor that produces plausible-looking data is not a working
sensor. This was only caught by asking what a specific arc *should* return and checking that it did.

---

## V-06. 360 degree coverage: improved, and still short of the claim

**Status: partially reproduced. The reference claims more than this model delivers, and the gap is
measured rather than glossed.**

The platform sheet specifies two SICK nanoScan3 scanners giving "360 degree visual protection around
robot". The merged scan is measured directly for contiguous blind sectors.

The metric matters as much as the number. A count of empty bins is nearly useless here: re-binning
polar data from an off-centre origin at the same angular resolution necessarily leaves scattered
single-bin holes, which are an artifact of the re-binning and harmless. A **contiguous run** is a
blind sector, and that is the only thing worth asserting.

| scanner optics | empty bins | largest contiguous gap |
|---|---|---|
| recessed inboard at (0.340, 0.245) | 273 of 2118 (12.9%) | **11.56 degrees** |
| flush at the corner, (0.405, 0.295) | 128 of 2118 (6.0%) | **4.25 degrees** |

**Why recessing fails, and it is structural rather than a tuning error.** From (0.340, 0.245) a ray
aimed at the opposite front corner must drop 45 mm in y to clear the chassis roof at y = 0.200, which
at 45 degrees costs 45 mm in x and puts it at x = 0.385, still inside the chassis half-length of
0.400. It re-enters the body before clearing it. Seeing tangentially along the vehicle's own side
requires the optics to be at or beyond the envelope corner, full stop.

**The fix.** The link origin is now the optical centre, flush at the corner and 5 mm proud, with the
housing drawn 60 mm inboard so the physical body still sits in the recess and inside the published
envelope. That is how a corner-mounted safety scanner is actually packaged. A test in
`test_platform_spec.py` now asserts the optics are **not** inboard of the corner, and bounds how far
proud they may stand; the previous version of that assertion required the opposite and had encoded
the defect.

**What remains, stated plainly.** A 4.25 degree seam persists at the two diagonal corners, roughly
0.37 m of unobserved arc at 5 m range. **So the model does not fully reproduce the sheet's 360
degree claim.** The residual is vehicle occlusion at grazing incidence, not a merge error: a unit
test drives the same merge with two ideal unoccluded scanners at the deployed 2118-bin resolution
and the largest gap is at most one bin, so the arithmetic is clean and the seam is geometry.

**Consequence to carry forward.** The protective field design in the safety phase must treat these
two sectors as unobserved rather than assuming full coverage, or it will claim protection the sensor
set does not provide. Closing the seam entirely would need a chamfered chassis, which needs a mesh
rather than the box primitives used here.

---

## V-07. People detection from one scan plane

**Status: measured. Recall and localisation are good; precision is poor, for a reason that is
structural and that the next two components exist to fix.**

Scored by `src/amr_evaluation/tools/score_detections.py` against the `static_people` scenario, which
states exactly where four pedestrians stand. This is the first use of the simulator as a **label
oracle** rather than as a source of control input, which is the inversion described at the top of
this file.

40 frames, 4 people, a match counted within 0.40 m:

| metric | value |
|---|---|
| recall | **0.875** |
| precision | **0.168** |
| localisation error, mean | 9.7 cm |
| localisation error, p50 | 5.4 cm |
| localisation error, p95 | 33.2 cm |

Per person: 100% at 2.00 m ahead, 100% at 2.50 m behind, 95% at 2.20 m to the side, and **55% at
1.28 m**, the closest one. The near miss rate is the more interesting number and is discussed below.

**Precision is 0.168 because a shelf upright is a leg.** 16.5 false positives per frame, but from
only 32 distinct positions, and 9 of those appear in at least 80 percent of frames and account for
56 percent of all false positives. They are not noise. They are warehouse structure: rack uprights,
pallet-jack legs, clutter. A round, leg-sized, leg-separated pair of vertical cylinders is exactly
what a shelf frame presents on a plane 150 mm off the floor.

**No parameter tuning will fix this, and tuning would be the wrong response.** The detector is
already discriminating on the only two things a single plane offers, size and roundness, and a shelf
upright matches a calf on both. Narrowing the width band to exclude this particular warehouse's
uprights would be fitting to one scene rather than detecting people. The result stands as measured.

**What actually separates people from structure, in the order the plan builds them:**
1. **Height above the scan plane.** The RGB-D channel sees that a person has a torso at 1.2 m and a
   rack upright continues to the ceiling. One plane cannot see either.
2. **Motion.** A rack upright is in the same place every frame, which is precisely what the analysis
   above measured. A tracker's VELOCITY ESTIMATE separates the two. Note that track confirmation
   does not: a rack upright is seen in every frame and so confirms immediately. Measured in V-08:
   the motion test lifts precision 4.4 times and halves recall, because a person standing still has
   no velocity either.

So this number is the baseline the fused detector and the tracker have to beat, and it is recorded
so that improvement can be quantified rather than asserted.

**The 55% at 1.28 m is a separate effect** and was chased in V-11. The cause stated here originally,
that the legs merge into one wide cluster, was a HYPOTHESIS and it was wrong. See V-11 for what the
scan actually shows.

---

## V-08. Tracking: what the motion test buys, and what it costs

**Status: measured. A real improvement, not a sufficient one.**

Scored by `src/amr_evaluation/tools/score_tracks.py` against the `walking_people` scenario: three
pedestrians walking fixed legs and one worker standing still. 60 frames, a match counted within
0.50 m. Ground truth from the simulator pose feed, used only for scoring.

| | all confirmed tracks | moving tracks only |
|---|---|---|
| precision | 0.071 | **0.312** |
| recall | **0.575** | 0.242 |
| localisation p50 | 4.9 cm | 9.0 cm |
| id switches | 3 | **1** |

**Classifying by velocity improves precision about 4.4 times**, from 0.071 to 0.312, and cuts ID
switches from 3 to 1. That is the measured answer to what tracking contributes.

**It also halves recall, and that is not a bug.** The scenario deliberately contains a stationary
worker. A person standing still has, by definition, no velocity to distinguish them from a rack
upright, so the motion test discards them. Leaving the standing worker out of the scenario would
have produced a much prettier number and a dishonest one.

**Correcting something stated in V-07.** That entry said a tracker removes persistent static returns
"almost by construction". That is wrong and the tracker's own tests now say so explicitly. Track
confirmation counts how often something is seen, and a rack upright is the most confirmable object
in a warehouse: it confirms immediately. Only the velocity estimate separates them, and only for
people who are moving.

**Precision 0.312 is still poor**, and no amount of tracker tuning closes it, for the same reason as
V-07. What remains is height: a torso at 1.2 m that a rack upright does not have. That is the RGB-D
channel, and these figures are the baseline it has to beat.

### A measurement bug worth recording, because it looked exactly like a result

The first run of this scoring reported precision 0.019 and recall 0.129, numbers so bad they would
have read as a broken tracker. They were a broken measurement. The scorer hardcoded the robot's
spawn pose to convert ground truth into the sensor frame, and `walker_slow` had been given a path
running straight down the robot's own row. A velocity-controlled pedestrian does not respond to
contact, so a 75 kg person simply shoved the 97 kg vehicle 2.2 m backwards, and every ground-truth
position was wrong by that much.

Two fixes, one of them structural: the scorer now **reads the robot pose from the oracle every
frame** rather than assuming it, and the pedestrian path no longer crosses the vehicle. The general
lesson matches V-05: a number produced by a broken measurement is indistinguishable from a number
produced by a discovery, and the only defence is to sanity-check the inputs before believing the
output.

---

## V-09. Camera field of view: an ambiguity resolved against a second source

**Status: resolved. The model was wrong by 27 degrees per camera and is now corrected.**

The platform sheet lists "2 pcs 3D camera Intel RealSense D435" and, under the same heading,
"FoV horizontal angle: 114 deg". It never says whether that figure describes one camera or the pair.
The model took the optimistic reading, 114 degrees per camera, and the description carried an
explicit note that no detection or coverage figure could be published until a second source settled
it. This entry is that settlement.

The archived Intel D435i specification gives a depth field of view of **87 deg +/- 3 horizontal by
58 deg +/- 1 vertical**. A single camera therefore cannot be 114 degrees, so the platform figure is
the pair.

| quantity | before | after | source |
|---|---|---|---|
| horizontal FoV per camera | 114 deg | **87 deg** | Intel sheet |
| vertical FoV per camera | not modelled | **58 deg** | Intel sheet |
| toe-out per camera | 20 deg, chosen | **13.5 deg, derived** | (114 - 87) / 2 |
| near clip | 0.25 m | **0.105 m** | Intel Min-Z |
| far clip | 10 m, assumed | **10 m** | Intel maximum range |

Two 87 degree cameras toed out 13.5 degrees span 114 degrees, which reproduces the platform figure
exactly. The two documents now agree, and `test_platform_spec.py` asserts the derivation so it
cannot drift.

**A conflation corrected at the same time.** The near clip had been set from the platform sheet's
0.25 m "minimum distance in front of robot for ground view". That is a mounting-geometry figure,
the closest point on the floor the camera can see given where it sits, and it is not the sensor's
minimum depth. Those are different quantities and had been used interchangeably. Min-Z is 0.105 m.

**A provenance claim corrected too.** The camera mass carried the source "taken as the published
mass of the named Intel RealSense D435 module". The Intel sheet gives dimensions and no mass, so
that citation was to a document that had not been read. It now says the mass is estimated and that
neither sheet publishes it. The module dimensions, which the sheet does give as 90 x 25 x 25 mm,
moved from estimated to datasheet and happened to match the estimates exactly.

---

## V-10. The depth channel's vertical field of view limits it more than expected

**Status: measured, and it changed the design.**

The height channel exists to separate a standing person from a rack upright, which neither the scan
plane nor the tracker can do. The intended discriminator was simple: a person tops out around
1.75 m, a rack upright keeps going.

That test turns out to be unusable at close range, and the geometry says why. The camera sits at
0.27 m with a 58 degree vertical field of view, so the highest point it can see at horizontal
distance r is 0.27 + r*tan(29 deg):

| range | highest visible point | a 1.75 m person |
|---|---|---|
| 1.5 m | 1.10 m | cut off |
| 2.5 m | 1.66 m | cut off |
| 4.0 m | 2.49 m | fits |
| 6.0 m | 3.60 m | fits |

**Closer than about 2.7 m every tall object is truncated to the same apparent height.** A 3 m rack
upright and a 1.75 m person both appear to stop at the visible ceiling, so "it tops out where a
person tops out" is not merely weak there, it is meaningless. This was found because a unit test
asserting that a 3 m column is structure failed at 2.5 m, which looked like a detector bug and was
a geometry lesson.

**What changed.** Clusters now carry a `truncated` flag and the visible ceiling at their range. When
a cluster is truncated the height test is **skipped rather than trusted**, and `looksLikeStructure`
refuses to assert anything, because a truncated column might be a person or a pillar and choosing
would be a guess. Two tests cover it, one at 5 m where the top is observable and one at 2.5 m where
it is not.

**So the discriminator that actually carries the load is the WIDTH PROFILE**, not the height: a
person is narrow at the calves and wider at the torso, a post is the same width all the way up. That
works at any range where the torso is visible, which is everywhere beyond about 1.7 m. A test
renders a post cut off at exactly 1.60 m, so its top falls inside the person band, and confirms the
width profile still rejects it.

**Carried forward.** The near field below about 1.7 m is now poorly served by BOTH channels: the leg
detector merges the legs into one wide cluster (V-07, 55 percent detection at 1.28 m) and the depth
channel cannot see the torso. That is exactly where a protective stop matters most, and it is the
open problem for the safety phase rather than something to be quietly averaged away.

---

## V-11. The near field: the cause was not what I claimed, and the fix is not classification

**Status: cause diagnosed, partially fixed, and the safety argument re-grounded with a measurement.**

V-07 measured 55 percent detection on a pedestrian at 1.28 m and asserted a cause: that close in, a
person subtends a large angle and the two calves merge into one cluster too wide for the leg test.
**That was a hypothesis written as if it were a finding, and it was wrong.**

A unit test made it fail. Synthesising a pedestrian at 1.2 m, the legs resolved and paired perfectly
well. So the merge story could not be right, and the running system was asked directly what it
sees around that person:

```
  10 contiguous runs of returns under 3 m, around the right bearing:
    3 pts   1.32 m   width 0.022 m
    3 pts   1.31 m   width 0.019 m
    4 pts   1.30 m   width 0.022 m
   16 pts   1.04 m   width 0.249 m
    1 pt    0.99 m
    2 pts   0.95 m   width 0.009 m
   14 pts   0.98 m   width 0.205 m
```

**Not merged. Fragmented.** Runs of one, two, three and four points separated by two and three bin
holes, every fragment too short or too narrow to classify.

**The cause is the merge itself.** Two scanners mounted 0.5 m off the robot centre are re-binned
about that centre, and the angular mapping is non-uniform, so output bins are simply skipped. Close
in it is severe. This is the same aliasing that shows up as scattered single-bin gaps in V-06, where
it was harmless; here it destroys the evidence.

**Fix.** The clusterer may now bridge up to three consecutive empty bins, but only when the measured
points either side agree in space by the same adaptive threshold used for adjacency, and the
allowance scales with how many bins were skipped. A genuine occlusion boundary still breaks the run.
Note this required contradicting an earlier test whose comment read "a hole is absence of
measurement, bridging it would invent continuity". That is true of a RAW scan and false of a
re-binned one, and the distinction had been missed.

| | before | after |
|---|---|---|
| precision | 0.168 | **0.218** |
| recall | 0.875 | **0.900** |
| false positives over 40 frames | 692 | **517** |
| person at 1.28 m | 55% | **60%** |
| person at 2.20 m | 95% | **100%** |

**60 percent is still poor.** Bridging holes treated a symptom; the representation was the cause,
and V-12 fixes it properly. The reframing below stands regardless, and is the more important half.

### The protective stop must not depend on classification, and does not

A protective field trips on **any** return inside it. It does not ask what the return is, and it is
not allowed to: that is what makes it a protective device rather than a perception system, and it is
how ISO 3691-4 and EN ISO 13849 protective fields work.

So the right question is not "can the detector name that pedestrian" but "are the returns there at
all". Measured, same scenario, same 40 frames:

| | rate |
|---|---|
| frames with any return on the person at 1.28 m | **40 / 40, 100%** |
| frames where the detector classified them as a person | 24 / 40, 60% |

**The returns are present in every single frame.** What fails is naming them, not seeing them.

That relocates the 60 percent from a safety problem to a behaviour problem. A field-based protective
stop would trip every time. What imperfect classification costs is the *smooth* behaviour built on
top: yielding early, predicting where someone will walk, choosing which side to pass. Those degrade;
the stop does not.

**Consequence, carried into the safety phase.** The protective stop will be built on the merged scan
directly, through `nav2_collision_monitor`, and will not consume `people_detections` or
`people_tracks` at all. Wiring classification into a protective function would make safety depend on
a component measured at 0.218 precision, which would be the wrong architecture no matter how good
the classifier later becomes.

---

## V-12. Fixing the representation, not the symptom

**Status: root cause fixed. Recall 1.000 on the static scenario.**

V-11 bridged holes in the binned scan, which lifted the near-field rate from 55 to 60 percent. That
treated a symptom. The cause is that **a LaserScan is a lossy container for two sensors at different
origins**: re-binning about the robot centre skips bins, and no amount of bridging recovers what the
binning discarded.

The merger now also publishes every accepted return **un-binned**, as a point cloud, and the
detector clusters those. The binned scan remains, because a costmap and a collision monitor both
want a LaserScan and for that purpose the holes are harmless: a protective field asks whether a
return is inside it, not whether its neighbours are contiguous.

**The first attempt at this made things worse, which is worth recording.** Clustering the merged
points by walking them in bearing order dropped recall to 0.250 and reduced the pedestrian at 2.2 m
to a three-point fragment. The reason is specific to a merged sensor set: **the two scanners see
different surfaces of the same object**, so bearing order interleaves them, consecutive points jump
between surfaces, and the run splits. Any ordering-based clustering has this failure.

A grid plus connected components has no ordering, so neither failure mode can occur. A test asserts
the result is invariant under a permutation of the input.

**Cell size then had to be chosen, not guessed.** At 0.06 m the eight-neighbour reach is 0.085 m,
which exceeds the 0.090 m gap between two 110 mm calves only marginally, and the pedestrian at
2.50 m merged into one 0.31 m cluster, too wide for the leg test: **detection went to zero for that
person while the near-field one went to 100 percent.** At 0.040 m the reach is 0.057 m, comfortably
below the leg gap, and a leg stays whole out past 12 m where its own point spacing is 0.036 m.

| | binned scan | + hole bridging | un-binned points |
|---|---|---|---|
| recall | 0.875 | 0.900 | **1.000** |
| precision | 0.168 | 0.218 | 0.175 |
| localisation p50 | 5.4 cm | 5.2 cm | **4.5 cm** |
| person at 1.28 m | 55% | 60% | **100%** |
| person at 2.50 m | 100% | 100% | **100%** |

**Every person, in every frame.** Precision is essentially unchanged, which is expected and is not
this fix's job: it is structural, a rack upright really is a leg-shaped object, and the height
channel and the motion test are what address it.

---

## V-13. The protective stop, and three bugs on the way to it

**Status: working and measured. See docs/safety_concept.md for the geometry.**

| command | active protective field | obstacle at 0.778 m inside it | result |
|---|---|---|---|
| 0.25 m/s | 0.561 m from base_link | no | moves, at 0.081 m/s |
| 0.80 m/s | 0.854 m from base_link | yes | **stops, 0.000 m/s** |

That single table shows the protective stop firing, the field switching with speed so the same
obstacle is safe at one speed and not another, and the warning field slowing the vehicle to 0.081
from a 0.25 m/s command at the configured 0.3 ratio.

**Three failures on the way, each of which presented as something else.**

1. **The monitor published the wrong message type.** `diff_drive_controller` 4.x consumes
   `TwistStamped`; the monitor defaults to plain `Twist`. Both types then appear on the same topic,
   the controller ignores the one it does not want, and the robot never moves with **no error
   anywhere**. It looked exactly like a permanent protective stop. Found because `ros2 topic hz`
   refused the topic, reporting it carried two types. Fixed by `enable_stamped_cmd_vel`, now
   asserted by a test.

2. **The vehicle stopped for its own scanner pods.** The optics sit flush at the envelope corner and
   the housing is a 107 x 80 mm box rotated 45 degrees, so the pods reach **28.7 mm proud** of the
   published envelope. The self-return filter used a 20 mm margin, those returns survived it, landed
   inside the protective field, and the vehicle held a permanent stop against itself. The margin is
   now derived from the pod geometry and a test asserts it covers them.

3. **Nav2 wants polygon points as a string**, not a numeric array. Configuring with an array fails
   with `parameter points has invalid type`, and the lifecycle manager reports only `failed to
   change state`. The node had to be run standalone to see the real message.

**And one of my own claims was false.** The platform spec recorded control latency as *measured*
end to end. It was not measured; it is an estimate, and it feeds the protective field size directly.
The provenance gate caught it only indirectly. That is the second fabricated citation in this
project, after the camera mass, and both were found by reading rather than by a test, which is now
written into the gate's own docstring as a limit of what it can check.

---

## Known limitations of the model

Recorded here rather than discovered later. None of these are bugs; they are places where the model
is deliberately narrower than the machine.

**Caster joint states are not published.** The four casters are passive and are simulated correctly
by the physics engine, but they are not declared in `ros2_control`, so nothing publishes their joint
positions and their frames therefore do not appear in TF. This is faithful to the real platform,
where casters carry no encoders, but it means RViz draws them at their zero position no matter what
the robot is doing. If a demonstration video needs them to look right, a simulator-only joint state
publisher can supply them, and that would have to be labelled as sim-only.

**`scan_time` is zero on the laser messages.** Gazebo does not populate the field. Nothing in the
stack currently uses it, but any consumer that does would need it filled in.

**No jerk limit.** See V-01. The velocity smoother steps acceleration rather than ramping it, which
is why the model reaches top speed sooner than the reference.

**A cosmetic CMake dev warning on build.** `colcon build --symlink-install` emits a CMP0009 policy
warning from ament's own install code when it globs a directory containing symlinks. It comes from
upstream, not from this project, and setting the policy locally does not reach the scope that
raises it. Left alone rather than papered over; builds are otherwise clean.

**Camera field of view is the optimistic reading of an ambiguous sheet.** See the note in
`urdf/sensors.xacro`. No detection or coverage figure may be published until the Intel data sheet is
archived.

---

## Repeatability

**The drive check was not repeatable, and the fix is recorded because the failure was instructive.**
Run twice in a row, the second run began its acceleration ramp at roughly 1.75 m/s left over from
the first, and duly reported reaching top speed in 4.99 m at an effective 0.35 m/s2, above the limit
the controller enforces. It looked like a physics finding and was measurement error: the check
commanded zero for a fixed number of spins instead of waiting for the robot to actually stop. It now
blocks until velocity is below tolerance for ten consecutive samples, and two back-to-back runs
produce identical figures.

The simulation benchmark repeats to within a few percent on the expensive configurations. The cheap
ones vary by around 8 percent, because their per-step cost is a small difference between two larger
wall times, so those are quoted to two significant digits.
