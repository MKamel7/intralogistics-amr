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

## V-15. Animated pedestrians: the cause was duplicate nodes, not physics

**Status: working.** Measured over 100 s: all three walkers travel back and forth inside their
lanes, 2.0 to 2.4 m of extent against 51 to 83 m of path, all resting at z = 0.020 m, with the
standing worker correctly stationary.

**The cause was not physics at all, and I spent a long time believing it was.** Sixteen
`ground_truth_publisher` nodes and eleven `pedestrian_driver` nodes were running simultaneously,
left behind by repeated launches. Killing a launch does not reliably kill the nodes it started.
Eleven drivers were publishing conflicting velocity commands to the same three pedestrians, which is
why every measurement contradicted the last and why the same configuration appeared to work and then
not work.

**This invalidated a long run of measurements**, and the wasted effort is the lesson: before
believing a behavioural measurement, check `ros2 node list | sort | uniq -d` for duplicates.

Two genuine model fixes were also needed and are kept:

- **Inertia matching the collision body.** The figure carried a real person's centre of mass at
  0.95 m over a 0.22 m collision puck, which tips at about 13 degrees of lean. It now sits at 0.12 m
  with that cylinder's tensor.
- **No collision geometry.** Lanes were cleared against the 150 mm scan plane, but a body collision
  reaching 1.2 m can still snag on a shelf beam that the probe never saw. A driven pedestrian is an
  input to the simulation, not a subject of it, and the lidar renders visuals so the scanner sees the
  figure exactly as before.

Lane placement is also no longer guesswork: the free floor was measured by probing the live scan on a
grid, which showed x = 3 blocked at every row, exactly where two walkers and the standing worker had
been placed.

### Approaches that failed, kept because each was instructive

| approach | result |
|---|---|
| kinematic link | the velocity plugin integrated it out of the world, to z = 19 m |
| prismatic rail welded to `world` | a model joined to `world` is treated as fixed and its joints do not articulate; verified by commanding the joint directly |
| removing floor friction | no effect |

## OPEN-1 (RESOLVED, see V-15)

**Status: was not working; the entry above supersedes this.**

Static pedestrians work correctly and are what the demo scenario uses: all four rest on the floor and
are detected in 100 percent of frames (V-12). Walking pedestrians do not.

Five approaches were tried and each failed differently:

| approach | result |
|---|---|
| dynamic body, human inertia (CoM 0.95 m over a 0.22 m puck) | tipped at ~13 degrees of lean, climbed the racking, hovered at z = 0.27 m |
| inertia moved into the collision puck | upright and stable, but barely moves |
| floor friction removed | still barely moves |
| kinematic link | the velocity plugin integrated it out of the world, to z = 19 m |
| prismatic rail welded to `world` | a model joined to `world` is treated as fixed and its joints do not articulate; verified by commanding the joint directly in Gazebo |

Current state: gravity disabled on the body, lanes placed on floor **measured** clear from the live
scan rather than guessed. Two of three walkers move 0.5 to 0.7 m of a 3 m lane; one does not move.

**What the evidence says.** The command demonstrably reaches the plugin (verified on the Gazebo
topic), the figure is upright and at the right height, and the lane is clear. So the remaining
problem is in how `VelocityControl` interacts with this body, not in the driver, the bridge or the
scenario.

**The right fix is a small Gazebo system plugin in C++** that sets the model pose directly along a
waypoint list, which is how moving obstacles are normally done and which removes the dynamics
question entirely. That is scoped work, not another guess, and it belongs with R4 of the revised
plan.

## V-14. SLAM: three separate faults, each silent

**Status: working.** Map 276 x 414 at 5 cm resolution, 88.1 m2 mapped free and 1.9 m2 occupied, with
`map -> odom` published.

Getting there took three fixes and **not one of them logged an error**. The node ran, the scan was
healthy at 3102 returns per frame, and nothing at all appeared on `/map`.

1. **The merged scan was malformed.** It declared `angle_max = angle_min + 2*pi`, so the first and
   last ray described the same bearing and a consumer stepping from `angle_min` by
   `angle_increment` ends up with one ray too many. Now one increment short of a full turn, with a
   test. This was a real bug and fixing it was correct, but it was **not** the cause.
2. **The scan had no frame of its own.** Its `frame_id` was `base_link`, which is also
   slam_toolbox's `base_frame`, leaving the laser transform as identity. The merged scan is now
   published in `base_scan`. Also not the cause on its own.
3. **`async_slam_toolbox_node` is a LIFECYCLE node and nothing was managing it.** It sat in
   `unconfigured` indefinitely, which is why it declared no parameters at all: `ros2 param get
   /slam_toolbox resolution` returned "Parameter not set" and the node logged only
   "Failed to get parameters: resolution". A `nav2_lifecycle_manager` now brings it up. **This was
   the cause.**

The diagnostic that cracked it was checking `ros2 lifecycle get`, which is exactly the check that
cracked the identical problem on `nav2_collision_monitor` earlier in the project. A silent node
should be checked for lifecycle state before anything else.

A fourth thing was found along the way and is worth recording separately: the params file was keyed
`slam_toolbox:` and is now keyed `/**:`. That change is harmless and more robust, but it did not fix
anything, because an unconfigured lifecycle node has no parameters to set under any key.

---

## V-16. Navigation: seven silent faults, none of them where they appeared to be

The vehicle would not drive. Over several days this presented, every time, as a controller tuning
problem: the commanded speed sat at a fraction of what was asked for, or the vehicle spun on the
spot, or it crawled and gave up. It was never once a controller tuning problem. Seven separate
faults produced the same symptom, and every one of them was silent.

**Nothing published on `/odom`.** ros2_control publishes `/diff_drive_controller/odom`; every Nav2
default is plain `/odom`. MPPI reads its measured velocity from that topic and limits each command
to what the acceleration limit allows from there, so with no odometry it believed the vehicle was
permanently stationary and every command was 0 + ax_max * model_dt = 0.015 m/s, forever. Measured:
0.019 m/s commanded, 0.24 m travelled in 180 s. No node logged anything; the topic simply had no
publisher and the subscriber waited.

**Two publishers on `/clock`.** An orphaned bridge from an earlier launch survived a teardown. With
two clock sources, simulated time jumps backwards, every node clears its TF buffer several times a
second, and 19 percent of `map` to `base_link` lookups fail. After the fix, 1.4 percent.

**The collision monitor left INACTIVE.** Its lifecycle activation timed out during a crowded
start-up. With no monitor, nothing forwards commands to the wheels. Everything else reported
healthy.

**An uncovered velocity silences the monitor.** See ADR 0009. Reverse and spot turns above 1 rad/s
matched no polygon, and the monitor responded by publishing nothing rather than by stopping.

**The warning field scaled instead of capped.** See ADR 0009. Stable fixed point at 0.0064 m/s.

**The survey offered goals the planner must refuse.** Goals were required to have 0.45 m of
clearance against an inscribed radius of 0.501 m, so essentially every goal was inscribed-inflated
and therefore lethal. Eleven consecutive "no valid path found". With no path there is nothing to
follow, so the behaviour tree ran its recoveries, and the spinning that looked so much like a
controller fault was the `spin` recovery doing its job correctly.

**Reachability meant two different things.** The survey's flood fill walked free cells; the planner
needs a corridor of at least twice the inscribed radius. So the survey could offer a goal reachable
only through a gap the vehicle does not fit through. Fixed with a Euclidean distance transform so
the fill traverses only cells the vehicle fits in.

The lesson is not any one of these. It is that six of the seven were invisible from the topic list
and none produced an error. `tools/preflight.py` now checks all of them in about fifteen seconds:
one clock publisher, every lifecycle node active, every link of the command chain with a publisher,
TF resolving, sensors flowing. It would have found the first four immediately.

Two of its own checks were wrong when first written, in the same way the system was: it counted
publishers before discovery had settled and reported 0 publishers for a topic it simultaneously
measured flowing at 14.4 Hz, and it counted TF failures during the listener's buffer-fill period
and scored a healthy system at 11.7 percent against a 10 percent threshold. Both are fixed and both
are commented, because a diagnostic that lies is worse than no diagnostic.

---

## V-17. Orphaned nodes: the teardown script was wrong for most of the project's life

`ros2 node list` showed **19 `/battery_model`, 6 `/leg_detector` and 5 `/people_tracker`** nodes
alive at once. Load average was 47 on 12 cores and the MPPI control loop had starved to 2.5 Hz
against a required 20 Hz, which of course presented as a navigation problem.

The cause was `tools/stop_all.sh`. It matched a hand-maintained list of process names that included
the simulator, the bridges and the Nav2 servers, and omitted every node this workspace builds
itself. Each restart therefore left those behind, across dozens of restarts.

The first fix was also wrong, and instructively so: it resolved `/proc/PID/exe` and killed anything
executing from the workspace install tree. That catches C++ nodes. A Python node's executable is
`/usr/bin/python3`, so every Python node in the project survived it. The list of survivors is
exactly the list of Python nodes: battery_model, pedestrian_driver, ground_truth_publisher,
truth_map_publisher, survey_runner.

The script now tests both the resolved executable and the command line, and reports what it could
not stop rather than exiting quietly. Verified: 50 processes stopped, 0 survivors, load average
falling from 47 to 17.8.

Related and worth recording: `pkill -f 'gz sim'` matches the shell running the pkill, because that
shell's own command line contains the pattern. The shell dies partway through the pattern list, the
rest never runs, and the caller sees exit 144 while half the stack is still alive. This cost three
separate debugging rounds before it was understood.

---

## V-18. Mapping quality, measured rather than looked at

Every mapping fault in this project survived because the map was judged by eye. `score_map.py` now
compares the SLAM map against the ground truth floorplan, searching the alignment offset rather
than assuming it.

Current figures, on the map produced by an unattended survey from a standing start:

| | |
|---|---|
| true floor, vehicle height band | 236.2 m2 |
| mapped as free | 212.5 m2 |
| coverage of the true floor | 78.2 % |
| precision of the mapped free space | 87.0 % |
| IoU on free space | 70.1 % |
| obstacle recall, within one 50 mm cell | 10.7 % |
| **claimed free but really obstacle** | **6.93 m2, 3.26 % of what it calls free** |

Two of these need reading carefully rather than quoting.

**Obstacle recall is low and that is mostly the metric, not the map.** The ground truth marks every
mesh outline, including interior and hidden surfaces no scanner can ever see, while SLAM marks only
cells where a beam actually returned. Matching is already given one 50 mm cell of tolerance, without
which two correct outlines of the same wall offset by a single cell score zero and the figure read
2.4 percent. The number is kept because it is honest, not because it is flattering.

**The last row is the one that matters**, because it is the dangerous direction: floor the planner
would route through that is really obstacle. It is the residual under-racking space, and it is why
ADR 0007 declares keepout zones rather than relying on the map being right.

A bug in this project's own map writer was found while establishing these figures. The generated
YAML used `free_thresh: 0.25`. The PGM writes unknown as 205, which decodes to an occupancy of
exactly 0.196, so at 0.25 every unknown cell decoded as FREE. That inflated the ground truth floor
from 236.2 m2 to 312.8 m2 and would have quietly flattered every coverage number measured against
it. The ROS convention is 0.196 precisely so unknown lands on the boundary.

---

## V-19. The transport task, and the layering mistake that hid inside it

The first complete pick and deliver cycle:

    cycle 1: complete in 123 s, 17.6 m driven, 0 protective stops, 0 s held up
             [goods_in 86 s, dispatch 27 s]

Before the fix below, that same delivery leg timed out at 240 s having covered
12.1 m of an 8.1 m journey, with no protective stops, no planner errors and
nothing else wrong. Three separate causes were eliminated on the way, and each
looked plausible while it lasted.

**The vehicle was stopping for itself.** 12 of 12 protective stops in one run
came from a return at exactly 0.440 m, minimum equal to median equal to maximum,
always forward, and absent from BOTH costmaps. A real obstacle gives varying
ranges and appears in the costmap; fixed geometry does neither. 0.400 m of half
length plus a 0.035 m self filter margin is 0.435 m, so the vehicle's own
structure sat 5 mm outside its own filter. The margin had been DERIVED as 28.7 mm
from the scanner housing geometry, which was sound and incomplete. Measured, the
reach is 40 mm. At 0.060 m the stop count went to zero.

**The payload feature reintroduced a fault this project had already fixed.** The
laden acceleration limit was being applied to MPPI's `ax_max`. That parameter is
not a physical limit, it is the bound used to generate candidate trajectories,
and at 0.3 m/s2 every one of the 2000 samples lands within 0.015 m/s of the last
command, so the optimiser has no gradient and returns its prior. ADR 0008 records
exactly this happening and being fixed by raising `ax_max` to 1.0. The transport
task then set it back to 0.3 the moment the vehicle picked something up.
Measured on the laden leg: 0.013 to 0.022 m/s commanded for four minutes, which
is 0.3 * 0.05 = 0.015 m/s to three figures. The unladen pick-up leg on the same
run arrived normally, which is what finally made the pattern visible.

The limit now goes to the VELOCITY SMOOTHER, which is a rate limiter between the
controller and the wheels. A load that must not slide is a constraint on the
commanded profile, and a smoother is what enforces those. It is not a constraint
on how a controller explores its options, and conflating the two cost a day.

**A diagnostic that lied.** The tool written to find this tracked distance to
`/goal_pose`, which the mission never publishes because it drives through the
NavigateToPose action. It fell back to the first plan's endpoint and held it for
the whole run, then reported that the plan ended 8.17 m from the goal. That was
an artefact of the tool, not a finding, and it is recorded because it was very
nearly reported as one. It now tracks the live plan endpoint.

Still outstanding: 1 of 2 cycles completes. The second failed at the dispatch
station after 8 s, which is a goal rejection rather than a drive failure, and is
not yet diagnosed.

---

## V-20. Five cycles: the transport task works, and fails in one specific way

Five consecutive cycles in a single run, which is the first measurement here
with enough repeats to separate a real effect from run to run noise:

| cycle | result | time |
|---|---|---|
| 1 | complete | 69 s |
| 2 | complete | 66 s |
| 3 | failed | 10 s |
| 4 | failed |  9 s |
| 5 | failed | 25 s |

    2 of 5 complete, mean cycle time 67 s, mean distance 17.5 m,
    mean speed 0.26 m/s, safety cost 4 percent of cycle time

**The completed cycles are tight: 69 s and 66 s, a 3 s spread.** That matters
beyond the headline, because earlier single-run comparisons in this document
were made against a baseline that varied by a factor of five, and several
conclusions were drawn from them more confidently than the evidence allowed. A
3 s spread says the 175 s cycle measured before the MPPI critic rebalance was a
genuinely different regime rather than a bad draw, so that change did what it
appeared to do.

**Every failure is one fault.** Three TF extrapolation errors, three "Unable to
transform goal pose into costmap frame", three failed cycles. The correspondence
is exact. No failed cycle had any other cause.

**And it degrades rather than failing randomly.** Cycles 1 and 2 succeed, then 3,
4 and 5 all fail within 6 to 10 seconds. Something accumulates across the run.
The leading hypothesis is that slam_toolbox's map to odom publication slows as
the pose graph grows, widening the window in which a request for "now" outruns
the newest transform. That is a hypothesis and it is written here as one; the
measurement that would confirm it is logging the map to odom publish interval
across a five cycle run and checking whether it grows.

Raising the consumers' transform tolerance did not help, and the reason is worth
keeping: a tolerance governs how far into the PAST a lookup may reach. This
request is in the future. Setting slam_toolbox's own `transform_timeout` to
publish the transform valid 0.2 s ahead did not close it either, so the gap is
larger than 0.2 s by the time it fails, which is itself consistent with a
widening interval rather than fixed jitter.

The two candidate responses are recorded as ADR 0010, deliberately left Proposed
rather than decided, because the measurement that would choose between them has
not been taken.

---

## V-21. The goal transform failure: two hypotheses refuted, cause still open

The transport task completes 2 of 5 cycles. Every failure is one fault, and it
has survived two explanations, both of which were tested rather than assumed.

**Refuted: slam_toolbox's pose graph slowing its publication.** Measured, 15140
`map -> odom` publications across five cycles hold a flat 20.0 ms mean and
22.9 ms p95 from first window to last, with the single worst gap occurring at
STARTUP and improving afterwards. ADR 0010 rested on this and is Rejected. Both
of its options would have fixed nothing, and one of them was attractive enough
to have been built and believed.

**Refuted: the mission stamping goals with its own clock.** Goals are now
stamped zero, meaning latest available. Failures went from 3 to 6.

**Where it actually fails.** `controller_server`, transforming into the LOCAL
costmap's frame, which is `odom`, hence a `map -> odom` lookup:

    Exception in transformPose
    Requested time 292.980000 but the latest data is at time 292.944000
    looking up transform from frame [map] to frame [odom]

36 ms into the future, consistently, and roughly two publish periods. The local
costmap's `transform_tolerance` is 0.5 s, so the lookup should wait rather than
throw, and it does not.

**The cause, and it was the third thing looked at.** slam_toolbox's `restamp_tf`
defaults to false, which stamps `map -> odom` from the SCAN time rather than
from the clock. Measured over 599 publications, the stamp sat ahead of "now" by
between 40 ms and 28 SECONDS, mean 3.4 s. A 28 second spread on a transform
published every 20 ms is not jitter, and the low end of that range is the same
order as the 36 ms failures.

Setting `restamp_tf: true` closed it:

| | before | after |
|---|---|---|
| goal transform failures | 6 | **0** |
| cycles completed | 2 of 5 | **4 of 5** |

The lesson is not the parameter. It is that the first two explanations were
each coherent, each supported by a real observation, and each wrong, and the
only thing that separated them was measuring the quantity that would have
distinguished them BEFORE building anything. ADR 0010 was written and rejected
without a line of its implementation being started, which is the whole point of
having written it down as Proposed.

---

## V-22. A number that was asserted rather than measured, and what it cost

Throughout this project's navigation work the test warehouse was described as
having 1.35 m aisles, and a chain of reasoning was built on it: that the
MiR250's 1.002 m circumscribed diameter left only 0.348 m of slack, that this
explained the marginal clearance failures, and that a smaller platform would
therefore fix them.

The 1.35 m was never measured. It is `corridor_width_default` from the MiR250
DATASHEET, the corridor width the manufacturer quotes for their vehicle, and it
was repeated as though it described this building.

Measured properly, from the ground truth map at robot height, corridor width
being twice the distance to the nearest obstacle at each free cell:

| percentile | corridor width |
|---|---|
| p5 | 0.20 m |
| p25 | 0.64 m |
| median | 1.34 m |
| p75 | 2.30 m |

| corridor at least | fraction of floor |
|---|---|
| 0.82 m, the MP-400 | 67.8 % |
| 1.00 m, the MiR250 | 63.2 % |

So the building is not the constraint that was claimed, the two platforms differ
by five percentage points of reachable floor rather than by a transformation,
and **the cause of the marginal clearance failures is still unknown**.

Three consequences, recorded so the wrong ones are not carried forward:

    do NOT widen the aisles. The measurement does not support it.
    the MP-400's value is architectural, not clearance. It proves the platform
      abstraction and it has better documented provenance. It is not a fix.
    the clearance failures need their own investigation, which has not started.

This is the same failure as the stale log reads earlier in this document: an
assertion that sounded right, was never checked, and directed real work. The
difference is that this one was caught by someone asking whether the aisles
could simply be widened, which forced the number to be looked at.

---

## V-23. The MP-400 spec cited a source that describes a different sensor

The second platform arrived with five failing tests and a handover note saying
they were mechanical errors in estimated values. Three of the five were
something else, and the difference matters more than the fix.

**The scanner mounting positions were taken from the wrong sensor.** The spec
carried `scanner_mount_x` 0.230 m and `scanner_mount_y` 0.000 m, labelled
`datasheet` against MP-400 manual section 1.3.3, which locates a front scanner
at X 230 mm and a rear one at X -354 mm, both on the centre line. Those figures
are real and correctly transcribed. They are also the positions of **the
MP-400's own laser scanners**, the unrated ones the same file says at length
this project deliberately does not fit, on the grounds that the whole safety
concept rests on a rated device with a published response time.

So the strongest looking provenance in the sensor block, a figure given to the
millimetre in an operating manual, was describing a sensor that is not on the
vehicle. It survived because the label said `datasheet` and the citation
checked out; nobody asked whether the cited row was about the same part.

It was not cosmetic. A centre-line pair and a corner pair are different safety
concepts, and `urdf/sensors.xacro`, the self filter and `generate_fields.py`
are all built for the corner pair, mirroring one mount through four sign
combinations. With `scanner_mount_y` at zero the description places both
scanners on top of each other on the centre line at plus and minus 230 mm,
facing 45 degrees off axis, which is not a machine. The pair is now mounted as
it is on the MiR250 build, optics 5 mm proud of the chamfered corner, and both
values are labelled `estimated`, which is what they are. The manual's LS1 and
LS2 rows are kept in the provenance so the next reader can see they were
considered and why they were not taken.

**Three more values in the same block contradicted the archived SICK sheet**
while labelled `datasheet`: response time 0.05 s against a published 70 ms,
angular resolution 0.10 deg against 0.17 deg, and warning field range 40 m
against 10 m. Only the first of those failed a test, because
`test_scanner_update_rate_matches_response_time` compares it against the update
rate. The other two were internally consistent and would have passed
indefinitely: `scanner_samples` was 2750, which is exactly 275 / 0.10, so the
consistency check confirmed a value derived from a wrong input. The sensor
block is a copy of the MiR250 one, and it was corrupted in the copying.

**The corridor targets were not published at all.** `corridor_width_90_turn`
was 0.850 m against a body diagonal of 0.813 m, which is what
`test_robot_fits_the_corridor_it_claims` caught. The MP-400 manual quotes no
corridor or doorway widths; it is an operating manual and gives dimensions,
ratings and sensor positions, not application envelopes. All four figures were
invented and sat in a block whose header calls it published figures.

They are now derived, and the derivation is stated in the spec: the two
platforms carry the same scanners, the same protective field supplement and the
same safety configuration, so the clearance that configuration demands around
the body is taken as the same and only the body changes.

| target | MiR250, published | clearance implied | MP-400, derived |
|---|---|---|---|
| corridor, default footprint | 1.350 m | +0.770 m on width | 1.329 m |
| corridor, dynamic footprint | 1.000 m | +0.420 m on width | 0.979 m |
| doorway, default footprint | 1.300 m | +0.720 m on width | 1.279 m |
| corridor, 90 degree turn | 0.950 m | -0.038 m on diagonal | 0.775 m |

**Status: these four are estimates of a target, which is weaker than anything
else in this file, and they must not be quoted as manufacturer figures.**
`corridor_width_dynamic` is read by `generate_fields.py` to size a protective
field, so it is load bearing; the generator derives a 1.003 rad/s rotation
threshold from it on this platform. Both it and `self_filter_margin`, carried
across from the MiR250 measurement, need their own measurement on this vehicle
before the MP-400 is run in anger.

The two remaining failures were what the handover described: a provenance line
reading only "not published", and a false positive from the reasoning-marker
heuristic in `test_platform_spec.py`, whose own comment says to widen the marker
list rather than reword the prose. It was widened.

The pattern is the same one as V-22. A number was carried because its source
looked authoritative, and the check that would have caught it, asking what the
cited row is actually about, was never made.

---

## V-24. Generating the Nav2 configuration, and a diagnostic that lied about the safety layer

The second platform could not be run until the navigation configuration was
generated from the platform spec rather than hand-written against the MiR250.
The reason is worth stating precisely, because it is not "the numbers were
wrong". Every number in `nav2.yaml` was right, for a vehicle 200 mm wider than
the one it would have been driving.

`config/nav2.yaml` is now `config/nav2.yaml.in`, a template that keeps every
comment, plus `tools/generate_nav2.py`, which substitutes what depends on the
vehicle and writes `nav2.<platform>.yaml`. A test asserts the committed files
match the generator, comparing TEXT rather than parsed YAML: most of that file
is reasoning, and a parsed comparison would let someone rewrite the reasoning
while the numbers still matched.

### What is derived, and what deliberately is not

A number is derived only if it is a function of the vehicle. Critic weights,
server timeouts, plugin choices and the ranges of sensors both platforms share
are not, and deriving them from the chassis would be numerology wearing
provenance as a disguise. Three commissioning constants are stated once in the
generator instead of being spread through the file: the commissioned speed
fraction, the ordinary-braking cap, and the inflation clearance band.

| parameter | MiR250 | MP-400 | derivation |
|---|---|---|---|
| footprint | 810 x 590 mm | 600 x 569 mm | scanner optical centres |
| inflation radius | 0.5510 m | 0.4634 m | circumscribed radius + 0.05 m |
| vx_max | 1.00 m/s | 0.75 m/s | half the rated top speed |
| ax_min | -1.00 m/s2 | -1.00 m/s2 | min(unladen rating, 2/3 emergency) |
| local costmap | 6 m | 5 m | 2 x speed x MPPI horizon, rounded up |
| voxel layer | 12 x 0.10 m | 10 x 0.10 m | vehicle envelope height |

**The braking cap is not decoration.** The MP-400's unladen acceleration rating
is 2.4 m/s2 and its emergency deceleration is 1.5 m/s2. Carried across
literally, as every other limit in that file was, the controller would brake
harder in ordinary driving than the protective fields assume it can in an
emergency, and every stopping distance behind those fields would be computed
from a rate the vehicle routinely beats. Nothing in the stack would report it.
There is now a test for it by name.

**Regenerating the MiR250 changed two things and nothing else.** The footprint
is identical to four decimal places, and the inflation radius moves from 0.55 m
to 0.5510 m, because the committed 0.55 was a round number with a post-hoc
justification attached: its comment said "the 0.295 m inscribed radius plus a
body's width of margin", and 0.55 minus 0.295 is not a body's width or half of
one. Rather than reverse-engineer a formula to land on the legacy value, the
radius is now the circumscribed radius plus a stated 0.05 m band. One
millimetre, and the deliverable's behaviour is unchanged.

### The collision monitor was never platform-selected either

`robot.launch.py` loaded one `collision_monitor.yaml` whatever platform it was
given. That was safe with one vehicle and is not safe with two, and it is the
one configuration that must never be almost right. Both it and the Nav2
configuration are now selected by name with no fallback file, so an unknown
platform fails the launch loudly instead of quietly driving on another
machine's protective fields. `test_fields.py` now runs over every platform
spec; the MP-400's field set passes all 19 protective-field properties
unchanged.

### The diagnostic that lied

The first MP-400 bringup reported `collision_monitor active` and then failed
preflight sixty seconds later with the monitor inactive, having never been
anything else.

`run_stack.sh` tested for readiness with `ros2 lifecycle get | grep -q active`.
`ros2 lifecycle get` prints `active [3]` or `inactive [2]`, and **`grep active`
matches `inactive`**, so the check passed on the first poll no matter what
state the node was in.

It failed on the node it could least afford to lie about. The retry block
directly below it exists to catch exactly this, an inactive collision monitor,
and it could never fire, because the condition guarding it was satisfied by the
word it was searching for. The script then went on to launch navigation against
a lifecycle service that was still busy, and the manager's configure request
timed out six milliseconds after it was made.

The grep is now anchored. This is the third entry in this file in the same
family as the stale log reads: a check that reported success without ever
testing the thing it claimed to test. It is the most expensive kind, because it
does not merely fail to find a fault, it actively asserts there is none.

### A third place the MiR250 was hardcoded, found only by running it

With the fields and Nav2 both generated, the MP-400 drove a full five-cycle
transport task. Cycle one completed in 79 s over 29.1 m with no protective
stops. The mission log said this:

    unloaded: acceleration limit set to 1.0 m/s2
    loaded: acceleration limit set to 0.3 m/s2

Those are the MiR250's figures. The MP-400's manual publishes a single
acceleration rating of 2.4 m/s2. `transport_task.py` declared both limits as
ROS parameters with the MiR250 numbers as DEFAULTS, under a comment reading
"Both from the platform spec", and nothing ever passed them, so the default was
the value. The comment described an intention rather than the code.

Nothing failed, and that is the point. The vehicle was driven at a fifth of its
own rating, every cycle completed, and every log line reported a figure that
looked like provenance. The only symptom available was a slower number that
looked like a result, which is what makes it worth a regression test rather
than a fix.

`transport.launch.py` now reads both from the spec for the platform the stack
was brought up with, `run_stack.sh` passes the platform through to it, and
`test_transport_limits.py` asserts the values match the spec for every
platform, that laden is never the more permissive of the two, and that an
unknown platform raises rather than defaults.

**Where the ceiling actually is.** On the MP-400 the two limits are equal,
because its manual publishes one rating, so the laden and unladen switching is
a no-op on that platform. The cycle times below are therefore not limited by
acceleration at all but by the commissioned speed of 0.75 m/s, which is half
the platform rating by the same commissioning rule the MiR250 gets. That is a
decision, not a measurement, and it is the first thing to revisit if this
platform's cycle time matters.

---

## V-25. The MP-400 completes 1 of 5 cycles, and the obvious explanation is refuted

**Status: OPEN. No cause established. Do not describe this platform as
working.**

With the footprint, protective fields, speed limits and acceleration limits all
generated from its own spec, the MP-400 was given the standard five-cycle
transport run, `--cameras off --rviz off --cycles 5`, the same invocation the
MiR250's recorded 4 of 5 comes from.

    1 of 5 cycles completed

| cycle | outcome | time | driven | protective stops |
|---|---|---|---|---|
| 1 | complete | 117 s | 36.1 m | 1 |
| 2 | failed to reach dispatch | 154 s | 68.3 m | 32 |
| 3 | failed to reach dispatch | 176 s | 71.8 m | 39 |
| 4 | failed to reach goods_in | 28 s | 0.0 m | 0 |
| 5 | failed to reach goods_in | 28 s | 0.0 m | 0 |

Cycles 4 and 5 never moved at all. The signature is the planner refusing:

    GridBased plugin failed to plan from (-0.67, 3.33) to (-0.83, 2.65):
      "Start occupied"
    GridBased plugin failed to plan from (1.08, 2.32) to (-1.58, -5.45):
      "no valid path found"                                        x28

and every recovery aborting on top of it, `backup failed` and `spin failed`,
both reporting "Collision Ahead". So the vehicle believed itself to be inside an
obstacle and believed every escape from it was also inside one.

### What has been refuted

**It is not that the smaller vehicle drove into a gap it could not get out of.**
That was the obvious reading, it matches a failure this project has had before,
and it is wrong. Measured on the surveyed map, which is the one in the same
frame as the pose the planner reported:

| position | clearance | note |
|---|---|---|
| (-0.67, 3.33), "Start occupied" | 1.200 m | wide open floor |
| (1.08, 2.32), 28 planning failures | 0.400 m | tight, but see below |
| both station goals | 1.65 to 1.75 m | wide open |

A start declared OCCUPIED in 1.200 m of clear floor is not a geometry problem.
The MP-400's inscribed radius is 0.2845 m. Whatever marked that cell, it was not
the width of the aisle.

**It is not the self filter.** That was the second hypothesis, and V-23 had
already flagged `self_filter_margin` as carried over from the MiR250 without
being re-measured, so it was the natural suspect: leaked returns from the
vehicle's own pods would mark its own cell and produce exactly this. The
arithmetic does not support it. The filter is a rectangle of
`half_length + margin` by `half_width + margin`, which on this platform is
0.355 by 0.3395 m, and the pods reach 0.3237 by 0.3082 m. They are inside it,
with 31 mm to spare on the binding axis.

### A measurement error worth recording, because it nearly became a finding

The first pass at the clearance figures above used `warehouse_truth_robot.yaml`
and reported that BOTH station goals sat inside obstacles, one of them with zero
clearance. That would have been a dramatic result and it was nonsense: the
ground truth map and the SLAM map do not share an origin, so mission
coordinates cannot be looked up in the truth map without aligning the frames
first. The truth map is for scoring, and scoring means comparing like with like.

This is the third time in this project that the ground truth map has produced a
confident wrong answer when reached for casually.

### The control run: it is MP-400 specific, and the lead is DISTANCE

A MiR250 control was run on the same machine, minutes later, with the identical
invocation, because the recorded 4 of 5 comes from a different session and no
regression can be attributed without a same-day baseline.

    MiR250 control:  5 of 5 cycles, mean 74 s, mean 19.1 m
    MP-400:          1 of 5 cycles

So the rig is healthy, and it is better than healthy: the control beat the
project's own recorded 4 of 5. The failure belongs to the platform.

The discriminating quantity is not time and it is not the protective stops. It
is DISTANCE DRIVEN between two fixed stations:

| | MiR250 | MP-400 |
|---|---|---|
| cycles completed | 5 of 5 | 1 of 5 |
| distance, completed cycles | 15.2 to 20.5 m, mean 19.1 | 36.1 m |
| distance, failed cycles | none | 68.3 and 71.8 m |

The MP-400 drives roughly twice the distance on a cycle it completes and
roughly 3.6 times on the ones it fails, between the same two station poses on
the same map. Its one completed cycle took 31 s longer than the MiR250's, and
21 m of extra distance at its commissioned 0.75 m/s accounts for 28 s of that.
So it is not creeping, hesitating or being held up by people. It is driving a
much longer route at speed, which means the route it is given is much longer.

That points at path selection rather than at the vehicle being stuck, and the
"Start occupied" refusals are then more likely a consequence of wherever the
long route puts it than the original fault. The three parameters that differ
and could plausibly change route choice are the inflation radius, 0.4634 m
against 0.5510 m, the local costmap window, 5 m against 6 m, and the footprint
itself. NONE of these has been tested by changing one and re-running, which is
the obvious next step and is cheap: one run per variant, about eight minutes
each.

### Hypothesis 1, the inflation radius: TESTED AND REFUTED

The MP-400 was given the MiR250's inflation radius of 0.5510 m in place of its
own 0.4634 m, one variable changed, everything else including the footprint
left alone, and re-run. Confirmed live on the running system before measuring:
footprint 0.3000 by 0.2845, inflation 0.551.

    2 of 3 cycles, distances 73.2, 51.7 and 119.5 m, mean 85.6 m

Distance did not fall towards the MiR250's 19.1 m. It ROSE, against a baseline
of 36.1, 68.3 and 71.8 m. The inflation radius is not the explanation, and the
derivation in generate_nav2.py does not need revisiting on this evidence.

**But note how weak a single run is here, because the variance is the story.**

| | spread of distance per cycle |
|---|---|
| MiR250, 5 cycles | 15.2 to 20.5 m |
| MP-400, 5 cycles | 36.1 to 71.8 m |
| MP-400 with MiR250 inflation, 3 cycles | 51.7 to 119.5 m |

The MiR250 varies by 5.3 m across five cycles of the same journey. The MP-400
varies by 35 m, and one cycle took 390 s with 65 protective stops and 109 s held
up. This is not a platform that is uniformly slower, it is a platform that is
unstable, and n=3 against that spread cannot cleanly eliminate anything. Read
the refutation above as "inflation is not a fix", not as "inflation is
irrelevant".

It also means the existing open item about cycle time variance, 70 to 176 s on
the MiR250, may be the same phenomenon seen faintly on a platform where it does
not yet break anything.

### Hypothesis 2, the scan plane height: TESTED AND REFUTED

`scanner_mount_height` is 0.110 m on the MP-400 against the MiR250's 0.150 m,
and V-23 records that the 0.110 was taken from the manual's Z position for the
MP-400's OWN scanners. Every run here used `--cameras off`, so the merged 2D
scan at that height was the ONLY thing marking obstacles, and a plane 40 mm
lower in a warehouse sees pallet feet and rack footplates that a higher one
passes over. It was the best available candidate.

The spec was set to 0.150 m, everything regenerated from it, and five cycles
run.

    1 of 5 cycles, distances 68.0, 41.6, 39.8, 51.6, 15.6 m

Identical to the baseline's 1 of 5, and the distances did not fall towards the
MiR250's 19.1 m. Refuted. The spec is back at 0.110 m, which at least has a
real anchor behind it.

### Where it actually fails: the DISPATCH leg

Four hypotheses are now dead, and the useful result came from reading what was
already in the logs rather than from another run. Across all three MP-400 runs,
thirteen cycles:

| leg | failures | leg time when it does arrive |
|---|---|---|
| goods_in | 3, and 2 of those after the vehicle was already stuck | 25 to 56 s |
| dispatch | 6 | 45 to 158 s |

    MiR250 control, same two legs: goods_in 24 to 32 s, dispatch 24 to 48 s,
    ten legs, no failures

This is not a vehicle that is globally slower or globally lost. It reaches
`goods_in` at (-1.58, -5.45) reliably and at a time comparable to the MiR250,
and it fails approaching `dispatch` at (-0.83, 2.65). That also places the very
first "Start occupied" refusal, at (-0.67, 3.33), just 0.70 m from the dispatch
station.

It explains why the two parameter experiments changed nothing: inflation radius
and scan plane height are global, and the fault is local to one approach.

**The leading candidate is now the station approach poses**, which were authored
against the MiR250 and carry an approach heading the goal checker enforces to
0.25 rad. A vehicle with a different footprint, a different circumscribed
radius and a smaller inflation radius has to reach the same pose in the same
pocket of floor. That has not been tested. `tools/track_goal.py` on a failing
dispatch leg against the MiR250's successful one is the measurement, and it is
what the existing open item on cycle time variance already recommends.

### CAUSE FOUND for the distance: an asymmetric acceleration envelope

`tools/track_goal.py` on a failing leg returned the answer the tool was built to
give:

    CAME WITHIN 0.02 m at t=92s and then moved away to 0.95 m
      the vehicle reached the goal and did not stop there, so the goal
      checker is not being satisfied
    commanding motion in 188/200 samples, peak 0.75 m/s

The distance to the goal oscillated between 0.09 and 1.22 m with the throttle
pinned at 0.7 m/s. The vehicle was not lost and it was not blocked. It was
arriving and failing to stop, orbiting the station until the leg timed out.

The cause was introduced by generate_nav2.py in this same session. It capped
BRAKING against the emergency reserve and left ACCELERATION at the platform
rating:

| | MiR250 | MP-400 |
|---|---|---|
| ax_max | 1.00 | 2.40 |
| ax_min | -1.00 | -1.00 |

On the MiR250 `min(1.0 rating, 0.667 x 1.5)` is 1.0, which equals its rating, so
the envelope came out symmetric BY COINCIDENCE and the fault was invisible. On
the MP-400 it produced a vehicle that accelerates 2.4 times harder than it can
brake, so every trajectory MPPI sampled towards the goal overshot it.

The reasoning that produced it was that acceleration has no safety coupling.
That is true and it is beside the point: it has a CONTROL coupling. The envelope
is now one figure in both directions, which leaves the MiR250 output unchanged
and gives the MP-400 1.00 against 1.00. `test_the_vehicle_can_brake_as_hard_as
_it_accelerates` asserts it on both the controller and the smoother.

**Measured, it is decisive on the quantity it was meant to fix:**

| | before | after | MiR250 |
|---|---|---|---|
| distance, completed cycle | 70.1 m | 19.9 m | 19.1 m |
| all cycles | 50.8 to 104.7 m | 5.0 to 27.1 m | 15.2 to 20.5 m |

### And it exposed a SECOND fault, which is now the dominant one

Still 1 of 5, but failing in a completely different way.

| | before | after |
|---|---|---|
| protective stops, cycle 1 | 3 | 71 |
| held up by safety | 1 s | 400 s of 484 s |
| commanding motion | 188/199 samples | 93/199 |
| mean speed | | 0.10 m/s |

The vehicle is no longer driving too far. It is barely driving, held by
protective stops for about 80 percent of the run.

**The hypothesis, NOT yet tested**, is the deadlock class that generate_fields.py
documents at length for the MiR250: a vehicle creeping out of rest selects an
all-round rotation field, the field is violated by the racking, it is held
stopped, and so it never reaches a speed that would select a narrower field.
Halving the MP-400's acceleration from 2.4 to 1.0 keeps it in the creep band
longer, which would make it more exposed to exactly that.

If it is that, the root is a number V-23 already flagged. The rotation ladder is
sized against `corridor_width_dynamic`, which on this platform is 0.979 m and is
an ESTIMATE derived from the MiR250's published figures, not a measurement of
this building. Its rot_2 field is 0.489 m half width and rot_3 is 0.602 m,
against a building whose measured median corridor is 1.34 m but whose 25th
percentile is 0.64 m.

The discriminating measurement is the free width actually available along the
route against the half width of the field selected at each moment. Do that
before changing anything.

### THE SAME FAULT AGAIN, reaching the wheels by another route

Fixing the envelope in the generator was not enough, and the reason was only
found by watching a live run rather than reading logs. The mission log said:

    unloaded: acceleration limit set to 2.4 m/s2

while the generated configuration held 1.00. `set_payload` writes `max_accel`
onto the velocity smoother at RUNTIME and does not touch `max_decel`, so the
last rate limiter before the collision monitor was running 2.40 against -1.00.
The generator's fix had only ever reached MPPI's sampler.

The path was introduced earlier in the same session, by the change that made
transport.launch.py read the acceleration limits from the platform spec. That
change fixed a real provenance fault, and it introduced a real behaviour fault,
and the test written alongside it passed: it asserted the mission's value
matched the spec, which it did. It never asked whether the spec value was
PERMITTED. A test can confirm a wrong thing precisely.

`accel_limits` now clamps both figures to the envelope, read from the generated
Nav2 configuration rather than recomputed, so the rule stays in one place.
MiR250 is unchanged at 0.3 and 1.0; the MP-400 goes from 2.4 to 1.0.

**RETRACTED. The measurement that appeared to confirm this was confounded, and
the confound was a layer that was silently absent.**

The run that produced 2 of 5, with 0.5 protective stops per cycle and no time
held up, was the ONLY run of the day whose keepout mask never published. That
was not noticed at the time because the failure announces itself only as a WARN
per costmap update, and because the same run was the one being watched for the
acceleration clamp. Two variables moved and the improvement was attributed to
one of them.

Across every run of the day, checking `Filter mask was not received`:

| run | platform | keepout mask | cycles |
|---|---|---|---|
| 19:17 | MiR250 | present | 5 of 5 |
| 20:01 | MP-400 | present | 1 of 5 |
| 20:17 | MP-400 | present | 1 of 5 |
| 20:42 | MP-400 | ABSENT, 1834 warnings | 2 of 5 |
| 21:19 | MP-400 | present | 0 of 5 |

So the MiR250 baseline of 5 of 5 IS valid, and the only good MP-400 result is
the one taken without the vehicle's no-go zones.

**What is still true.** The mission was setting the smoother to 2.4 m/s2 while
the generated configuration held 1.00, and it never touched `max_decel`. That is
a defect on its own terms and the clamp for it is correct. What is NOT
established is that fixing it improved anything, and the honest reading of the
table above is that it did not.

**The deadlock hypothesis is reinstated, and sharpened.** With the mask present
the MP-400 is close to immobile: 6.0 m in 243 s with 28 protective stops and 169
seconds held up, then 1.6 m in 243 s with 247 seconds held up. It is not
planning badly, it is being stopped. The MiR250 in the same building with the
same mask is unaffected.

**The candidate that fits all of it** is that the inflation radius and the
protective fields are derived INDEPENDENTLY and are not consistent with each
other. generate_nav2.py takes inflation from the circumscribed radius plus a
0.05 m band, giving the MP-400 0.4634 m against the MiR250's 0.5510 m.
generate_fields.py sizes the all-round rotation fields from stopping distance
and the corridor target, giving the MP-400 half widths up to 0.602 m. So the
planner will happily route the smaller vehicle into a gap that its own
protective field cannot fit through, and the monitor then stops it there. The
MiR250 is protected from this by accident, because its larger inflation radius
keeps it out of those gaps in the first place.

If that is right, the rule is that the inflation radius must be at least the
half width of the widest field the vehicle can select at planning speed, and
neither generator currently knows about the other.

**NOT TESTED.** The earlier inflation experiment does not count: it ran with the
acceleration bug present and, as the table above shows, cannot be trusted on the
mask either. Re-run it with the mask confirmed present before believing
anything.

### What still fails, and a defect found while looking at it

Cycles 4 and 5 failed in THREE SECONDS with 0.0 m driven, both from the
identical pose, both `"Start occupied"`. The vehicle ends cycle 3 somewhere it
cannot plan out of and never moves again. That terminal wedge survives both
fixes and is unexplained.

While reading that log, something else:

    KeepoutFilter: Filter mask was not received        x441

`filter_mask_server` configured and never activated, so the keepout mask was
never published and BOTH costmaps ran the whole mission with no keepout zones
at all. The vehicle was planning through floor that was declared permanently
forbidden before it was switched on.

**`tools/preflight.py` reported 17 of 17 checks passed while that was
happening.** It did not check the mask, and its lifecycle list did not include
either filter server, which are precisely the two nodes documented in
navigation.launch.py as the ones that time out under load. Both are now checked
and both servers are in the lifecycle list.

This is the third diagnostic-that-lied in this file, after the stale log reads
and the `grep active` that matched `inactive`. The pattern is identical: a
check that reports success without testing the thing it claims to test.

That hypothesis, that a missing mask routed the vehicle into rack bays where the
scan then walled it in, was TESTED AND REFUTED. With the mask present the
vehicle is worse, not better: 0 of 5, and barely moving.

FIXING THE FILTER SERVERS TOOK TWO ATTEMPTS, and the first was wrong for an
instructive reason. The launch file's own comment attributed the failure to a
service timeout under load, that explanation was taken at face value, and the
navigation manager was staggered on the strength of it. The log says something
simpler:

    023.212  manager: "Configuring filter_mask_server"
    023.370  filter_mask_server: "lifecycle node launched ... Creating"

The manager asks 158 ms before the node exists to answer. Nav2's lifecycle
manager does not wait for the nodes it manages to finish constructing, and the
navigation group is worse, because controller_server builds two costmaps and the
MPPI optimiser first and its manager gave up 20 ms after asking even with an 8
second head start. The manager has no wait-for-node option and no service-call
timeout parameter, both checked against the installed library, so both managers
are now delayed: 12 s for the filters, 30 s for navigation.

The gate worked as intended in between. The failed attempt printed
`KEEPOUT FILTER INACTIVE` and exited, instead of driving five more cycles
without the vehicle's no-go zones.

### What has NOT been tested

Recorded so the next person does not mistake them for conclusions.

    the keepout mask hypothesis immediately above, which is now the leading one.
    the dispatch approach pose, which the leg analysis pointed at and which the
      acceleration fix may or may not have already addressed.
    the local costmap window, 5 m against 6 m, and the footprint itself.
    whether the costmap is contaminated, and by what. "Start occupied" in 1.2 m
      of clear floor means something marked that cell. The candidates are the
      camera voxel layer, the keepout filter, and a localisation jump putting
      the reported pose somewhere the vehicle is not.
    whether the protective stops are cause or consequence. The monitor state
      accounting printed "clear 134%" for one cycle, which is impossible and is
      its own small bug.

Four hypotheses have been tested and refuted here and none of them was the
armchair favourite at the time it was tested. That is the method working, and it
is worth saying plainly that the cause is still unknown.

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
