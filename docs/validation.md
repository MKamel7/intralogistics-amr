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
