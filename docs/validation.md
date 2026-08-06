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

**Status: not yet measured.** Targets recorded, battery model not built.

13 h at maximum payload, 17.4 h unloaded, 22 h standby, from a 1.63 kWh pack. These constrain the
energy model tightly enough to be a real check on it.
