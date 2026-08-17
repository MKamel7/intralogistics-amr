# Faults that hid behind coherent explanations

`docs/validation.md` is the full record, 66 numbered findings and 4900 lines,
and it is long because it is a laboratory notebook. This is the short version.

Every fault below had a persuasive wrong explanation attached to it, and none of
them was an algorithm. That is the part worth arguing about in an interview:
nothing here was fixed by tuning a planner or picking a better controller. They
were configuration, provenance, and diagnostics lying across a layer boundary,
which is what actually goes wrong on a deployment.

## If you read nothing else

| | the number | the mistake |
|---|---|---|
| Aisle width | median 1.34 m, p25 **0.64 m** | a datasheet corridor figure repeated until it sounded like a measurement of the building |
| Scanner placement | correctly cited, **wrong sensor** | a provenance label is a claim about a PART, not about a document |
| Self filter geometry | 33.1 mm to **55.1 mm** of field coverage, no field resized | called a hardware limit for three findings; the filter was a bounding box and the vehicle is not a box |
| Test suite | `colcon test` ran 289 cases, `pytest` ran 337 | **eleven test files** were never run by the build, and all of them passed |
| Latency, RETRACTED | p99 1260 ms became **p95 124 ms** | percentiles from 43 samples across runs of 6 to 17, with the probe's own "samples is thin" warning printed in the same output |
| Goal tolerance | set to 200 mm, parks **212 mm** out | a tolerance is a stopping condition, not an accuracy specification |
| Human aware layer, RETRACTED | shipped disabled, then **7.33 % to 5.00 %** | "the effect is smaller than the noise" is a claim about the experiment, not the effect |
| Station orientation | **every goal ever sent used yaw 0** | the generator wrote `yaw`, the mission read `approach_yaw` with a default, so a guess replaced a computed value silently |
| Precision docking | built, measured, **not shipped**: 84 % false positives | "it steers nothing so there is no reason to gate it" was true in every clause and wrong in its conclusion |

**Five of the faults were in the measuring instruments rather than in the
robot.** That ratio is uncomfortable and it is the honest headline.

**One feature was built, measured and not shipped.** Precision docking works on
synthetic geometry to under a millimetre and reports 84 percent false positives
in a real building, so the node is kept, unit tested, and not launched. Building
it found two defects that had nothing to do with docking.

---

## 1. An aisle width nobody had measured

For most of this project's life the test warehouse was described as having
1.35 m aisles. A chain of reasoning was built on it: the MiR250's 1.002 m
circumscribed diameter left only 0.348 m of slack, that explained the marginal
clearance failures, and a smaller platform would therefore fix them.

The 1.35 m was `corridor_width_default` from the MiR250 **datasheet**, the
corridor width the manufacturer quotes for their vehicle, repeated until it
sounded like a measurement of the building.

Measured properly from the ground truth map at robot height:

| percentile | corridor width |
|---|---|
| p5 | 0.20 m |
| p25 | 0.64 m |
| median | 1.34 m |
| p75 | 2.30 m |

The median is 1.34 m, so the number was nearly right and completely unfounded.
The p25 of 0.64 m is the interesting one: a quarter of that building is
narrower than the robot, which no amount of navigation tuning addresses.

**What it cost:** a second platform was adopted partly to fix clearance failures
it does not fix. It reaches 67.8 % of the floor against the MiR250's 63.2 %.
Five percentage points, not a transformation.

**What caught it:** someone asking whether the aisles could simply be widened,
which forced the number to be looked at.

Recorded as V-22.

---

## 2. Provenance that cited the wrong sensor

Every physical constant in this project carries a recorded source, and a test
fails the build when one loses it. That gate has been in place since early on.
It did not catch this.

The MP-400 platform spec placed its safety scanners using figures from the
manufacturer's operating manual, section 1.3.3, quoted to the millimetre:

```
scanner_mount_x: 0.230   {kind: datasheet, src: "MP-400 manual 1.3.3, LS1 X-pos 230 mm"}
scanner_mount_y: 0.000   {kind: datasheet, src: "MP-400 manual 1.3.3, LS1 and LS2 Y-pos 0"}
```

Correctly transcribed, correctly cited, and describing **a different sensor than
the one fitted**. Section 1.3.3 locates the MP-400's own laser scanners: the
unrated ones the same file explains at length this project deliberately does not
use, because the safety concept rests on a rated device with a published
response time.

It was not cosmetic. A centre-line scanner pair and a corner pair are different
safety concepts. The robot description, the self filter and the field generator
are all built for a corner pair, so those figures placed both scanners on the
centre line facing 45 degrees off axis, which is not a machine.

Three more values in the same block contradicted the archived SICK datasheet
while labelled `datasheet`. Only one of them failed a test, because
`scanner_samples` was 2750, which is exactly 275° / 0.10°, so the
consistency check confirmed a value derived from a wrong input.

**The lesson that generalises:** a provenance label is a claim about a **part**,
not just about a document. The citation checked out. Nobody asked whether the
cited row described the component actually fitted.

Recorded as V-23.

---

## 3. A limit fixed in one place and silently re-widened in another

This one was mine, introduced during the same session that found it.

Making the Nav2 configuration generated per platform meant deriving the
acceleration envelope from the vehicle spec. Braking was capped against the
emergency deceleration, so the emergency rate stays a genuine reserve.
Acceleration was left at the platform rating, on the reasoning that acceleration
has no safety coupling.

True, and beside the point. It has a **control** coupling:

| | MiR250 | MP-400 |
|---|---|---|
| `ax_max` | 1.00 | 2.40 |
| `ax_min` | -1.00 | -1.00 |

On the MiR250 the two come out equal by coincidence, so the fault was invisible.
On the MP-400 it produced a vehicle that accelerates 2.4 times harder than it
can brake, and every trajectory the controller sampled toward a goal overshot
it. `tools/track_goal.py` named it in one line:

```
CAME WITHIN 0.02 m at t=92s and then moved away to 0.95 m
  the vehicle reached the goal and did not stop there
commanding motion in 188/200 samples, peak 0.75 m/s
```

It was arriving and failing to stop, orbiting the station until the leg timed
out. Fixing the generator dropped distance per cycle from 70.1 m to 19.9 m
against a MiR250 baseline of 19.1 m.

**And it still was not fixed.** The mission layer writes the smoother's
acceleration limit at runtime and never touches the braking limit, so the last
rate limiter before the wheels was running 2.40 against -1.00 regardless. That
path had been created in the same edit, hours earlier, while fixing a genuine
provenance fault, and the test written alongside it passed, because it asserted
the mission's value matched the spec. It never asked whether the spec value was
*permitted*.

**The lesson:** any value with two authors will eventually have two values. And
a test can confirm a wrong thing precisely.

Recorded as V-24 and V-25.

---

## 4. Three diagnostics that reported success without testing anything

The most expensive class, because they do not merely fail to find a fault, they
actively assert there is none.

**A stale log read as a fresh result.** Three times in one session, and once a
previous run's numbers were nearly reported as new. Every run now gets a
timestamped directory and a `latest` symlink, so a stale read is impossible
rather than unlikely.

**`grep active` matching `inactive`.** `ros2 lifecycle get` prints `active [3]`
or `inactive [2]`, and the readiness check was unanchored. It failed on the node
it could least afford to lie about: the retry block that exists to catch an
inactive collision monitor could never fire, because the condition guarding it
was satisfied by the word it was searching for. Measured on one bringup, the
script reported the monitor active sixty seconds before preflight found it
inactive, having never been anything else.

**A preflight passing while a safety layer was absent.** `filter_mask_server`
configured and never activated, so the keepout mask never published and both
costmaps ran an entire five-cycle mission with **no keepout zones**. The only
signal was a warning per costmap update, 441 of them, and the preflight reported
17 of 17 checks passing throughout. It did not check the mask, and its lifecycle
list omitted both filter servers, which are the exact two nodes the launch file
documents as timing out under load.

That last one also invalidated a measurement. The run that appeared to confirm
the acceleration fix was the only one of the day missing its keepout layer: two
variables moved, and the improvement was credited to the one being watched. The
conclusion was retracted the same evening.

**The rule adopted since:** every diagnostic gets a test that proves it can
fail. A check nobody has seen fail is not a check.

---

## 5. A safety defect called a hardware limit for three findings

The scan merger deletes returns inside the vehicle, so it does not see its own
body. The forward protective fields sit inside the region it deletes, which
means they cover ground the sensor is not allowed to report on. Five hypotheses
were refuted on it by measurement over three sessions. The sixth was accepted
without any:

> Closing the rest needs a smaller pod or a different mounting, which is a
> hardware change and not something a configuration file can assert its way
> out of.

Every attempt had asked how LARGE the blind margin should be. None asked what
SHAPE the filter was. It is a bounding box, and the scanner pods stand proud
only at two diagonal corners, occupying 132 mm of a 590 mm side:

| | margin 0.060 | 0.032 | shaped |
|---|---|---|---|
| forward field lateral coverage | 5.1 mm | 33.1 mm | **55.1 mm** |
| fields resized | no | no | **no** |

The regenerated field configuration is byte identical to the committed one, so
the whole gain came from the blind zone shrinking underneath fields that did not
move. Two earlier attempts had tried to buy the same coverage by enlarging the
FIELDS and both measured worse than the defect: one trapped the vehicle against
a rack with 1057 commands in and 0 out, the other dropped the second platform
from 3 of 3 cycles to 2 of 9.

**What caught it:** being asked whether any of the open findings were fixable,
which forced the filter's implementation to be read rather than its parameter.

Recorded as V-49, correcting V-46.

---

## 6. Eleven test files the build never ran

`colcon test` reported 289 pytest cases. A direct `pytest src` reported 337.
Nobody had put the two numbers side by side.

The gap was eleven files that exist in `test/` and were never registered with
`ament_add_pytest_test`: every probe that produces a safety number, and the test
asserting the survey exits non-zero when a station is off the map.

**All eleven passed.** That is why it survived the whole project. A failing
unregistered test would have been caught the first time somebody ran pytest by
hand; a passing one is invisible from both directions.

A twelfth failed the other way. One package was the only `ament_python` one, so
`colcon test` ran pytest where it collected nothing, exited 5, and reported the
package FAILED on every build while its nineteen tests passed under a direct
run. A red package nobody could explain and a green suite missing a seventh of
itself, at the same time, for the same reason.

The fix that generalises: **walk the tree instead of naming the files.** The
replacement test discovers every test file in the source tree, so a new one is
covered the day it is written.

Recorded as V-50.

---

## 7. The instruments were wrong more often than the robot

Four faults were found in the stack over three days. Five were found in the
things measuring it.

**The wrong clock.** Six probes read wall time in a world running on simulated
time. Five used it only for their own run length, which is self consistent
whichever clock it is, so no published figure moved. The sixth subtracted a
clock reading from a message stamp and printed the epoch as a duration.

**The wrong pairing, and this one had published numbers on it.** A latency
sample was armed on a protective stop and closed when the vehicle stopped
moving. It was armed even when the vehicle was already stationary, so nothing
closed it until some later unrelated stop and the interval spanned two events
that were never connected.

| | n | p50 | p95 | max | sd |
|---|---|---|---|---|---|
| unguarded | 302 | 88 | 128 | **980** | 56 |
| guarded | 397 | 84 | 124 | **144** | 24 |

Twenty decisions per run were armed while stationary. Those twenty were the
tail. The stamps had already said so and nobody had looked: at the 872 ms
sample the command stream was publishing every 52 ms throughout and no arrival
gap exceeded 72 ms, which makes a genuine 872 ms interval impossible.

**The wrong statistic.** A social navigation probe argued in its own docstring
that "the denominator matters more than the count" and divided every figure in
its table by exposure, then printed the closest approach and the median of per
person minima as its SUMMARY. Those are extrema and can only grow as a run gets
longer, so a run that drove further looked worse at identical behaviour.

**The wrong duration.** A prediction that an unsecured 100 kg load would creep
11.5 mm per hard stop measured 0.0 mm. The friction arithmetic was right; the
assumption that the deceleration exceeded the friction limit for the whole
190 ms stop was not. A protective stop is a spike and a tail, the excess lasts
single 4 ms physics steps, and slip goes with the square of that time.

**The wrong reference frame.** A cargo box was placed at the vehicle's map frame
pose and created at those coordinates in the WORLD frame, so it materialised
outside the building and fell to the floor while the vehicle drove a full cycle
carrying nothing. The mission log said "placed payload_0 on the plate".

The common shape: **in each case the measured part was correct and the assumed
part carried the error**, which is exactly why the results looked plausible.

Recorded as V-52, V-56, V-61 and V-64.

---

## 8. What was retracted, and why that matters more than what was found

Two of this project's headline claims were withdrawn after being measured
properly.

**"The latency estimate is refuted."** It was the first of three things the
handover said were most worth knowing. The measurement behind it took p95
796 ms and p99 1260 ms from 43 samples pooled across five runs of 17, 6, 7, 7
and 6, and the probe printed `7 samples is thin. Prefer 20 or more before
changing a spec.` in the same output. Guarded and re-measured over 397 samples
the p95 is 124 ms against an estimate of 100. At the commissioned speed the
estimate is short by 18 mm, not 522.

**"The human aware costmap layer cannot be shown to help."** It shipped
disabled on the conclusion that its effect was smaller than the spread of the
metric. Three separate design faults each produced a plausible null: the metric
was an extremum, cycle completion was confounded with the arm, and task type
dominated both. Held to one task it reduces time in a person's intimate space
from 7.33 % to 5.00 %, twelve times the reproducibility of the metric, for six
percent of survey duration and no measured loss of mobility.

Both retractions are written into `docs/validation.md` beside the original
claim, with the wrong sentence left standing rather than edited away, because
the mistake is the useful part.

---

## 9. A field written, committed, regenerated, and never read

Found while building precision docking, which is the first feature in this
project that needs the parked ORIENTATION to be right rather than only the
position.

The generator writes `yaw` for every station. The mission read
`station.get('approach_yaw', 0.0)`. Two different keys, so **every navigation
goal this project ever sent used a yaw of 0**, and the computed orientation was
written, committed, regenerated per platform and never once consumed.

**The default is the whole defect.** A subscript would have raised `KeyError`
on the first mission ever run. `.get(key, 0.0)` substituted a plausible number,
the vehicle drove to a plausible pose, and nothing looked wrong for the entire
life of the project.

It was hiding two things. The 180 degree spot turn that V-63 measured at
`goods_in` existed only because the effective goal yaw was always east; with
the key fixed the parked heading error there went from +143.9 degrees to +9.8.
And it made the new dock invisible, because a vehicle parked facing east has an
aisle-end dock behind it and the detector searches forward. The first docking
run reported `0 found, 140 not found` for its whole length. The detector was
right and the goal was wrong.

**What caught it:** a feature that could not work unless the value was correct.
Not a test, not a review. Every earlier feature tolerated the wrong answer.

---

## What this adds up to

The pattern is the same throughout, and it is not a coding pattern. It is
that **a system can be wrong while every individual file is right**: a correctly
transcribed datasheet row for the wrong part, a correctly derived limit
overwritten by a correct-looking runtime call, a correctly worded check that
matches the wrong string.

The defences that actually worked were not clever. Recording where each number
came from. Writing the deciding measurement down before building the fix. And
refusing to draw a conclusion from a single run against a system with 35 m of
variance, which is the one this project learned last and most expensively.

Two ADRs and one hypothesis in this repository were **rejected after the
deciding measurement was taken and before a line of their implementation was
written**. Two headline claims were **retracted** after being measured properly.
That is the habit the rest of it is built to support.

The defence that worked most often was not a technique. It was writing the
decision rule down before the data arrived, and then honouring it when the
number came back inconvenient.
