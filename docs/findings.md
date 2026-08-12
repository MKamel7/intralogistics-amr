# Four faults that hid behind coherent explanations

`docs/validation.md` is the full record, and it is long because it is a
laboratory notebook. This is the short version: four faults worth reading about,
chosen because each one had a persuasive wrong explanation attached to it, and
because none of them was an algorithm.

That last point is the one worth arguing about in an interview. Nothing here was
fixed by tuning a planner or picking a better controller. Every one was
configuration, provenance or diagnostics lying across a layer boundary, which is
what actually goes wrong on a real deployment.

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

## What this adds up to

The pattern across all four is the same, and it is not a coding pattern. It is
that **a system can be wrong while every individual file is right**: a correctly
transcribed datasheet row for the wrong part, a correctly derived limit
overwritten by a correct-looking runtime call, a correctly worded check that
matches the wrong string.

The defences that actually worked were not clever. Recording where each number
came from. Writing the deciding measurement down before building the fix. And
refusing to draw a conclusion from a single run against a system with 35 m of
variance — which is the one this project learned last and most expensively.

Two ADRs and one hypothesis in this repository were **rejected after the
deciding measurement was taken and before a line of their implementation was
written**. That is the habit the rest of it is built to support.
