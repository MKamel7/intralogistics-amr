# Handover

Written 2026-08-12. Read this first, then `docs/adr/` and `docs/validation.md`.

## What works

An AMR surveys a warehouse it has never seen, builds a map, then repeatedly
collects a load at one station and delivers it to another while people walk
around it. A safety layer sits after the planner and can override it.

    4 of 5 transport cycles complete, about 70 s for a 17 m cycle
    safety costs 4 percent of cycle time
    SLAM maps 96 percent of the building unattended

This is the deliverable. Everything below is improvement, not repair.

## The test suite is GREEN, and the second platform runs

    130 tests, 0 failures

The five `mp400_class` failures are fixed and the platform is CONFIGURED but
NOT VALIDATED. It brings up, passes all 17 preflight checks, and drives on its
own footprint, fields, speed limits and acceleration. It then completes 1 of 5
transport cycles against the MiR250's 4 of 5, and the cause is open. See V-25,
and do not describe this platform as working.

The suite grew from 77 to 130 collectable tests because `test_fields.py` now
runs over every platform spec rather than the MiR250 alone, plus new
`test_nav2_config.py` and `test_transport_limits.py`.

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
    tools/run_stack.sh --platform mp400_class --cameras off   # the second platform
    tools/run_stack.sh --run survey            # map the building
    tools/preflight.py                          # 17 health checks, ~15 s
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

## The plan, in order

**1. The second platform is CONFIGURED. It is not validated. Half a day.**
   The plumbing is done: the five failures are fixed, `nav2.yaml` is generated
   per platform from `config/nav2.yaml.in` by
   `amr_navigation/tools/generate_nav2.py`, the collision monitor is generated
   and selected per platform too, the transport task takes its acceleration
   limits from the spec, and all three have tests. See V-23 and V-24.

   WHAT IS LEFT IS THE PART THAT MATTERS. The MP-400 completes 1 of 5 transport
   cycles against a same-day MiR250 control of 5 of 5, so the fault is the
   platform's and not the rig's. It drives 2 to 4 times the distance between the
   same two stations and is wildly variable, 36 to 72 m per cycle against the
   MiR250's 15 to 21 m.

   V-25 has the evidence. Two hypotheses are already refuted, including the
   obvious one that it wedges in gaps too small for it, which the clearance
   measurement kills. The leading untested candidate is the SCAN PLANE HEIGHT:
   0.110 m against the MiR250's 0.150 m, and with `--cameras off` that scan is
   the only thing marking obstacles. Test it by putting 0.150 into the spec and
   re-running several cycles. Do not write it up as the cause until you have.

   Two smaller things from the same work:

   The MP-400's `self_filter_margin` and its four corridor targets are carried
   across from the MiR250 rather than measured on this vehicle, and
   `corridor_width_dynamic` sizes a protective field. Measure them before this
   platform is run in anger.

   `mir250_class.yaml` holds a mapping under `platform:` and `mp400_class.yaml`
   holds a bare string. Nothing reads it, which is why no test caught it, and
   the generators take the name from the filename instead. Worth making
   consistent before a third platform.

   The MP-400's commissioned speed is 0.75 m/s, half its rating, by the same
   rule the MiR250 gets. Its acceleration limits are now equal laden and
   unladen because its manual publishes one rating, so the load switching is a
   no-op there. Both are decisions rather than measurements.

**2. Close the credibility gaps. Half a day.**
   `control_latency` is estimated and feeds protective field sizing directly. It
   is the first thing a functional safety reviewer will ask about. Measure it end
   to end: inject a step command, timestamp scan to command to wheel response,
   take the distribution rather than the mean.
   Then switch protective field sets with load state, which the transport task's
   docstring records as an open coupling.

**3. Explain the variance. Half a day.**
   Cycle times run 70 to 176 s on identical journeys, always with a slow
   `goods_in` leg. Use `tools/track_goal.py` on a slow one against a fast one.

**4. Portfolio pass. Half a day.**
   README, a figure or two from the RViz navigation layout, and decide whether
   the repo goes public.

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
