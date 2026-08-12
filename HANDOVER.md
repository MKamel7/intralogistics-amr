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

## The test suite is currently RED, deliberately

    156 tests, 6 failures

All six failures are `mp400_class`, the half-finished second platform added at
the end of the last session. The MiR250 build, which is the deliverable, is
green: it was 141 tests and 0 failures before the new platform was introduced,
and none of those 141 changed.

The suite went red because adding a platform spec automatically parameterises
the existing tests over it, which is the abstraction working as intended. The
first task below clears it. If you want a green build before touching anything,
`git rm src/amr_description/config/platforms/mp400_class.yaml` restores it, but
finishing the platform is half a day and worth more.

## Start here

    cd ~/intralogistics-amr-fleet
    python3 -m pytest src/amr_description/test/test_platform_spec.py -k mp400 -v

Read every assertion in full before changing anything. Five failures, all in
`estimated` values of `mp400_class.yaml`, all mechanical.

## Running it

    tools/run_stack.sh --cameras off --run mission --cycles 5
    tools/run_stack.sh --run survey            # map the building
    tools/preflight.py                          # 17 health checks, ~15 s
    tools/stop_all.sh                           # always use this, never pkill

Logs go to a timestamped directory under `/tmp/amr-logs/`, with `latest` as a
symlink. Never read a fixed filename: three times in one session a stale log was
read as a fresh result, and once a previous run's numbers were nearly reported
as new.

`--cameras off` is the fleet tier and roughly halves CPU load. The keepout zones
cover the racking, so the cameras are not load-bearing for that case.

## The plan, in order

**1. Finish the second platform. Half a day.**
   Fix the five `mp400_class` test failures, then generate `nav2.yaml` from the
   platform spec the way `generate_fields.py` already generates
   `collision_monitor.yaml`, with a test that fails if the committed file drifts.
   The generation step is the blocker: until it exists the MP-400 gets a correct
   body and MiR250 navigation tuning. Then run it.

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
    /ground_truth/ is measurement only and must never reach the control path
    git authorship is Mohamed Kamel only, no AI attribution anywhere
    no em dashes, en dashes or double hyphens in any written document
