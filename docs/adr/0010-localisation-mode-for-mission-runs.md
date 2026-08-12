# 0010. How to stop the goal transform race: bound SLAM, or localise on a saved map

Status:   Rejected. The hypothesis it rests on was measured and is false.
Date:     2026-08-12

## Context

The transport task completes a 17.5 m pick and deliver cycle in 67 s, measured
over five consecutive cycles with a 3 s spread on the ones that complete. Two of
those five completed. All three failures had exactly one cause, and the
correspondence was exact: three TF extrapolation errors, three "Unable to
transform goal pose into costmap frame", three failed cycles. See V-20.

The controller asks to transform the goal at "now", slam_toolbox has published
`map -> odom` a few milliseconds earlier, and tf2 refuses a lookup in the
future. The controller then aborts the entire goal rather than waiting a cycle.

Two things are already known and rule out the easy answers:

    Raising the consumers' transform tolerance does nothing, because a tolerance
    governs how far into the PAST a lookup may reach, not the future.

    Setting slam_toolbox's own `transform_timeout` to 0.2 s did not close it
    either, so by the time it fails the gap exceeds 200 ms.

The failure also DEGRADES. Cycles 1 and 2 succeed, then 3, 4 and 5 fail within 6
to 10 seconds each. Something accumulates over a run. The leading hypothesis is
that slam_toolbox's publication slows as its pose graph grows, which fits both
the direction and the timing, but it has not been measured.

## Options

**A. Bound the SLAM cost.** Keep running SLAM during missions and stop its
optimisation from growing without limit: a shorter loop closure search window, a
larger `minimum_time_interval`, a cap on the pose graph.

Keeps one mode instead of two, so nothing new has to be commissioned, and the
vehicle keeps improving its map while it works, which matters in a warehouse
whose contents move. Against it: it treats a symptom whose mechanism is assumed
rather than proven, the bound would need retuning for a larger building, and a
vehicle that is still mapping is still capable of surprising its own planner
with a loop closure mid journey.

**B. Localise on the saved map, with AMCL.** Survey once, save, then run missions
against the saved map. AMCL publishes `map -> odom` at a steady rate that does
not depend on how long the vehicle has been running, because there is no pose
graph to grow.

This is the production arrangement for essentially every deployed AMR: you
survey a site once at commissioning and localise thereafter. The AMCL
configuration is already written and has never been exercised, the keepout mask
in ADR 0007 is already authored against the saved map's frame, and the survey
already saves one. It also removes the growth mechanism entirely rather than
bounding it. Against it: it is a second mode to commission and keep working, the
map goes stale as the warehouse changes, and it puts weight on a localisation
path this project has never run.

## Decision

NOT YET TAKEN, and deliberately so. The measurement that separates these has not
been made: log the `map -> odom` publish interval across a five cycle run and
see whether it grows.

    if it grows          the mechanism is confirmed, and B removes it while A
                         only bounds it. B, with A's parameters as a fallback
                         for sites where a live map is genuinely needed.

    if it does not grow  the hypothesis is wrong, both options are treating the
                         wrong thing, and the degradation is something else that
                         accumulates over a run. Neither option should be taken
                         on the strength of a story that the data contradicts.

B is the more likely outcome and is worth building regardless, because
localisation on a surveyed map is a capability this project claims and does not
yet demonstrate. That is an argument for building it, not for assuming it fixes
this.

## Consequences

Recording this as Proposed rather than Accepted is the point. The temptation was
to take option B immediately, since it is the better long term architecture and
the pieces are already in place. Doing so would have meant fixing a bug by
building a feature and never learning whether the bug was what I thought it was,
which is how three separate faults in this project stayed hidden behind
plausible explanations.


## Outcome, 2026-08-12

**Rejected, and this is the value of having left it Proposed.**

The discriminating measurement was taken: 15140 `map -> odom` publications across
a five cycle run.

    window        n      mean    median       p95     worst
    0-  60s    2961    19.9ms    20.0ms    22.7ms   336.7ms
    60- 120s    3000    20.0ms    20.0ms    22.7ms    30.5ms
    240- 300s    3000    20.0ms    20.0ms    22.9ms    32.2ms

The interval does not grow. It is flat at 20.0 ms from start to finish, and the
single worst gap occurs at STARTUP and improves thereafter. slam_toolbox's
publication is healthy and its pose graph is not the mechanism.

Both options in this ADR would therefore have fixed nothing. Option B in
particular was attractive enough to build on its own merits, and building it
would have appeared to work, because the real cause is intermittent and a fresh
run completes its first cycles either way.

The actual cause was in this project's own mission node, which stamped each
NavigateToPose goal with its own clock. Under simulated time, nodes receive
/clock a few milliseconds apart, so that stamp can sit marginally ahead of what
the controller's TF buffer has seen. A lookup in the future fails regardless of
tolerance, because a tolerance reaches into the past. A goal pose in the map
frame is not time varying, so it is now stamped zero, meaning "latest available".

**Localising on a saved map remains worth building**, for the reasons in option B
that have nothing to do with this bug: it is what a deployed AMR does, the AMCL
configuration and the saved map already exist, and the capability is currently
claimed but not demonstrated. It should be raised as its own ADR, argued on its
own merits, and not smuggled in as a bug fix.
