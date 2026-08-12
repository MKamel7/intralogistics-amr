# 0008. Navigation margin belongs to the mission layer, not to the planner

Status:   Accepted
Date:     2026-08-12

## Context

Nav2 treats a cell as blocked once it is within the vehicle's inscribed radius of an obstacle,
0.501 m for this 810 by 590 mm platform. `footprint_padding` grows that radius.

Both settings of it were tried on the running system and both failed, in opposite directions.

At 0.02 m the planner will happily thread a gap that clears the vehicle by 9 mm. Nine millimetres
survives neither map noise nor localisation drift, and the vehicle duly followed such a path into a
pocket with 0.450 m of true clearance, below its own inscribed radius, where every candidate
trajectory is in collision and it cannot move at all.

At 0.08 m the effective inscribed radius becomes 0.613 m, and this warehouse contains real,
drivable positions with 0.550 m of clearance. The vehicle reached one of them perfectly well and
the planner then declared its own position blocked, which is a worse failure than the one the
change was meant to cure.

The diagnosis is that padding was being asked to do something it cannot express. It is a hard
constraint. It can say "the vehicle does not fit here". It cannot say "prefer roomy routes".

## Decision

Keep the planner's margin modest and put the preference in the layer that chooses where to go.

**Planner**: `footprint_padding: 0.03`, an effective inscribed radius of 0.543 m. Enough that a
route is not threaded with millimetres to spare, small enough that legitimate floor stays legal.

**Mission layer**: the survey targets only floor with 0.70 m of clearance, and its reachability
search traverses only such floor. That leaves the planner 0.157 m of margin on every journey it is
asked to make, while still allowing it to use the whole building if it has to.

The asymmetry is the design, not an inconsistency.

A related decision belongs here. `allow_unknown` was also tried both ways. Allowing the planner
into unsurveyed floor lets it route into gaps it knows nothing about, which is how the vehicle got
under the racking in the first place. Forbidding it appeared to fail worse, refusing every goal,
but that turned out to be a fault in the survey rather than in the setting: its reachability search
walked free cells rather than cells the vehicle fits in, so it kept offering goals reachable only
through gaps too narrow to use. With that corrected, `allow_unknown: false` is right. A vehicle
sharing a floor with people should not drive into a space on the assumption that it is empty.

## Consequences

Makes easy: the survey completes. From a standing start it maps 88 m2 to 218.9 m2 and terminates on
its own criterion, having traversed the whole building without wedging.

Makes explicit: two layers now have to agree about clearance, and if the survey's figure is lowered
below the planner's effective inscribed radius the old failure returns. Both numbers are commented
with the other's value for that reason.

Reveals: reachability must mean the same thing to whoever picks the goal and whoever plans the
route. That was the actual bug behind several days of apparent controller problems.
