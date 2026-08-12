# 0007. Keepout zones are commissioning data, authored from the site layout

Status:   Accepted
Date:     2026-08-12

## Context

ADR 0005 added depth cameras so the planner can see the shelf body above the scan plane, and it
works: the vehicle stopped driving under racking. But it works by rediscovering, on every run, a
fact that never changes, and only for as long as the cameras function correctly.

Racking is bolted to the floor. Its position is known before the vehicle is switched on. Making the
vehicle's ability to stay out of it contingent on depth perception is a fragile design: a dirty
lens, a failed camera or a low sun should degrade performance, not remove a hard constraint.

Every real AMR deployment declares these zones. The zone editor is usually the first tool a
commissioning engineer opens after the survey.

The obvious objection is that this project has a strict rule, ADR 0006, that world knowledge must
not reach the navigation stack. The objection is worth taking seriously and it does not apply here,
for a reason worth writing down rather than assuming.

## Decision

Author a keepout mask over the racking and serve it to both costmaps through a
`nav2_costmap_2d::KeepoutFilter`.

A costmap FILTER rather than a layer, deliberately. Layers describe what has been observed. A
filter applies a decision that was made before the vehicle was switched on, and that distinction is
the whole point.

**The provenance rule.** A keepout mask has the status of the building drawing an integrator is
handed on day one. It states where the operator has decided the vehicle may not go. It is not a
measurement of where obstacles are, and it is not used to judge any result:

    ground truth map   what is really there, used to MARK the robot's homework.
                       Derived from every collision mesh. Evaluation only.
    keepout mask       where the operator forbids the vehicle to go.
                       Derived from the racking only, and hand-authored from a
                       floor plan on a real site.

Both are generated from the same world file, which is exactly why the distinction is stated
explicitly at the top of `tools/build_keepout_mask.py` rather than left to be inferred.

**The workflow is the real one**: survey the building, save the map, author zones against that
saved map, then operate on it. The mask is written in the surveyed map's frame, which means
re-surveying invalidates it. That is a genuine commissioning step on a real site too, not an
oversight, and it is flagged in the config.

## Consequences

Makes easy: a hard, deterministic constraint that does not depend on perception. Verified at the
exact spot the vehicle previously wedged, world (-5.21, -0.59), which now reads cost 100 in the
global costmap while the aisles and the start position remain free. 70.8 m2 declared across 7 rack
instances.

Makes easy, later: the same filter infrastructure serves speed-limit zones by changing one `type`
field, which is the natural next use.

Makes hard: the mask is tied to a specific saved map's frame and must be re-authored after a
re-survey.

Does not change: the SLAM map still records the space between shelf legs as free, because that is
genuinely what the scanner saw. The keepout constrains what the vehicle is PERMITTED to use, not
what it observed. Conflating the two would have been the wrong fix, and it is why the map score
barely moved when this was added.
