# 0006. Ground truth is an evaluation oracle and never an input

Status:   Accepted
Date:     2026-08-12

## Context

The simulator knows everything: where every object is, where every person is, where the vehicle
really is. That knowledge is enormously useful for saying whether a result is any good, and
enormously corrupting if it leaks into the thing being judged.

The pressure to leak it is real and it is not always obvious. Two examples from this project, one
resisted and one committed and then reverted:

Scenario pedestrians planned their walks against the robot's SLAM map. That coupling looked
conservative and was simply wrong: at startup the map is 88 m2 of which 6.4 percent had the
clearance a walker needs, so all three walkers correctly refused to move and stood still for an
entire run. A person in a warehouse knows the building. Only the robot has to discover it.

Separately, every mapping fault in this project survived because the map was judged by eye. A map
that is the right shape and 30 percent too small looks fine in RViz. A map built by a vehicle that
never moved looks like a perfectly good map of one room. Both happened here.

## Decision

Derive ground truth artefacts from the world, publish them under a `/ground_truth/` namespace, and
treat that namespace as measurement only.

Three artefacts exist:

**The pose oracle**, `/ground_truth/poses`, the true pose of every model. Used to score detection
and tracking, and to check where the vehicle actually ended up.

**The ground truth floorplan**, built by rasterising every collision mesh in the world SDF into an
occupancy grid. Published latched on `/ground_truth/map`. Two are generated, because the height
band that blocks is not a property of the building but of who is moving through it: a person band
of 0.06 to 1.90 m and a vehicle band of 0.06 to 0.35 m. Scoring a scan-plane map against a
person-height map would count the vehicle's correct observations as errors.

**The scorer**, `tools/score_map.py`, which reports coverage, precision, recall and IoU of the SLAM
map against the floorplan, searching the alignment offset rather than assuming it, because a map
that is accurate and displaced is a localisation error and a map that is accurate and distorted is
a mapping error, and they deserve different answers.

The rule: nothing in the navigation, perception or safety stack subscribes to `/ground_truth/`, and
scenario figures may use the floorplan because a human being genuinely does know the building.

## Consequences

Makes easy: mapping quality becomes a number. The current map scores 78.2 percent coverage, 87.0
percent precision and 70.1 percent IoU, and reports 3.26 percent of what it calls free as actually
obstacle, which is the dangerous direction and is called out on its own line.

Makes hard: nothing may be quoted as a perception result if it was computed with oracle input. The
discipline has to be enforced by review, since a subscription is one line.

Watch for: the keepout mask in ADR 0007 is derived from the same world file and is NOT covered by
this rule, because it is declared site knowledge rather than measured truth. The two are easy to
confuse and the distinction is the subject of that ADR.
