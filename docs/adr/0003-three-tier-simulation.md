# 0003. Split the simulation into three tiers rather than running one full-fidelity scene

Status:   Accepted
Date:     2026-08-06

## Context

The target scenario is several AMRs moving loads through a warehouse shared with walking people,
each robot carrying two safety scanners and two RGB-D cameras. The development machine is a 12th
generation Intel i5-1235U (2 performance cores, 8 efficiency cores) with Iris Xe graphics, 15 GB
RAM and no CUDA. That will not carry the full scene in real time, and pretending otherwise would
surface as a problem weeks into the work rather than at the start.

The imported world was measured on its own before any robot was added: 25 model instances,
headless, no sensors, **20.0 s of simulated time in 2.44 s wall, real-time factor 8.2, at 38
percent CPU and 216 MB RSS**. The scenery is close to free, so the entire budget belongs to robots
and sensors.

A measurement note that cost time and is worth recording: a run with `<real_time_factor>` left at
1.0 reports RTF 0.9 at 23 percent CPU and measures nothing, because the simulator throttles itself
to real time. Benchmark with it set to 0.

## Decision

Run three tiers, each with a defined purpose, rather than one scene that has to serve every need.

1. **Fleet tier.** Headless, 2D safety scanners only, 3 to 5 robots, pedestrians as simple movers.
   Traffic, reservation, allocation, throughput and deadlock metrics. Runs faster than real time and
   is what CI executes.
2. **Perception tier.** One robot, full sensor suite, small scenario. Detection, tracking,
   prediction, docking pose estimation and localisation accuracy.
3. **Bag tier.** rosbag2 recorded once from the perception tier, then perception developed and
   regression-tested offline against the recording.

`nav2_loopback_sim` sits below all three for navigation, behaviour-tree and fleet logic that needs
no physics at all.

## Consequences

Makes easy: perception iteration decoupled from simulator load entirely, which is the largest
single productivity gain available here. CI can run a meaningful system test on a laptop-class
runner. Each tier can be tuned for its own purpose instead of compromising.

Makes hard: two world configurations and a scenario definition that has to work across tiers, so
the scenario schema has to be designed rather than grown. Any result must state which tier produced
it, and the KPI harness records that in the run manifest.

Rules out: a single screenshot or video that shows everything at once. Demo material will be
composed from more than one tier, and the documentation says so rather than implying one scene did
it all.
