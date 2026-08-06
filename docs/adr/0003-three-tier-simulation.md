# 0003. Split the simulation into three tiers rather than running one full-fidelity scene

Status:   Accepted
Date:     2026-08-06

## Context

The target scenario is several AMRs moving loads through a warehouse shared with walking people,
each robot carrying two safety scanners and two RGB-D cameras. The development machine is a 12th
generation Intel i5-1235U (2 performance cores, 8 efficiency cores) with Iris Xe graphics, 15 GB
RAM and no CUDA. That will not carry the full scene in real time, and pretending otherwise would
surface as a problem weeks into the work rather than at the start.

The world was measured on its own before any robot was added, and the tier split below was
predicted from that. It has since been measured properly, and the prediction held.

MEASURED, by `src/amr_evaluation/tools/benchmark_sim_cost.py`, on an i5-1235U:

| configuration | us/step | marginal RTF | verdict |
|---|---|---|---|
| world only, no robot | 52.8 | 75.7x | scenery is nearly free |
| 1 robot, no sensors | 302.6 | 13.2x | |
| 1 robot, 2 safety scanners | 668.5 | 6.0x | |
| 1 robot, scanners + 2 RGB-D | 1977.6 | 2.0x | **perception tier** |
| 3 robots, scanners only | 2016.4 | 2.0x | **fleet tier** |
| 5 robots, scanners only | 3509.5 | 1.1x | fleet tier ceiling |
| 3 robots, scanners + RGB-D | 6330.7 | 0.6x | **below real time** |

Fixed startup is about 2.2 s per run regardless of configuration.

Three measurement traps were found while producing that table, all of which gave wrong answers
first:

1. **The real-time throttle.** With `<real_time_factor>` left at 1.0 the simulator sleeps to track
   real time and reports about 0.9 at 23 percent CPU no matter how much headroom exists. Set it
   to 0.
2. **Lazy sensor rendering.** Gazebo renders a camera only while its topic has a subscriber. With
   nothing attached, an earlier run reported that adding two RGB-D cameras made the simulation
   slightly *faster*. Subscribers must be attached for the whole measured run.
3. **Startup cost.** Loading the simulator and the warehouse costs about 2.2 s, far more than a few
   thousand physics steps, so a single short run measures process startup rather than simulation.
   An earlier figure of "real-time factor 8.2" for the world alone came from this and understated
   it by roughly nine times; the correct steady-state figure is 75.7x. Take the slope between two
   run lengths so fixed costs cancel.

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

## Validation

The measured table above is what the decision predicted: a fleet tier of 3 robots runs at 2.0x and
5 robots at 1.1x, a perception tier of 1 fully equipped robot runs at 2.0x, and the configuration
this ADR exists to avoid, 3 robots carrying cameras, runs at 0.6x, below real time. Cameras are the
dominant cost by a wide margin: adding two of them to one robot triples the per-step cost.

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
