# 0009. The collision monitor caps speed, and its bands must cover every commandable velocity

Status:   Accepted
Date:     2026-08-12

## Context

`nav2_collision_monitor` sits between the controller and the wheels and is the last thing to touch
a command. Two properties of it were discovered the hard way, and both present as a vehicle that
will not move with no error logged anywhere.

**An uncovered velocity silences it.** If a commanded velocity matches no velocity polygon, the
monitor does not pass the command through and does not stop the vehicle. It publishes nothing at
all, so the wheels receive no command while the controller upstream carries on at 20 Hz. Found by
noticing that `/diff_drive_controller/cmd_vel` had no publisher while `/cmd_vel_raw` had one. The
bands covered forward motion only, so reversing at -0.15 m/s, which the controller does to back out
of a dead end and which the backup recovery does by design, fell straight through the gap. So did
any spot turn above 1.0 rad/s on a platform capable of 1.5.

**A multiplicative slowdown is a trap, not a slowdown.** The warning field scaled the command by
0.3. Downstream of a controller that is acceleration limited and closes its loop on measured
velocity, that has a stable fixed point:

    v = r * (v + a*dt)   ->   v = r*a*dt / (1 - r)

With r = 0.3, a = 0.3 m/s2 and dt = 0.05 s that is 0.0064 m/s. Measured on the running system:
0.0128 m/s commanded, 0.0026 to 0.0064 m/s actual, 0.10 m travelled in 45 seconds. The vehicle was
not being stopped. It was being asymptotically throttled to a standstill by a warning field doing
exactly what it had been configured to do.

## Decision

**The band set must be total.** Every velocity the vehicle can be commanded matches some polygon:
four forward bands, a reverse band sized on the 0.30 m/s reverse limit, and a ladder of rotation
bands at near-zero linear velocity. A test walks the whole commandable velocity space and asserts
no point falls outside every band.

**Rotation is a ladder, not one band.** A single all-round field covering every rotation rate is
sized for the fastest one, 0.766 m to each side, which is wider than the half width of the 1.00 m
dynamic corridor the vehicle claims to work in. The effect was a deadlock rather than a slow robot:
starting from rest the first commands are millimetres per second, that selected the all-round
field, the field was inside the racking, so the vehicle was held stopped and never reached a speed
that would have given it a narrower field. The field now steps with the rotation rate, so a vehicle
barely turning gets a field barely larger than itself.

**The warning field caps speed rather than scaling it.** `action_type: limit`, with a linear cap of
0.30 m/s and an angular cap of the corridor-derived 0.825 rad/s. The controller ramps to the cap
and holds it, which is what "slow down near people" is supposed to mean.

Every one of these is derived from the platform spec and regenerated, never hand-edited, and a test
asserts the committed file matches what the generator produces.

## Consequences

Makes easy: three regression tests that state the failure classes directly rather than by proxy.
One asserts no commandable velocity is uncovered. One asserts that crawling out of rest is not more
constrained, sideways or ahead, than cruising, which is the deadlock class. One asserts the warning
field limits rather than scales.

Reveals a general lesson worth keeping: a safety layer that fails silent is more dangerous than one
that fails loud, and three of the four hardest faults in this project were silent by construction.
That is what `tools/preflight.py` exists for.
