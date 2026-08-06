# 0002. Model a MiR250-class AMR, parameterised, rather than a specific vendor platform

Status:   Accepted
Date:     2026-08-06

## Context

The original robot was a SolidWorks export scaled to a 200 x 160 mm, 2.5 kg chassis with an
invented mass and skid boxes standing in for casters. Nothing about it was traceable to a real
machine, and a portfolio piece aimed at intralogistics needs a robot whose numbers can be defended.

Two candidate references were compared against their public data sheets.

**MiR250** publishes roughly 40 usable constants: envelope 800 x 580 x 300 mm, 83 kg tare plus a
14 kg battery, 250 kg payload, 200 mm drive wheels and 125 mm casters, 25 to 28 mm ground clearance,
2.0 m/s with a 0.3 m/s2 acceleration limit at full payload, operational corridor widths of 1350 mm
default and 1000 mm with a dynamic footprint, 950 mm for a 90 degree turn, minimum detectable object
of 20 mm at 1000 mm and 70 mm at 2500 mm, a 1.63 kWh battery at 47.7 V nominal giving 13 h loaded
and 17.4 h unloaded, and a named sensor set (2 x SICK nanoScan3, 2 x Intel RealSense D435 with a
114 degree horizontal field of view).

**KUKA KMP 600-S diffDrive and KMP 600P** publish about 8: payload, envelope for the 600P only,
speed, acceleration and deceleration, IP54, positioning accuracy, runtime and charge time. No wheel
diameters, no masses, no battery capacity, no corridor widths, no detectable object size, no named
sensors.

KUKA is a target employer, which argued for the KMP.

## Decision

Model a **MiR250-class** AMR: geometry and performance derived from the public MiR250
specification, with our own livery and no MiR branding, documented as a class rather than as a
vendor model.

Generate the description from a platform spec YAML (`config/platforms/`) so a 600 kg KMP-class
variant is a configuration file and an extra row in the results table, not a rewrite.

## Consequences

Makes easy: every constant cites an archived data sheet line, per the project's provenance rule.
Three published figures (1000 mm dynamic-footprint corridor, 950 mm 90 degree turn, 20 mm object at
1000 mm) become claims the planner and scanner model can be **validated against** rather than tuned
to, which is worth more than any additional feature. The two-scanner plus dual-3D-camera plus
differential-with-casters architecture is the market-standard one, so the work transfers to any
intralogistics employer.

Makes hard: the sensor model needs the SICK nanoScan3 data sheet as a second source, because the
MiR sheet names the part without giving its aperture, range, angular resolution or response time.
Anything still not sourced (motor torque curve, friction, inertia tensors) has to be listed in
`docs/platform_spec.md` as derived, estimated or tuned.

Rules out: claiming to have modelled a specific vendor's robot. That is deliberate. Cloning a
target employer's platform is the weaker interview position, because at that employer every gap is
visible to the person best equipped to see it, and elsewhere it is neutral. Alignment with KUKA is
carried by the VDA 5050 fleet interface instead, which also speaks to Jungheinrich, Magazino,
SSI Schaefer and Still.
