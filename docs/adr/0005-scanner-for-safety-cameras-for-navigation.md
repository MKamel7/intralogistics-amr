# 0005. The scanner is the safety sensor, the cameras are navigation sensors

Status:   Accepted
Date:     2026-08-12

## Context

The vehicle carries two 2D safety scanners at 150 mm and two forward RGB-D cameras. Until now only
the scanners were used, for everything.

That failed, and it failed in a way worth recording because it is invisible from the topic list.
Warehouse racking stands on legs. At 150 mm a rack is a row of thin posts with wide gaps between
them, and those gaps are drivable as far as any 2D consumer of that scan can tell. The planner
believed them. The vehicle drove under the shelving and wedged, with 0.400 m of clearance against
its own 0.501 m inscribed radius, unable to move in any direction while the planner produced a
fresh valid path every second.

Measured on this world: obstacles occupying the band a person walks through cover 108.7 m2, and
obstacles in the band the scanner sees cover 16.4 m2. The scanner is blind to roughly 85 percent of
what should block this vehicle. No amount of controller tuning fixes a costmap that does not
contain the obstacle.

The obvious response, feeding the cameras into everything, is wrong for a different reason. A
protective stop is a safety function. It must rest on a rated device with a known response time and
a defined failure mode, not on a depth camera whose output degrades with a dirty lens, a sunlit
aisle or a reflective surface.

## Decision

Split the two sensors by what they are for, and never cross the streams.

**The 2D scanners are the safety sensor.** They alone feed `nav2_collision_monitor`. The protective
and warning fields are derived from their published response time. Nothing else may become a
`observation_source` for the monitor, and a test asserts this.

**The RGB-D cameras are navigation sensors.** Their point clouds feed a `VoxelLayer` in both
costmaps, marking between 0.10 m and the 1.20 m vehicle envelope height, so the planner sees the
shelf body above the scan plane. They never reach the collision monitor.

A voxel layer rather than an obstacle layer, because a depth camera has to CLEAR in three
dimensions. A 2D obstacle layer raytraces in the ground plane and cannot distinguish "nothing
there" from "nothing there at this height", so marks from an overhang never clear and the costmap
fills with ghosts.

## Consequences

Makes easy: an honest answer to "why did you add 3D perception", supported by the 108.7 against
16.4 m2 measurement rather than by an assertion that 3D is better. The safety argument stays
defensible, because the protective function still rests on one rated device.

Makes hard: CPU. Two depth streams at 640x480 pushed the bridge to 72 percent of a core and starved
the MPPI control loop from 20 Hz down to between 8 and 19 Hz. The depth stream is therefore
rendered at 320x240, which at the 4 m marking range puts samples 24 mm apart against a 50 mm
costmap cell. Sampling four times finer than the grid being written to is cost, not information.

Accepts: the cameras face forward, so a reversing vehicle navigates on the scan plane alone. This
is why reverse is capped at 0.30 m/s and discouraged by a controller critic rather than forbidden.

## Note on a convention that cost a debugging round

Gazebo Harmonic publishes the RGB-D point cloud in the camera's LINK convention, x forward, y left,
z up, not in the REP 103 optical convention. Stamping it as the optical frame makes every consumer
apply a rotation the data has already accounted for. Labelled optical, 41.8 percent of the cloud
landed below the floor, which is impossible, and 17173 points landed inside the vehicle's own
footprint, so the planner answered "Start occupied" while the vehicle sat in an aisle with 3.05 m
of clearance. Labelled as the link frame, both counts are exactly zero. The optical frame is still
defined, because it is correct for image geometry, and a test asserts the sensor does not stamp
with it.
