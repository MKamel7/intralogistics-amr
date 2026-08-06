# 0001. Target ROS 2 Jazzy and Gazebo Harmonic, not Humble and Gazebo Classic

Status:   Accepted
Date:     2026-08-06

## Context

The package was written for ROS 2 Humble and Gazebo Classic 11. The development machine runs
Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Sim 8.11.0 (Harmonic). Neither Humble nor Gazebo Classic
is installed, and all three simulator plugins in the URDF (`libgazebo_ros_diff_drive.so`,
`libgazebo_ros_ray_sensor.so`, `libgazebo_ros_p3d.so`) are Classic-only.

So the repository could not be built or run on the machine it is developed on, and since it is
public, it could not be run by anyone cloning it either. Gazebo Classic reached end of life in
January 2025.

The alternative was to keep Humble and Classic inside a container. That works, and it is cheaper in
the short term.

## Decision

Migrate to ROS 2 Jazzy and Gazebo Harmonic. Replace the Classic plugins with `ros2_control` plus
`gz_ros2_control`, a `gpu_lidar` sensor, and a `ros_gz_bridge` configuration.

## Consequences

Makes easy: development on the actual machine, a stack a reader can reproduce, `ros2_control` so
the same description would drive real hardware, and reuse of this work for the later
LLM-commanded-robot project which also targets Jazzy.

Makes hard: the world and all scenery had to be ported (see ADR 0003), and any future reference to
a Humble-era tutorial needs translating.

Rules out: running the original coursework demos unchanged. The convoy demo is being demoted anyway,
and the coursework state is preserved under a `v0.1-coursework` tag.
