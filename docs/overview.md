# Overview

## Purpose

The project models a spatial 7DOF robotic arm and provides a compact simulation stack for kinematic analysis, resolved-rate motion, simplified dynamics, joint-space control, and 3D visualization.

## Scope

- The arm uses standard DH geometry with seven revolute joints.
- The default chain is a compact industrial-style pose inspired by the KUKA
  LBR iiwa 14 R820, with joint angle offsets, zero lateral link offsets, and an
  explicit tool extension.
- The task-space focus is end-effector position control in 3D plus a fixed
  down-axis alignment task for the tool z-axis, with soft self-collision
  repulsion handled in the nullspace.
- End-effector roll about the final wrist axis remains redundant and is left
  free for the singularity-aware and collision-aware nullspace objectives.
- The implementation favors a clear educational model over a full rigid-body dynamics engine.

## Core Interfaces

- `FK`: forward kinematics and frame/point extraction
- `Jacobian`: geometric Jacobian and singularity analysis
- `IK`: inverse differential kinematics for tool-position motion, down-axis alignment, and soft self-collision repulsion
- `Dynamics`: simplified joint-space dynamics
- `Controller`: joint-space PD tracking with gravity compensation and torque saturation

## System Map

```text
DH geometry
  -> forward kinematics
  -> geometric Jacobian
  -> inverse differential kinematics
  -> controller
  -> simplified dynamics
  -> numerical integration
  -> visualization and demos
```

## Units

- Joint angles in radians
- Angular velocities in rad/s
- Lengths in meters
- Linear velocities in m/s
- Torques in N m
