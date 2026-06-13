# Assumptions and Limitations

## Intentional Simplifications

- The dynamics model is reduced to diagonal inertia, viscous damping, and gravity.
- Coriolis and centrifugal coupling terms are omitted.
- The controller is joint-space and does not solve a full orientation-tracking task.
- The inverse-kinematics stage prioritizes tool-position tracking.
- The current gravity torque model is the intended final v1 model, even though it remains intentionally lightweight.

## Modeling Tradeoffs

- The arm is designed to be spatial rather than planar, but it is still a compact teaching model.
- The tool frame is aligned with `z4` so joint 4 acts as a roll DOF for the tool origin.
- Singularity handling uses damping rather than constraint-based optimization.
- Numerical integration uses a fixed-step semi-implicit Euler method instead of a higher-order solver.

## What Is Not Modeled

- Full rigid-body coupling across joints
- Friction models beyond simple viscous damping
- Compliance, backlash, and actuator electronics
- Real hardware limits beyond basic saturation behavior

## Report Use

This document is the right place to capture what is intentionally abstracted away so the future academic report can distinguish model choices from physical reality.
