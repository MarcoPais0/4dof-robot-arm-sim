# Assumptions and Limitations

## Intentional Simplifications

- The dynamics model is reduced to diagonal inertia, viscous damping, and gravity.
- Coriolis and centrifugal coupling terms are omitted.
- The controller is joint-space and does not solve a full orientation-tracking task.
- The inverse-kinematics stage prioritizes tool-position tracking plus a fixed down-axis constraint, while leaving roll about the tool axis free.
- Self-collision is handled as a soft capsule repulsion between non-adjacent links rather than a hard constraint.
- The current gravity torque model is the intended final v1 model, even though it remains intentionally lightweight.
- The arm geometry is a generic 7DOF industrial-style chain with KUKA LBR iiwa 14 R820-inspired proportions, zero lateral link offsets, and a short tool extension, not a manufacturer-accurate KUKA replication.
- The visualization shell is Qt Widgets plus pyqtgraph; the 3D card uses a lightweight Qt OpenGL viewport rather than a full graphics engine.
- The validated runtime is Python `3.13`, with Python `3.12` as fallback. Python `3.14` is not treated as a supported baseline for this repo because the GUI/OpenGL package layout has been unreliable there.

## Modeling Tradeoffs

- The arm is designed to be spatial rather than planar, but it is still a compact teaching model.
- The tool frame is aligned with `z7` so joint 7 acts as the roll DOF for the tool origin.
- The dashboard makes the seven joint origins explicit with markers and a `7 DOF` badge, and it now renders the FK chain as cylindrical link segments with a visible tool-point marker plus chip-based joint and link selection rather than a wireframe line strip.
- The refreshed DH table leaves some FK origins coincident in the home pose, so a few nominal spans are zero-length and the thicker cylinders are there to keep the visible spans readable.
- The live charts use rounded 5-tick y-axis ladders that rescale automatically rather than a fully continuous auto-ranging plot.
- Singularity handling uses damping plus a nullspace objective that repels low-`σ_min` states and adds a tiny posture bias.
- The self-collision model uses capsules and a finite-difference repulsion gradient, so it is smooth enough for the demo but not a hard geometric safety guarantee. Pairs that only touch at a shared FK endpoint are ignored to avoid false positives on the zero-offset chain.
- Numerical integration uses a fixed-step semi-implicit Euler method instead of a higher-order solver.

## What Is Not Modeled

- Full rigid-body coupling across joints
- Friction models beyond simple viscous damping
- Compliance, backlash, and actuator electronics
- Collision against external obstacles or the environment
- Real hardware limits beyond basic saturation behavior
- A browser-style or Qt Quick dashboard layer

## Report Use

This document is the right place to capture what is intentionally abstracted away so the future academic report can distinguish model choices from physical reality.
