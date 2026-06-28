# Demos

## Purpose

The demos show whether the 7DOF kinematic model, inverse kinematics, controller, and dynamics work together as intended.
Run them from the repository venv with `./.venv/bin/python src/robot_arm_3d_demo.py --mode cartesian` for the main demo or `--mode joint` for the validation demo.
Use the repository venv created for the project. The startup check in [`environment.md`](environment.md) should pass before launching either demo.
Startup is validated before the window opens, so missing Qt packages, a missing display backend, or missing OpenGL support fail fast with a direct message.

## Cartesian Tracking Demo

This is the main demo for the project. It exercises the full task-space pipeline.
It opens as a dark, minimal desktop dashboard sized to the available display,
with the simulation as the dominant card on the right and a stack of separate
metric cards on the left, separated by visible gutters. The visual theme keeps a
graphite background, darker card panels, and high-contrast blue, red, green, and
orange accents. The simulation card now renders the FK chain as cylindrical link
segments with visible joint-origin markers, the tool point, a `7 DOF` badge, a
joint-origin chip row, and a second chip row for the rendered link segments.
Each metric card shows the chart title in its header strip, a fixed hover info
badge immediately before the title, and a live value beside it. Tooltips open on
click or after a short hover delay, use a black background with white text, and
show a technical line plus a plain-language line. Titles stay on one line.
Clicking a joint chip toggles the dashboard into that joint's view, and
clicking the same chip again returns to the original per-demo graphs without
clearing the buffered history. The left metric column takes roughly 40% of the
window width.
The `J1` through `J7` chips in the simulation header show each joint's DH
parameters on hover, and clicking one of them switches all four charts to that
joint's angle, velocity, torque, and power. The `B-J1` through `J7-tool` link
chips highlight the corresponding segment in the 3D view without changing the
metric cards, and their hover tooltips spell out the segment endpoints.
Use the normal OS title-bar controls to close it, or press `Esc` or `q` as a
keyboard fallback.
Resolved-rate IK now solves a 5D task: tool position plus tool z-axis down
alignment, with soft self-collision repulsion and singularity-aware redundancy
left over for the nullspace objective. The joint-space controller plus
simplified dynamics track that reference while the target sequence advances.

The arm now starts from a bent industrial-style home pose with KUKA LBR iiwa 14
R820-inspired link proportions, joint angle offsets, zero lateral link offsets,
and a visible tool extension. A few FK origins coincide in the home pose, so
the demo uses thicker cylinders and explicit selection chips to keep the
compact meter-scale workspace easy to read.

### Runtime Loop

```text
x_d, z_tool, capsules -> x, z_tool + self-collision repulsion -> 5D task -> q_dot_cmd -> q_ref -> controller -> tau -> dynamics -> q
```

### What It Shows

- sequential target tracking in 3D on the 7-joint arm
- 15 randomly scattered targets per run
- active target highlighting
- end-effector trace through space
- a visible tool-tip marker and the full arm path from base through the tool
- soft self-collision repulsion between non-adjacent link capsules
- joint-chip selection that changes the metric cards
- link-chip selection that highlights the chosen segment in the 3D view
- default metric plots for:
  - Cartesian position error norm
  - desired end-effector speed norm
  - all-joint speed norm
  - minimum singular value of the 5D task Jacobian
- clicking a joint chip switches the same four cards to that joint's angle, velocity, torque, and power
- clicking a link chip highlights the chosen segment without changing the charts
- each metric plot uses a rounded 5-tick y-axis ladder that resizes automatically
- the plots show grid lines only at the labeled major tick values
- the time axis uses 2-second ticks
- metric histories keep the last 15 seconds
- metric axes follow the visible range of the active view
- the active target advances once both the tool-point error and tool-axis misalignment fall below tolerance, or the fallback step limit is hit
- the active target is highlighted by a blinking red marker while the blue target cloud omits that point until selection advances
- joint-specific charts appear immediately from buffered history instead of restarting from empty
- clicking the active joint chip again restores the original Cartesian graphs without clearing history
- the native window close button ends the demo cleanly, with `Esc` and `q` as fallbacks
- tool-axis misalignment is tracked internally for validation, but it does not get its own dashboard card
- internal self-collision telemetry tracks the minimum capsule distance, the active penalty, and whether the safety band is active, but it does not get its own dashboard card

## Joint-Space Step Response

This demo is a validation path for controller and dynamics behavior in isolation.
It uses the same dashboard layout, with the simulation still dominant on the
right and the metric cards stacked on the left.

### Runtime Loop

```text
q_ref -> controller -> tau -> dynamics -> integration -> q
```

### What It Shows

- controller response to a step input on the 7-joint chain
- default metric plots for:
  - joint 1 position error norm
  - all-joint velocity error norm
  - all-joint actuator torque norm
  - all-joint maximum joint error magnitude
- clicking a joint chip switches the same four cards to that joint's angle, velocity, torque, and power
- each metric plot uses a rounded 5-tick y-axis ladder that resizes automatically
- the plots show grid lines only at the labeled major tick values
- the time axis uses 2-second ticks
- metric histories keep the last 15 seconds
- metric axes follow the visible range of the active view
- clicking the active joint chip again returns to the original joint-space graphs without clearing history
- the same keyboard fallback applies here

## Demo Expectations

- The Cartesian demo should be the default showcase for the project.
- The joint-space demo should remain available for controller tuning and response inspection.
- Visualization should remain tied to the FK-derived geometry so the rendered arm matches the model.
- The demo window should launch as a standard resizable desktop window in a session with a usable display backend.
- The demos should not rely on a custom in-window exit button; the OS title-bar controls are the primary close path.
