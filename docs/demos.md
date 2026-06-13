# Demos

## Purpose

The demos show whether the kinematic model, inverse kinematics, controller, and dynamics work together as intended.
Run them with `python3 src/robot_arm_3d_demo.py --mode cartesian` for the main demo or `--mode joint` for the validation demo.

## Cartesian Tracking Demo

This is the main demo for the project. It exercises the full task-space pipeline.
It displays the 3D arm animation and live metric plots in the same window.

### Runtime Loop

```text
x_d -> x -> v_des -> q_dot_cmd -> q -> x
```

### What It Shows

- sequential target tracking in 3D
- 15 randomly scattered targets per run
- active target highlighting
- end-effector trace through space
- live metric plots for:
  - position error norm
  - commanded Cartesian speed norm
  - joint-speed norm
  - minimum singular value of `J_v`
- live status values for the current target index and tracking state

## Joint-Space Step Response

This demo is a validation path for controller and dynamics behavior in isolation.
It also displays live metric plots in the same window.

### Runtime Loop

```text
q_ref -> controller -> tau -> dynamics -> integration -> q
```

### What It Shows

- joint position over time
- joint velocity over time
- controller response to a step input
- live metric plots for:
  - joint-position error norm
  - joint-velocity error norm
  - torque norm
  - maximum absolute joint error
- live summary values for rise time, settling time, and overshoot

## Demo Expectations

- The Cartesian demo should be the default showcase for the project.
- The joint-space demo should remain available for controller tuning and response inspection.
- Visualization should remain tied to the FK-derived geometry so the rendered arm matches the model.
