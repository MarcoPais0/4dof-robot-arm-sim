# Final 4DOF 3D Robotic Arm Implementation Plan

## Purpose

This document defines the intended final implementation for the 4DOF robotic arm
project. It describes the target architecture, mathematical model, control
structure, demo layer, and completion criteria.

## Architecture Principles

- The project models a spatial 4DOF arm with standard DH geometry and an
  explicit tool frame with nonzero translation along `z4`.
- The task is `3D` tool-position control with one redundant DOF; tool
  orientation is modeled but not actively controlled.
- The redundant DOF is end-effector roll about `z4` for the default tool
  geometry, not an independent position actuator.
- The system is structured around five core interfaces: `FK`, `Jacobian`,
  `IK`, `Dynamics`, and `Controller`.
- These interfaces define the stable mathematical and control boundaries of the
  stack.
- Geometry, numerical integration, demos, visualization, and reporting are
  concrete modules.
- The selected methods are standard DH forward kinematics with point
  extraction, a geometric Jacobian, damped least-squares resolved-rate IK,
  simplified diagonal dynamics, joint-space PD plus gravity compensation, and
  semi-implicit Euler integration.
- The design favors simplicity over extensibility in v1.
- The retained demos are joint-space step response and task-space sequential
  target tracking.

## Implementation Dependency Flow

The topic sequence in this document follows the intended implementation order:

```text
Manipulator Definition and Frames
      ->
   DH Table
      ->
Forward Kinematics and Point Extraction
      ->
Geometric Jacobian
      ->
Inverse Differential Kinematics
      ->
Dynamics
      ->
Joint-Space Control
      ->
Numerical Integration
      ->
Demo Layer
```

The runtime loop used inside the demos remains `controller -> dynamics ->
integration` at each timestep, but the document topics are ordered by build
dependency rather than closed-loop execution order.

## System Model and Notation

- Joint state and references: `q`, `q_dot`, `q_ddot`, `q_ref`, `q_dot_ref`,
  `q_dot_cmd`
- Joint actuation and dynamics: `tau`, `tau_cmd`, `M(q)`, `D`, `g(q)`, `e`,
  `e_dot`
- DH geometry: `a_i`, `alpha_i`, `d_i`, `theta_i`, `T_(i-1)^i`
- Pose and task space: `T_0^4(q)`, `T_4^tool`, `T_0^tool(q)`, `x`, `x_d`,
  `x_dot`, `v`, `v_des`
- Jacobian and gains: `J(q)`, `J_v(q)`, `J_v^+`, `K_x`, `dt`

Unless otherwise noted, joint angles are in radians, angular velocities in
rad/s, lengths in meters, linear velocities in m/s, accelerations in SI units,
and torques in N m.

## Core Interfaces

- `FK`:
  - Input: `q`
  - Output: `T_0^4(q)`, `T_0^tool(q)`, plotted joint/tool points
  - Method: standard DH forward kinematics with FK-owned point extraction
- `Jacobian`:
  - Input: `q` and FK transforms
  - Output: `J(q)`, `J_v(q)`
  - Method: geometric Jacobian from the transform chain
- `IK`:
  - Input: `x_d`, `x`, `J_v(q)`
  - Output: `q_dot_cmd`, `q_ref_next`, `q_dot_ref`
  - Method: damped least-squares resolved-rate IK with adaptive damping
- `Dynamics`:
  - Input: `q`, `q_dot`, `tau`
  - Output: `q_ddot`
  - Method: simplified diagonal joint-space dynamics with viscous damping and
    gravity
- `Controller`:
  - Input: `q`, `q_dot`, `q_ref`, `q_dot_ref`
  - Output: `tau_cmd`, `tau`
  - Method: joint-space PD plus gravity compensation with saturation

## Topic 1: Manipulator Definition and Frames

The topics below are intentionally ordered by implementation dependency so the
project can be built from geometry outward to demos.

### Objective

Define the robot as a spatial serial chain of four revolute joints and establish
the frame assignment used throughout the project.

### Frame Set

- Base frame: `{0}`
- Joint frames: `{1}`, `{2}`, `{3}`, `{4}`
- Tool frame: `{tool}`

### Design Decisions

- All joints are revolute.
- The tool frame is modeled explicitly after joint 4.
- The tool frame origin is offset along `z4` so joint 4 acts as an
  end-effector roll joint without moving the tool point.

## Topic 2: Standard DH Convention

### Objective

Define the final arm geometry with a sufficiently non-degenerate spatial DH
table.

### DH Transform

```text
T_(i-1)^i =
[ cos(theta_i)  -sin(theta_i) cos(alpha_i)   sin(theta_i) sin(alpha_i)   a_i cos(theta_i) ]
[ sin(theta_i)   cos(theta_i) cos(alpha_i)  -cos(theta_i) sin(alpha_i)   a_i sin(theta_i) ]
[      0                sin(alpha_i)               cos(alpha_i)                  d_i        ]
[      0                     0                           0                         1         ]
```

### Target DH Table

```text
a_i = [0, L2, L3, 0]
alpha_i = [pi/2, 0, pi/2, 0]
d_i = [L1, 0, 0, 0]
theta_i = q_i + theta_offset_i
```

### Tool Transform

```text
T_4^tool = Trans_z(L_tool), with L_tool > 0
```

### Design Decisions

- `theta_offset_i` remains available for deliberate home-frame alignment.
- The nonzero `alpha_3` term prevents joint 3 and joint 4 axes from being
  collinear.
- The tool offset is modeled through `T_4^tool` along `z4`.
- Any custom `T_4^tool` override must preserve positive `z4` translation and
  tool-axis alignment with `z4`.
- Aligning the tool with `z4` partially decouples positioning from
  end-effector roll: changing `q4` changes the tool orientation about `z4`,
  but does not move the tool origin for this geometry.
- This is not a complete kinematic decoupling because joints 1 through 3 still
  determine the position and direction of the `z4` axis.

## Topic 3: Forward Kinematics

### Objective

Compute the joint-4 and tool pose with respect to the base frame, and expose
the FK-derived point data needed by the visualization layer.

### Governing Equations

```text
T_0^4(q) = T_0^1(q1) T_1^2(q2) T_2^3(q3) T_3^4(q4)
T_0^tool(q) = T_0^4(q) T_4^tool
p_k = T_0^k[0:3, 3]
p_tool = T_0^tool[0:3, 3]
```

### Design Decisions

- Public FK outputs are `T_0^tool(q)`, `T_0^4(q)`, and FK-derived point
  extraction.
- FK returns full homogeneous transforms.
- Point extraction stays inside FK.
- With `T_4^tool = Trans_z(L_tool)`, FK should keep `p_tool` invariant under
  changes to `q4` when `q1`, `q2`, and `q3` are fixed.

## Topic 4: Geometric Jacobian

### Objective

Relate joint velocities to tool spatial velocity through a geometric Jacobian.

### Governing Equations

```text
x_dot = [v; omega] = J(q) q_dot
v_i = z_(i-1) x (p_e - p_(i-1))
omega_i = z_(i-1)
J_i = [v_i; omega_i]
```

### Design Decisions

- The geometric Jacobian is constructed from the transform chain.
- All Jacobian quantities are expressed in the base frame.
- The primary control block is `J_v`.
- Because `T_4^tool = Trans_z(L_tool)`, the `q4` column of `J_v` is zero for
  the default geometry: joint 4 contributes roll/orientation, not tool-origin
  translation.
- For a sufficiently non-degenerate spatial geometry and away from
  singularities, `J_v` should achieve rank `3`.
- Singularity analysis should use SVD-derived quantities such as rank, condition
  number, and manipulability.

## Topic 5: Damped Least-Squares Inverse Differential Kinematics

### Objective

Map a desired tool linear velocity to a feasible joint-velocity command using
the position Jacobian.

### Governing Equations

```text
v_des_raw = K_x (x_d - x)
v_des = sat_v(v_des_raw, v_max)
sigma_min_value = min_singular_value(J_v)
lambda = lambda_0 + k_lambda / (sigma_min_value + epsilon)
J_v^+ = J_v^T (J_v J_v^T + lambda^2 I_3)^(-1)
q_dot_cmd = J_v^+ v_des
q_ref_next = q_ref + dt q_dot_cmd
q_dot_ref = q_dot_cmd
```

### Design Decisions

- `q_dot_cmd` is integrated into `q_ref_next`, and `q_dot_ref = q_dot_cmd`.
- Position IK acts through `J_v`; with the default `z4` tool alignment, it
  should not rely on joint 4 to move the tool origin.
- The damping term `lambda` increases as the smallest singular value decreases.
- This damping rule prioritizes stability over accuracy near singularities.
- The magnitude of `v_des` is limited to improve numerical stability and to
  avoid excessive joint velocities near singularities.

## Topic 6: Simplified Joint-Space Dynamics

### Objective

Provide a minimal but useful dynamic model for control and simulation.

### Governing Equations

The full rigid-body equation is:

```text
M(q) q_ddot + C(q, q_dot) q_dot + g(q) = tau
```

The project approximation is:

```text
M q_ddot = tau - D q_dot - g(q)
M approx diag(I_1, I_2, I_3, I_4)
D approx diag(d_1, d_2, d_3, d_4)
```

### Design Decisions

- The inertia matrix is diagonal and configuration-independent.
- The rigid-body term `C(q, q_dot) q_dot` is omitted.
- `D q_dot` represents viscous damping.
- Practical constraints include torque limits and torque saturation.

## Topic 7: Joint-Space PD Control

### Objective

Generate torque commands for joint-space tracking while remaining consistent
with the simplified dynamics model.

### Governing Equations

```text
e = q_ref - q
e_dot = q_dot_ref - q_dot
tau_cmd = K_p e + K_d e_dot + g(q)
tau = sat(tau_cmd, tau_min, tau_max)
```

### Design Decisions

- Each joint is controlled independently.
- The derivative term uses velocity error rather than finite-difference position
  estimates.
- The `+ g(q)` term is included to approximately cancel the `- g(q)` term in the
  dynamics.

## Topic 8: Numerical Integration

### Objective

Standardize time stepping for the simulation stack.

### Governing Equations

```text
q_ddot = M^(-1) (tau - D q_dot - g(q))
q_dot_next = q_dot + dt q_ddot
q_next = q + dt q_dot_next
```

### Design Decisions

- The project uses a fixed time step.
- The acceleration `q_ddot` is evaluated from the current state.
- Semi-implicit Euler uses the updated velocity in the position step.

## Topic 9: Demo Architecture

### Purpose

The demos provide controller-only and full-pipeline validation.

### Demo 1: Joint-Space Step Response

#### Objective

Analyze controller behavior in isolation through joint-space references.

#### Loop

```text
q_ref -> controller -> tau -> dynamics -> integration -> q
```

#### Required Outputs

- Joint position versus time
- Joint velocity versus time
- Error versus time
- Torque versus time
- Rise time
- Settling time
- Overshoot

### Demo 2: Task-Space Sequential Target Tracking

#### Objective

Validate the full pipeline through sequential spatial targets.

#### Loop

```text
x_d -> x -> v_des_raw = K_x (x_d - x) -> v_des = sat_v(v_des_raw, v_max)
-> sigma_min_value = min_singular_value(J_v) -> lambda
-> q_dot_cmd = J_v^+ v_des -> q_ref_next / q_dot_ref
-> controller -> dynamics -> integration -> q -> x
```

#### Required Visualization

- All target points shown as blue balls
- Active target shown in red
- End-effector trace shown continuously

## Reporting and Analysis Requirements

The final implementation should support a report that includes:

- the complete DH table,
- the frame diagram,
- forward-kinematics derivation,
- Jacobian interpretation,
- SVD-based singularity analysis,
- damping-strategy explanation,
- simplified-dynamics justification,
- controller-structure justification,
- numerical-integration stability discussion,
- demo results and interpretation,
- workspace analysis,
- units and modeling assumptions.

## Implementation Priorities

1. Finalize the manipulator definition, frame assignment, spatial DH geometry,
   and explicit tool frame as concrete model data.
2. Implement the `FK` interface for both the joint-4 frame and the tool frame,
   together with FK-owned point extraction for visualization and plotting.
3. Implement the `Jacobian` interface and validate its spatial rank behavior.
4. Implement the `IK` interface with `J_v`-based inverse differential
   kinematics, resolved-rate reference generation, and singularity-aware
   damping.
5. Extend the `Dynamics` interface with diagonal inertia, viscous damping,
   gravity, and actuator constraints.
6. Implement the `Controller` interface with joint-space PD plus gravity
   compensation and torque saturation.
7. Use one concrete semi-implicit Euler stepping module for all simulations in
   v1.
8. Deliver the two final demos and their reporting outputs as direct consumers
   of the core interfaces.

## Acceptance Criteria

The final implementation is considered complete when:

- the arm geometry is spatial and uses the target DH table with
  `alpha = [pi/2, 0, pi/2, 0]`,
- `T_4^tool` includes a nonzero translation `L_tool > 0` along `z4`,
- forward kinematics returns both joint-4 and tool pose consistently and
  exposes FK-derived joint/tool points for plotting without a separate
  interface layer,
- changing only `q4` changes tool roll/orientation but leaves the tool origin
  position unchanged,
- the document and implementation consistently describe a `3D` position task
  with one redundant DOF,
- only five stable interfaces are defined: `FK`, `Jacobian`, `IK`,
  `Dynamics`, and `Controller`,
- geometry/model definition, numerical integration, demos, and
  visualization/reporting are described as concrete modules or helpers rather
  than standalone interfaces,
- each topic presents one selected approach only,
- the Jacobian section states the base-frame convention explicitly,
- the Jacobian section explains that the default `z4` tool alignment makes the
  `q4` position-Jacobian column zero,
- `J_v` supports position-first inverse differential kinematics with a
  documented damping rule,
- the damping rule explicitly defines `sigma_min_value` and
  `lambda = lambda_0 + k_lambda / (sigma_min_value + epsilon)`,
- the controller pipeline includes `v_des`, `q_dot_cmd`, `q_ref_next`, and
  `q_dot_ref`,
- the simplified dynamics section explicitly distinguishes omitted rigid-body
  `C(q, q_dot) q_dot` terms from viscous damping `D q_dot`,
- the controller section explicitly states the gravity-compensation
  cancellation intent and uses PD control,
- all simulations use a single concrete semi-implicit Euler stepping path,
- the joint-space step demo reports plots and performance metrics,
- the sequential target demo shows active targets and end-effector trace,
- the report can be written directly from this design without needing another
  architectural rewrite.
