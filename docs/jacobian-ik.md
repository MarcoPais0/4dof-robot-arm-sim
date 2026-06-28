# Jacobian and Inverse Differential Kinematics

## Geometric Jacobian

The base-frame geometric Jacobian maps joint rates to end-effector spatial velocity:

```text
x_dot = [v; omega] = J(q) q_dot
v_i = z_(i-1) x (p_e - p_(i-1))
omega_i = z_(i-1)
J_i = [v_i; omega_i]
```

The position Jacobian `J_v` is the upper 3x7 block of `J`.

With the default tool alignment aligned to the final wrist axis, the terminal roll joint contributes orientation but not tool-origin translation, so the last column of `J_v` is zero even though the tool frame now includes a short extension beyond joint 7.

## Task Jacobian

The inverse-kinematics loop now uses a 5D primary task:

```text
J_task = [J_v
          J_align]
```

where `J_align` is built from the tool-frame z-axis expressed in the world x/y basis:

```text
z_tool = T_0^tool[0:3, 2]
e_align = [z_tool.x, z_tool.y]
J_align = [e_x^T; e_y^T] (-[z_tool]_x J_omega)
```

The two alignment rows keep the tool axis pointed toward world `-Z` while leaving roll about the tool axis free.

## Singularity Analysis

The task Jacobian is analyzed through its singular values:

- numerical rank
- smallest singular value
- condition number
- manipulability

These quantities support both documentation and damping logic.

## Inverse Differential Kinematics

The resolved-rate controller is task-priority based:

```text
e_pos = x_d - x
v_pos = sat_v(K_x e_pos)
e_align = [z_tool.x, z_tool.y]
v_align = -k_axis e_align
v_task = [v_pos; v_align]

sigma_min_value = min_singular_value(J_task)
lambda = lambda_0 + k_lambda / (sigma_min_value + epsilon)
J_task^+ = damped_least_squares(J_task, lambda)
q_dot_primary = J_task^+ v_task

U_coll = 0.5 * k_coll * max(0, d_safe - d)^2
q_dot_coll = -grad(U_coll)

q_dot_null = -k_null grad(
    1 / (sigma_min_value + eps)
    + small hemisphere safeguard
    + tiny joint-centering bias
)
q_dot_cmd = q_dot_primary + (I - J_task^+ J_task) (q_dot_coll + q_dot_null)
q_ref_next = q_ref + dt q_dot_cmd
q_dot_ref = q_dot_cmd
```

## Self-Collision Repulsion

Each arm link is treated as a capsule built from the FK points:

```text
base->J1, J1->J2, ..., J7->tool
```

The solver checks all non-adjacent capsule pairs, because adjacent links share a joint and should not be treated as a collision.
Pairs that only meet at a shared FK endpoint are also skipped, because the zero-offset iiwa-style DH table can place some nominal origins at the same position in the home pose.

The repulsion model is intentionally simple:

```text
U_coll = 0.5 * k_coll * max(0, d_safe - d)^2
```

- `U_coll` is the soft penalty energy for an unsafe pair.
- `k_coll` is the repulsion strength.
- `d` is the current shortest distance between the two capsules.
- `d_safe` is the trigger distance where repulsion begins.
- `max(0, d_safe - d)` means the penalty is zero once the pair is safely separated.

In this project, `d_safe` is computed as:

```text
d_safe = r_i + r_j + 0.01 m
```

where `r_i` and `r_j` are the capsule radii. The current implementation uses a single fixed radius for all link capsules, derived from the rendered link radius.

The gradient of `U_coll` is converted into a joint-space correction with central finite differences, then projected through the task nullspace so it stays below the primary position + down-axis task.

## Design Notes

- Damping grows as the smallest singular value decreases.
- The method prioritizes stability near singularities over perfect tracking accuracy.
- The solver keeps the tool on the down-pointing branch with a small hemisphere safeguard rather than trying to solve a full 6D pose task.
- Collision avoidance is implemented as a soft self-collision repulsion between non-adjacent link capsules; external obstacle collision is still out of scope.
