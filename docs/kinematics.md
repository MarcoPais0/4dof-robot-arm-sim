# Kinematics

## Frame Assignment

- Base frame: `{0}`
- Joint frames: `{1}`, `{2}`, `{3}`, `{4}`, `{5}`, `{6}`, `{7}`
- Tool frame: `{tool}`

The chain is a 7R spatial manipulator. The tool frame is modeled explicitly after joint 7.

## DH Geometry

The project uses standard Denavit-Hartenberg transforms:

```text
T_(i-1)^i =
[ cos(theta_i)  -sin(theta_i) cos(alpha_i)   sin(theta_i) sin(alpha_i)   a_i cos(theta_i) ]
[ sin(theta_i)   cos(theta_i) cos(alpha_i)  -cos(theta_i) sin(alpha_i)   a_i sin(theta_i) ]
[      0                sin(alpha_i)               cos(alpha_i)                  d_i        ]
[      0                     0                           0                         1         ]
```

Default parameters:

```text
a_i = [0, 0, 0, 0, 0, 0, 0]
alpha_i = [-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2, 0]
d_i = [0.34, 0, 0.40, 0, 0.40, 0, 0.126]
theta_offset_i = [0.0, -pi/2, pi/2, -pi/2, 0.0, pi/2, 0.0]
theta_i = q_i + theta_offset_i
```

The tool transform is:

```text
T_7^tool = translation along +z7 by default, with a short visible tool extension
```

## Forward Kinematics

The forward chain is:

```text
T_0^7(q) = T_0^1(q1) ... T_6^7(q7)
T_0^tool(q) = T_0^7(q) T_7^tool
p_k = T_0^k[0:3, 3]
p_tool = T_0^tool[0:3, 3]
```

The tool origin stays fixed under changes to the terminal wrist roll angle when the preceding joints are held constant, because the tool offset is aligned with `z7`.

## Design Notes

- FK returns full homogeneous transforms.
- Point extraction remains part of FK so visualization does not need a separate geometry layer.
- The chosen DH table now uses iiwa 14 R820-inspired proportions with zero lateral offsets, so the home pose contains a few coincident FK origins and the visible spans come from the axial `d_i` values.
- `DEFAULT_SEGMENT_LENGTHS` is derived from the home pose, which keeps the simplified gravity model aligned with the refreshed FK chain.
- The dashboard renders the seven joint origins as visible markers, a `7 DOF` badge, and the explicit tool point so the arm path matches the FK chain.
- The task-space controller uses a 5D primary task: three position rows plus two rows that keep the tool z-axis aligned with world down, while leaving roll about the tool axis free.
