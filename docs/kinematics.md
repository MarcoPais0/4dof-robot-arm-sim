# Kinematics

## Frame Assignment

- Base frame: `{0}`
- Joint frames: `{1}`, `{2}`, `{3}`, `{4}`
- Tool frame: `{tool}`

The chain is a 4R spatial manipulator. The tool frame is modeled explicitly after joint 4.

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
a_i = [0, L2, L3, 0]
alpha_i = [pi/2, 0, pi/2, 0]
d_i = [L1, 0, 0, 0]
theta_i = q_i + theta_offset_i
```

The tool transform is:

```text
T_4^tool = Trans_z(L_tool), with L_tool > 0
```

## Forward Kinematics

The forward chain is:

```text
T_0^4(q) = T_0^1(q1) T_1^2(q2) T_2^3(q3) T_3^4(q4)
T_0^tool(q) = T_0^4(q) T_4^tool
p_k = T_0^k[0:3, 3]
p_tool = T_0^tool[0:3, 3]
```

The tool origin stays fixed under changes to `q4` when `q1`, `q2`, and `q3` are held constant, because the tool offset is aligned with `z4`.

## Design Notes

- FK returns full homogeneous transforms.
- Point extraction remains part of FK so visualization does not need a separate geometry layer.
- The chosen DH table is intentionally non-degenerate so the spatial structure is easy to analyze.

