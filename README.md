# 4DOF 3D Robotic Arm DH Simulation

Spatial 4DOF robotic arm simulation built around standard DH geometry, a
geometric Jacobian, damped differential inverse kinematics, simplified joint
dynamics, and PyVista visualization.

## What is included

- `src/forward_kinematics.py`: DH model and forward kinematics
- `src/geometric_jacobian.py`: geometric Jacobian and singularity analysis
- `src/inverse_differential_kinematics.py`: damped least-squares resolved-rate IK
- `src/joint_space_dynamics.py`: simplified joint-space dynamics
- `src/joint_space_controller.py`: joint-space PID controller
- `src/robot_arm_3d_demo.py`: interactive 3D arm demo
- `src/pid_tuning_sandbox.py`: small 1-DOF PID tuning sandbox
- `FINAL_4DOF_3D_IMPLEMENTATION_PLAN.md`: design and implementation notes

## Requirements

- Python 3.10+
- NumPy
- PyVista
- Matplotlib

Install the dependencies with:

```bash
pip install -r requirements.txt
```

## Run

Run the modules directly from the repository root:

```bash
python src/forward_kinematics.py
python src/geometric_jacobian.py
python src/inverse_differential_kinematics.py
python src/joint_space_dynamics.py
python src/joint_space_controller.py
python src/pid_tuning_sandbox.py
python src/robot_arm_3d_demo.py
```

The main visualization entrypoint is `src/robot_arm_3d_demo.py`.
