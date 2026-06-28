# Environment

## Purpose

This project is intended to run from a local virtual environment.
The GUI stack is constrained to a conservative set so the Qt dashboard starts with a known plugin layout and the OpenGL imports are explicit.

## Supported Runtime

- The demo should be launched from the repository venv, not from the system Python

## Setup

```bash
cd /Users/marco.cruz.pais/Documents/GitHub/spatial-7dof-robot-arm-sim
rm -rf .venv
python3 -m venv .venv
```

Activate it, install dependencies, and run the environment check:

```bash
source .venv/bin/activate
python --version
which python

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python scripts/check_environment.py
python scripts/smoke_check.py
python src/robot_arm_3d_demo.py --mode cartesian
python src/robot_arm_3d_demo.py --mode joint
```

If you prefer not to activate the venv, use the absolute venv interpreter path instead of `python`:

```bash
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python scripts/check_environment.py
./.venv/bin/python scripts/smoke_check.py
./.venv/bin/python src/robot_arm_3d_demo.py --mode cartesian
./.venv/bin/python src/robot_arm_3d_demo.py --mode joint
```

## What The Check Verifies

- `numpy`, `PySide6`, `pyqtgraph`, `OpenGL`, `OpenGL.GL`, and `pyqtgraph.opengl` import correctly
- the Qt plugin directory resolved from `PySide6` exists
- the Cocoa platform plugin file `libqcocoa.dylib` exists under the Qt `platforms` directory

## Startup Behavior

- The dashboard sets `QT_PLUGIN_PATH` and `QT_QPA_PLATFORM_PLUGIN_PATH` from the `PySide6` installation before `QApplication` is created.
- On macOS, startup also checks that the process is attached to an active console session before trying to construct the Qt application object.
- If the environment is incomplete, startup now reports the exact missing component instead of a generic reinstall message.
- If you see missing `numpy`, `PySide6`, `pyqtgraph`, or Qt plugin errors, force-reinstall the project dependencies from the repository root:

```bash
./.venv/bin/python -m pip install --force-reinstall -r requirements.txt
```

- The demo expects a working desktop session with display and OpenGL support.
