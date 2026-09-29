# litearm-mujoco

Official MuJoCo-based simulation environment for the **LiteArm 7-DOF robotic arm**.
API-compatible with `litearm-python` — swap `Arm` with `MujocoArm` and your
control code runs identically in simulation and on hardware.

## Features

- 🔄 **Drop-in API compatibility** — Same interface as `litearm-python`. `MujocoArm` replaces `Arm` directly.
- 🖥️ **Three operating modes** — Standalone simulation / Mirror tracking / Dual control
- 🎮 **Native MuJoCo rendering** — Real-time visualization of arm motion
- 🧪 **No hardware required** — Develop and test motion logic without a physical arm
- 🐍 **Pure Python** — Zero compilation.

## Installation

```bash
pip install litearm-mujoco
```

The SDK, [`litearm-python`](https://github.com/nexform-tech/litearm-python), is
a hard dependency and pip installs it for you — this package returns the SDK's
own `Msg` / `RobotState` / `CartPlan` types, so it cannot be simulated away.

> ⚠ The SDK is not published on PyPI yet, so it is pinned to a git tag in
> `pyproject.toml`. `git` must be on your `PATH` for the install to resolve.

Or from source:

```bash
git clone https://github.com/nexform-tech/litearm-mujoco.git
cd litearm-mujoco
pip install -e ".[dev]"
```

Standalone simulation needs no hardware.

## Quick Start

### Mode 1 — Standalone Simulation

```python
from litearm_mujoco import MujocoArm

with MujocoArm(render=True) as arm:
    # Joint-space motion
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # Cartesian straight-line motion
    pos, R = arm.get_tcp_pose()
    arm.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.1)

    # Read state
    state = arm.get_state()
    print(state["q"])
```

### Mode 2 — Mirror Mode (sim follows real arm)

```python
from litearm_mujoco import litearm_core, MujocoArm

# Connect to the real arm over USB CDC (omit port= to auto-discover)
real = litearm_core.Arm(port="/dev/ttyACM0").connect()

# Create simulation and start mirroring
sim = MujocoArm(render=True)
sim.start()
sim.mirror_from(real)  # sim tracks real arm in real time
```

### Mode 3 — Dual Control (control both simultaneously)

```python
from litearm_mujoco import DualArm

dual = DualArm(real_port="/dev/ttyACM0", render=True)
dual.start()

# One command — both arms move!
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

dual.close()
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│               Your Python Program                │
│                                                   │
│   arm = MujocoArm()  ← can replace litearm_core.Arm│
│   arm.movej(...)                                  │
│   arm.get_state()                                 │
└──────────┬────────────────────┬─────────────────┘
           │                    │
    ┌──────▼──────┐      ┌─────▼──────────┐
    │ Standalone   │      │ Dual / Mirror   │
    │ Simulation   │      │                 │
    │              │      │ MuJoCo +        │
    │ MuJoCo       │      │ litearm-python    │
    │ physics      │      │ → USB CDC       │
    │ engine       │      │ → STM32 fw      │
    │ PID ctrl     │      │                 │
    │ FK/IK        │      │ Real + Sim      │
    │              │      │ together        │
    └──────────────┘      └─────────────────┘
```

## Examples

| Example | Description | Needs real arm |
|---------|-------------|:---:|
| `01_hello_sim.py` | Create simulation + read state | ❌ |
| `02_movej_sim.py` | Joint & Cartesian motion + FK/IK | ❌ |
| `03_trajectory.py` | Record & replay trajectories | ❌ |
| `04_mirror_real.py` | Sim mirrors real arm | ✅ |
| `05_dual_control.py` | Control both arms simultaneously | ✅ |

```bash
python3 examples/01_hello_sim.py
python3 examples/02_movej_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --port /dev/ttyACM0
python3 examples/05_dual_control.py --port /dev/ttyACM0
```

## API Reference

> ⚠ This table describes the surface **before** the `litearm-python` realignment.
> `MujocoArm`'s own methods still carry the old names (`movel`/`movec`/`movep`);
> aligning them to `move_l`/`move_c`/`move_path` is the follow-up commit.

| litearm_core.Arm | MujocoArm | Notes |
|-------------|-----------|-------|
| `Arm(endpoint=...)` | `MujocoArm(render=True)` | Constructor |
| `movej(q, speed)` | `movej(q, speed)` | ✅ Identical |
| `movel(pose, speed)` | `movel(pose, speed)` | ✅ Identical |
| `movec(via, goal)` | `movec(via, goal)` | ✅ Identical |
| `movep(poses)` | `movep(poses)` | ✅ Identical |
| `get_state()` | `get_state()` | ✅ Identical |
| `get_tcp_pose()` | `get_tcp_pose()` | ✅ Identical |
| `fk(q)` | `fk(q)` | ✅ Identical |
| `ik(pos, R)` | `ik(pos, R)` | ✅ Identical |
| `replay_joint_path(path)` | `replay_joint_path(path)` | ✅ Identical |
| `replay_trajectory(traj)` | `replay_trajectory(traj)` | ✅ Identical |
| `record_trajectory()` | `record_trajectory()` | ✅ Simulated |
| `hold()` / `zero_gravity()` | `hold()` / `zero_gravity()` | ✅ Identical |
| `request_stop()` | `request_stop()` | ✅ Identical |
| `enable/disable()` | `enable/disable()` | ✅ Identical |
| `set_gains(kp, kd)` | `set_gains(kp, kd)` | ✅ Identical |
| `device("hand_0")` | `device("hand_0")` | ✅ Simulated proxy |
| `arm.hand.open()` | `arm.hand.open()` | ✅ Simulated proxy |

## Development

Prefer the dev container: open the folder in VS Code, press `F1`, and run
**Dev Containers: Reopen in Container** for a ready-to-run environment with
Python 3.12, MuJoCo, and all extras preinstalled. See
[Dev Container](docs/DEVELOPER_GUIDE.md#dev-container) in the developer guide.

Without Docker:

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## License

Proprietary

---

[中文文档](README_zh-CN.md) | [开发者指南](docs/DEVELOPER_GUIDE.md)