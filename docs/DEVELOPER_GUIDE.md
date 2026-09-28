# litearm-mujoco Developer Guide

> litearm-mujoco v0.3.0

## Project Structure

```
litearm-mujoco/
├── pyproject.toml              # Build config (setuptools)
├── setup.cfg                   # Legacy pip compatibility
├── setup.py                    # Legacy pip compatibility
├── MANIFEST.in                 # Package assets (STL + XML)
├── README.md                   # User-facing documentation
├── README_zh-CN.md             # Chinese user documentation
├── .gitignore
├── src/
│   └── litearm_mujoco/
│       ├── __init__.py         # Exports: MujocoArm, DualArm, MirrorMode, litearm_core
│       ├── _sdk.py             # Single point of contact with the real-arm SDK
│       ├── arm.py              # Core: MujocoArm class (500+ lines, litearm_core.Arm API)
│       ├── controller.py       # PID joint controller + trapezoidal trajectory generator
│       ├── kinematics.py       # FK/IK (damped least squares) + path planning
│       ├── mirror.py           # DualArm (dual control) + MirrorMode (mirror tracking)
│       ├── trajectory.py       # JointTrajectory / TrajectoryFrame (sim-only extension)
│       └── assets/
│           ├── litearm7.xml    # 7-DOF MuJoCo MJCF model (aligned to real URDF)
│           └── meshes/         # 9 real STL meshes (shipped with package)
├── examples/
│   ├── 01_hello_sim.py         # Standalone sim + read state (no hardware)
│   ├── 02_movej_sim.py         # Joint & Cartesian motion + FK/IK (no hardware)
│   ├── 03_trajectory.py        # Record & replay trajectories (no hardware)
│   ├── 04_mirror_real.py       # Mirror mode — sim follows real arm
│   └── 05_dual_control.py      # Dual control — real + sim together
├── tests/
│   ├── __init__.py
│   └── test_mujoco_arm.py      # 29 test cases (9 test classes)
└── docs/
    └── DEVELOPER_GUIDE.md      # This file
```

## Core Architecture

```
┌──────────────────────────────────────────────────┐
│               Your Python Program                 │
│                                                    │
│  arm = MujocoArm(render=True)  ←  swap litearm_core.Arm
│  arm.movej(...) / arm.get_state() / arm.close()    │
└──────────┬──────────────────┬────────────────────┘
           │                  │
    ┌──────▼──────┐    ┌─────▼──────────────┐
    │ Standalone   │    │ Dual / Mirror      │
    │ Simulation   │    │                    │
    │              │    │ MuJoCo +           │
    │ MuJoCo       │    │ litearm-python       │
    │ physics      │    │ → USB CDC          │
    │ engine       │    │ → STM32 firmware   │
    │ PID control  │    │                    │
    └──────────────┘    └────────────────────┘
```

### Module Responsibilities

| Module | Responsibility |
|--------|---------------|
| `_sdk.py` | The one place the SDK's import name appears; re-exports `Msg`, `RobotState`, `CartPlan`, error classes, plus the `state_q()` envelope-unwrapping helper |
| `trajectory.py` | `JointTrajectory` / `TrajectoryFrame` — a simulation-only extension (litearm-python has no portable trajectory type) |
| `arm.py` | Main class `MujocoArm` — `litearm_core.Arm` API compatibility, background physics thread |
| `controller.py` | `JointPIDController` position controller + `TrajectoryGenerator` trapezoidal velocity profiles |
| `kinematics.py` | `Kinematics` class: FK, IK (damped least squares), path planning (linear/arc/multi-waypoint) |
| `mirror.py` | `DualArm` (controls real + sim together), `MirrorMode` (sim tracks real arm) |
| `litearm7.xml` | MuJoCo MJCF model: 7 hinge joints, torque motors, real STL mesh geometry |

### Data Flow

```
arm.movej(q_target, speed=0.5)
        │
        ▼
┌───────────────────┐
│ Trajectory Gen     │  ← TrajectoryGenerator.linear_trajectory()
│ Trapezoidal profile│     Generates joint-space path
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ PID Controller     │  ← JointPIDController.set_target(q_des)
│ Sets target pos    │
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ Background sim     │  ← _sim_loop() @ 500Hz
│ thread             │     PID computes torque → mj_step() advances physics
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ State cache        │  ← _state_cache (updated every step)
│ get_state() reads  │
└───────────────────┘
```

## Design Principles

### 1. API Compatibility

`MujocoArm` implements the `litearm_core.Arm` API surface. Users can swap
`litearm_core.Arm(port=...)` with `MujocoArm(render=True)` and run identical
control code in simulation.

> ⚠ The surface was originally written against the legacy `litearm-python` 0.1
> (pylitearm-era) API — `movel` / `movec` / `movep` / `enter_teleop` /
> `joint_impedance` / `devices`. Renaming those to litearm-python's
> `move_l` / `move_c` / `move_path` (and dropping the ones litearm-python
> deliberately does not have) is the follow-up commit.

### 2. Three Operating Modes

| Mode | Class | How to use | Needs hardware |
|------|-------|------------|:---:|
| Standalone | `MujocoArm` | `MujocoArm(render=True)` | ❌ |
| Mirror | `MujocoArm` + `mirror_from()` | `sim.mirror_from(real)` | ✅ |
| Dual control | `DualArm` | `DualArm(real_port=...)` | ✅ |

### 3. Thread Safety

- The simulation loop runs in a background thread, protected by `self._lock`
- Kinematics computations use a separate `_kin_data` (`MjData` copy) to avoid
  racing with the simulation thread
- `get_state()` returns a snapshot of the cached state without blocking the
  simulation loop

### 4. Dependencies

The SDK (`litearm-python`, import name `litearm`) is a **hard** dependency:
`_sdk.py` imports it at module load, so even standalone simulation will not
import without it. That is deliberate — `MujocoArm` returns the SDK's own `Msg`
/ `RobotState` / `CartPlan` types, which only works if they are the real ones
(`isinstance` checks and `except IKError` behave identically in sim and on
hardware).

It is declared in `pyproject.toml`, but pinned to a git tag rather than a
version, because it is not published on PyPI yet. Bump the tag deliberately.
Because it is a git URL, installing this package needs `git` on `PATH`.

Import names go through `_sdk.py` only. The same 2.x code also exists under the
name `litearm_core` (how it was developed); the shim accepts that as a fallback
so a local checkout keeps working. `_sdk.py` rejects the legacy zenoh-based
`litearm` 0.1 explicitly, since it shares the import name but has no `Msg`
envelope.

## MuJoCo Model (litearm7.xml)

- 7 hinge joints, torque-motor actuated
- Kinematic chain (link pos/quat, joint axis/range) aligned to the real LiteArm URDF
- Zero-configuration TCP = [0, 0, 0.814] m (matches real hardware)
- Mass/inertia/CoM from URDF (total mass ≈ 2.93 kg)
- Visual geometry: 9 real STL meshes shipped with the package
- Motors: `gear="1"` (direct torque control)

## Controller Parameters

Default gains are aligned to the real LiteArm MIT follow-mode gains:

| Joint | kp (Nm/rad) | kd (Nm·s/rad) |
|-------|-------------|----------------|
| 1-2 | 260 | 5 |
| 3-4 | 150 | 4-5 |
| 5-7 | 50 | 2.5 |

The simulation loop includes gravity + Coriolis feed-forward compensation
(`qfrc_bias`) to eliminate steady-state sag. The PID controller includes
integral anti-windup.

## Dependencies

### Declared
- `mujoco>=3.0`
- `numpy>=1.21`
- `litearm-python` — hard import, not on PyPI, pinned to a git tag

### Dev
- `pytest`, `pytest-timeout`

## Known Limitations

- Controller is joint-space PD + gravity/Coriolis feed-forward. Full
  computed-torque feed-forward and friction compensation are not included.
- Dexterous hand / gripper / teach pendant are simulated proxies returning
  fixed values. litearm-python has no peripheral bus at all, so these are
  simulation-only and are slated for removal with the API realignment.
- System management / logging / teleop are API-compatible no-ops. Same: the
  follow-up realignment removes them rather than keeping no-op fakes.
- IK uses damped least squares with local convergence; may not converge for
  large Cartesian displacements far from the seed configuration.

## License

Proprietary