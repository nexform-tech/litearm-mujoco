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
│       ├── __init__.py         # Exports: MujocoArm, DualArm, MirrorMode, litearm
│       ├── arm.py              # Core: MujocoArm class (500+ lines, full litearm.Arm API)
│       ├── controller.py       # PID joint controller + trapezoidal trajectory generator
│       ├── kinematics.py       # FK/IK (damped least squares) + path planning
│       ├── mirror.py           # DualArm (dual control) + MirrorMode (mirror tracking)
│       ├── _litearm/           # Minimal vendored SDK (trajectory types only)
│       │   ├── __init__.py     # Lazy-loads litearm-python when available
│       │   └── types.py        # JointTrajectory / TrajectoryFrame
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
│  arm = MujocoArm(render=True)  ←  swap litearm.Arm│
│  arm.movej(...) / arm.get_state() / arm.close()    │
└──────────┬──────────────────┬────────────────────┘
           │                  │
    ┌──────▼──────┐    ┌─────▼──────────────┐
    │ Standalone   │    │ Dual / Mirror      │
    │ Simulation   │    │                    │
    │              │    │ MuJoCo + Zenoh     │
    │ MuJoCo       │    │ → litearm-server   │
    │ physics      │    │ → real hardware    │
    │ engine       │    │                    │
    │ PID control  │    │                    │
    └──────────────┘    └────────────────────┘
```

### Module Responsibilities

| Module | Responsibility |
|--------|---------------|
| `arm.py` | Main class `MujocoArm` — full `litearm.Arm` API compatibility, background physics thread |
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

`MujocoArm` implements the complete `litearm.Arm` API surface (50+ methods). Users can
swap `litearm.Arm(endpoint=...)` with `MujocoArm(render=True)` and run identical
control code in simulation.

### 2. Three Operating Modes

| Mode | Class | How to use | Needs hardware |
|------|-------|------------|:---:|
| Standalone | `MujocoArm` | `MujocoArm(render=True)` | ❌ |
| Mirror | `MujocoArm` + `mirror_from()` | `sim.mirror_from(real)` | ✅ |
| Dual control | `DualArm` | `DualArm(real_endpoint=...)` | ✅ |

### 3. Thread Safety

- The simulation loop runs in a background thread, protected by `self._lock`
- Kinematics computations use a separate `_kin_data` (`MjData` copy) to avoid
  racing with the simulation thread
- `get_state()` returns a snapshot of the cached state without blocking the
  simulation loop

### 4. Zero-Dependency Standalone Mode

Standalone simulation requires only `mujoco>=3.0` and `numpy>=1.21`.
Mirror/Dual modes require the `[mirror]` extra, which installs `eclipse-zenoh`
and `protobuf`. The dependency on `litearm-python` is lazy — it is imported
only when `DualArm` or `mirror_from()` is actually used.

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

### Core
- `mujoco>=3.0`
- `numpy>=1.21`

### Optional (mirror/dual mode)
- `eclipse-zenoh>=1.0`
- `protobuf>=4.0`

## Known Limitations

- Controller is joint-space PD + gravity/Coriolis feed-forward. Full
  computed-torque feed-forward and friction compensation are not included.
- Dexterous hand / gripper / teach pendant are simulated proxies returning
  fixed values.
- System management / logging / teleop are API-compatible no-ops.
- IK uses damped least squares with local convergence; may not converge for
  large Cartesian displacements far from the seed configuration.

## License

Proprietary