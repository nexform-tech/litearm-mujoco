# litearm-mujoco 开发者指南

> litearm-mujoco v0.3.0

## 项目结构

```
litearm-mujoco/
├── pyproject.toml              # 构建配置 (setuptools)
├── setup.cfg                   # 兼容旧版 pip
├── setup.py                    # 兼容旧版 pip
├── MANIFEST.in                 # 打包资产 (STL + XML)
├── README.md                   # 用户文档（英文）
├── README_zh-CN.md             # 用户文档（中文）
├── .gitignore
├── src/
│   └── litearm_mujoco/
│       ├── __init__.py         # 导出：MujocoArm, DualArm, MirrorMode, litearm_core
│       ├── _sdk.py             # 与真机 SDK 的唯一接触点（导入名收在这一处）
│       ├── arm.py              # 核心：MujocoArm 类（500+行，litearm_core.Arm API）
│       ├── controller.py       # PID 关节控制器 + 梯形速度轨迹生成器
│       ├── kinematics.py       # FK/IK（阻尼最小二乘）+ 路径规划
│       ├── mirror.py           # DualArm（双控）+ MirrorMode（镜像跟踪）
│       ├── trajectory.py       # JointTrajectory / TrajectoryFrame（仿真独有扩展）
│       └── assets/
│           ├── litearm7.xml    # 7-DOF MuJoCo MJCF 模型（与真机 URDF 对齐）
│           └── meshes/         # 9 个真实 STL 网格（随包分发）
├── examples/
│   ├── 01_hello_sim.py         # 独立仿真 + 读取状态（无需硬件）
│   ├── 02_movej_sim.py         # 关节与笛卡尔运动 + FK/IK（无需硬件）
│   ├── 03_trajectory.py        # 轨迹录制与回放（无需硬件）
│   ├── 04_mirror_real.py       # 镜像模式 — 仿真跟随实臂
│   └── 05_dual_control.py      # 双控模式 — 实臂 + 仿真同步
├── tests/
│   ├── __init__.py
│   └── test_mujoco_arm.py      # 29 个测试用例（9 个测试类）
└── docs/
    └── DEVELOPER_GUIDE.md      # 本文档（英文）
```

## 核心架构

```
┌──────────────────────────────────────────────────┐
│                  你的 Python 程序                  │
│                                                    │
│  arm = MujocoArm(render=True)  ← 可替换 litearm_core.Arm
│  arm.movej(...) / arm.get_state() / arm.close()    │
└──────────┬──────────────────┬────────────────────┘
           │                  │
    ┌──────▼──────┐    ┌─────▼──────────────┐
    │ 独立仿真     │    │ 双控 / 镜像         │
    │             │    │                    │
    │ MuJoCo      │    │ MuJoCo +           │
    │ 物理引擎    │    │ litearm-python       │
    │ PID 控制    │    │ → USB CDC          │
    │             │    │ → STM32 固件        │
    └─────────────┘    └────────────────────┘
```

### 模块职责

| 模块 | 职责 |
|------|------|
| `_sdk.py` | SDK 导入名唯一出现的地方；重导出 `Msg` / `RobotState` / `CartPlan` / 异常类，以及拆 `Msg` 信封的 `state_q()` |
| `trajectory.py` | `JointTrajectory` / `TrajectoryFrame` — 仿真独有扩展（litearm-python 没有可移植轨迹类型） |
| `arm.py` | 主类 `MujocoArm` — `litearm_core.Arm` API 兼容，后台物理线程 |
| `controller.py` | `JointPIDController` 位置控制器 + `TrajectoryGenerator` 梯形速度剖面 |
| `kinematics.py` | `Kinematics` 类：正运动学、逆运动学（阻尼最小二乘）、路径规划（直线/圆弧/多航点） |
| `mirror.py` | `DualArm`（同时控制实臂+仿真）、`MirrorMode`（仿真跟踪实臂） |
| `litearm7.xml` | MuJoCo MJCF 模型：7 个铰链关节、力矩电机、真实 STL 网格几何 |

### 数据流

```
arm.movej(q_target, speed=0.5)
        │
        ▼
┌───────────────────┐
│ 轨迹生成器         │  ← TrajectoryGenerator.linear_trajectory()
│ 梯形速度剖面       │     生成关节空间路径
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ PID 控制器         │  ← JointPIDController.set_target(q_des)
│ 设置目标位置       │
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ 后台仿真线程       │  ← _sim_loop() @ 500Hz
│                   │     PID 计算力矩 → mj_step() 推进物理
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ 状态缓存           │  ← _state_cache（每步更新）
│ get_state() 读取   │
└───────────────────┘
```

## 设计原则

### 1. API 兼容性

`MujocoArm` 实现 `litearm_core.Arm` 的 API 接口。用户只需将
`litearm_core.Arm(port=...)` 替换为 `MujocoArm(render=True)`，即可在仿真中
运行相同的控制代码。

> ⚠ 现有方法面当初是对着老的 `litearm-python` 0.1（pylitearm 时代）写的 —
> `movel` / `movec` / `movep` / `enter_teleop` / `joint_impedance` / `devices`。
> 把它们改成 litearm-python 的 `move_l` / `move_c` / `move_path`，并删掉
> core 刻意没有的那些，是后续那次提交的事。

### 2. 三种操作模式

| 模式 | 类 | 使用方式 | 需要硬件 |
|------|-----|----------|:---:|
| 独立仿真 | `MujocoArm` | `MujocoArm(render=True)` | ❌ |
| 镜像模式 | `MujocoArm` + `mirror_from()` | `sim.mirror_from(real)` | ✅ |
| 双控模式 | `DualArm` | `DualArm(real_port=...)` | ✅ |

### 3. 线程安全

- 仿真循环在后台线程中运行，通过 `self._lock` 保护数据访问
- 运动学计算使用独立的 `_kin_data`（`MjData` 副本），避免与仿真线程冲突
- `get_state()` 返回状态缓存的快照，不阻塞仿真循环

### 4. 依赖

SDK（`litearm-python`，导入名 `litearm`）是**硬依赖**：`_sdk.py` 在模块加载时就
import 它，所以连独立仿真在没装它时也 import 不进来。这是刻意的 — `MujocoArm`
返回的是 SDK 自己的 `Msg` / `RobotState` / `CartPlan` 类型，只有拿到真类型，
`isinstance` 与 `except IKError` 才能和真机行为一致。

它已写进 `pyproject.toml`，但钉的是 git tag 而不是版本号 —— 因为还没发布到 PyPI。
升级要手动改 tag。因为是 git URL，装本包需要 `PATH` 上有 `git`。

导入名只在 `_sdk.py` 一处出现。同一份 2.x 代码还有 `litearm_core` 这个名字
（开发时用的），shim 把它当兜底，这样本机的旧检出仍能用。`_sdk.py` 会显式拒绝
老的 zenoh 版 `litearm` 0.1 —— 它导入名相同但没有 `Msg` 信封。

## MuJoCo 模型 (litearm7.xml)

- 7 个铰链关节，力矩电机驱动
- 运动学链（连杆 pos/quat、关节 axis/range）与真实 LiteArm URDF 对齐
- 零位 TCP = [0, 0, 0.814] m（与真机一致）
- 质量/惯量/质心取自 URDF（总质量约 2.93 kg）
- 视觉几何：9 个真实 STL 网格，随包分发
- 电机：`gear="1"`（直接力矩控制）

## 控制器参数

默认增益与真机 MIT 跟随模式对齐：

| 关节 | kp (Nm/rad) | kd (Nm·s/rad) |
|------|-------------|----------------|
| 1-2 | 260 | 5 |
| 3-4 | 150 | 4-5 |
| 5-7 | 50 | 2.5 |

仿真循环包含重力 + 科氏力前馈补偿（`qfrc_bias`），消除稳态下垂。
PID 控制器包含积分抗饱和（anti-windup）。

## 依赖

### 已声明依赖
- `mujoco>=3.0`
- `numpy>=1.21`
- `litearm-python` — 硬依赖，未发布到 PyPI，钉在 git tag 上

### 开发依赖
- `pytest`、`pytest-timeout`

## Dev Container（开发容器）

本节面向不想在宿主机上安装 Python 和 MuJoCo 的开发者。预构建的开发容器镜像
内置 Python 3.12、MuJoCo 及其 OpenGL 运行库，并已安装本包（含 `dev` 与
`mirror` 附加依赖）。

### 环境要求

- Docker Desktop（Windows、macOS）或 Docker Engine（Linux）
- VS Code 及 Dev Containers 扩展

### 打开容器

1. 在 VS Code 中打开本仓库目录。
2. 按 `F1` 运行 **Dev Containers: Reopen in Container**。

VS Code 会先拉取预构建镜像
`ghcr.io/nexform-tech/litearm-mujoco-dev:latest`；当拉取失败时（宿主机未登录
`ghcr.io` 或没有网络），VS Code 会基于 `.devcontainer/Dockerfile` 在本地构建
同样的镜像。两条路径得到相同的环境。

不要在容器内以 root 身份执行 `pip install`。容器用户是 `vscode`，用
`sudo pip install` 会把 root 属主的文件写进挂载的工作区，可能破坏可编辑安装。

镜像按挂载的工作区路径预装了本包，因此修改源码后可编辑安装仍然生效。如果
你修改了 `pyproject.toml` 中的依赖而镜像早于该修改，请在容器内刷新安装：

```bash
pip install -e ".[dev,mirror]"
```

### 验证环境

```bash
pytest -q
python3 examples/01_hello_sim.py
```

### 使用 MuJoCo 查看器

只有当容器能访问显示器时，示例才会打开查看器窗口。没有显示器时示例照常
运行：物理循环正常工作，并打印 "Running headless" 警告。

在 Windows 上安装 VcXsrv 并关闭访问控制，然后在终端会话中设置显示：

```bash
export DISPLAY=host.docker.internal:0
python3 examples/02_movej_sim.py
```

在 Linux 上把 `DISPLAY` 指向宿主机显示（如 `export DISPLAY=:0`）。如果窗口
仍无法打开，可尝试 `export MUJOCO_GL=egl`。

不要把 `DISPLAY` 的值提交到 `devcontainer.json`。显示环境是宿主机的属性，
不是仓库的属性。

### 镜像如何构建

`.github/workflows/devcontainer.yml` 会在每个涉及 `.devcontainer/`、
`pyproject.toml` 或 `setup.cfg` 的拉取请求上构建镜像并在其中运行测试套件，
变更合入 `main` 后向 GHCR 推送 `latest` 和一个提交标签。如需手动重建，在
GitHub 上打开该工作流并点击 **Run workflow**。

## 已知限制

- 控制器为关节空间 PD + 重力/科氏力前馈，未包含完整计算力矩前馈与摩擦补偿
- 灵巧手/夹爪/示教板为模拟代理，返回固定值。litearm-python 根本没有外围总线，
  所以这些是仿真独有，将在 API 对齐时一并删除
- 系统管理/日志/遥操为 API 兼容的空实现。同上，后续对齐会删除而不是留空壳
- IK 使用阻尼最小二乘法，具有局部收敛性；对远离种子构型的大位移笛卡尔运动可能不收敛

## License

Proprietary