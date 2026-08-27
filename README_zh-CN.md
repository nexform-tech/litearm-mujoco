# litearm-mujoco

LiteArm 七轴机械臂的官方 MuJoCo 仿真环境。与 `litearm-python` API 完全兼容 —
将 `Arm` 替换为 `MujocoArm`，你的控制代码即可在仿真和真机上无差别运行。

## 特性

- 🔄 **API 无缝替换** — 与 `litearm-python` 接口完全一致。`MujocoArm` 可直接替换 `Arm`。
- 🖥️ **三种操作模式** — 独立仿真 / 镜像跟随 / 双控同步
- 🎮 **原生 MuJoCo 渲染** — 机械臂运动实时可视化
- 🧪 **无需硬件** — 不连接物理机械臂也可以开发和测试运动逻辑
- 🐍 **纯 Python** — 零编译。`pip install` 即用。

## 安装

```bash
# 独立仿真（无需硬件通信）
pip install litearm-mujoco

# 镜像 / 双控模式（需要硬件通信）
pip install "litearm-mujoco[mirror]"
```

或从源码安装：

```bash
git clone https://gitee.com/nexform-tech/litearm-mujoco.git
cd litearm-mujoco
pip install -e ".[dev]"
```

## 快速开始

### 模式 1 — 独立仿真

```python
from litearm_mujoco import MujocoArm

with MujocoArm(render=True) as arm:
    # 关节空间运动
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # 笛卡尔直线运动
    pos, R = arm.get_tcp_pose()
    arm.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.1)

    # 读取状态
    state = arm.get_state()
    print(state["q"])
```

### 模式 2 — 镜像模式（仿真跟随实臂）

```python
from litearm_mujoco import litearm, MujocoArm

# 连接真实机械臂
real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

# 创建仿真并启动镜像
sim = MujocoArm(render=True)
sim.start()
sim.mirror_from(real)  # 仿真实时跟随实臂运动
```

### 模式 3 — 双控模式（同时控制实臂和仿真）

```python
from litearm_mujoco import DualArm

dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
dual.start()

# 一条命令 — 实臂和仿真同时运动！
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

dual.close()
```

## 架构

```
┌─────────────────────────────────────────────────┐
│                  你的 Python 程序                │
│                                                   │
│   arm = MujocoArm()  ← 可替换为 litearm.Arm      │
│   arm.movej(...)                                  │
│   arm.get_state()                                 │
└──────────┬────────────────────┬─────────────────┘
           │                    │
    ┌──────▼──────┐      ┌─────▼──────────┐
    │ 独立仿真     │      │ 双控 / 镜像     │
    │             │      │                │
    │ MuJoCo      │      │ MuJoCo + Zenoh │
    │ 物理引擎    │      │ → litearm-     │
    │             │      │   server       │
    │ PID 控制器  │      │                │
    │ FK/IK      │      │  实臂 + 仿真   │
    │             │      │  同时运动      │
    └─────────────┘      └────────────────┘
```

## 示例

| 示例 | 说明 | 需要实臂 |
|------|------|:---:|
| `01_hello_sim.py` | 创建仿真 + 读取状态 | ❌ |
| `02_movej_sim.py` | 关节与笛卡尔运动 + FK/IK | ❌ |
| `03_trajectory.py` | 轨迹录制与回放 | ❌ |
| `04_mirror_real.py` | 仿真镜像跟随实臂 | ✅ |
| `05_dual_control.py` | 同时控制实臂和仿真 | ✅ |

```bash
python3 examples/01_hello_sim.py
python3 examples/02_movej_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --endpoint tcp/192.168.31.139:7447
python3 examples/05_dual_control.py --endpoint tcp/192.168.31.139:7447
```

## API 对照

| litearm.Arm | MujocoArm | 说明 |
|-------------|-----------|------|
| `Arm(endpoint=...)` | `MujocoArm(render=True)` | 构造实例 |
| `movej(q, speed)` | `movej(q, speed)` | ✅ 完全一致 |
| `movel(pose, speed)` | `movel(pose, speed)` | ✅ 完全一致 |
| `movec(via, goal)` | `movec(via, goal)` | ✅ 完全一致 |
| `movep(poses)` | `movep(poses)` | ✅ 完全一致 |
| `get_state()` | `get_state()` | ✅ 完全一致 |
| `get_tcp_pose()` | `get_tcp_pose()` | ✅ 完全一致 |
| `fk(q)` | `fk(q)` | ✅ 完全一致 |
| `ik(pos, R)` | `ik(pos, R)` | ✅ 完全一致 |
| `replay_joint_path(path)` | `replay_joint_path(path)` | ✅ 完全一致 |
| `replay_trajectory(traj)` | `replay_trajectory(traj)` | ✅ 完全一致 |
| `record_trajectory()` | `record_trajectory()` | ✅ 模拟实现 |
| `hold()` / `zero_gravity()` | `hold()` / `zero_gravity()` | ✅ 完全一致 |
| `request_stop()` | `request_stop()` | ✅ 完全一致 |
| `enable/disable()` | `enable/disable()` | ✅ 完全一致 |
| `set_gains(kp, kd)` | `set_gains(kp, kd)` | ✅ 完全一致 |
| `device("hand_0")` | `device("hand_0")` | ✅ 模拟代理 |
| `arm.hand.open()` | `arm.hand.open()` | ✅ 模拟代理 |

## 开发

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## License

Proprietary

---

[English README](README.md) | [开发者指南](docs/DEVELOPER_GUIDE.md)