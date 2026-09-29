"""litearm-mujoco: LiteArm 七轴机械臂 MuJoCo 仿真。

与 litearm-python API 兼容，支持三种模式：

1. 独立仿真：替换 litearm_core.Arm，无需连接真实机械臂
2. 镜像模式：仿真跟随真实机械臂状态同步运动
3. 双控模式：同时向真实机械臂和仿真发送指令

Usage::

    from litearm_mujoco import MujocoArm, DualArm, MirrorMode

    # 模式 1: 独立仿真
    with MujocoArm(render=True) as arm:
        arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)
        state = arm.get_state()

    # 模式 2: 双控（port 省略时由 litearm-python 自动发现 CDC 串口）
    dual = DualArm(real_port="/dev/ttyACM0", render=True)
    dual.start()
    dual.movej([0.0]*7, speed=0.2)  # 实臂 + 仿真同时运动
    dual.close()

    # 模式 3: 镜像
    import litearm_core
    real = litearm_core.Arm(port="/dev/ttyACM0").connect()
    sim = MujocoArm(render=True)
    sim.start()
    sim.mirror_from(real)  # 仿真跟随实臂
"""

__version__ = "0.3.0"

from .arm import MujocoArm
from .mirror import DualArm, MirrorMode
from ._sdk import sdk as litearm_core

__all__ = [
    "__version__",
    "MujocoArm",
    "DualArm",
    "MirrorMode",
    "litearm_core",
]