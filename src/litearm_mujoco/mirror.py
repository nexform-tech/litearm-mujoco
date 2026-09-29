"""DualArm — control real and simulated arms simultaneously.

Provides DualArm for sending commands to both real and simulated arms,
and MirrorMode for having the simulation track the real arm's state.

Usage::

    from litearm_mujoco import DualArm, MirrorMode

    # Mode 1: Dual control — send commands to both
    dual = DualArm(real_port="/dev/ttyACM0")
    dual.start()
    dual.movej([0.0]*7, speed=0.2)  # Both arms move!
    dual.close()

    # Mode 2: Mirror — simulation follows real arm
    import litearm_core
    real = litearm_core.Arm(port="/dev/ttyACM0").connect()
    sim = MujocoArm(render=True)
    sim.start()
    sim.mirror_from(real)  # sim tracks real arm
"""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ._sdk import sdk as litearm_core, state_q
from .arm import MujocoArm


class DualArm:
    """Control both a real arm and a simulated arm simultaneously.

    Sends the same motion commands to both arms. The simulation can be
    optionally rendered for visualization.

    Usage::

        dual = DualArm(real_port="/dev/ttyACM0", render=True)
        dual.start()

        # Both arms execute the same motion
        dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

        dual.close()
    """

    def __init__(
        self,
        real_port: Optional[str] = None,
        sim_model_path: Optional[str] = None,
        render: bool = True,
        mirror_first: bool = True,
        **sim_kwargs: Any,
    ) -> None:
        """Initialize DualArm.

        Args:
            real_port: Serial port of the real arm's CDC interface, e.g.
                ``"/dev/ttyACM0"``. ``None`` lets litearm-python auto-discover it.
                The arm is connected (and its firmware version checked) here.
            sim_model_path: Path to MuJoCo model (default: built-in).
            render: Whether to open the MuJoCo viewer.
            mirror_first: If True, start mirroring real arm state to simulation
                         before sending any commands.
            **sim_kwargs: Additional arguments passed to MujocoArm.
        """
        self._real = litearm_core.Arm(port=real_port).connect()
        self._sim = MujocoArm(model_path=sim_model_path, render=render, **sim_kwargs)
        self._sim.start()

        self._mirror_first = mirror_first
        self._mirroring = False
        self._mirror_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start(self) -> None:
        """Start the simulation and optional mirroring."""
        if self._mirror_first:
            self._start_mirroring()

    def _start_mirroring(self) -> None:
        """Start mirroring real arm state to simulation."""
        self._mirroring = True
        self._mirror_thread = threading.Thread(
            target=self._mirror_loop, daemon=True, name="dual_mirror"
        )
        self._mirror_thread.start()
        time.sleep(0.5)

    def _mirror_loop(self) -> None:
        """Continuously read real arm state and update simulation."""
        while self._mirroring:
            try:
                q = state_q(self._real.get_state())
                if q:
                    with self._lock:
                        self._sim.set_joint_positions(q)
                        self._sim._controller.set_target(np.asarray(q))
            except Exception:
                pass
            time.sleep(0.02)  # ~50 Hz

    def _pause_mirroring(self) -> None:
        self._mirroring = False

    def _resume_mirroring(self) -> None:
        self._mirroring = True

    # ── Motion Control ─────────────────────────────────────────────────────────

    def movej(
        self, q_target: List[float], speed: float = 1.0,
        settle_s: float = 1.0
    ) -> Tuple[Any, Any]:
        """Send movej to both arms.

        Returns ``(real_result, sim_result)``: a ``RobotState`` from the real
        arm (litearm-python) and, until the simulation's own API is aligned, a
        bool from the simulated one.

        ``settle_s`` is a simulation-only settling knob — litearm-python's
        ``movej`` has no equivalent.
        """
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movej(q_target, speed=speed,
                                             settle_s=settle_s)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.movej(q_target, speed=speed)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    def move_l(
        self, pose_goal: Any, speed: float = 1.0,
        settle_s: float = 0.8
    ) -> Tuple[Any, Any]:
        """Send a Cartesian straight-line move to both arms.

        Returns ``(real_result, sim_result)``, the real one being a
        litearm-python ``CartPlan``.
        """
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movel(pose_goal, speed=speed,
                                             settle_s=settle_s)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.move_l(pose_goal, speed=speed)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    def move_c(
        self, pose_via: Any, pose_goal: Any, speed: float = 1.0,
        settle_s: float = 0.8
    ) -> Tuple[Any, Any]:
        """Send a Cartesian arc to both arms.

        litearm-python requires the arc's *start* pose to match the arm's current
        TCP, so this reads it from the real arm first rather than making every
        caller do it. Returns ``(real_result, sim_result)``, the real one being
        a ``CartPlan``.
        """
        self._pause_mirroring()

        # litearm-python rejects a start pose that does not match the measured
        # TCP (6 mm / 0.03 rad), so it has to be read from the arm itself.
        pose_start = self._real.get_tcp().value

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movec(pose_via, pose_goal, speed=speed,
                                             settle_s=settle_s)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.move_c(pose_start, pose_via, pose_goal,
                                         speed=speed)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    def move_path(
        self, poses_goal: List[Any], speed: float = 1.0,
        settle_s: float = 0.8
    ) -> Tuple[Any, Any]:
        """Send a Cartesian multi-waypoint move to both arms.

        Returns ``(real_result, sim_result)``, the real one being a
        ``CartPlan``.
        """
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movep(poses_goal, speed=speed,
                                             settle_s=settle_s)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.move_path(poses_goal, speed=speed)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    # ── State ──────────────────────────────────────────────────────────────────

    def get_real_state(self) -> Any:
        """Get real arm state.

        Returns litearm-python's ``Msg`` envelope — the ``RobotState`` is at
        ``.value`` (``None`` when no frame has arrived yet).
        """
        return self._real.get_state()

    def get_sim_state(self) -> Optional[dict]:
        """Get simulation state."""
        return self._sim.get_state()

    def get_real_tcp(self) -> Optional[tuple]:
        """Get the real arm's TCP as a 6-tuple (position + rpy), or None."""
        return self._real.get_tcp().value

    def get_sim_tcp_pose(self) -> Tuple[List[float], List[List[float]]]:
        """Get simulation TCP pose."""
        return self._sim.get_tcp_pose()

    # ── Emergency Stop ─────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """Emergency stop both arms."""
        self._real.emergency_stop()
        self._sim.request_stop()

    def reset(self) -> None:
        """Clear the stop latch on both arms."""
        self._real.reset()
        self._sim.clear_stop()

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    @property
    def real(self) -> Any:
        return self._real

    @property
    def sim(self) -> MujocoArm:
        return self._sim

    def close(self) -> None:
        """Close both arms."""
        self._mirroring = False
        if self._mirror_thread and self._mirror_thread.is_alive():
            self._mirror_thread.join(timeout=2.0)
        try:
            self._sim.close()
        except Exception:
            pass
        try:
            self._real.close()
        except Exception:
            pass

    def __enter__(self) -> "DualArm":
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"DualArm(real={self._real!r}, sim={self._sim!r})"


class MirrorMode:
    """Explicit mirror mode: simulation tracks real arm.

    Usage::

        import litearm_core
        from litearm_mujoco import MujocoArm, MirrorMode

        real = litearm_core.Arm(port="/dev/ttyACM0").connect()
        sim = MujocoArm(render=True)
        sim.start()

        mirror = MirrorMode(real, sim)
        mirror.start()  # sim follows real arm

        mirror.stop()
        sim.close()
        real.close()
    """

    def __init__(
        self, real_arm: Any, sim_arm: MujocoArm, rate_hz: float = 50.0
    ) -> None:
        self._real = real_arm
        self._sim = sim_arm
        self._rate_hz = rate_hz
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start mirroring."""
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="mirror_mode"
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop mirroring."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._sim.stop_mirroring()

    def _loop(self) -> None:
        """Main mirror loop."""
        dt = 1.0 / self._rate_hz
        while self._running:
            try:
                q = state_q(self._real.get_state())
                if q:
                    self._sim.set_joint_positions(q)
                    self._sim._controller.set_target(np.asarray(q))
            except Exception:
                pass
            time.sleep(dt)