"""MujocoArm — MuJoCo simulation of the LiteArm 7-DOF robot.

API-compatible with litearm.Arm. You can swap between real and simulated arms
without changing your control code:

    # Real arm
    arm = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

    # Simulation
    arm = MujocoArm()

    # Same API for both:
    arm.movej([0.0]*7, speed=0.5)
    state = arm.get_state()
    arm.close()
"""
from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import mujoco
import numpy as np

from .controller import DEFAULT_KD, DEFAULT_KP, N_JOINTS, JointPIDController, TrajectoryGenerator
from .kinematics import Kinematics

# Path to the default MuJoCo model
_ASSETS_DIR = Path(__file__).parent / "assets"
_DEFAULT_MODEL = _ASSETS_DIR / "litearm7.xml"


def _resolve_model_path(model_path: Optional[str] = None) -> str:
    """Resolve the model path, trying multiple locations."""
    if model_path is not None and os.path.exists(model_path):
        return model_path

    candidates = [
        _DEFAULT_MODEL,
        Path("src/litearm_mujoco/assets/litearm7.xml"),
        Path("assets/litearm7.xml"),
    ]

    for p in candidates:
        if p.exists():
            return str(p)

    raise FileNotFoundError(
        f"MuJoCo model not found. Tried: {[str(c) for c in candidates]}. "
        "Provide model_path= explicitly."
    )


class MujocoArm:
    """MuJoCo simulation of LiteArm 7-DOF robot arm.

    API-compatible with litearm.Arm. All motion methods are blocking (they run
    the simulation until the motion completes), matching the real arm's behavior.

    Usage::

        # Standalone simulation
        with MujocoArm(render=True) as arm:
            arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)
            state = arm.get_state()
            print(state["q"])

        # Mirror real arm
        import litearm
        real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
        sim = MujocoArm(render=True)
        sim.start()
        sim.mirror_from(real)  # sim follows real arm state
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        render: bool = False,
        dt: float = 0.002,
        kp: Union[float, List[float]] = DEFAULT_KP,
        kd: Union[float, List[float]] = DEFAULT_KD,
        max_velocity: float = 3.0,
        n_joints: int = N_JOINTS,
        ee_site_name: str = "tcp",
        key_callback: Optional[Callable[[int], None]] = None,
    ) -> None:
        """Initialize the MuJoCo simulation.

        Args:
            model_path: Path to MJCF/XML model file. Default: built-in litearm7.xml.
            render: Whether to open the MuJoCo viewer window.
            dt: Physics timestep (seconds).
            kp: Position gain for joint controller.
            kd: Velocity gain for joint controller.
            max_velocity: Maximum joint velocity (rad/s).
            n_joints: Number of joints (default 7).
            ee_site_name: Name of the end-effector site in the model.
        """
        model_path = _resolve_model_path(model_path)

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._kin_data = mujoco.MjData(self._model)  # Separate data for FK/IK (thread-safe)
        self._n_joints = n_joints
        self._ee_site_name = ee_site_name
        self._dt = dt
        self._model.opt.timestep = dt

        # Controllers
        self._controller = JointPIDController(kp=kp, kd=kd, n_joints=n_joints)
        self._traj_gen = TrajectoryGenerator(max_velocity=max_velocity, dt=dt)
        self._kinematics = Kinematics(
            self._model, self._kin_data, ee_site_name=ee_site_name, n_joints=n_joints
        )

        # State
        self._lock = threading.Lock()
        self._sim_running = False
        self._sim_thread: Optional[threading.Thread] = None
        self._stopped = False
        self._enabled = True

        # Mirror mode
        self._mirror_arm: Optional[Any] = None
        self._mirror_thread: Optional[threading.Thread] = None
        self._mirroring = False

        # Simulated devices
        self._devices: Optional[Any] = None

        # Viewer
        self._viewer: Optional[Any] = None
        self._render = render
        self._key_callback = key_callback
        if render:
            self._start_viewer()

        self._controller.set_target(np.zeros(n_joints))
        self._state_cache: Optional[dict] = None

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the background simulation thread."""
        if self._sim_running:
            return
        self._sim_running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, daemon=True, name="mujoco_sim"
        )
        self._sim_thread.start()

    def close(self) -> None:
        """Stop simulation and close viewer."""
        self._sim_running = False
        self._mirroring = False

        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        if self._mirror_thread and self._mirror_thread.is_alive():
            self._mirror_thread.join(timeout=2.0)
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def __enter__(self) -> "MujocoArm":
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"MujocoArm(n_joints={self._n_joints}, render={self._render})"

    # ── Simulation Loop ────────────────────────────────────────────────────────

    def _sim_loop(self) -> None:
        """Background simulation loop running at 1/dt Hz."""
        while self._sim_running:
            loop_start = time.time()

            with self._lock:
                # Mirror mode: read real arm state and set as target
                if self._mirroring and self._mirror_arm is not None:
                    try:
                        real_state = self._mirror_arm.get_state()
                        if real_state and real_state.get("q"):
                            q_real = np.asarray(real_state["q"], dtype=float)[:self._n_joints]
                            self._controller.set_target(q_real)
                    except Exception:
                        pass

                # Compute control
                q_actual = self._data.qpos[:self._n_joints].copy()
                dq_actual = self._data.qvel[:self._n_joints].copy()

                if self._enabled and not self._stopped:
                    # Gravity + Coriolis feed-forward (computed-torque style)
                    ff_torque = self._data.qfrc_bias[:self._n_joints].copy()
                    tau = self._controller.compute(
                        q_actual, dq_actual, self._dt, ff_torque=ff_torque
                    )
                else:
                    tau = np.zeros(self._n_joints)

                self._data.ctrl[:self._n_joints] = tau

                # Step physics
                mujoco.mj_step(self._model, self._data)

                # Update state cache
                self._state_cache = self._build_state_dict()

                # Render
                if self._viewer is not None and self._viewer.is_running():
                    self._viewer.sync()

            # Real-time sync
            elapsed = time.time() - loop_start
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    def _start_viewer(self) -> None:
        """Launch the MuJoCo passive viewer."""
        try:
            from mujoco import viewer
            self._viewer = viewer.launch_passive(
                self._model, self._data, key_callback=self._key_callback,
            )
        except Exception:
            print("[MujocoArm] Warning: Could not launch viewer. Running headless.")
            self._viewer = None

    # ── State Reading ──────────────────────────────────────────────────────────

    def get_state(self, refresh: bool = False) -> Optional[dict]:
        """Get latest robot state.

        Returns state dict matching litearm.Arm.get_state() format:
        {q, dq, tau, fault, errs, temps, state, feedback, watchdog, ...}
        """
        with self._lock:
            if self._state_cache is None:
                self._state_cache = self._build_state_dict()
            return dict(self._state_cache)

    def _build_state_dict(self) -> dict:
        """Build a state dict from current MuJoCo data."""
        q = self._data.qpos[:self._n_joints].tolist()
        dq = self._data.qvel[:self._n_joints].tolist()
        tau = self._data.qfrc_actuator[:self._n_joints].tolist()

        state_str = "ready"
        if self._stopped:
            state_str = "stopping"
        elif not self._enabled:
            state_str = "disconnected"

        return {
            "q": q,
            "dq": dq,
            "tau": tau,
            "fault": [],
            "errs": [0] * self._n_joints,
            "temps": [(i, 25) for i in range(self._n_joints)],
            "state": state_str,
            "feedback": {
                "max_age_s": 0.001,
                "joints": [
                    {"joint": i, "received": 1000, "age_s": 0.001, "fresh": True}
                    for i in range(self._n_joints)
                ],
                "stale_joints": [],
            },
            "watchdog": {
                "enabled": True,
                "timeout_s": 0.5,
                "mode": "stop",
                "tripped": self._stopped,
                "last_kick_age_s": 0.001,
            },
            "robot_serial": "MUJOCO-SIM-001",
            "config_checksum_sha256": "sim",
        }

    def get_tcp_pose(self) -> Tuple[List[float], List[List[float]]]:
        """Get current TCP pose as (position, rotation_matrix)."""
        with self._lock:
            return self._kinematics.fk(self._data.qpos[:self._n_joints].tolist())

    # ── Pure Computation (no simulation step needed) ───────────────────────────

    def fk(self, q: List[float]) -> Tuple[List[float], List[List[float]]]:
        """Forward kinematics: joint angles → (position, rotation_matrix)."""
        return self._kinematics.fk(q)

    def ik(
        self,
        pos_d: List[float],
        R_d: List[List[float]],
        q_seed: Optional[List[float]] = None,
    ) -> Tuple[List[float], bool]:
        """Inverse kinematics: (position, rotation) → (q, success)."""
        return self._kinematics.ik(pos_d, R_d, q_seed)

    def plan_movel(
        self, q_start: List[float], pose_goal: Any
    ) -> List[List[float]]:
        """Plan a straight-line Cartesian path."""
        return self._kinematics.plan_movel(q_start, pose_goal)

    def plan_movec(
        self, q_start: List[float], pose_via: Any, pose_goal: Any
    ) -> List[List[float]]:
        """Plan a circular-arc Cartesian path."""
        return self._kinematics.plan_movec(q_start, pose_via, pose_goal)

    def plan_movep(
        self, q_start: List[float], poses_goal: List[Any]
    ) -> List[List[float]]:
        """Plan a multi-waypoint Cartesian path."""
        return self._kinematics.plan_movep(q_start, poses_goal)

    # ── Motion Control (blocking) ──────────────────────────────────────────────

    def movej(
        self,
        q_target: List[float],
        speed: float = 1.0,
        settle_s: float = 1.0,
        max_cycles: Optional[int] = None,
        allow_start_collision_recovery: bool = False,
        **kwargs: Any,
    ) -> bool:
        """Move to joint target (blocking)."""
        self._ensure_running()
        q_target = np.asarray(q_target, dtype=float)[:self._n_joints]

        with self._lock:
            q_current = self._data.qpos[:self._n_joints].copy()

        traj = self._traj_gen.linear_trajectory(q_current, q_target, speed=speed)
        traj_duration = len(traj) * self._dt / speed

        self._controller.set_target(q_target)

        total_wait = traj_duration + settle_s
        settle_start = time.time()
        while time.time() - settle_start < total_wait:
            self._control_sleep_with_abort(self._dt)
            if self._stopped:
                return False

        return True

    def recover_joint_limits(
        self,
        speed: float = 0.05,
        settle_s: float = 0.5,
        max_cycles: Optional[int] = None,
        inset_rad: float = 0.0,
        **kwargs: Any,
    ) -> bool:
        """Slowly return out-of-limit joints to safe boundaries."""
        self._ensure_running()
        with self._lock:
            q_current = self._data.qpos[:self._n_joints].copy()

        q_safe = q_current.copy()
        for j in range(self._n_joints):
            lo = self._model.jnt_range[j][0] + inset_rad
            hi = self._model.jnt_range[j][1] - inset_rad
            if lo < hi:
                q_safe[j] = np.clip(q_safe[j], lo, hi)

        if np.allclose(q_current, q_safe):
            return True

        return self.movej(q_safe.tolist(), speed=speed, settle_s=settle_s)

    def movel(
        self,
        pose_goal: Any,
        speed: float = 1.0,
        settle_s: float = 0.8,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Move in a straight Cartesian line (blocking)."""
        self._ensure_running()

        pos_goal, R_goal = pose_goal
        pos_goal = np.asarray(pos_goal, dtype=float)
        R_goal = np.asarray(R_goal, dtype=float)

        with self._lock:
            q_current = self._data.qpos[:self._n_joints].tolist()

        path = self._kinematics.plan_movel(q_current, (pos_goal, R_goal), num_waypoints=50)

        for q_des in path:
            self._control_sleep_with_abort(self._dt * 10)
            self._controller.set_target(np.asarray(q_des))
            if self._stopped:
                return False

        self._controller.set_target(np.asarray(path[-1]))
        settle_start = time.time()
        while time.time() - settle_start < settle_s:
            self._control_sleep_with_abort(self._dt)
            if self._stopped:
                return False

        return True

    def movec(
        self,
        pose_via: Any,
        pose_goal: Any,
        speed: float = 1.0,
        settle_s: float = 0.8,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Move in a circular arc through via-point (blocking)."""
        self._ensure_running()

        with self._lock:
            q_current = self._data.qpos[:self._n_joints].tolist()

        path = self._kinematics.plan_movec(q_current, pose_via, pose_goal)

        for q_des in path:
            self._control_sleep_with_abort(self._dt * 10)
            self._controller.set_target(np.asarray(q_des))
            if self._stopped:
                return False

        settle_start = time.time()
        while time.time() - settle_start < settle_s:
            self._control_sleep_with_abort(self._dt)
            if self._stopped:
                return False

        return True

    def movep(
        self,
        poses_goal: List[Any],
        speed: float = 1.0,
        settle_s: float = 0.8,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Move through multiple Cartesian waypoints (blocking)."""
        self._ensure_running()

        with self._lock:
            q_current = self._data.qpos[:self._n_joints].tolist()

        path = self._kinematics.plan_movep(q_current, poses_goal)

        for q_des in path:
            self._control_sleep_with_abort(self._dt * 10)
            self._controller.set_target(np.asarray(q_des))
            if self._stopped:
                return False

        settle_start = time.time()
        while time.time() - settle_start < settle_s:
            self._control_sleep_with_abort(self._dt)
            if self._stopped:
                return False

        return True

    def replay_joint_path(
        self,
        q_path: List[List[float]],
        speed: float = 1.0,
        settle_s: float = 0.5,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Replay a sequence of joint configurations."""
        self._ensure_running()

        if goto_start and len(q_path) > 0:
            self.movej(q_path[0], speed=goto_speed)

        for q_des in q_path:
            self._control_sleep_with_abort(self._dt / speed)
            self._controller.set_target(np.asarray(q_des))
            if self._stopped:
                return False

        settle_start = time.time()
        while time.time() - settle_start < settle_s:
            self._control_sleep_with_abort(self._dt)
            if self._stopped:
                return False

        return True

    def replay_trajectory(
        self,
        traj_q: Any,
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        max_cycles: Optional[int] = None,
        check_singularity: bool = True,
        **kwargs: Any,
    ) -> bool:
        """Replay a JointTrajectory or path."""
        self._ensure_running()

        if hasattr(traj_q, 'q'):
            q_path = traj_q.q
        elif hasattr(traj_q, 'to_dict'):
            d = traj_q.to_dict()
            q_path = [f["q"] for f in d.get("frames", [])]
        elif isinstance(traj_q, dict):
            frames = traj_q.get("frames", [])
            q_path = [f["q"] for f in frames]
        else:
            q_path = traj_q

        return self.replay_joint_path(
            q_path, speed=speed, settle_s=0.5,
            goto_start=goto_start, goto_speed=goto_speed,
        )

    def replay_timed_trajectory(
        self,
        traj_q: List[List[float]],
        traj_t: List[float],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        simplify_tolerance_rad: float = 0.01,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Replay a measured trajectory on its recorded time axis."""
        self._ensure_running()

        if goto_start and len(traj_q) > 0:
            self.movej(traj_q[0], speed=goto_speed)

        if len(traj_t) < 2:
            return self.replay_joint_path(traj_q, speed=speed)

        t0 = traj_t[0]
        for q_des, t in zip(traj_q, traj_t):
            dt = (t - t0) / speed
            t0 = t
            self._control_sleep_with_abort(max(dt, self._dt))
            self._controller.set_target(np.asarray(q_des))
            if self._stopped:
                return False

        return True

    def play_trajectory(
        self,
        trajectory: Union[Any, str],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        verify_robot: bool = True,
        simplify_tolerance_rad: float = 0.01,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Load and replay a saved trajectory."""
        try:
            from ._litearm.types import JointTrajectory
        except ImportError:
            raise ImportError(
                "play_trajectory with file path requires litearm-mujoco[mirror] dependencies. "
                "Install with: pip install litearm-mujoco[mirror]"
            )

        if isinstance(trajectory, str):
            traj = JointTrajectory.load(trajectory)
        elif isinstance(trajectory, JointTrajectory):
            traj = trajectory
        else:
            traj = JointTrajectory.from_dict(trajectory)

        return self.replay_trajectory(traj, speed=speed, goto_start=goto_start,
                                      goto_speed=goto_speed)

    def record_trajectory(
        self,
        output: str = "trajectories",
        duration_s: Optional[float] = None,
        sample_rate_hz: float = 100.0,
        filter_alpha: float = 0.15,
        name: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """Record a trajectory (simulated)."""
        try:
            from ._litearm.types import JointTrajectory, TrajectoryFrame
        except ImportError:
            raise ImportError(
                "record_trajectory requires litearm-mujoco[mirror] dependencies. "
                "Install with: pip install litearm-mujoco[mirror]"
            )

        self._ensure_running()

        if duration_s is None:
            duration_s = 5.0

        dt_sample = 1.0 / sample_rate_hz
        n_samples = int(duration_s / dt_sample)
        frames = []

        old_enabled = self._enabled
        self._enabled = False

        t_start = time.time()
        for i in range(n_samples):
            target_t = t_start + (i + 1) * dt_sample
            sleep_t = target_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)

            with self._lock:
                q = self._data.qpos[:self._n_joints].tolist()
                dq = self._data.qvel[:self._n_joints].tolist()

            frames.append(TrajectoryFrame(
                t=i * dt_sample, q=q, dq=dq,
            ))

        self._enabled = old_enabled

        return JointTrajectory(
            frames=frames, name=name or "sim_recording",
            sample_rate_hz=sample_rate_hz, filter_alpha=filter_alpha,
        )

    def hold(
        self,
        kp_scale: float = 3.0,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Hold current position with increased stiffness."""
        self._ensure_running()
        with self._lock:
            q_current = self._data.qpos[:self._n_joints].copy()
        self._controller.set_target(q_current)
        return True

    def zero_gravity(
        self,
        max_cycles: Optional[int] = None,
        duration_s: Optional[float] = None,
        measured_overspeed_factor: Optional[float] = None,
        vel_max: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> bool:
        """Enable zero-gravity (free-drag) mode."""
        self._ensure_running()
        self._enabled = False
        if duration_s is not None:
            time.sleep(duration_s)
            self._enabled = True
        return True

    def joint_impedance(
        self,
        q_des: List[float],
        K: Any,
        B: Any,
        tau_max: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Joint-space impedance control (simplified)."""
        self._ensure_running()
        self._controller.set_target(np.asarray(q_des, dtype=float)[:self._n_joints])
        return True

    def cartesian_impedance(
        self,
        q_des: List[float],
        K_cart: Any,
        B_cart: Any,
        v_des: Optional[Any] = None,
        tau_max: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        sigma_min_thresh: Optional[float] = None,
        max_ori_err: Optional[float] = None,
        measured_overspeed_factor: Optional[float] = None,
        vel_max: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> bool:
        """Cartesian-space impedance control (simplified)."""
        self._ensure_running()
        self._controller.set_target(np.asarray(q_des, dtype=float)[:self._n_joints])
        return True

    def joint_follow(
        self,
        K: Optional[Any] = None,
        B: Optional[Any] = None,
        speed_limit: Optional[Any] = None,
        accel_limit: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        duration_s: Optional[float] = None,
        **kwargs: Any,
    ) -> bool:
        """Follow an external target provider."""
        self._ensure_running()
        return True

    # ── Emergency Stop ─────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        """Emergency stop the simulation."""
        self._stopped = True
        with self._lock:
            self._data.ctrl[:self._n_joints] = 0.0

    def clear_stop(self) -> None:
        """Clear the stop condition."""
        self._stopped = False

    # ── Enable / Disable ───────────────────────────────────────────────────────

    def enable(self) -> None:
        """Enable motors and hold current pose."""
        self._enabled = True
        self._stopped = False
        with self._lock:
            q_current = self._data.qpos[:self._n_joints].copy()
        self._controller.set_target(q_current)

    def disable(self) -> None:
        """Disable motors (arm will drop under gravity)."""
        self._enabled = False
        with self._lock:
            self._data.ctrl[:self._n_joints] = 0.0

    # ── Parameter Tuning ───────────────────────────────────────────────────────

    def set_gains(self, kp: Optional[Any] = None, kd: Optional[Any] = None) -> Dict:
        """Set PD controller gains."""
        if kp is not None:
            self._controller.kp = np.broadcast_to(
                np.atleast_1d(np.asarray(kp, dtype=float)), self._n_joints
            ).copy()
        if kd is not None:
            self._controller.kd = np.broadcast_to(
                np.atleast_1d(np.asarray(kd, dtype=float)), self._n_joints
            ).copy()
        return {"kp": self._controller.kp.tolist(), "kd": self._controller.kd.tolist()}

    def get_gains(self) -> Dict:
        """Get current PD gains."""
        return {"kp": self._controller.kp.tolist(), "kd": self._controller.kd.tolist()}

    def clear_faults(self) -> List[Tuple[int, int]]:
        """Clear motor faults (no-op in simulation)."""
        return []

    def set_payload(
        self, mass: float, com: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ) -> Dict:
        return {"mass": mass, "com": list(com)}

    def get_payload(self) -> Dict:
        return {"mass": 0.0, "com": [0.0, 0.0, 0.0]}

    def set_installation(
        self,
        base_rpy: Optional[List[float]] = None,
        gravity: Optional[List[float]] = None,
    ) -> Dict:
        return {"base_rpy": base_rpy, "gravity": gravity}

    def get_installation(self) -> Dict:
        return {"base_rpy": [0, 0, 0], "gravity": [0, 0, -9.81]}

    # ── System / Settings / Trajectory / Device / Teleop ───────────────────────

    def get_system_stats(self) -> Dict[str, Any]:
        return {"cpu": 0.0, "memory": 0.0, "board_temp": 25.0, "uptime": 0.0}

    def get_logs(self, page: int = 1, size: int = 50, search: str = "") -> Dict[str, Any]:
        return {"logs": [], "total": 0, "page": page, "size": size}

    def restart_service(self) -> Dict[str, Any]:
        return {"status": "ok"}

    def get_joint_limits(self) -> Dict[str, Any]:
        limits = {}
        for j in range(self._n_joints):
            limits[f"joint{j}"] = {
                "min": float(self._model.jnt_range[j][0]),
                "max": float(self._model.jnt_range[j][1]),
            }
        return limits

    def set_joint_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        return limits

    def get_zero_offsets(self) -> Dict[str, Any]:
        return {"offsets": [0.0] * self._n_joints}

    def set_zero_offsets(self, offsets: Dict[str, Any]) -> Dict[str, Any]:
        return offsets

    def get_end_effector(self) -> Dict[str, Any]:
        return {"type": "none"}

    def set_end_effector(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return config

    def get_cartesian_limits(self) -> Dict[str, Any]:
        return {"x": [-1, 1], "y": [-1, 1], "z": [0, 1.5]}

    def set_cartesian_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        return limits

    def get_collision_config(self) -> Dict[str, Any]:
        return {}

    def set_collision_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return config

    def start_recording(self) -> Dict[str, Any]:
        return {"status": "recording"}

    def stop_recording(self) -> Dict[str, Any]:
        return {"status": "stopped"}

    def discard_recording(self) -> Dict[str, Any]:
        return {"status": "discarded"}

    def get_recording_state(self) -> Dict[str, Any]:
        return {"recording": False}

    def get_playback_state(self) -> Dict[str, Any]:
        return {"playing": False}

    def list_trajectories(self) -> Dict[str, Any]:
        return {"trajectories": []}

    def save_trajectory(
        self, id: str, name: str, points: List[List[float]],
        duration: Optional[float] = None,
    ) -> Dict[str, Any]:
        return {"id": id, "name": name, "saved": True}

    def delete_trajectory(self, id: str) -> Dict[str, Any]:
        return {"id": id, "deleted": True}

    def list_device_types(self) -> List[Dict[str, Any]]:
        return [
            {"category": "hand", "subtype": "sim_hand", "label": "Simulated Hand"},
            {"category": "gripper", "subtype": "sim_gripper", "label": "Simulated Gripper"},
        ]

    def connect_device(
        self, category: str, subtype: str, device_id: str = "end_0",
        can_iface: str = "", config: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        return {"device_id": device_id, "connected": True}

    def disconnect_device(self, device_id: str = "end_0") -> Dict[str, Any]:
        return {"device_id": device_id, "disconnected": True}

    def get_active_device(self, device_id: str = "end_0") -> Dict[str, Any]:
        return {"device_id": device_id, "active": False}

    def enter_teleop(self, mode: str, **params: Any) -> Dict[str, Any]:
        return {"mode": mode, "active": True}

    def exit_teleop(self) -> Dict[str, Any]:
        return {"active": False}

    def get_teleop_status(self) -> Dict[str, Any]:
        return {"active": False, "mode": "none"}

    # ── Device (for API compatibility) ─────────────────────────────────────────

    def device(self, device_id: str) -> Any:
        """Return a simulated device proxy."""
        if self._devices is None:
            self._devices = _SimDeviceManager()
        return self._devices.get(device_id)

    @property
    def devices(self) -> Any:
        """Device manager (simulated). Supports ``arm.devices["hand_0"]`` syntax."""
        if self._devices is None:
            self._devices = _SimDeviceManager()
        return self._devices

    @property
    def hand(self) -> Any:
        """Backward-compatible hand property."""
        return self.device("hand_0")

    # ── Mirror Mode ────────────────────────────────────────────────────────────

    def mirror_from(self, real_arm: Any, rate_hz: float = 50.0) -> None:
        """Start mirroring the state of a real arm into this simulation.

        Args:
            real_arm: A litearm.Arm instance connected to a real robot.
            rate_hz: Mirroring update rate (Hz).

        Usage::

            import litearm
            real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
            sim = MujocoArm(render=True)
            sim.start()
            sim.mirror_from(real)
            # Now sim follows real arm's motion
        """
        self._mirror_arm = real_arm
        self._mirroring = True
        self._ensure_running()

    def stop_mirroring(self) -> None:
        """Stop mirroring the real arm."""
        self._mirroring = False
        self._mirror_arm = None

    # ── Internal Helpers ───────────────────────────────────────────────────────

    def _ensure_running(self) -> None:
        """Start the simulation thread if not already running."""
        if not self._sim_running:
            self.start()
            time.sleep(0.05)

    def _control_sleep_with_abort(self, duration: float) -> None:
        """Sleep for a duration, but abort early if stopped."""
        if duration <= 0:
            return
        check_interval = min(duration, 0.01)
        end = time.time() + duration
        while time.time() < end:
            if self._stopped:
                return
            time.sleep(min(check_interval, end - time.time()))

    def set_joint_positions(self, q: List[float]) -> None:
        """Directly set joint positions (for mirror mode)."""
        q_arr = np.asarray(q, dtype=float)[:self._n_joints]
        with self._lock:
            self._data.qpos[:self._n_joints] = q_arr
            self._data.qvel[:self._n_joints] = 0.0
            mujoco.mj_forward(self._model, self._data)
            self._controller.set_target(q_arr)


class _SimDevice:
    """Simulated device proxy (returns canned responses)."""

    def __init__(self, device_id: str) -> None:
        self._device_id = device_id

    @property
    def device_id(self) -> str:
        return self._device_id

    def open(self) -> bool: return True
    def close(self) -> bool: return True
    def set_gesture(self, gesture: str) -> bool: return True
    def list_gestures(self) -> list: return ["open", "close", "pinch"]
    def set_force(self, force: float) -> bool: return True
    def get_state(self) -> dict: return {"connected": True}
    def get_status(self) -> dict: return {"ok": True}
    def get_info(self) -> dict: return {"type": "sim"}
    def connect(self) -> bool: return True
    def disconnect(self) -> bool: return True
    def clear_faults(self) -> bool: return True
    def finger_move(self, pose: list) -> bool: return True
    def set_speed(self, speed: list) -> bool: return True
    def set_torque(self, torque: list) -> bool: return True
    def set_width(self, width: float) -> bool: return True
    def get_width(self) -> float: return 0.5
    def get_joints(self) -> list: return [0.0] * 7
    def get_buttons(self) -> dict: return {}

    def __repr__(self) -> str:
        return f"_SimDevice({self._device_id!r})"


class _SimDeviceManager:
    """Simulated device manager (API-compatible with litearm.DeviceManager).

    Supports ``arm.devices["hand_0"]`` and ``arm.devices.get("hand_0")``.
    """

    def __init__(self) -> None:
        self._devices: dict = {}

    def get(self, device_id: str) -> "_SimDevice":
        if device_id not in self._devices:
            self._devices[device_id] = _SimDevice(device_id)
        return self._devices[device_id]

    def __getitem__(self, device_id: str) -> "_SimDevice":
        return self.get(device_id)

    def __contains__(self, device_id: str) -> bool:
        return device_id in self._devices

    def __repr__(self) -> str:
        return f"_SimDeviceManager({list(self._devices.keys())})"