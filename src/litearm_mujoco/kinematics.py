"""Forward and inverse kinematics using MuJoCo.

Provides FK/IK/planning functions that match the litearm.Arm API:
- fk(q) → (position, rotation_matrix)
- ik(pos, R, q_seed) → (q, success)
- plan_movel / plan_movec / plan_movep
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import mujoco
import numpy as np


def _quat_to_rotmat(quat: np.ndarray) -> np.ndarray:
    """Convert [w, x, y, z] quaternion to 3x3 rotation matrix."""
    w, x, y, z = quat
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y],
    ])


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to [w, x, y, z] quaternion."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return np.array([w, x, y, z])


def _rotation_error(R_current: np.ndarray, R_desired: np.ndarray) -> np.ndarray:
    """Compute rotation error as angular velocity (3-vector).

    Uses the log map of SO(3): the angular velocity that would rotate
    R_current to R_desired in unit time.
    """
    R_err = R_desired @ R_current.T
    trace = np.clip(R_err[0, 0] + R_err[1, 1] + R_err[2, 2], -1.0, 3.0)
    theta = np.arccos((trace - 1.0) / 2.0)

    if abs(theta) < 1e-10:
        return np.zeros(3)

    w_hat = (R_err - R_err.T) / (2.0 * np.sin(theta))
    omega = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])
    return omega * theta


def _pose_to_list(
    pos: np.ndarray, rot: np.ndarray
) -> Tuple[List[float], List[List[float]]]:
    """Convert numpy arrays to list format matching litearm.Arm."""
    return pos.tolist(), rot.tolist()


class Kinematics:
    """Forward and inverse kinematics using MuJoCo.

    Usage::

        kin = Kinematics(model, data, ee_site_name="tcp")
        pos, R = kin.fk(q)
        q_sol, ok = kin.ik(pos_d, R_d, q_seed)
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        ee_site_name: str = "tcp",
        n_joints: int = 7,
    ) -> None:
        self._model = model
        self._data = data
        self._ee_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name
        )
        self._n_joints = n_joints

    def fk(self, q: List[float]) -> Tuple[List[float], List[List[float]]]:
        """Forward kinematics: joint angles → (position, rotation_matrix)."""
        q_arr = np.asarray(q, dtype=float)[:self._n_joints]

        saved_qpos = self._data.qpos.copy()
        self._data.qpos[:self._n_joints] = q_arr
        mujoco.mj_forward(self._model, self._data)

        pos = self._data.site_xpos[self._ee_site_id].copy()
        rot = self._data.site_xmat[self._ee_site_id].reshape(3, 3).copy()

        self._data.qpos[:] = saved_qpos
        mujoco.mj_forward(self._model, self._data)

        return _pose_to_list(pos, rot)

    def ik(
        self,
        pos_d: List[float],
        R_d: List[List[float]],
        q_seed: Optional[List[float]] = None,
        max_iter: int = 200,
        tol_pos: float = 1e-4,
        tol_rot: float = 1e-4,
        damping: float = 0.1,
    ) -> Tuple[List[float], bool]:
        """Inverse kinematics using damped least squares.

        Args:
            pos_d: Desired position [x, y, z].
            R_d: Desired 3x3 rotation matrix.
            q_seed: Initial guess (default: current config).
            max_iter: Maximum iterations.
            tol_pos: Position tolerance (m).
            tol_rot: Rotation tolerance (rad).
            damping: Damping factor for Levenberg-Marquardt.

        Returns:
            (q_solution, success).
        """
        pos_d = np.asarray(pos_d, dtype=float)
        R_d = np.asarray(R_d, dtype=float)

        if q_seed is None:
            q = self._data.qpos[:self._n_joints].copy()
        else:
            q = np.asarray(q_seed, dtype=float)[:self._n_joints].copy()

        saved_qpos = self._data.qpos.copy()

        for _ in range(max_iter):
            self._data.qpos[:self._n_joints] = q
            mujoco.mj_forward(self._model, self._data)

            pos = self._data.site_xpos[self._ee_site_id].copy()
            R = self._data.site_xmat[self._ee_site_id].reshape(3, 3).copy()

            pos_err = pos_d - pos
            rot_err = _rotation_error(R, R_d)
            err = np.concatenate([pos_err, rot_err])

            if (
                np.linalg.norm(pos_err) < tol_pos
                and np.linalg.norm(rot_err) < tol_rot
            ):
                self._data.qpos[:] = saved_qpos
                mujoco.mj_forward(self._model, self._data)
                return q.tolist(), True

            # Compute Jacobian
            jacp = np.zeros((3, self._model.nv))
            jacr = np.zeros((3, self._model.nv))
            mujoco.mj_jacSite(self._model, self._data, jacp, jacr, self._ee_site_id)
            J = np.vstack([jacp[:, :self._n_joints], jacr[:, :self._n_joints]])

            # Damped least squares: dq = J^T (J J^T + λ²I)⁻¹ err
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + damping**2 * np.eye(6), err)

            # Line search
            alpha = 1.0
            for _ in range(10):
                q_new = q + alpha * dq
                for j in range(self._n_joints):
                    if self._model.jnt_range[j] is not None:
                        lo = self._model.jnt_range[j][0]
                        hi = self._model.jnt_range[j][1]
                        if lo < hi:
                            q_new[j] = np.clip(q_new[j], lo, hi)
                if np.all(np.abs(q_new - q) < 0.5):
                    q = q_new
                    break
                alpha *= 0.5
            else:
                q = q_new

        self._data.qpos[:] = saved_qpos
        mujoco.mj_forward(self._model, self._data)

        return q.tolist(), False

    def plan_movel(
        self,
        q_start: List[float],
        pose_goal: Tuple[List[float], List[List[float]]],
        num_waypoints: int = 50,
    ) -> List[List[float]]:
        """Plan a straight-line Cartesian path from q_start to pose_goal."""
        q_start = np.asarray(q_start, dtype=float)[:self._n_joints]
        pos_goal, R_goal = pose_goal
        pos_goal = np.asarray(pos_goal, dtype=float)
        R_goal = np.asarray(R_goal, dtype=float)

        saved_qpos = self._data.qpos.copy()
        self._data.qpos[:self._n_joints] = q_start
        mujoco.mj_forward(self._model, self._data)
        pos_start = self._data.site_xpos[self._ee_site_id].copy()
        R_start = self._data.site_xmat[self._ee_site_id].reshape(3, 3).copy()

        path = [q_start.tolist()]
        q_current = q_start.copy()

        for i in range(1, num_waypoints + 1):
            t = i / num_waypoints
            pos_i = pos_start + t * (pos_goal - pos_start)
            R_i = self._slerp(R_start, R_goal, t)

            q_i, ok = self.ik(pos_i, R_i, q_seed=q_current.tolist())
            if not ok:
                self._data.qpos[:] = saved_qpos
                mujoco.mj_forward(self._model, self._data)
                return path

            path.append(q_i)
            q_current = np.asarray(q_i)

        self._data.qpos[:] = saved_qpos
        mujoco.mj_forward(self._model, self._data)

        return path

    def plan_movec(
        self,
        q_start: List[float],
        pose_via: Tuple[List[float], List[List[float]]],
        pose_goal: Tuple[List[float], List[List[float]]],
        num_waypoints: int = 50,
    ) -> List[List[float]]:
        """Plan a circular-arc Cartesian path through a via-point."""
        q_start = np.asarray(q_start, dtype=float)[:self._n_joints]
        pos_via, R_via = pose_via
        pos_goal, R_goal = pose_goal
        pos_via = np.asarray(pos_via, dtype=float)
        pos_goal = np.asarray(pos_goal, dtype=float)
        R_via = np.asarray(R_via, dtype=float)
        R_goal = np.asarray(R_goal, dtype=float)

        saved_qpos = self._data.qpos.copy()
        self._data.qpos[:self._n_joints] = q_start
        mujoco.mj_forward(self._model, self._data)
        pos_start = self._data.site_xpos[self._ee_site_id].copy()
        R_start = self._data.site_xmat[self._ee_site_id].reshape(3, 3).copy()
        self._data.qpos[:] = saved_qpos
        mujoco.mj_forward(self._model, self._data)

        center = self._circle_center(pos_start, pos_via, pos_goal)
        r = np.linalg.norm(pos_start - center)

        v_start = pos_start - center
        v_via = pos_via - center
        v_goal = pos_goal - center

        theta_start = np.arctan2(v_start[1], v_start[0])
        theta_via = np.arctan2(v_via[1], v_via[0])
        theta_goal = np.arctan2(v_goal[1], v_goal[0])

        while theta_via < theta_start:
            theta_via += 2 * np.pi
        while theta_goal < theta_via:
            theta_goal += 2 * np.pi
        total_theta = theta_goal - theta_start

        path = [q_start.tolist()]
        q_current = q_start.copy()

        for i in range(1, num_waypoints + 1):
            t = i / num_waypoints
            theta = theta_start + t * total_theta
            pos_i = center + r * np.array([np.cos(theta), np.sin(theta), 0])
            pos_i[2] = pos_start[2] + t * (pos_goal[2] - pos_start[2])
            R_i = self._slerp(R_start, R_goal, t)

            q_i, _ = self.ik(pos_i, R_i, q_seed=q_current.tolist())
            path.append(q_i)
            q_current = np.asarray(q_i)

        return path

    def plan_movep(
        self,
        q_start: List[float],
        poses_goal: List[Tuple[List[float], List[List[float]]]],
        waypoints_per_segment: int = 30,
    ) -> List[List[float]]:
        """Plan a multi-waypoint Cartesian path."""
        q_current = np.asarray(q_start, dtype=float)[:self._n_joints]
        full_path = [q_current.tolist()]

        saved_qpos = self._data.qpos.copy()
        self._data.qpos[:self._n_joints] = q_current
        mujoco.mj_forward(self._model, self._data)
        pos_current = self._data.site_xpos[self._ee_site_id].copy()
        R_current = self._data.site_xmat[self._ee_site_id].reshape(3, 3).copy()
        self._data.qpos[:] = saved_qpos
        mujoco.mj_forward(self._model, self._data)

        for pose_goal in poses_goal:
            pos_goal, R_goal = pose_goal
            pos_goal = np.asarray(pos_goal, dtype=float)
            R_goal = np.asarray(R_goal, dtype=float)

            for i in range(1, waypoints_per_segment + 1):
                t = i / waypoints_per_segment
                pos_i = pos_current + t * (pos_goal - pos_current)
                R_i = self._slerp(R_current, R_goal, t)

                q_i, ok = self.ik(pos_i, R_i, q_seed=q_current.tolist())
                if not ok:
                    return full_path
                full_path.append(q_i)
                q_current = np.asarray(q_i)

            pos_current = pos_goal
            R_current = R_goal

        return full_path

    @staticmethod
    def _circle_center(
        p1: np.ndarray, p2: np.ndarray, p3: np.ndarray
    ) -> np.ndarray:
        """Compute the center of a circle passing through three 3D points."""
        v1 = p2 - p1
        v2 = p3 - p1
        n = np.cross(v1, v2)
        if np.linalg.norm(n) < 1e-10:
            return (p1 + p3) / 2.0

        n12 = np.cross(v1, n)
        n13 = np.cross(v2, n)
        mid12 = (p1 + p2) / 2.0
        mid13 = (p1 + p3) / 2.0

        A = np.column_stack([n12, -n13])
        b = mid13 - mid12
        try:
            ts, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            center = mid12 + ts[0] * n12
        except np.linalg.LinAlgError:
            center = (p1 + p2 + p3) / 3.0

        return center

    @staticmethod
    def _slerp(R1: np.ndarray, R2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between two rotation matrices."""
        q1 = _rotmat_to_quat(R1)
        q2 = _rotmat_to_quat(R2)

        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            q = q1 + t * (q2 - q1)
            q = q / np.linalg.norm(q)
        else:
            theta = np.arccos(np.clip(dot, -1.0, 1.0))
            sin_theta = np.sin(theta)
            s1 = np.sin((1 - t) * theta) / sin_theta
            s2 = np.sin(t * theta) / sin_theta
            q = s1 * q1 + s2 * q2

        return _quat_to_rotmat(q)