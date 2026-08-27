#!/usr/bin/env python3
"""Tests for MujocoArm — MuJoCo simulation of LiteArm 7-DOF robot arm."""
from __future__ import annotations

import time

import numpy as np
import pytest

from litearm_mujoco import MujocoArm


# ── Fixtures ────────────────────────────────────────────────────────────────────

@pytest.fixture
def arm_headless():
    """Create a headless MujocoArm (no rendering)."""
    arm = MujocoArm(render=False)
    arm.start()
    yield arm
    arm.close()


# ── Init & Lifecycle ────────────────────────────────────────────────────────────

class TestInitAndLifecycle:
    def test_create_headless(self):
        """Create arm without rendering."""
        arm = MujocoArm(render=False)
        assert arm is not None
        arm.close()

    def test_context_manager(self):
        """Use with statement."""
        with MujocoArm(render=False) as arm:
            assert arm is not None
            state = arm.get_state()
            assert state is not None
            assert "q" in state

    def test_repr(self):
        """repr should show config."""
        arm = MujocoArm(render=False)
        r = repr(arm)
        assert "MujocoArm" in r
        arm.close()


# ── State ────────────────────────────────────────────────────────────────────────

class TestState:
    def test_get_state(self, arm_headless):
        """get_state() returns valid dict."""
        state = arm_headless.get_state()
        assert isinstance(state, dict)
        assert "q" in state
        assert len(state["q"]) == 7
        assert "dq" in state
        assert len(state["dq"]) == 7
        assert "tau" in state
        assert "state" in state
        assert "robot_serial" in state

    def test_get_tcp_pose(self, arm_headless):
        """get_tcp_pose() returns (pos, R)."""
        pos, R = arm_headless.get_tcp_pose()
        assert len(pos) == 3
        assert len(R) == 3
        assert all(len(row) == 3 for row in R)

    def test_feedback_in_state(self, arm_headless):
        """State includes feedback dict."""
        state = arm_headless.get_state()
        assert "feedback" in state
        fb = state["feedback"]
        assert "joints" in fb
        assert len(fb["joints"]) == 7


# ── Kinematics ───────────────────────────────────────────────────────────────────

class TestKinematics:
    Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]

    def test_fk(self, arm_headless):
        """FK returns valid pose."""
        pos, R = arm_headless.fk(self.Q_HOME)
        assert len(pos) == 3
        assert len(R) == 3
        # Rotation matrix should be orthonormal
        R_np = np.array(R)
        assert np.allclose(R_np @ R_np.T, np.eye(3), atol=1e-6)

    def test_ik_roundtrip(self, arm_headless):
        """IK should recover q from FK result."""
        pos, R = arm_headless.fk(self.Q_HOME)
        q_sol, ok = arm_headless.ik(pos, R)
        assert ok
        assert np.allclose(q_sol, self.Q_HOME, atol=0.03)

    def test_plan_movel(self, arm_headless):
        """plan_movel returns a path."""
        pos, R = arm_headless.fk(self.Q_HOME)
        pos_goal = [pos[0], pos[1], pos[2] - 0.05]
        path = arm_headless.plan_movel(self.Q_HOME, (pos_goal, R))
        assert len(path) >= 2
        assert len(path[0]) == 7


# ── Motion Control ───────────────────────────────────────────────────────────────

class TestMotionControl:
    Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]

    def test_movej(self, arm_headless):
        """movej blocks and returns True."""
        ok = arm_headless.movej(self.Q_HOME, speed=0.5, settle_s=0.3)
        assert ok is True

    def test_movej_then_zero(self, arm_headless):
        """movej to home then back to zero."""
        arm_headless.movej(self.Q_HOME, speed=0.5, settle_s=0.3)
        arm_headless.movej([0.0] * 7, speed=0.5, settle_s=0.3)
        state = arm_headless.get_state()
        assert np.allclose(state["q"], [0.0] * 7, atol=5e-2)

    def test_movel(self, arm_headless):
        """movel straight line."""
        pos, R = arm_headless.get_tcp_pose()
        pos_goal = [pos[0], pos[1], pos[2] - 0.05]
        ok = arm_headless.movel((pos_goal, R), speed=0.2, settle_s=0.3)
        assert ok is True

    def test_hold(self, arm_headless):
        """hold returns True."""
        ok = arm_headless.hold()
        assert ok is True

    def test_zero_gravity(self, arm_headless):
        """zero_gravity returns True."""
        ok = arm_headless.zero_gravity()
        assert ok is True


# ── Emergency Stop ───────────────────────────────────────────────────────────────

class TestEmergencyStop:
    def test_request_stop(self, arm_headless):
        """request_stop() sets stopped flag."""
        arm_headless.request_stop()
        time.sleep(0.01)  # Let sim thread update state cache
        state = arm_headless.get_state()
        assert state["state"] == "stopping"

    def test_clear_stop(self, arm_headless):
        """clear_stop() clears the stopped flag."""
        arm_headless.request_stop()
        arm_headless.clear_stop()
        state = arm_headless.get_state()
        assert state["state"] == "ready"

    def test_enable_disable(self, arm_headless):
        """enable/disable cycle."""
        arm_headless.disable()
        time.sleep(0.01)  # Let sim thread update state cache
        state = arm_headless.get_state()
        assert state["state"] == "disconnected"

        arm_headless.enable()
        time.sleep(0.01)
        state = arm_headless.get_state()
        assert state["state"] == "ready"


# ── Parameters ───────────────────────────────────────────────────────────────────

class TestParameters:
    def test_get_gains(self, arm_headless):
        """get_gains returns kp/kd."""
        gains = arm_headless.get_gains()
        assert "kp" in gains
        assert "kd" in gains
        assert len(gains["kp"]) == 7

    def test_set_gains(self, arm_headless):
        """set_gains updates and returns gains."""
        gains = arm_headless.set_gains(kp=[100.0] * 7, kd=[2.0] * 7)
        assert gains["kp"] == [100.0] * 7
        assert gains["kd"] == [2.0] * 7

    def test_get_joint_limits(self, arm_headless):
        """get_joint_limits returns limits for all 7 joints."""
        limits = arm_headless.get_joint_limits()
        assert len(limits) == 7
        for j in range(7):
            assert f"joint{j}" in limits
            assert "min" in limits[f"joint{j}"]
            assert "max" in limits[f"joint{j}"]

    def test_clear_faults(self, arm_headless):
        """clear_faults returns empty list."""
        faults = arm_headless.clear_faults()
        assert faults == []


# ── API Compatibility ────────────────────────────────────────────────────────────

class TestApiCompatibility:
    """Verify all litearm.Arm methods exist on MujocoArm."""

    EXPECTED_METHODS = {
        # Lifecycle
        "start", "close",
        # State
        "get_state", "get_tcp_pose",
        # Kinematics
        "fk", "ik", "plan_movel", "plan_movec", "plan_movep",
        # Motion
        "movej", "movel", "movec", "movep",
        "replay_joint_path", "replay_trajectory", "replay_timed_trajectory",
        "hold", "zero_gravity",
        # Emergency
        "request_stop", "clear_stop",
        # Enable/Disable
        "enable", "disable",
        # Gains
        "set_gains", "get_gains",
        "clear_faults",
        # Payload/Installation
        "set_payload", "get_payload",
        "set_installation", "get_installation",
        # System
        "get_system_stats", "get_logs", "restart_service",
        # Settings
        "get_joint_limits", "set_joint_limits",
        "get_zero_offsets", "set_zero_offsets",
        "get_end_effector", "set_end_effector",
        "get_cartesian_limits", "set_cartesian_limits",
        "get_collision_config", "set_collision_config",
        # Recording
        "start_recording", "stop_recording", "discard_recording",
        "get_recording_state", "get_playback_state",
        # Trajectory
        "list_trajectories", "save_trajectory", "delete_trajectory",
        "play_trajectory", "record_trajectory",
        # Device
        "list_device_types", "connect_device", "disconnect_device",
        "get_active_device", "device",
        # Teleop
        "enter_teleop", "exit_teleop", "get_teleop_status",
        # Impedance
        "joint_impedance", "cartesian_impedance", "joint_follow",
        # Limits
        "recover_joint_limits",
    }

    def test_all_methods_exist(self, arm_headless):
        """All expected API methods exist on MujocoArm."""
        for name in self.EXPECTED_METHODS:
            assert hasattr(arm_headless, name), f"Missing method: {name}"
            assert callable(getattr(arm_headless, name)), f"Not callable: {name}"

    def test_device_proxy(self, arm_headless):
        """device() returns a simulated proxy."""
        dev = arm_headless.device("hand_0")
        assert dev is not None
        assert dev.open() is True
        assert dev.close() is True
        assert dev.get_state() == {"connected": True}

    def test_devices_dict_access(self, arm_headless):
        """devices['hand_0'] works."""
        dev = arm_headless.devices["hand_0"]
        assert dev is not None

    def test_hand_property(self, arm_headless):
        """arm.hand shortcut works."""
        h = arm_headless.hand
        assert h is not None
        assert h.open() is True


# ── Joint Positions ──────────────────────────────────────────────────────────────

class TestSetJointPositions:
    def test_set_joint_positions(self, arm_headless):
        """set_joint_positions() directly sets the arm."""
        q = [0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0]
        arm_headless.set_joint_positions(q)
        time.sleep(0.1)
        state = arm_headless.get_state()
        assert np.allclose(state["q"], q, atol=1e-2)


# ── Controller ───────────────────────────────────────────────────────────────────

class TestController:
    def test_controller_tracks_target(self, arm_headless):
        """Controller drives arm toward target."""
        q_target = [0.0, 0.3, 0.0, -0.5, 0.0, 0.3, 0.0]
        arm_headless.movej(q_target, speed=0.5, settle_s=0.5)
        state = arm_headless.get_state()
        assert np.allclose(state["q"], q_target, atol=5e-2)


# ── Trajectory ───────────────────────────────────────────────────────────────────

class TestTrajectory:
    def test_record_trajectory(self, arm_headless):
        """record_trajectory returns a JointTrajectory."""
        traj = arm_headless.record_trajectory(duration_s=0.5, sample_rate_hz=50.0)
        assert traj is not None
        assert len(traj.frames) > 0

    def test_play_trajectory(self, arm_headless):
        """play_trajectory replays a recorded trajectory."""
        traj = arm_headless.record_trajectory(duration_s=0.5, sample_rate_hz=50.0)
        ok = arm_headless.play_trajectory(traj, speed=1.0)
        assert ok is True