#!/usr/bin/env python3
"""Tests for the litearm-python integration — the SDK shim, the Msg envelope
handling, and the mirror path.

Everything here runs offline: ``litearm_core.testing.FakeTransport`` scripts
firmware replies, so no real arm is needed.
"""
from __future__ import annotations

import importlib
import inspect
import pathlib
import sys
import time

import pytest

import litearm_mujoco
from litearm_mujoco import DualArm, MujocoArm
from litearm_mujoco._sdk import sdk, state_q
from litearm_mujoco.trajectory import JointTrajectory, TrajectoryFrame

# The tests need the SDK's *real* types as ground truth, but the import name is
# resolved through the shim rather than written out here, so the litearm /
# litearm_core split stays a one-file change instead of a tree-wide one.
Arm = sdk.Arm
Msg = sdk.Msg
RobotState = sdk.RobotState
JointState = sdk.JointState
FakeTransport = importlib.import_module(f"{sdk.__name__}.testing").FakeTransport


# ── Fixtures ────────────────────────────────────────────────────────────────────

@pytest.fixture
def fake_arm():
    """A connected litearm-python Arm backed by a scripted fake transport.

    ⚠ Two things here are load-bearing:

    * The placeholder port — ``Arm``'s ``transport_factory`` injection point
      sits *after* ``connect()``'s ``find_cdc_port()`` check, so with
      ``port=None`` and no hardware attached this raises TransportError
      before the factory is ever consulted.
    * ``auto_status`` — without it the fake only answers commands and never
      emits the ~100Hz status stream a real arm sends, so ``get_state()``
      keeps returning its connect-time zeros forever.
    """
    fake = FakeTransport()
    fake.auto_status = True
    arm = Arm(port="fake", transport_factory=lambda p: fake).connect()
    yield arm, fake
    arm.close()


# ── The shim ────────────────────────────────────────────────────────────────────

class TestSdkShim:
    """_sdk.py is the only place the SDK's import name appears."""

    def test_resolves_to_a_2x_sdk(self):
        # `litearm` is the declared, released name; `litearm_core` is the same
        # 2.x code under the name it was developed under, still accepted so a
        # local checkout keeps working. Anything else is a bug.
        assert sdk.__name__ in {"litearm", "litearm_core"}, sdk.__name__

    def test_has_msg_envelope(self):
        # The guard in _sdk.py uses this attribute to tell the 2.x line apart
        # from the legacy zenoh-based litearm 0.1, which shares the import name.
        assert hasattr(sdk, "Msg")

    def test_reexports_the_types_the_package_returns(self):
        for name in ("Msg", "RobotState", "JointState", "CartPlan", "JointParam"):
            assert hasattr(sdk, name)

    def test_reexports_catchable_errors(self):
        for name in ("LiteArmError", "IKError", "InvalidCommandError"):
            assert hasattr(sdk, name)

    def test_legacy_zenoh_sdk_would_be_rejected(self, monkeypatch):
        """The 0.1 litearm has the same import name but no Msg.

        The shim is exec'd in a scratch namespace instead of reloaded, so the
        real one the rest of the session holds keeps working.
        """
        import litearm_mujoco._sdk as shim

        class LegacyLitearm:
            __version__ = "0.1.0"

        monkeypatch.setitem(sys.modules, "litearm", LegacyLitearm())
        source = pathlib.Path(shim.__file__).read_text()
        with pytest.raises(ImportError, match="legacy litearm-python 0.1"):
            exec(compile(source, shim.__file__, "exec"), {"__name__": "shim_under_test"})


# ── The top-level alias ─────────────────────────────────────────────────────────

class TestPublicAlias:
    def test_litearm_core_is_exported(self):
        assert litearm_mujoco.litearm_core is sdk
        assert "litearm_core" in litearm_mujoco.__all__

    def test_old_litearm_alias_is_gone(self):
        assert not hasattr(litearm_mujoco, "litearm")
        assert "litearm" not in litearm_mujoco.__all__


# ── Msg envelope unwrapping ─────────────────────────────────────────────────────

class TestStateQ:
    """litearm-python returns Msg[Optional[RobotState]]; Msg is always truthy
    and has no .get, so the legacy `state and state.get("q")` idiom failed
    silently inside a bare except. state_q() is what the mirror loops use."""

    @staticmethod
    def _robot_state(values):
        return RobotState(joints=[JointState(q=v) for v in values])

    def test_unwraps_msg_envelope(self):
        msg = Msg(self._robot_state([0.1, 0.2, 0.3]), 100.0, 0.0)
        assert state_q(msg) == pytest.approx([0.1, 0.2, 0.3])

    def test_none_value_returns_none(self):
        # Msg is truthy even when it carries no frame — the trap.
        assert bool(Msg(None, 0.0, 0.0)) is True
        assert state_q(Msg(None, 0.0, 0.0)) is None

    def test_bare_robot_state(self):
        assert state_q(self._robot_state([1.0, 2.0])) == pytest.approx([1.0, 2.0])

    def test_legacy_dict_state_still_works(self):
        assert state_q({"q": [1.0, 2.0]}) == pytest.approx([1.0, 2.0])

    def test_empty_and_none(self):
        assert state_q(None) is None
        assert state_q({"q": []}) is None
        assert state_q(Msg(self._robot_state([]), 0.0, 0.0)) is None


# ── Offline integration against a fake arm ──────────────────────────────────────

class TestFakeTransportConnection:
    def test_connect_and_read_state(self, fake_arm):
        arm, fake = fake_arm
        assert arm.n == 7

        fake.q = [0.1] * 7
        state = arm.get_state(refresh=True).value
        assert state is not None
        assert state.q[0] == pytest.approx(0.1, abs=1e-6)

    def test_movej_updates_fake_configuration(self, fake_arm):
        arm, fake = fake_arm
        arm.movej([0.3] * 7, speed=0.5)
        assert fake.q[0] == pytest.approx(0.3, abs=1e-6)


class TestMirrorFromFakeArm:
    """Regression test for the silent-mirroring bug.

    Before, the loop did `real.get_state()` then `state.get("q")`. Under
    litearm-python that raised AttributeError on the Msg envelope, which the
    loop's bare `except Exception: pass` swallowed — the simulation simply
    never moved, with no error.
    """

    def test_sim_follows_the_real_arm(self, fake_arm):
        real, fake = fake_arm

        sim = MujocoArm(render=False)
        sim.start()
        try:
            sim.set_joint_positions([0.0] * 7)
            fake.q = [0.3] * 7

            # The fake must actually be reporting the new configuration,
            # otherwise a stalled status stream would be blamed on mirroring.
            deadline = time.time() + 2.0
            while time.time() < deadline:
                if state_q(real.get_state(refresh=True)) == pytest.approx([0.3] * 7):
                    break
                time.sleep(0.02)
            assert state_q(real.get_state()) == pytest.approx([0.3] * 7)

            sim.mirror_from(real)

            mirror_deadline = time.time() + 5.0
            while time.time() < mirror_deadline:
                if sim.get_state()["q"][0] > 0.15:
                    break
                time.sleep(0.05)

            assert sim.get_state()["q"][0] > 0.15, (
                "simulation did not follow the real arm — mirroring is failing "
                "silently (check the Msg envelope unwrapping in the mirror loop)"
            )
        finally:
            sim.stop_mirroring()
            sim.close()


# ── DualArm signature ───────────────────────────────────────────────────────────

class TestDualArmSignature:
    """litearm-python connects over USB CDC (a port), not a zenoh endpoint.
    Constructing a DualArm needs hardware, so assert on the signature."""

    def test_uses_real_port(self):
        params = inspect.signature(DualArm.__init__).parameters
        assert "real_port" in params
        assert params["real_port"].default is None

    def test_old_endpoint_kwarg_is_gone(self):
        params = inspect.signature(DualArm.__init__).parameters
        assert "real_endpoint" not in params
        assert "real_arm_id" not in params


# ── Trajectory types (simulation-only extension) ────────────────────────────────

class TestTrajectoryModule:
    """litearm-python has no portable trajectory type, so these stay local."""

    def test_round_trips_through_json(self, tmp_path):
        traj = JointTrajectory(
            frames=[
                TrajectoryFrame(t=0.0, q=[0.0] * 7, dq=[0.0] * 7),
                TrajectoryFrame(t=0.01, q=[0.1] * 7, dq=[0.0] * 7),
            ],
            name="roundtrip",
        )
        path = tmp_path / "traj.json"
        traj.save(str(path))

        loaded = JointTrajectory.load(str(path))
        assert loaded.name == "roundtrip"
        assert len(loaded.frames) == 2
        assert loaded.frames[1].q[0] == pytest.approx(0.1)

    def test_importable_from_the_old_documented_path_is_gone(self):
        # examples used to do `from litearm_mujoco._litearm.types import ...`
        with pytest.raises(ImportError):
            __import__("litearm_mujoco._litearm.types")
