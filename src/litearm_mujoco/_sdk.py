"""Single point of contact with the real-arm SDK.

Everything else in this package imports the SDK's names from here, so the
import name lives in exactly one place.

The SDK is the ``litearm-python`` distribution (import name ``litearm``) — a
thin USB-CDC protocol layer: the firmware owns S-curve/IK/dynamics, the PC side
only sends points and reads frames. It is pinned to a git tag in pyproject.toml
because it is not on PyPI yet.

``litearm_core`` is the same 2.x code under the name it was developed under;
some local checkouts still carry it, so it is accepted as a fallback. Whatever
is installed, the import name appears in exactly this file.

⚠ The legacy ``litearm-python`` v0.1.0 (zenoh, remote ``litearm-server``,
``DeviceManager``) also imports as ``litearm``. It has no ``Msg`` envelope, so
importing it here would fail far from the cause; the guard below rejects it.
"""
from __future__ import annotations

from typing import List, Optional

try:
    import litearm as sdk
except ImportError:  # pragma: no cover - depends on the installed SDK
    try:
        import litearm_core as sdk  # type: ignore[no-redef]
    except ImportError:
        raise ImportError(
            "litearm-mujoco requires litearm-python 2.x (import name 'litearm'). "
            "Neither 'litearm' nor 'litearm_core' could be imported — install "
            "litearm-python and make sure it is on sys.path."
        ) from None

# The ``Msg`` envelope is what tells the two ``litearm`` flavours apart: the
# 2.x line returns it from every "read one frame" getter, the 0.1 zenoh line
# never had it. Checking here keeps a legacy install from surfacing later as
# an AttributeError somewhere unrelated.
if not hasattr(sdk, "Msg"):  # pragma: no cover - depends on the installed SDK
    raise ImportError(
        "litearm-mujoco requires litearm-python 2.x, but "
        f"{getattr(sdk, '__file__', 'an unknown module')} "
        f"(version {getattr(sdk, '__version__', 'unknown')}) was imported instead. "
        "That looks like the legacy litearm-python 0.1 zenoh SDK. "
        "Install litearm-python 2.x and make sure it shadows it on sys.path."
    )

# ── Re-exported SDK surface ───────────────────────────────────────────────────
# Types
Msg = sdk.Msg
RobotState = sdk.RobotState
JointState = sdk.JointState
CartPlan = sdk.CartPlan
JointParam = sdk.JointParam

# Errors a caller of the simulated arm may need to catch
LiteArmError = sdk.LiteArmError
IKError = sdk.IKError
InvalidCommandError = sdk.InvalidCommandError
MotionTimeoutError = sdk.MotionTimeoutError
MotorFaultError = sdk.MotorFaultError
NotConnectedError = sdk.NotConnectedError
TransportError = sdk.TransportError
CommandRejectedError = sdk.CommandRejectedError

def state_q(state_obj: object) -> Optional[List[float]]:
    """Extract joint positions from a state returned by either arm.

    litearm-python's ``get_state()`` returns a ``Msg`` *envelope* around an
    ``Optional[RobotState]``, so the positions live at ``msg.value.q`` — not
    at ``msg["q"]``. Since ``Msg`` is always truthy and has no ``get``, the
    legacy ``if state and state.get("q")`` idiom raises ``AttributeError``
    inside a bare ``except`` and mirroring silently stops. Unwrapping here
    keeps that failure mode out of the mirror loops.

    Also tolerates a bare ``RobotState`` and the legacy dict-shaped state, so
    mirroring works sim-to-sim as well as against real hardware.

    Returns ``None`` when there is no usable frame (no state yet, or no joints).
    """
    st = getattr(state_obj, "value", state_obj)  # unwrap the Msg envelope
    if st is None:
        return None
    q = getattr(st, "q", None)
    if q is None and isinstance(st, dict):
        q = st.get("q")
    if not q:
        return None
    return [float(v) for v in q]


__all__ = [
    "sdk",
    "state_q",
    "Msg",
    "RobotState",
    "JointState",
    "CartPlan",
    "JointParam",
    "LiteArmError",
    "IKError",
    "InvalidCommandError",
    "MotionTimeoutError",
    "MotorFaultError",
    "NotConnectedError",
    "TransportError",
    "CommandRejectedError",
]
