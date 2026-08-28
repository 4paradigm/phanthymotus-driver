"""Q5 card call argument normalization."""

from __future__ import annotations


def prepare_call_args(tool_def: dict, args: dict) -> dict:
    """Return the user-provided parameters without adding hidden approvals."""
    del tool_def
    return dict(args)


def q5_active_status(client) -> dict:
    """Read the Q5 FSM evidence required before a physical command."""
    reader = getattr(client, "sensor_snapshot", None)
    if not callable(reader):
        return {"available": False, "fresh": False, "state": None,
                "state_label": "UNAVAILABLE", "source": "/xbot_state"}
    try:
        raw = reader("robot_status") or {}
    except Exception:
        raw = {}
    state = raw.get("state")
    labels = {0: "INIT", 1: "SELF_TEST", 2: "IDLE", 3: "READY", 4: "ACTIVE",
              5: "SHUTDOWN", 6: "OTA", 7: "E_STOP", -1: "ERROR"}
    return {
        "available": bool(raw.get("available", False)),
        "fresh": bool(raw.get("fresh", False)),
        "state": state,
        "state_label": labels.get(state, "UNKNOWN"),
        "source": "/xbot_state",
    }


Q5_CONTROL_READY_STATES = (3, 4)


def q5_is_control_ready(client) -> tuple[bool, dict]:
    """Allow commands only when the fresh Q5 FSM reports READY or ACTIVE."""
    status = q5_active_status(client)
    status["control_ready_states"] = list(Q5_CONTROL_READY_STATES)
    return bool(status["available"] and status["fresh"] and
                status["state"] in Q5_CONTROL_READY_STATES), status


# Compatibility alias for cards that have not yet adopted the clearer name.
q5_is_active = q5_is_control_ready
