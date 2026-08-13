"""Run a visible, robot-free final-dispatch smoke scenario."""

from __future__ import annotations

import json
import secrets
import time
import uuid

from runtime import ShadowRuntime


def _frame(runtime: ShadowRuntime, session: dict) -> dict:
    pose = {
        "position": [0.0, 1.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    return {
        "schema_version": 1,
        "mode": "shadow",
        "boot_id": runtime.boot_id,
        **session,
        "sequence": 1,
        "client_monotonic_ns": 1,
        "deadman": True,
        "clutch_sequence": 1,
        "tracking": {
            "head": True,
            "left_controller": True,
            "right_controller": True,
        },
        "head": dict(pose),
        "left_controller": dict(pose),
        "right_controller": dict(pose),
        "controllers": {
            "left": {"axes": [0.0, 0.0], "buttons": [1.0]},
            "right": {"axes": [0.0, 0.0], "buttons": [1.0]},
        },
        "base_twist": {
            "linear": [0.1, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.1],
        },
    }


def _wait_for_apply(runtime: ShadowRuntime, sequence: int) -> dict:
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        dispatch = runtime.status()["dispatch"]
        if dispatch["last_would_apply_sequence"] == sequence:
            return dispatch
        time.sleep(0.005)
    raise RuntimeError("recording adapter did not acknowledge the motion intent")


def run_smoke() -> dict:
    runtime = ShadowRuntime(auto_watchdog=False)
    session = {
        "session_id": str(uuid.uuid4()),
        "epoch": 1,
        "fence": secrets.token_urlsafe(32),
    }
    identity = {"boot_id": runtime.boot_id, **session}
    try:
        prepared = runtime.prepare_shadow(session)
        submitted = runtime.submit_shadow_frame(
            _frame(runtime, session),
            source="mcp_diagnostic",
        )
        applied = _wait_for_apply(runtime, 1)
        held = runtime.soft_stop(identity)
        released = runtime.release(identity)
        decisions = []
        for record in released["dispatch"]["adapter"]["records"]:
            if record["kind"] == "would_apply":
                decisions.append(
                    {"kind": "would_apply", "sequence": record["sequence"]}
                )
            elif record["kind"] == "would_stop":
                decisions.append(
                    {"kind": "would_stop", "reason": record["reason"]}
                )
        proof = {
            "mode": prepared["mode"],
            "actuation_enabled": prepared["actuation_enabled"],
            "prepared_state": prepared["state"],
            "motion_state": submitted["state"],
            "last_would_apply_sequence": applied["last_would_apply_sequence"],
            "soft_stop": {
                "state": held["state"],
                "reason": held["reason"],
                "acknowledged": held["dispatch"]["stop_acknowledged"],
            },
            "release": {
                "state": released["state"],
                "authority_valid": released["authority_valid"],
                "stop_acknowledged": released["dispatch"]["stop_acknowledged"],
            },
            "recorded_decisions": decisions,
        }
        if proof["actuation_enabled"] is not False:
            raise RuntimeError("Shadow smoke unexpectedly enabled actuation")
        if proof["last_would_apply_sequence"] != 1:
            raise RuntimeError("motion intent did not reach recording dispatch")
        if not proof["soft_stop"]["acknowledged"]:
            raise RuntimeError("soft-stop was not acknowledged")
        if proof["release"] != {
            "state": "released",
            "authority_valid": False,
            "stop_acknowledged": True,
        }:
            raise RuntimeError("release did not revoke and stop the session")
        if session["fence"] in json.dumps(proof, sort_keys=True):
            raise RuntimeError("public proof disclosed the session fence")
        return proof
    finally:
        runtime.close()


def main() -> None:
    print(json.dumps(run_smoke(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
