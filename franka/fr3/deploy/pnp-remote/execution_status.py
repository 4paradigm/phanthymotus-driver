"""Publish a single coherent status snapshot for the remote card."""
import json
import os
from pathlib import Path
import time

from arm_config import ARMS

STATUS = Path(__file__).with_name("execution.json")
FOLLOW_HEARTBEAT_LIMIT = 1.0


def report(stage, **details):
    snapshot = {"stage": stage, "updated_monotonic": time.monotonic(), **details}
    pending = STATUS.with_name(f".execution.{os.getpid()}.tmp")
    pending.write_text(json.dumps(snapshot, allow_nan=False))
    pending.replace(STATUS)


def snapshot(path=None):
    try:
        return json.loads((path or STATUS).read_text())
    except FileNotFoundError:
        return {}


def report_failure(exc):
    current = snapshot()
    if current.get("stage") != "failed":
        failed_stage = current.pop("stage", "startup")
        current.pop("updated_monotonic", None)
        report("failed", failed_stage=failed_stage, error=str(exc), **current)


def run_checked(main):
    try:
        main()
    except Exception as exc:
        report_failure(exc)
        raise


def age_snapshot(current):
    if current:
        age = time.monotonic() - current.pop("updated_monotonic")
        current["age_seconds"] = round(age, 3)
        current["heartbeat_fresh"] = 0 <= age <= FOLLOW_HEARTBEAT_LIMIT
    return current


if __name__ == "__main__":
    current = age_snapshot(snapshot())
    if current:
        current["arms"] = {
            side: age_snapshot(snapshot(STATUS.with_name(f"execution-{side}.json")))
            for side in current.get("selected_arms", []) if side in ARMS
        }
        print("Execution=" + json.dumps(current, allow_nan=False))
