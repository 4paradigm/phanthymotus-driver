"""Release only freshly confirmed aligned workers in this service session."""
import json
from pathlib import Path
import time


def release():
    root = Path(__file__).parent
    session = json.loads((root / "execution.json").read_text())
    selected = session.get("selected_arms", [])
    if not selected or any(side not in ("right", "left") for side in selected):
        raise RuntimeError("No aligned arm session exists")
    for side in selected:
        state = json.loads((root / f"execution-{side}.json").read_text())
        age = time.monotonic() - state["updated_monotonic"]
        if state.get("stage") != "vendor_aligned" or not 0 <= age <= 1:
            raise RuntimeError(f"{side} alignment is not freshly confirmed")
        if state.get("master_delta_rad", 1) > .05:
            raise RuntimeError(f"{side} master moved after alignment; align again")
    for side in selected:
        (root / f"follow-gate-{side}").touch()


if __name__ == "__main__":
    release()
