"""Store Teach sessions using the vendor frame writer."""
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import sys
from tempfile import TemporaryDirectory

GRIPPER_OPEN_WIDTH_M = .08
PLAYBACK_TIMESTAMP = "fr3_observed_monotonic_ns"

NAME = re.compile(r"(?:franka|fr3)-(right|left)-\d{8}T\d{6}(?:\.\d{6})?(?:Z|[+-]\d{4})")


def start_recording(env, data_dir):
    marker = env.get("FR3_RECORD_MARKER")
    if not marker or not Path(marker).is_file():
        return None
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    path = Path(data_dir).expanduser() / "teleop" / f"franka-{env['FR3_ARM_SIDE']}-{timestamp}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def record_frame(path, timestamp, observation, action, writer, *, observed_ns):
    # Only expose complete vendor frames to the listing and playback readers.
    with TemporaryDirectory(prefix=".frame-", dir=path) as directory:
        frame = {**observation, PLAYBACK_TIMESTAMP: observed_ns}
        writer(Path(directory), timestamp, frame, action)
        name = timestamp.isoformat() + ".pkl"
        (Path(directory) / name).replace(path / name)


def main(args):
    root = Path.home() / "bc_data" / "teleop"
    if args == ["list"]:
        names = sorted((p.name for p in root.iterdir()
                        if NAME.fullmatch(p.name) and p.is_dir() and not p.is_symlink()
                        and any(p.glob("*.pkl"))), reverse=True) if root.is_dir() else []
        print(json.dumps(names))
        return
    if len(args) == 2 and args[0] == "delete" and NAME.fullmatch(args[1]):
        path = root / args[1]
        if path.is_symlink() or not path.is_dir():
            raise ValueError("Unknown Teach recording")
        shutil.rmtree(path)
        return
    raise ValueError("Invalid trajectory operation")


if __name__ == "__main__":
    main(sys.argv[1:])
