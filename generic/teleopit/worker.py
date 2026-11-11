"""Private child protocol: commands on stdin, structured events on an inherited pipe."""

from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import sys
import threading
import traceback

# Source checkout and unified Docker build both make common/ available here.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "common" / "logsafe.py").is_file():
        sys.path.insert(0, str(_parent))
        break
try:
    from common import logsafe
    logsafe.install(check_fd=False)
except ImportError:
    pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--event-fd", type=int, required=True)
    args = parser.parse_args()
    stop_event, pause_event = threading.Event(), threading.Event()
    with os.fdopen(args.event_fd, "w", encoding="utf-8", buffering=1) as output:
        def emit(event: dict) -> None:
            event = dict(event)
            preview = event.pop("preview_jpeg", None)
            if preview is not None:
                event["preview_jpeg_b64"] = base64.b64encode(preview).decode("ascii")
            output.write(json.dumps(event, allow_nan=False, ensure_ascii=False) + "\n")
            output.flush()

        try:
            options = json.loads(sys.stdin.readline())

            def commands() -> None:
                try:
                    for line in sys.stdin:
                        command = json.loads(line).get("command")
                        if command == "pause":
                            pause_event.set()
                        elif command == "resume":
                            pause_event.clear()
                        elif command == "stop":
                            break
                finally:
                    stop_event.set()
                    pause_event.clear()

            threading.Thread(target=commands, name="teleopit-commands", daemon=True).start()
            from backend import run_backend
            run_backend(options, emit, stop_event, pause_event)
            return 0
        except Exception as exc:
            traceback.print_exc()
            emit({"event": "error", "error": f"{type(exc).__name__}: {exc}"})
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
