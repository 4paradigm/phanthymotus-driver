"""Native-reader preflight. Inspect paths only; never open a serial device."""

import argparse
import json
import os
from pathlib import Path
import platform
import stat


def check(glove_port=None, hand_port=None, files_only=False):
    import serial

    errors = []
    resolved = {}
    if not files_only:
        for label, value in [("glove", glove_port), ("hand", hand_port)]:
            if label == "hand" and os.environ.get("GLOVE_PREVIEW_ONLY") == "1":
                continue
            if not value:
                errors.append(f"missing {label} port")
                continue
            path = Path(value)
            try:
                if not stat.S_ISCHR(path.stat().st_mode) or not os.access(
                    path, os.R_OK | os.W_OK
                ):
                    errors.append(
                        f"{label} port must be an accessible character device"
                    )
                resolved[label] = str(path.resolve())
            except OSError as error:
                errors.append(f"{label}: {error}")
        if len(resolved) == 2 and len(set(resolved.values())) == 1:
            errors.append("hand and receiver must differ")
    return {
        "ok": not errors,
        "backend": "native_serial",
        "sdk_required": False,
        "architecture": platform.machine(),
        "serial_devices": resolved,
        "errors": errors,
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--files-only", action="store_true")
    args = ap.parse_args()
    result = check(
        os.environ.get("GLOVE_SERIAL_PORT"),
        os.environ.get("L20_SERIAL_PORT"),
        args.files_only,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["ok"] else 1)
