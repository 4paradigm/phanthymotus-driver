#!/usr/bin/env python3
"""Create the tracked-source Tianyi pose_check temporary-service bundle."""

from __future__ import annotations

import argparse
import tarfile
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
R1 = REPOSITORY_ROOT / "unitree" / "r1"
TIANYI = Path(__file__).resolve().parent
FILES = {
    R1 / "pose_check.py": "pose_check.py",
    R1 / "pose_estimator.py": "pose_estimator.py",
    R1 / "pose_mcp.py": "pose_mcp.py",
    R1 / "pose_ros_service.py": "pose_ros_service.py",
    R1 / "pose_session.py": "pose_session.py",
    TIANYI / "bridged_publisher.py": "bridged_publisher.py",
}


def build_bundle(output: Path) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output, "w:gz") as archive:
        for source, archive_name in FILES.items():
            if not source.is_file():
                raise FileNotFoundError(source)
            archive.add(source, arcname=archive_name)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=Path("/tmp/tianyi-pose-check-bridge.tar.gz"))
    args = parser.parse_args()
    print(build_bundle(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
