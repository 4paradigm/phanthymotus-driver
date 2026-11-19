"""Read-only foot geometry from SDK joint snapshots; no robot command interface.

Reuses leg_odometry's Bumi URDF chains. Results are model estimates in base_link,
not measurements of floor contact, balance, or incoming DDS freshness.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from leg_odometry import LEG_CHAINS, JOINT_INDEX, foot_geometry

LEG_IDS = frozenset(JOINT_INDEX[name] for chain in LEG_CHAINS.values() for name, _, _ in chain)
ALIGNMENT_TOLERANCE_M = 0.04  # Same fore-aft band as the upstream parallel finish.


def leg_positions(sample):
    if not isinstance(sample, dict) or not isinstance(sample.get("joint_states"), list):
        raise ValueError("A motion_state snapshot with joint_states is required")
    positions = {}
    for joint in sample["joint_states"]:
        if not isinstance(joint, dict):
            raise ValueError("Joint states must be objects")
        motor_id = joint.get("motor_id")
        if type(motor_id) is not int:
            raise ValueError("Joint motor_id must be an integer")
        if motor_id not in LEG_IDS:
            continue
        position = joint.get("position")
        if motor_id in positions:
            raise ValueError("Duplicate leg motor ID")
        if type(position) not in (int, float) or not math.isfinite(position):
            raise ValueError("Leg positions must be finite radians")
        positions[motor_id] = float(position)
    if set(positions) != LEG_IDS:
        raise ValueError("All twelve leg motor positions are required")
    return positions


def source_freshness(sample, now=None):
    """Internal reader contract; never infer DDS reception from a successful poll.

    A reader with actual receive evidence may supply source_received_monotonic
    using this process's clock and source_freshness_verified=True. The current
    vendor Python binding supplies neither. JSON demos cannot authorize motion.
    """
    now = time.monotonic() if now is None else now
    stamp = sample.get("source_received_monotonic")
    valid = type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= stamp <= now
    age = now - stamp if valid else None
    verified = sample.get("source_freshness_verified") is True and valid
    return {"source_freshness_verified": bool(verified),
            "source_age_s": age,
            "source_recent": bool(verified and age <= 0.5)}


def describe_feet(sample):
    positions = leg_positions(sample)
    feet = {}
    for side, chain in LEG_CHAINS.items():
        angles = [positions[JOINT_INDEX[name]] for name, _, _ in chain]
        heel, toe = foot_geometry(angles, chain)
        feet[side] = {"sole_center_m": [(a + b) / 2 for a, b in zip(heel, toe)],
                      "heel_m": heel, "toe_m": toe}
    delta = [a - b for a, b in zip(feet["left"]["sole_center_m"], feet["right"]["sole_center_m"])]
    return {"state": "observed", "command_sent": False, "pose_confirmed": False,
            "source": "SDK joint snapshot and leg_odometry forward kinematics",
            "frame": "base_link", "unit": "m", "accuracy_validated": False,
            "floor_contact_verified": False, "balance_verified": False,
            "model": "BUMI_V3.0_260119 URDF chains",
            **source_freshness(sample),
            "feet": feet,
            "separation": {"left_minus_right_xyz_m": delta,
                           "lateral_width_m": abs(delta[1]),
                           "fore_aft_offset_m": abs(delta[0]),
                           "height_difference_m": abs(delta[2]),
                           "center_distance_m": math.sqrt(sum(v * v for v in delta))},
            "fore_aft_aligned": abs(delta[0]) <= ALIGNMENT_TOLERANCE_M,
            "alignment_tolerance_m": ALIGNMENT_TOLERANCE_M,
            "note": "Fore-aft alignment does not mean feet together; no contact or safe stance is inferred."}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--sample", type=Path, help="Saved motion_state JSON snapshot")
    group.add_argument("--demo", action="store_true", help="Synthetic zero-angle calculation; no hardware")
    args = parser.parse_args(argv)
    try:
        if args.demo:
            sample = {"joint_states": [{"motor_id": i, "position": 0.0} for i in sorted(LEG_IDS)]}
        else:
            sample = json.loads(args.sample.read_text(encoding="utf-8"))
        # Offline JSON is not live reception evidence, even if it contains stamps.
        sample = dict(sample, source_freshness_verified=False)
        result = describe_feet(sample)
        result.update(input_kind="synthetic_demo" if args.demo else "saved_snapshot", offline=True)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError) as error:
        print(json.dumps({"state": "error", "command_sent": False, "error": str(error)}), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
