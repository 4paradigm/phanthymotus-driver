"""Build a built-in hand_teach preset from a glove recording (offline, not shipped in the image).

    python3 linkerhand/l20_lite/tools/build_teach_preset.py \
        "<recording dir>/port9012_60fps.jsonl" --name 拇指四指交替 --slug thumb_fingers_alternate
    python3 linkerhand/l20_lite/tools/build_teach_preset.py --index-test

The recording's 9012 lines are copied verbatim to raw.jsonl, so the preset can be
rebuilt and checked byte for byte (tests/test_linkerhand_l20_lite_teach.py does).
--index-test writes the hand-made first-motion check instead: only the index
finger root moves, 254 -> 200 -> 254 over 2 s.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import hand_trajectory  # noqa: E402


def build(lines, *, name, recording=None, utc_offset_hours=8):
    frames = hand_trajectory.parse_records(lines)
    if not frames:
        raise ValueError(f"no {hand_trajectory.SOURCE_FORMAT} frames found")
    first = next(json.loads(line) for line in lines if line.strip())
    return hand_trajectory.build_trajectory(
        frames, name=name, source="preset", source_recording=recording,
        recorded_at=hand_trajectory.recorded_at(first["ts"], utc_offset_hours))


# Open hand, thumb against the index finger, sways 128 and thumb roll 255 (rules R1-R4 hold).
INDEX_TEST_BASE = [254, 255, 254, 254, 254, 254, 128, 128, 128, 255]
INDEX_TEST_NAME = "食指单关节测试"
INDEX_TEST_SLUG = "index_root_test"


def index_test_keyframes():
    bent = list(INDEX_TEST_BASE)
    bent[hand_trajectory.INDEX_ROOT] = 200
    return [(0, list(INDEX_TEST_BASE)), (1000, bent), (2000, list(INDEX_TEST_BASE))]


def build_keyframes(keyframes, *, name):
    """Preset linearly interpolated between (time_ms, driver positions) keyframes."""
    times = [time_ms for time_ms, _positions in keyframes]
    poses = [hand_trajectory.check_positions(list(positions)) for _time_ms, positions in keyframes]
    points = hand_trajectory.validate_points(
        hand_trajectory.resample(times, poses, max_gap_ms=max(times) - min(times)))
    samples = {"version": 1, "sample_ms": hand_trajectory.SAMPLE_MS,
               "joint_names": list(hand_trajectory.JOINT_NAMES), "points": points}
    manifest = {
        "version": 2, "name": hand_trajectory.normalize_name(name), "source": "preset", "side": "right",
        "source_format": "keyframes", "source_recording": None, "recorded_at": None,
        "state": "recorded", "sample_ms": hand_trajectory.SAMPLE_MS, "frame_count": len(keyframes),
        "point_count": len(points), "duration_s": (len(points) - 1) * hand_trajectory.SAMPLE_MS / 1000,
        "keyframes": [{"time_ms": time_ms, "positions": pose} for time_ms, pose in zip(times, poses)],
        "validation": {"passed": True, "rules_version": hand_trajectory.RULES_VERSION},
    }
    return manifest, samples


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", type=Path, nargs="?", help="recorder output port9012_60fps.jsonl")
    parser.add_argument("--name", help="trajectory name shown by list_trajectories")
    parser.add_argument("--slug", help="directory name under teach_presets/")
    parser.add_argument("--recording", help="source recording name (default: the source's directory)")
    parser.add_argument("--utc-offset", type=int, default=8, help="time zone of recorded_at")
    parser.add_argument("--index-test", action="store_true", help="write the index finger test preset")
    args = parser.parse_args()

    if args.index_test:
        manifest, samples = build_keyframes(index_test_keyframes(), name=INDEX_TEST_NAME)
        manifest = hand_trajectory.write_trajectory(hand_trajectory.PRESET_DIR / INDEX_TEST_SLUG, manifest, samples)
        print(json.dumps(hand_trajectory.summary(manifest), ensure_ascii=False))
        return
    if args.source is None or not args.name or not args.slug:
        parser.error("source, --name and --slug are required unless --index-test is given")
    lines = args.source.read_text(encoding="utf-8").splitlines(keepends=True)
    manifest, samples = build(lines, name=args.name, recording=args.recording or args.source.parent.name,
                              utc_offset_hours=args.utc_offset)
    manifest = hand_trajectory.write_trajectory(hand_trajectory.PRESET_DIR / args.slug, manifest, samples, lines)
    print(json.dumps(hand_trajectory.summary(manifest), ensure_ascii=False))


if __name__ == "__main__":
    main()
