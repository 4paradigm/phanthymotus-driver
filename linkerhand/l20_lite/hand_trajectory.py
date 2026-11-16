"""Glove recordings (UDP 9012) as L20 Lite joint trajectories.

Built-in presets and (later) live recordings share this pipeline: glove frames are
converted to the driver's joint order, filtered, resampled to 50 ms and checked
against the finger-collision rules before anything is allowed to play.

Values are 0..255 position units in Modbus register order, not degrees.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import unicodedata

from teach_messages import error_text, reason_text

SAMPLE_MS = 50
MAX_POINTS = 10000
MAX_BYTES = 32 * 1024 * 1024
# The glove streams at ~60 fps (largest gap seen in field recordings: 53 ms).
MAX_GAP_MS = 100
SOURCE_FORMAT = "linker.stroke6.flat.v1"
GLOVE_LABELS = ("thumb_pitch", "thumb_side", "index_pitch", "middle_pitch", "ring_pitch", "pinky_pitch")
PRESET_DIR = Path(__file__).resolve().with_name("teach_presets")

# Same as modbus_device.JOINT_NAMES (register order); kept here so the offline
# preset tool does not need the driver runtime.
JOINT_NAMES = [
    "thumb_root", "thumb_sway", "index_root", "middle_root", "ring_root",
    "pinky_root", "index_sway", "ring_sway", "pinky_sway", "thumb_roll",
]
# Column order of the vendor GUI, which glove2l20.py also writes.
GUI_JOINT_NAMES = [
    "拇指根弯", "食指根弯", "中指根弯", "无名指根弯", "小指根弯",
    "拇指摆", "食指摆", "无名指摆", "小指摆", "拇指旋转",
]
# driver[slot] = gui[GUI_TO_DRIVER[slot]]. A mistake here swaps thumb sway with a
# finger flex, which drives the thumb into the fingers.
GUI_TO_DRIVER = (0, 5, 1, 2, 3, 4, 6, 7, 8, 9)
THUMB_ROOT, THUMB_SWAY, INDEX_ROOT, THUMB_ROLL = 0, 1, 2, 9
FINGER_SWAYS = (6, 7, 8)

# Perpendicular-thumb detection, calibrated from the thumb-sway recording
# (_拇指摆_20261113_124644): low thumb_pitch together with low thumb_side.
TP_RANGE = (110, 230)
TS_RANGE = (120, 200)
# The glove has no finger sway channel.
FIXED_SWAY = 128
# Confirmed on the hand: the thumb roll is only safe at 255, so it never follows the glove.
THUMB_ROLL_SAFE = 255
MEDIAN_WINDOW = 3

RULES_VERSION = 2
SAFETY_RULES = (
    ("R1", "拇指旋转值必须保持为 255",
     lambda p: p[THUMB_ROLL] != THUMB_ROLL_SAFE),
    ("R2", "食指、无名指和小指没有侧摆输入，侧摆值必须保持为 128",
     lambda p: any(p[index] != FIXED_SWAY for index in FINGER_SWAYS)),
)


def to_driver_order(gui):
    return [gui[index] for index in GUI_TO_DRIVER]


def to_gui_order(driver):
    gui = [0] * len(GUI_TO_DRIVER)
    for slot, index in enumerate(GUI_TO_DRIVER):
        gui[index] = driver[slot]
    return gui


def _clamp255(value):
    return max(0, min(255, int(round(value))))


def _ramp(value, low, high):
    return max(0.0, min(1.0, (value - low) / (high - low)))


def glove_to_gui(right_hand, *, local_glove=False):
    """Convert six glove channels to GUI joint order."""
    values = dict(zip(GLOVE_LABELS, right_hand))
    pitch, side = values["thumb_pitch"], values["thumb_side"]
    # w=1: thumb perpendicular to the palm, w=0: against the index finger.
    w = (1 - _ramp(pitch, *TP_RANGE)) * (1 - _ramp(side, *TS_RANGE))
    # The SDK already provides thumb flexion. The UDP posture correction
    # can turn this bent-thumb reading back to straight.
    thumb_root = pitch if local_glove else pitch + w * (255 - pitch)
    return [
        _clamp255(thumb_root),  # unlike the fingers, a large glove value is straight
        _clamp255(255 - values["index_pitch"]),
        _clamp255(255 - values["middle_pitch"]),
        _clamp255(255 - values["ring_pitch"]),
        _clamp255(255 - values["pinky_pitch"]),
        _clamp255(255 * (1 - w)),
        FIXED_SWAY,
        FIXED_SWAY,
        FIXED_SWAY,
        THUMB_ROLL_SAFE,
    ]


def glove_to_driver(right_hand, *, local_glove=False):
    return to_driver_order(glove_to_gui(right_hand, local_glove=local_glove))


def parse_stroke(packet, side="right"):
    """The six values for the selected hand of one 9012 packet, or None if it is not a stroke frame."""
    if isinstance(packet, (str, bytes)):
        try:
            packet = json.loads(packet)
        except ValueError:
            return None
    if not isinstance(packet, dict) or packet.get("schemaId") != SOURCE_FORMAT:
        return None
    if packet.get("labels") != list(GLOVE_LABELS):
        return None
    if side not in ("left", "right"):
        return None
    available = packet.get("availableSides")
    if available is not None and (not isinstance(available, list) or side not in available):
        return None
    values = packet.get(f"{side}Hand")
    if (not isinstance(values, list) or len(values) != len(GLOVE_LABELS)
            or any(type(v) is not int or not 0 <= v <= 255 for v in values)):
        return None
    return values


def parse_records(lines, side="right"):
    """(time_ms, stroke) pairs from recorder lines {"t_rel": s, "raw": "<9012 JSON>"}."""
    frames = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except ValueError:
            continue
        values = parse_stroke(record.get("raw"), side) if isinstance(record, dict) else None
        if values is None or type(record.get("t_rel")) not in (int, float):
            continue
        frames.append((record["t_rel"] * 1000.0, values))
    return frames


def median_filter(frames, window=MEDIAN_WINDOW):
    """Per-joint running median; removes single-frame glove spikes, keeps the ends."""
    half = window // 2
    out = [list(frame) for frame in frames]
    for index in range(half, len(frames) - half):
        neighbours = frames[index - half:index + half + 1]
        out[index] = [sorted(column)[half] for column in zip(*neighbours)]
    return out


def resample(times_ms, frames, sample_ms=SAMPLE_MS, max_gap_ms=MAX_GAP_MS):
    """Linear interpolation onto a uniform grid starting at the first frame.

    A gap longer than max_gap_ms rejects the whole recording rather than bridging it.
    """
    if len(frames) < 2 or len(times_ms) != len(frames):
        raise ValueError("轨迹至少需要两帧手套数据")
    relative = [t - times_ms[0] for t in times_ms]
    for previous, current in zip(relative, relative[1:]):
        gap = current - previous
        if gap <= 0:
            raise ValueError("手套帧时间戳未按顺序递增")
        if gap > max_gap_ms:
            raise ValueError(f"手套数据间隔 {gap:.0f} ms，超过上限 {max_gap_ms} ms")
    points = []
    segment = 0
    while len(points) * sample_ms <= relative[-1]:
        target = len(points) * sample_ms
        if len(points) >= MAX_POINTS:
            raise ValueError(f"轨迹超过 {MAX_POINTS} 个采样点")
        while relative[segment + 1] < target:
            segment += 1
        start, end = relative[segment], relative[segment + 1]
        alpha = (target - start) / (end - start)
        values = [_clamp255(a + alpha * (b - a)) for a, b in zip(frames[segment], frames[segment + 1])]
        points.append({"time_ms": target, "positions": values})
    if len(points) < 2:
        raise ValueError(f"轨迹时长不足 {sample_ms} ms")
    return points


def check_positions(values):
    if (not isinstance(values, list) or len(values) != len(JOINT_NAMES)
            or any(type(v) is not int or not 0 <= v <= 255 for v in values)):
        raise ValueError("需要 10 个 0–255 范围内的整数关节值")
    return values


def violations(positions):
    """Ids of the safety rules a pose breaks (driver order)."""
    check_positions(positions)
    return [rule for rule, _reason, broken in SAFETY_RULES if broken(positions)]


def validate_points(points):
    if not isinstance(points, list) or not 2 <= len(points) <= MAX_POINTS:
        raise ValueError(f"轨迹需要 2–{MAX_POINTS} 个采样点")
    reasons = {rule: reason for rule, reason, _broken in SAFETY_RULES}
    for index, point in enumerate(points):
        if not isinstance(point, dict) or point.get("time_ms") != index * SAMPLE_MS:
            raise ValueError("轨迹采样时间不是等间隔")
        broken = violations(point.get("positions"))
        if broken:
            raise ValueError(f"采样点 {index}（{point['time_ms']} ms）违反 {broken[0]}："
                             f"{reasons[broken[0]]}")
    return points


def normalize_name(name):
    if not isinstance(name, str):
        raise ValueError("轨迹名称须为 1–80 个可打印字符")
    name = unicodedata.normalize("NFC", name).strip()
    if not name or len(name) > 80 or any(ord(c) < 32 or ord(c) == 127 for c in name):
        raise ValueError("轨迹名称须为 1–80 个可打印字符")
    return name


def build_trajectory(frames, *, name, source, side="right", source_recording=None, recorded_at=None):
    """Manifest (without checksum) and samples for (time_ms, stroke) glove frames."""
    if side not in ("left", "right"):
        raise ValueError("轨迹手侧必须是左手或右手")
    times = [time_ms for time_ms, _values in frames]
    local_glove = source_recording == "local_glove"
    converted = median_filter([
        glove_to_driver(values, local_glove=local_glove) for _time_ms, values in frames
    ])
    points = validate_points(resample(times, converted))
    samples = {"version": 1, "sample_ms": SAMPLE_MS, "joint_names": list(JOINT_NAMES), "points": points}
    manifest = {
        "version": 2, "name": normalize_name(name), "source": source, "side": side,
        "source_format": SOURCE_FORMAT, "source_recording": source_recording, "recorded_at": recorded_at,
        "state": "recorded", "sample_ms": SAMPLE_MS, "frame_count": len(frames),
        "point_count": len(points), "duration_s": (len(points) - 1) * SAMPLE_MS / 1000,
        "conversion": {"tp": list(TP_RANGE), "ts": list(TS_RANGE), "thumb_roll_fixed": THUMB_ROLL_SAFE,
                       "finger_sway_fixed": FIXED_SWAY, "median": MEDIAN_WINDOW, "max_gap_ms": MAX_GAP_MS},
        "validation": {"passed": True, "rules_version": RULES_VERSION},
    }
    if local_glove:
        manifest["conversion"]["thumb_root"] = "sdk_flexion"
    return manifest, samples


def recorded_at(epoch_s, utc_offset_hours=8):
    zone = timezone(timedelta(hours=utc_offset_hours))
    return datetime.fromtimestamp(epoch_s, zone).isoformat(timespec="seconds")


def encode(value):
    """Deterministic bytes, so a rebuilt preset is byte-identical and its checksum stable."""
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n").encode("utf-8")


def _atomic_write(path, data):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as file:
        file.write(data)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


def write_trajectory(directory, manifest, samples, raw_lines=None):
    """Write raw, samples, then the manifest (with the samples checksum) last."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if raw_lines is not None:
        _atomic_write(directory / "raw.jsonl", "".join(raw_lines).encode("utf-8"))
    data = encode(samples)
    _atomic_write(directory / "samples.json", data)
    manifest = {**manifest, "samples_sha256": hashlib.sha256(data).hexdigest()}
    _atomic_write(directory / "manifest.json", encode(manifest))
    return manifest


def _read(path):
    if path.is_symlink() or path.stat().st_size > MAX_BYTES:
        raise ValueError("轨迹文件类型无效或文件过大")
    return path.read_bytes()


def load_trajectory(directory):
    """Verified (manifest, samples); anything tampered with or out of rule is refused."""
    directory = Path(directory)
    if directory.is_symlink():
        raise ValueError("轨迹目录不能是符号链接")
    manifest = json.loads(_read(directory / "manifest.json"))
    if manifest.get("version") != 2 or manifest.get("state") != "recorded":
        raise ValueError(f"轨迹不完整：{reason_text(manifest.get('reason') or manifest.get('state'))}")
    validation = manifest.get("validation") or {}
    # Version 1 is compatible. The checksum and all points are still verified below.
    if validation.get("passed") is not True or validation.get("rules_version") not in (1, RULES_VERSION):
        raise ValueError("轨迹缺少兼容的手型校验记录")
    if manifest.get("sample_ms") != SAMPLE_MS or manifest.get("side") not in ("left", "right"):
        raise ValueError("轨迹的采样间隔或手侧不受支持")
    data = _read(directory / "samples.json")
    if hashlib.sha256(data).hexdigest() != manifest.get("samples_sha256"):
        raise ValueError("轨迹采样文件校验和不匹配")
    samples = json.loads(data)
    if samples.get("joint_names") != JOINT_NAMES or samples.get("sample_ms") != SAMPLE_MS:
        raise ValueError("轨迹的关节顺序或采样间隔与驱动不一致")
    validate_points(samples.get("points"))
    if len(samples["points"]) != manifest.get("point_count"):
        raise ValueError("轨迹采样点数与清单不一致")
    return manifest, samples


def summary(manifest):
    keys = ("name", "source", "side", "state", "recorded_at", "sample_ms", "duration_s", "point_count",
            "source_recording", "frame_count", "end_reason", "reason")
    result = {key: manifest[key] for key in keys if key in manifest}
    if "reason" in result:
        result["reason"] = reason_text(result["reason"])
    return result


def list_presets(root=PRESET_DIR, source="preset"):
    """Every trajectory directory with its verification result; invalid ones are listed, not hidden."""
    entries = []
    for directory in sorted(Path(root).glob("*/")):
        if not (directory / "manifest.json").exists():
            continue
        try:
            manifest, _samples = load_trajectory(directory)
            entries.append({**summary(manifest), "source": source, "valid": True})
        except (ValueError, OSError, KeyError) as exc:
            try:
                stored = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
                entry = {**summary(stored), "name": stored["name"]}
            except (ValueError, OSError, KeyError, TypeError):
                entry = {"name": directory.name}
            entries.append({**entry, "source": source, "valid": False, "error": error_text(exc, "轨迹清单或采样文件无法读取，请查看驱动日志")})
    return entries


def find_preset(name, root=PRESET_DIR):
    """Directory of the preset with this name, or None."""
    name = normalize_name(name)
    for directory in sorted(Path(root).glob("*/")):
        try:
            manifest = json.loads(_read(directory / "manifest.json"))
        except (ValueError, OSError):
            continue
        if isinstance(manifest, dict) and manifest.get("name") == name:
            return directory
    return None
