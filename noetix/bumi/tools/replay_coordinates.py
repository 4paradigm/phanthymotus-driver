"""Replay a Bumi coordinate recording and compare repeated observations of one point."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import closing
import csv
from dataclasses import dataclass
from itertools import combinations
import json
import math
from pathlib import Path
import sqlite3
from statistics import median
import sys

import cv2
import numpy as np

# The recorder can be copied into the running driver container as a standalone
# file. This offline tool runs from the checkout and reuses Bumi's TF convention.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from noetix.bumi.visual_odometry_contract import MAX_SKEW_NS, finite, quat_matrix, rotate


CSV_COLUMNS = ("image", "seconds", "segment", "station", "u", "v")


@dataclass
class Frame:
    stamp: int
    rgb: tuple
    depth: tuple
    info: dict
    meta: dict
    odom: dict
    mount: dict

    @property
    def image(self):
        return f"rgb_{self.stamp}.png"

    @property
    def segment(self):
        return self.odom["vendor"]["segment_id"]

    @property
    def reference(self):
        pose, vendor = self.odom["pose"], self.odom["vendor"]
        return (self.segment, pose["frame_id"], pose["child_frame_id"],
                self.rgb[1]["frame_id"], vendor["calibration_id"],
                vendor["pose_reference"], self.meta["epoch"], tuple(self.info["k"]),
                tuple(self.info["d"]), self.info["width"], self.info["height"])


def connect(path):
    db = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    if db.execute("PRAGMA user_version").fetchone()[0] != 1:
        db.close()
        raise ValueError("not a version 1 Bumi coordinate recording")
    return db


def index_rows(rows, stream):
    """An ambiguous stamp is never paired to an arbitrary image."""
    result = {}
    for row_id, stamp, payload in rows[stream]:
        result[stamp] = None if stamp in result else (row_id, payload)
    return result


def valid_poses(rows):
    poses = {}
    for _, stamp, data in rows:
        vendor, pose = data.get("vendor", {}), data.get("pose")
        segment = vendor.get("segment_id")
        if (data.get("schema") != "motus.odom/1" or vendor.get("source") != "rtabmap_rgbd_odometry"
                or vendor.get("status") != "tracking" or type(stamp) is not int or stamp <= 0
                or type(data.get("stamp_ms")) is not int or data["stamp_ms"] != stamp // 1_000_000
                or vendor.get("visual_stamp_ns") != stamp or not isinstance(pose, dict)
                or not isinstance(segment, str) or not segment
                or pose.get("frame_id") != segment + "_odom"
                or pose.get("child_frame_id") != segment + "_base"
                or "calibration_id" not in vendor or vendor.get("pose_reference") not in
                ("body", "body_axes", "rgb_optical")):
            continue
        provenance = vendor.get("camera_time_provenance")
        if not isinstance(provenance, dict) or not provenance.get("epoch"):
            continue
        signature = (pose, vendor["calibration_id"], vendor["pose_reference"], provenance["epoch"])
        if stamp in poses:
            if poses[stamp] is None or poses[stamp][1] != signature:
                poses[stamp] = None
        else:
            poses[stamp] = (data, signature)
    return poses


def paired_frames(db):
    recording = dict(db.execute("SELECT key, value FROM metadata"))
    rows = defaultdict(list)
    for row_id, stream, stamp, payload in db.execute(
            "SELECT id, stream, stamp_ns, header FROM samples ORDER BY id"):
        rows[stream].append((row_id, stamp, json.loads(payload)))
    rgb, depth, info, meta = (index_rows(rows, k) for k in ("rgb", "depth", "info", "camera_meta"))
    poses = valid_poses(rows["odom"])
    mounts = {}
    for _, _, tf in rows["tf_static"]:
        key = tf["parent"], tf["child"]
        mounts[key] = tf if key not in mounts or mounts[key] == tf else None
    result, skipped = {}, Counter()
    for stamp, image in sorted(rgb.items()):
        if not image or not info.get(stamp) or not meta.get(stamp):
            skipped["missing_or_ambiguous_rgb_info_metadata"] += 1
            continue
        data = meta[stamp][1]
        depth_stamp = data.get("depth_stamp_ns")
        if (type(depth_stamp) is not int or abs(stamp - depth_stamp) > MAX_SKEW_NS
                or data.get("rgb_stamp_ns") != stamp or not depth.get(depth_stamp)):
            skipped["missing_or_unsynced_depth"] += 1
            continue
        if not poses.get(stamp):
            skipped["no_matching_tracking_pose"] += 1
            continue
        odom, signature = poses[stamp]
        if not data.get("epoch") or data["epoch"] != signature[3]:
            skipped["camera_epoch_mismatch"] += 1
            continue
        _, color = image
        _, metres = depth[depth_stamp]
        intrinsics = info[stamp][1]
        geometry = (color["frame_id"], color["width"], color["height"])
        if (not geometry[0] or any(
                (h["frame_id"], h["width"], h["height"]) != geometry for h in (metres, intrinsics))
                or color["encoding"] != "bgr8" or metres["encoding"] != "32FC1"
                or metres["is_bigendian"] or intrinsics["distortion_model"] != "plumb_bob"):
            skipped["incompatible_image_geometry"] += 1
            continue
        mount = mounts.get((odom["pose"]["child_frame_id"], geometry[0]))
        if not mount:
            skipped["missing_or_conflicting_mount_tf"] += 1
            continue
        frame = Frame(stamp, image, depth[depth_stamp], intrinsics, data, odom, mount)
        result[frame.image] = frame
    return result, {
        "namespace": recording.get("namespace"),
        "recording_summary": json.loads(recording.get("summary", "null")),
        "stored_samples": {k: len(v) for k, v in rows.items()},
        "matched_frames": len(result), "skipped_rgb": dict(skipped),
        "segments": dict(Counter(f.segment for f in result.values())),
    }


def pixels(db, row, depth=False):
    row_id, header = row
    data, = db.execute("SELECT data FROM samples WHERE id = ?", (row_id,)).fetchone()
    height, width, step = header["height"], header["width"], header["step"]
    if height <= 0 or width <= 0 or step < width * (4 if depth else 3) or len(data) != height * step:
        raise ValueError("invalid image dimensions, stride or byte count")
    if depth:
        return np.ndarray((height, width), dtype="<f4", buffer=data, strides=(step, 4))
    return np.ndarray((height, width, 3), dtype=np.uint8, buffer=data, strides=(step, 3, 1))


def prepare(db, frames, stats, output, every):
    output.mkdir(parents=True, exist_ok=False)
    base_stamp = next(iter(frames.values())).stamp
    with (output / "observations.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(CSV_COLUMNS)
        for index, frame in enumerate(frames.values()):
            if index % every:
                continue
            # imencode avoids OpenCV filename issues when the Windows/WSL path has CJK.
            ok, png = cv2.imencode(".png", pixels(db, frame.rgb))
            if not ok:
                raise ValueError(f"could not encode {frame.image}")
            (output / frame.image).write_bytes(png.tobytes())
            writer.writerow((frame.image, f"{(frame.stamp - base_stamp) / 1e9:.3f}", frame.segment, "", "", ""))
    (output / "capture_summary.json").write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")


def project(db, frame, u, v, radius):
    depth = pixels(db, frame.depth, depth=True)
    height, width = depth.shape
    if not (0 <= u < width and 0 <= v < height):
        raise ValueError("pixel is outside the original RGB image")
    x, y = min(width - 1, int(u + .5)), min(height - 1, int(v + .5))
    patch = depth[max(0, y - radius):y + radius + 1, max(0, x - radius):x + radius + 1]
    valid = patch[np.isfinite(patch) & (patch > .1) & (patch < 10.)]
    if len(valid) < max(1, (patch.size + 1) // 2):
        raise ValueError("too few valid depth pixels at the reference point")
    z = float(np.median(valid))
    k = np.asarray(finite(frame.info["k"], 9)).reshape(3, 3)
    d = np.asarray(finite(frame.info["d"], 5))
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("invalid focal length")
    nx, ny = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), k, d)[0, 0]
    optical = finite([float(nx * z), float(ny * z), z], 3)
    # p_odom = T_odom_base * T_base_optical * p_optical (both TFs use xyzw).
    pose, mount = frame.odom["pose"], frame.mount
    base = [a + b for a, b in zip(rotate(quat_matrix(mount["quaternion"]), optical),
                                  finite(mount["translation"], 3))]
    point = [a + b for a, b in zip(rotate(quat_matrix(pose["quaternion"]), base),
                                   finite(pose["position"], 3))]
    return {"point_odom_m": point, "depth_m": z, "depth_valid_pixels": int(len(valid)),
            "depth_patch_span_m": float(np.ptp(valid))}


def analyze(db, frames, annotations, radius, tolerance):
    observations, used, reference = [], set(), None
    with annotations.open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        if not set(CSV_COLUMNS).issubset(reader.fieldnames or ()):
            raise ValueError("use the observations.csv generated by prepare")
        for row in reader:
            values = [str(row.get(k) or "").strip() for k in ("station", "u", "v")]
            if not any(values):
                continue
            image = row["image"]
            try:
                if not all(values) or image not in frames:
                    raise ValueError("enter station, u and v for a matched image from this capture")
                if image in used:
                    raise ValueError("image annotated more than once")
                used.add(image)
                frame = frames[image]
                if row["segment"] != frame.segment or (reference is not None and frame.reference != reference):
                    raise ValueError("observations must share a segment, pose frame, camera epoch and calibration")
                reference = frame.reference
                station, u, v = values[0], float(values[1]), float(values[2])
                point = project(db, frame, u, v, radius)
                pose = frame.odom["pose"]
                observations.append({"image": image, "station": station, "pixel": [u, v],
                                     "rgb_stamp_ns": frame.stamp,
                                     "depth_stamp_ns": frame.meta["depth_stamp_ns"],
                                     "robot_pose_odom_m": pose["position"],
                                     "robot_pose_quaternion_xyzw": pose["quaternion"], **point})
            except ValueError as error:
                raise ValueError(f"{annotations.name}:{reader.line_num} ({image}): {error}") from error
    groups = defaultdict(list)
    for row in observations:
        groups[row["station"]].append(row["point_odom_m"])
    if len(groups) < 2:
        raise ValueError("annotate the same reference point at two or more differently named stops")
    stations = {}
    for name, points in groups.items():
        center = [median(axis) for axis in zip(*points)]
        stations[name] = {"samples": len(points), "median_point_odom_m": center,
                          "max_offset_from_median_m": max(math.dist(p, center) for p in points)}
    separation = [{"stations": [a, b], "distance_m": math.dist(
                    stations[a]["median_point_odom_m"], stations[b]["median_point_odom_m"])}
                  for a, b in combinations(stations, 2)]
    spread = max(math.dist(a["point_odom_m"], b["point_odom_m"])
                 for a, b in combinations(observations, 2))
    return {
        "segment_id": reference[0], "frame_id": reference[1], "pose_child_frame": reference[2],
        "optical_frame": reference[3], "calibration_id": reference[4],
        "pose_reference": reference[5], "camera_epoch": reference[6],
        "max_point_separation_m": spread, "tolerance_m": tolerance,
        "within_tolerance": None if tolerance is None else spread <= tolerance,
        "stations": stations, "station_separations": separation, "observations": observations,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    export = sub.add_parser("prepare", help="export matched RGB frames and an annotation CSV")
    export.add_argument("capture", type=Path)
    export.add_argument("output", type=Path, help="new preview directory")
    export.add_argument("--every", type=int, default=5, help="export every Nth matched frame")
    check = sub.add_parser("analyze", help="project annotated pixels and report their spread")
    check.add_argument("capture", type=Path)
    check.add_argument("annotations", type=Path)
    check.add_argument("--output", type=Path, required=True, help="new JSON report")
    check.add_argument("--depth-radius", type=int, default=1, help="median depth patch radius in pixels")
    check.add_argument("--tolerance-m", type=float, help="optional maximum separation between any two samples")
    args = parser.parse_args()
    if args.command == "prepare" and args.every < 1:
        parser.error("--every must be positive")
    if args.command == "analyze":
        if not 0 <= args.depth_radius <= 10:
            parser.error("--depth-radius must be between 0 and 10")
        if args.tolerance_m is not None and not (math.isfinite(args.tolerance_m) and args.tolerance_m > 0):
            parser.error("--tolerance-m must be finite and positive")
    try:
        with closing(connect(args.capture)) as db:
            frames, stats = paired_frames(db)
            print(json.dumps(stats, ensure_ascii=False))
            if not frames:
                raise ValueError("no synchronized RGB-D, tracking pose and mount TF samples")
            if args.command == "prepare":
                prepare(db, frames, stats, args.output, args.every)
                return 0
            result = analyze(db, frames, args.annotations, args.depth_radius, args.tolerance_m)
            with args.output.open("x", encoding="utf-8") as file:
                json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
                file.write("\n")
            print(f"Maximum 3-D separation: {result['max_point_separation_m']:.4f} m")
            return 1 if result["within_tolerance"] is False else 0
    except (OSError, sqlite3.Error, ValueError, cv2.error) as error:
        parser.exit(2, f"replay: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
