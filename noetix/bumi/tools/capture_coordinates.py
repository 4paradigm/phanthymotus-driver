"""Record Bumi RGB-D, calibration, visual odometry and static TF to SQLite."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import shutil
import sqlite3
import time


def open_capture(path, namespace):
    # Reserve the filename without overwriting an existing field recording.
    path.touch(exist_ok=False)
    db = sqlite3.connect(path)
    db.executescript("""
        PRAGMA user_version = 1;
        CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE samples (
            id INTEGER PRIMARY KEY, stream TEXT NOT NULL, stamp_ns INTEGER,
            received_ns INTEGER NOT NULL, header TEXT NOT NULL, data BLOB
        );
        CREATE INDEX sample_stream_stamp ON samples(stream, stamp_ns);
    """)
    db.executemany("INSERT INTO metadata VALUES (?, ?)", [
        ("namespace", namespace), ("started_ns", str(time.time_ns())),
    ])
    db.commit()
    return db


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def transform_data(message):
    trans, rot = message.transform.translation, message.transform.rotation
    return {
        "parent": message.header.frame_id, "child": message.child_frame_id,
        "translation": [trans.x, trans.y, trans.z],
        "quaternion": [rot.x, rot.y, rot.z, rot.w],
    }


class Recorder:
    def __init__(self, db, namespace, max_bytes):
        self.db = db
        self.optical_frame = f"{namespace}_rgb_optical"
        self.max_bytes = max_bytes
        self.size = 0
        self.counts = Counter()
        self.tracking = 0
        self.invalid_json = Counter()
        self.full = False
        self.consecutive_tf = {}

    def write(self, stream, stamp, header, data=None):
        payload = json.dumps(header, separators=(",", ":"), allow_nan=False)
        size = len(payload.encode()) + (len(data) if data is not None else 0)
        if self.full or self.size + size > self.max_bytes:
            self.full = True
            return False
        self.db.execute("INSERT INTO samples(stream, stamp_ns, received_ns, header, data) VALUES (?,?,?,?,?)",
                        (stream, stamp, time.time_ns(), payload, data))
        self.size += size
        self.counts[stream] += 1
        return True

    def image(self, stream, msg):
        self.write(stream, stamp_ns(msg), {
            "frame_id": msg.header.frame_id, "width": msg.width, "height": msg.height,
            "encoding": msg.encoding, "is_bigendian": msg.is_bigendian, "step": msg.step,
        }, bytes(msg.data))

    def info(self, msg):
        self.write("info", stamp_ns(msg), {
            "frame_id": msg.header.frame_id, "width": msg.width, "height": msg.height,
            "distortion_model": msg.distortion_model, "k": list(msg.k), "d": list(msg.d),
            "r": list(msg.r), "p": list(msg.p),
        })

    def json_message(self, stream, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("expected a JSON object")
            if stream == "odom":
                vendor = payload.get("vendor")
                if not isinstance(vendor, dict):
                    raise ValueError("expected odometry vendor fields")
                stamp = vendor.get("visual_stamp_ns")
                if type(stamp) is not int:
                    stamp = None
                if self.write("odom", stamp, payload) and vendor.get("status") == "tracking":
                    self.tracking += 1
            else:
                stamp = payload.get("rgb_stamp_ns")
                if type(stamp) is not int:
                    raise ValueError("missing integer rgb_stamp_ns")
                self.write("camera_meta", stamp, payload)
        except (ValueError, RecursionError):
            self.invalid_json[stream] += 1

    def static_tf(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id != self.optical_frame:
                continue
            row = transform_data(tf)
            if row != self.consecutive_tf.get(row["parent"]):
                self.write("tf_static", stamp_ns(tf), row)
                self.consecutive_tf[row["parent"]] = row

    def summary(self, stop_reason):
        return {
            "samples": dict(self.counts), "tracking_odom_samples": self.tracking,
            "invalid_json": dict(self.invalid_json), "payload_bytes": self.size,
            "stop_reason": stop_reason,
        }


def record(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    max_bytes = int(args.max_gib * 1024**3)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(args.output.parent).free < max_bytes + 32 * 1024**2:
        raise ValueError("insufficient free space for the requested capture budget")
    db = open_capture(args.output, args.namespace)
    recorder = Recorder(db, args.namespace, max_bytes)
    stop_reason = "error"
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("bumi_coordinate_recorder")
        data_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        root = f"/{args.namespace}/odometry_input"
        node.create_subscription(Image, root + "/rgb", lambda msg: recorder.image("rgb", msg), data_qos)
        node.create_subscription(Image, root + "/depth", lambda msg: recorder.image("depth", msg), data_qos)
        node.create_subscription(CameraInfo, root + "/info", recorder.info, data_qos)
        node.create_subscription(String, root + "/camera_meta",
                                 lambda msg: recorder.json_message("camera_meta", msg), data_qos)
        node.create_subscription(String, f"/{args.namespace}/bumi_visual_odometry/state/odom",
                                 lambda msg: recorder.json_message("odom", msg), data_qos)
        node.create_subscription(TFMessage, "/tf_static", recorder.static_tf, tf_qos)
        print(f"Recording /{args.namespace} for up to {args.duration:g} s into {args.output}", flush=True)
        end = time.monotonic() + args.duration
        next_commit = time.monotonic() + 0.5
        while rclpy.ok() and not recorder.full and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_commit:
                db.commit()
                next_commit = time.monotonic() + 0.5
        stop_reason = "payload_budget" if recorder.full else "duration" if rclpy.ok() else "interrupted"
    except (KeyboardInterrupt, ExternalShutdownException):
        stop_reason = "interrupted"
    finally:
        try:
            summary = recorder.summary(stop_reason)
            db.execute("INSERT INTO metadata VALUES (?, ?)", ("summary", json.dumps(summary)))
            db.commit()
        finally:
            db.close()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
    print(json.dumps(summary), flush=True)
    required = ("rgb", "depth", "info", "camera_meta", "odom", "tf_static")
    if not recorder.tracking or any(not recorder.counts[name] for name in required):
        print("Capture incomplete: check namespace, rgbd_visual_odometry tracking and TF.")
        return 2
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", required=True, help="actual Bumi ROS namespace, e.g. ubuntu")
    parser.add_argument("--output", required=True, type=Path, help="new SQLite file")
    parser.add_argument("--duration", type=float, default=60, help="seconds, including DDS discovery")
    parser.add_argument("--max-gib", type=float, default=2, help="payload size budget in GiB")
    args = parser.parse_args()
    args.namespace = args.namespace.strip("/")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.namespace):
        parser.error("namespace must be the single ROS name used by the Bumi driver")
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 300:
        parser.error("--duration must be between 1 and 300 seconds")
    if not math.isfinite(args.max_gib) or not 0.05 <= args.max_gib <= 10:
        parser.error("--max-gib must be between 0.05 and 10")
    try:
        return record(args)
    except (ImportError, OSError, ValueError, sqlite3.Error) as error:
        parser.exit(2, f"capture: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
