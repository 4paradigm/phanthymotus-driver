"""Read-only Unitree G1 SLAM evidence capture (no mapping or motion commands).

Run in a G1 driver-compatible environment:
    python3 /work/slam_capture_probe.py --iface eth0 --duration 30

Own DDS domain: Unitree SDK domain 0, not ROS 2 domain 42.
This is an independent read-only subscriber, not a tracking/clock validator.
Do not treat receipt time as sensor acquisition time.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import queue
import time
from uuid import uuid4


SOURCES = (
    ('odom', 'rt/unitree/slam_mapping/odom', 'nav_msgs'),
    ('cloud', 'rt/utlidar/cloud_livox_mid360', 'sensor_msgs'),
    ('slam_info', 'rt/slam_info', 'std_msgs'),
    ('slam_key_info', 'rt/slam_key_info', 'std_msgs'),
)


def source_header(msg):
    """Copy the native header without interpreting its clock domain."""
    h = getattr(msg, 'header', None)
    stamp = getattr(h, 'stamp', None)
    return {
        'frame_id': getattr(h, 'frame_id', None),
        'stamp': {
            'sec': getattr(stamp, 'sec', None),
            'nanosec': getattr(stamp, 'nanosec', None),
        },
    }


def to_record(kind, msg, *, received_ns, received_monotonic_ns,
              max_status_chars=4096):
    """Bounded JSON evidence, with NO full point-cloud payload or clock claim."""
    if kind not in {k for k, _, _ in SOURCES}:
        raise ValueError('unknown capture topic kind')
    row = {
        'source_kind': kind,
        'source_topic': next(t for k, t, _ in SOURCES if k == kind),
        'received_ns': received_ns,
        'received_monotonic_ns': received_monotonic_ns,
        'measurement_clock_verified': False,
        'tracking_verified': False,
    }
    if kind == 'odom':
        pose = msg.pose.pose
        q, p = pose.orientation, pose.position
        row.update(
            source_header=source_header(msg),
            child_frame_id=msg.child_frame_id,
            position_xyz=[p.x, p.y, p.z],
            quaternion_xyzw=[q.x, q.y, q.z, q.w],
            covariance=list(msg.pose.covariance),
        )
    elif kind == 'cloud':
        row.update(
            source_header=source_header(msg),
            width=msg.width, height=msg.height,
            point_step=msg.point_step, row_step=msg.row_step,
            is_bigendian=msg.is_bigendian,
            is_dense=msg.is_dense,
            fields=[{'name': f.name, 'offset': f.offset,
                     'datatype': f.datatype, 'count': f.count}
                    for f in msg.fields],
            data_length_bytes=len(msg.data),
        )
    else:
        raw = getattr(msg, 'data', None)
        if raw is None:
            raw = getattr(msg, 'data_', None)
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', 'replace')
        raw = str(raw) if raw is not None else ''
        row['status_payload'] = raw[:max_status_chars]
        row['status_payload_truncated'] = len(raw) > max_status_chars
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read-only G1 SLAM DDS evidence sampler')
    parser.add_argument('--iface', default=os.environ.get('NETWORK_INTERFACE', 'eth0'))
    parser.add_argument('--duration', type=float, default=30., help='seconds, max 120')
    parser.add_argument('--output-dir',
                        default='/opt/phanthy-motus/data/slam_capture',
                        help='persistent host-mounted output directory')
    parser.add_argument('--max-records', type=int, default=20000)
    args = parser.parse_args(argv)
    if (not 0 < args.duration <= 120 or not math.isfinite(args.duration)
            or not 1 <= args.max_records <= 20000):
        parser.error('duration must be 0..120 and max-records must be 1..20000')

    # Lazy import: conversion and CLI validation remain ROS-free and testable.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    classes = {'nav_msgs': Odometry_, 'sensor_msgs': PointCloud2_, 'std_msgs': String_}
    ChannelFactoryInitialize(0, args.iface)
    generation = uuid4().hex  # sampler identity; NOT sensor epoch
    messages = queue.Queue(maxsize=256)
    counts = Counter()
    subscribers = []
    errors = Counter()

    def callback(kind):
        def on_message(msg):
            try:
                row = to_record(kind, msg, received_ns=time.time_ns(),
                                received_monotonic_ns=time.monotonic_ns())
                row['sampler_generation'] = generation
                # Reject unrepresentable JSON (NaN etc.) rather than write
                # non-standard JSON or repair the robot's original values.
                encoded = json.dumps(row, allow_nan=False, ensure_ascii=False)
                messages.put_nowait((kind, encoded))
            except (AttributeError, TypeError, ValueError, OverflowError):
                errors[kind + '_invalid'] += 1
            except queue.Full:
                errors[kind + '_overflow'] += 1
        return on_message

    for kind, topic, type_name in SOURCES:
        sub = ChannelSubscriber(topic, classes[type_name])
        sub.Init(callback(kind), 1)
        subscribers.append(sub)  # Keep DDS readers alive until sampling ends.
        print('subscribe', topic, flush=True)

    dest = Path(args.output_dir)
    dest.mkdir(parents=True, exist_ok=True)
    file = dest / ('g1_vendor_' + time.strftime('%Y%m%d_%H%M%S') + '.jsonl')
    written = 0
    end = time.monotonic() + args.duration
    with file.open('x', encoding='utf-8') as output:
        while time.monotonic() < end and written < args.max_records:
            try:
                kind, encoded = messages.get(timeout=.2)
            except queue.Empty:
                continue
            output.write(encoded + '\n')
            written += 1
            counts[kind] += 1

    summary = {
        'capture_file': str(file), 'duration_requested_s': args.duration,
        'sampler_generation': generation, 'counts': dict(counts),
        'errors': dict(errors),
        'measurement_clock_verified': False, 'tracking_verified': False,
        'note': 'read-only receipt and native-message evidence, not SLAM admission',
    }
    summary_file = file.with_suffix('.summary.json')
    summary_file.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n',
                            encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
