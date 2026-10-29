"""Read a bounded set of D435i motion samples without starting video streams.

Run with the intended SDK on PYTHONPATH and keep the camera stationary. This is
an IMU access/metadata probe, not VIO or an IMU noise calibration.
Stop other camera capture first: a separate RSUSB context may interrupt it.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import time


def motion_summary(samples, expected_hz, duration_s):
    if len(samples) < 2:
        raise ValueError('insufficient motion samples')
    if len(samples) < .8*expected_hz*duration_s:
        raise ValueError('insufficient motion coverage for the recording duration')
    stamps, domains, vectors = zip(*samples)
    if (len(set(domains)) != 1 or
            not all(math.isfinite(v) for t, _, xyz in samples for v in (t, *xyz)) or
            any(b <= a for a, b in zip(stamps, stamps[1:]))):
        raise ValueError('invalid motion values, clock domain or timestamp order')
    gaps = [b-a for a, b in zip(stamps, stamps[1:])]
    rate = (len(samples)-1)*1000/(stamps[-1]-stamps[0])
    if not .8*expected_hz <= rate <= 1.2*expected_hz or max(gaps) > 5000/expected_hz:
        raise ValueError(f'motion profile {expected_hz} Hz: measured {rate:.2f} Hz, '
                         f'max gap {max(gaps):.2f} ms')
    mean = [statistics.mean(xyz[i] for xyz in vectors) for i in range(3)]
    return dict(count=len(samples), expected_hz=expected_hz, rate_hz=rate,
                first_stamp_ms=stamps[0], last_stamp_ms=stamps[-1],
                max_gap_ms=max(gaps), timestamp_domain=domains[0], mean=mean,
                stddev=[statistics.pstdev(xyz[i] for xyz in vectors) for i in range(3)],
                mean_vector_norm=math.sqrt(sum(v*v for v in mean)))


def transform(extrinsics):
    # rs2_extrinsics.rotation is column-major; write an unambiguous matrix.
    r = [[extrinsics.rotation[col*3+row] for col in range(3)] for row in range(3)]
    t = list(extrinsics.translation)
    cross = [r[1][1]*r[2][2]-r[1][2]*r[2][1],
             r[1][2]*r[2][0]-r[1][0]*r[2][2],
             r[1][0]*r[2][1]-r[1][1]*r[2][0]]
    if (not all(math.isfinite(v) for v in [*t, *sum(r, [])]) or
            abs(sum(a*b for a, b in zip(r[0], cross))-1) > 1e-4 or
            any(abs(sum(a*b for a, b in zip(r[i], r[j]))-(i == j)) > 1e-4
                for i in range(3) for j in range(3))):
        raise ValueError('invalid SDK rigid transform')
    return dict(equation='p_color = R_color_motion * p_motion + t_color_motion',
                rotation_row_major=r, translation_m=t)


def capture(rs, seconds):
    context = rs.context()
    devices = list(context.query_devices())
    if len(devices) != 1:
        raise RuntimeError('probe requires exactly one RealSense device')
    device = devices[0]
    sensors = list(device.query_sensors())
    profiles = [p for s in sensors for p in s.get_stream_profiles()]
    color = next(p for p in profiles if p.stream_type() == rs.stream.color
                 and p.format() == rs.format.bgr8 and p.fps() == 30
                 and p.as_video_stream_profile().width() == 640
                 and p.as_video_stream_profile().height() == 480)
    motion = {}
    for name, kind, rate in [('gyro', rs.stream.gyro, 200), ('accel', rs.stream.accel, 250)]:
        choices = [p for p in profiles if p.stream_type() == kind]
        if not choices:
            raise RuntimeError('SDK does not expose '+name+'; check its HID backend')
        motion[name] = min(choices, key=lambda p: abs(p.fps()-rate))
    sensor = next(s for s in sensors if any(p.stream_type() == rs.stream.gyro
                                          for p in s.get_stream_profiles()))
    extrinsics = {name: transform(p.get_extrinsics_to(color)) for name, p in motion.items()}
    samples = {name: [] for name in motion}

    def receive(frame):
        kind = frame.get_profile().stream_type()
        name = 'gyro' if kind == rs.stream.gyro else 'accel'
        v = frame.as_motion_frame().get_motion_data()
        samples[name].append((frame.get_timestamp(),
                              str(frame.get_frame_timestamp_domain()), [v.x, v.y, v.z]))

    # Keep the device clock: global-time conversion changes domains at startup.
    sensor.set_option(rs.option.global_time_enabled, 0.)
    sensor.open(list(motion.values()))
    queue = rs.frame_queue(256, keep_frames=True)
    try:
        # Enqueue in C++ so the HID reader never waits for Python's GIL.
        sensor.start(queue)
        try:
            # Drain old HID reports before measuring a continuous window.
            deadline = time.monotonic()+2.
            while time.monotonic() < deadline:
                queue.try_wait_for_frame(100)
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                ready, frame = queue.try_wait_for_frame(100)
                if ready:
                    receive(frame)
        finally:
            sensor.stop()
    finally:
        sensor.close()
    summaries = {}
    for name, profile in motion.items():
        try:
            summaries[name] = dict(passed=True, **motion_summary(samples[name], profile.fps(), seconds))
        except ValueError as exc:
            summaries[name] = dict(passed=False, error=str(exc))
    return dict(passed=all(s['passed'] for s in summaries.values()),
                model=device.get_info(rs.camera_info.name),
                firmware=device.get_info(rs.camera_info.firmware_version),
                sdk_version=getattr(rs, '__version__', 'unknown'), duration_s=seconds,
                warmup_s=2., global_time_enabled=False,
                units=dict(gyro='rad/s', accel='m/s^2'),
                motion=summaries,
                motion_to_color=extrinsics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    try:
        import pyrealsense2 as rs
        result = capture(rs, args.seconds)
    except Exception as exc:
        result = dict(passed=False, error=str(exc))
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
