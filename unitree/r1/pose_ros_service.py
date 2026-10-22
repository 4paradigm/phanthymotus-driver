"""Standalone Tianyi RGB subscriber and pose_check MCP service."""
import argparse
import json
import os
import ssl
import sys
import threading
import time
import urllib.request
from pathlib import Path

try:
    # This service is commonly launched outside main.py on Tianyi. Install the
    # driver's line-atomic log writer before MediaPipe or ROS can emit output.
    # A temporary service runs from /tmp, while the driver image keeps common/
    # below /work, so expose that package explicitly in this entry point.
    if Path("/work/common/logsafe.py").is_file() and "/work" not in sys.path:
        sys.path.insert(0, "/work")
    from common import logsafe
    logsafe.install()
except ImportError:
    # Keep the standalone developer workflow usable outside the driver image.
    pass

from pose_estimator import MediaPipePoseEstimator
from pose_mcp import POSE_CHECK_VERSION, POSE_RESULT_TOPIC, PoseCard, make_server


class _FailureReporter:
    """Log a failure transition, then sample repeats instead of flooding logs."""

    def __init__(self, interval_seconds=10.0):
        self._interval_seconds = interval_seconds
        self._failures = {}
        self._lock = threading.Lock()

    def failure(self, key, detail):
        now = time.monotonic()
        with self._lock:
            previous = self._failures.get(key)
            count = 1 if previous is None else previous["count"] + 1
            should_log = previous is None or now - previous["logged_at"] >= self._interval_seconds
            self._failures[key] = {"count": count,
                                   "logged_at": now if should_log else previous["logged_at"]}
        if should_log:
            repeat = "" if count == 1 else f" (repeated {count} times)"
            print(f"[pose_check] {key}: {detail}{repeat}", flush=True)

    def recovered(self, key):
        with self._lock:
            failure = self._failures.pop(key, None)
        if failure is not None:
            print(f"[pose_check] {key} recovered after {failure['count']} failures",
                  flush=True)


def _raw_image_to_bgr(msg):
    """Convert ROS Image bytes to BGR without JPEG re-encoding.

    Tianyi publishes 30 raw RGB frames each second.  Turning every frame into
    JPEG only to decode it again before MediaPipe adds avoidable latency and CPU
    load, so this adapter keeps raw frames raw until model inference.
    """
    import cv2
    import numpy as np
    encoding = str(msg.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4,
                "mono8": 1, "8uc1": 1, "8uc3": 3, "8uc4": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported Image encoding: {msg.encoding}")
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if height <= 0 or width <= 0 or step < width * channels or raw.size < height * step:
        raise ValueError("invalid Image dimensions or buffer")
    rows = raw[:height * step].reshape(height, step)[:, :width * channels]
    frame = rows.reshape(height, width) if channels == 1 else rows.reshape(height, width, channels)
    if encoding == "rgb8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif encoding in ("rgba8", "8uc4"):
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR)
    elif encoding in ("mono8", "8uc1"):
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    # A ROS message buffer can be reused after this callback returns.  Keep an
    # owned frame for the asynchronous inference worker.
    return frame.copy()


def _create_domain_nodes(rclpy, context_type, executor_type):
    """Keep camera ingress on Tianyi's domain 0 and result egress on Core's 42."""
    camera_context = context_type()
    rclpy.init(context=camera_context, domain_id=0)
    core_context = context_type()
    rclpy.init(context=core_context, domain_id=42)
    camera_node = rclpy.create_node('pose_check_camera', context=camera_context)
    core_node = rclpy.create_node('pose_check_result', context=core_context)
    camera_executor = executor_type(context=camera_context)
    camera_executor.add_node(camera_node)
    # A node must be attached to an executor in its own context.  Without this
    # executor, the Domain 42 result publisher can appear locally usable while
    # never taking part reliably in Fast DDS discovery, leaving Decision Core
    # with a topic that has subscribers but no visible publisher.
    core_executor = executor_type(context=core_context)
    core_executor.add_node(core_node)
    return (camera_context, core_context, camera_node, core_node,
            camera_executor, core_executor)


def event_payloads(events, frame_id, session=None):
    """Build one compact, versioned result-topic message per state change."""
    if not events:
        return []
    base = {"schema": "pose_check.event.v1", "source": "pose_check",
            "frame_id": frame_id}
    if isinstance(session, dict) and "session_id" in session:
        progress = session.get("progress") or {}
        base.update({
            "session_id": session["session_id"],
            "state": session.get("state"),
            "progress": {key: progress.get(key) for key in (
                "repetitions", "target_repetitions", "elapsed_seconds",
                "calibrated", "phase") if key in progress},
        })
    return [{**base, **event, "events": [dict(event)]} for event in events]


def main():
    # FastDDS caches its profile process-wide. Select the Tianyi profile
    # before either ROS participant is created: it includes the robot camera
    # interface and loopback for Agent Core without exposing domain 42 on the
    # office LAN. The loopback-only core profile would hide the camera.
    dds_profile = os.environ.get('POSE_DDS_PROFILE', '/work/dds_profile.xml')
    if not os.path.isfile(dds_profile):
        raise RuntimeError(f'Tianyi DDS profile not found: {dds_profile}')
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = dds_profile

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String
    # The Tianyi driver cannot expose its in-process Domain 42 participant to
    # Agent Core directly: it runs under the vendor DDS profile.  The bundle's
    # socket_bridge process owns the loopback-visible participant instead.
    # Reuse that established egress path, rather than publishing straight from
    # this temporary service where DDS discovery would see no publisher.
    from bridged_publisher import create_bridged_publisher

    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--topic', default=os.environ.get('POSE_INPUT_TOPIC', 'auto'))
    parser.add_argument('--message-type', choices=('auto', 'raw', 'compressed'), default='auto')
    parser.add_argument('--port', type=int, default=15740)
    parser.add_argument('--max-fps', type=float, default=10.0,
                        help='maximum inference input rate; retain only newest frame')
    parser.add_argument('--max-width', type=int, default=640,
                        help='downscale frames wider than this before inference; 0 disables')
    args = parser.parse_args()
    if not 1 <= args.max_fps <= 60:
        parser.error('--max-fps must be between 1 and 60')
    if not 0 <= args.max_width <= 4096:
        parser.error('--max-width must be between 0 and 4096')
    model = MediaPipePoseEstimator(args.model, video=True)
    card = PoseCard(source="tianyi_orbbec_rgb" if args.topic == "auto" or
                    args.topic == "/ob_camera_head/color/image_raw" else args.topic)
    (camera_context, core_context, camera_node, core_node,
     camera_executor, core_executor) = _create_domain_nodes(
         rclpy, Context, SingleThreadedExecutor)
    result_topic = POSE_RESULT_TOPIC
    result_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10,
                            durability=DurabilityPolicy.VOLATILE)
    result_pub = create_bridged_publisher(core_node, String, result_topic,
                                          result_qos)

    def publish_events(events, session=None):
        for payload in event_payloads(events, card.frame_id, session):
            result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False,
                                                       allow_nan=False)))
            print(f"[pose_check] {payload['event']}: {payload.get('narration', '')}",
                  flush=True)

    def on_dispatch(action, value):
        # Only state changes go to Decision Core. Querying status or info does
        # not create a duplicate DDS message unless that query discovers a new
        # terminal transition (for example, a timeout).
        if isinstance(value, dict):
            publish_events(value.get("events"),
                           value if "session_id" in value else None)

    server = make_server(card, port=args.port, on_dispatch=on_dispatch)
    lock = threading.Lock()
    latest = [None]
    last_accepted_at = [None]
    stop = threading.Event()
    failures = _FailureReporter()

    def reserve_frame_time():
        """Throttle ingress before decode and retain just one current frame."""
        now = time.monotonic()
        with lock:
            previous = last_accepted_at[0]
            if previous is not None and now - previous < 1.0 / args.max_fps:
                return None
            last_accepted_at[0] = now
        return now

    def receive_compressed(msg):
        received_at = reserve_frame_time()
        if received_at is None:
            return
        with lock:
            latest[0] = ('jpeg', bytes(msg.data), received_at)

    def receive_raw(msg):
        received_at = reserve_frame_time()
        if received_at is None:
            return
        try:
            frame = _raw_image_to_bgr(msg)
        except Exception as exc:
            card.fail('camera_decode_failed')
            failures.failure('camera_decode_failed', exc)
            return
        failures.recovered('camera_decode_failed')
        with lock:
            latest[0] = ('bgr', frame, received_at)

    if args.topic != 'auto':
        candidates = [(args.topic, args.message_type if args.message_type != 'auto' else 'raw')]
    else:
        # A ROS topic cannot have Image and CompressedImage subscriptions at
        # once. Keep the message type paired with each known source.
        candidates = [
            ('/ob_camera_head/color/image_raw', 'raw'),
            ('/nvidia_desktop/camera/head', 'compressed'),
            ('/ubuntu/camera/main', 'compressed'),
        ]
    subscriptions = []
    raw_topics = [topic for topic, kind in candidates if kind == 'raw']
    compressed_topics = [topic for topic, kind in candidates if kind == 'compressed']
    if raw_topics:
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        subscriptions.extend(camera_node.create_subscription(Image, topic, receive_raw, qos)
                             for topic in raw_topics)
    if compressed_topics:
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        subscriptions.extend(camera_node.create_subscription(CompressedImage, topic,
                                                       receive_compressed, qos)
                             for topic in compressed_topics)

    def infer():
        while not stop.wait(0.02):
            with lock:
                item, latest[0] = latest[0], None
            if item is None:
                continue
            try:
                kind, frame, received_at = item
                if kind == 'bgr':
                    points = model.estimate_bgr(frame, max_width=args.max_width)
                else:
                    points = model.estimate(frame, max_width=args.max_width)
                session = card.ingest(points, received_at)
                publish_events(card.drain_events(), session)
                failures.recovered('inference_failed')
            except Exception as exc:
                card.fail('inference_failed')
                failures.failure('inference_failed', exc)

    def register():
        # Same local registration endpoint as the existing Tianyi bundle.
        context = ssl._create_unverified_context()
        payload = json.dumps({
            'id': 'tianyi-pose-check',
            'name': 'Tianyi Pose Check',
            'url': f'http://localhost:{args.port}/mcp',
            'transport': 'http',
            'category': 'driver',
        }).encode()
        while not stop.is_set():
            delay = 30
            try:
                req = urllib.request.Request('https://localhost:15678/api/mcp',
                    data=payload, headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, context=context, timeout=5) as response:
                    print(f'registration: {response.status}', flush=True)
                failures.recovered('registration_failed')
            except Exception as exc:
                failures.failure('registration_failed', exc)
                delay = 5
            stop.wait(delay)

    core_executor_thread = threading.Thread(target=core_executor.spin, daemon=True)
    workers = [threading.Thread(target=infer, daemon=True),
               threading.Thread(target=server.serve_forever, daemon=True),
               threading.Thread(target=register, daemon=True)]
    core_executor_thread.start()
    for worker in workers:
        worker.start()
    print(f'pose_check version={POSE_CHECK_VERSION}, MCP port={args.port}, topic={args.topic}, message_type={args.message_type}, max_fps={args.max_fps:g}, max_width={args.max_width}, candidates={candidates}, result_topic={result_topic}, camera_domain=0, result_domain=42, dds_profile={dds_profile}', flush=True)
    try:
        camera_executor.spin()
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        camera_executor.shutdown()
        core_executor.shutdown()
        core_executor_thread.join()
        for worker in workers:
            worker.join()
        model.close()
        result_pub.destroy()
        camera_node.destroy_node()
        core_node.destroy_node()
        rclpy.shutdown(context=camera_context)
        rclpy.shutdown(context=core_context)


if __name__ == '__main__':
    main()
