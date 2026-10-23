#!/usr/bin/env python3
"""
q5_bridge_worker.py — 独立子进程 DDS bridge，将 Q5 sensor snapshot 发布到 Domain 42 (FastDDS)。

架构：
  - 子进程（spawn 模式）拥有独立的 rclpy 节点（Domain 42/FastDDS）
  - 父进程通过 multiprocessing.Queue 推送 sensor snapshot
  - 数据驱动发布：收到 snapshot 立即转换为标准 ROS2 msg 并发布（实时，非 2Hz polling）
  - 发布类型：sensor_msgs/JointState, sensor_msgs/BatteryState, sensor_msgs/Imu,
             nav_msgs/Odometry, std_msgs/String 等

与 G1 safety_harness.py 模式一致：独立 subprocess 拥有自己的 DDS + ROS2 节点。

父进程调用方式：
    from q5_bridge_worker import BridgeWorker
    bridge = BridgeWorker("q5")
    bridge.start()
    ...
    bridge.shutdown()

子进程通过命令行 args 获取 Queue：
    python3 q5_bridge_worker.py _cmd_q_pid <pid> _sensor_q_pid <pid> _debug <0|1>
    Queue 对象由 spawn target 函数接收。
"""

import multiprocessing as mp
from collections import deque
import os
import queue
import sys
import threading
import time

from fastdds_transport import configure_bundled_fastdds_transport
from playback_telemetry import parse_audio_format


_SUPPORTED_SPEAKER_FORMATS = {"audio/pcm-16k", "pcm_16k_16bit_mono"}
_SPEAKER_FAILURE_DEDUPE_LIMIT = 256
_SPEAKER_FAILURE_MERGE_LIMIT = 64
_POSE_QUEUE_SIZE = 256
_MAX_POSE_JSON_BYTES = 1_048_576


def pose_message_item(msg, clock_ns=time.monotonic_ns,
                      max_raw_bytes: int = _MAX_POSE_JSON_BYTES) -> dict:
    """Convert one ROS String message without interpreting its source clock."""
    raw_json = str(msg.data)
    byte_limit = max(1, int(max_raw_bytes))
    if (len(raw_json) > byte_limit
            or len(raw_json.encode("utf-8")) > byte_limit):
        raise ValueError("Pose JSON exceeds maximum byte size")
    return {
        "kind": "pose",
        "raw_json": raw_json,
        "bridge_received_monotonic_ns": int(clock_ns()),
    }


def pose_barrier_item(token: str, clock_ns=time.monotonic_ns,
                      bridge_transport_drops: int = 0) -> dict:
    """Create an in-band marker ordered after earlier Pose callbacks."""
    return {
        "kind": "barrier",
        "token": str(token),
        "bridge_received_monotonic_ns": int(clock_ns()),
        "bridge_transport_drops": max(0, int(bridge_transport_drops)),
    }


def _increment_shared_counter(counter, amount: int = 1):
    with counter.get_lock():
        counter.value += int(amount)


def _shared_counter_value(counter) -> int:
    with counter.get_lock():
        return int(counter.value)


def _offer_pose(pose_q, item: dict, transport_drops) -> bool:
    """Offer without blocking, replacing the oldest item under overload."""
    try:
        pose_q.put_nowait(dict(item))
        return True
    except queue.Full:
        pass
    try:
        pose_q.get_nowait()
    except queue.Empty:
        _increment_shared_counter(transport_drops)
        return False
    _increment_shared_counter(transport_drops)
    try:
        pose_q.put_nowait(dict(item))
        return True
    except queue.Full:
        _increment_shared_counter(transport_drops)
        return False


def _offer_pose_barrier(pose_q, item: dict, transport_drops=None,
                        timeout_seconds: float = 0.25) -> bool:
    """Boundedly wait for a barrier slot without evicting a Pose frame."""
    del transport_drops
    try:
        pose_q.put(dict(item), timeout=max(0.0, float(timeout_seconds)))
        return True
    except queue.Full:
        return False


def _queue_speaker_failure(failure_q, failure: dict, seen_ids: dict,
                           overflow_epoch=None) -> bool:
    """Offer one failure without blocking; coalesce overload conservatively."""
    speech_id = str(failure.get("speech_id") or "")
    if not speech_id or speech_id in seen_ids:
        return False

    item = dict(failure)
    try:
        failure_q.put_nowait(item)
    except queue.Full:
        failed_epoch = int(item.get("speaker_epoch", -1))
        if overflow_epoch is not None:
            lock = overflow_epoch.get_lock()
            with lock:
                overflow_epoch.value = max(
                    int(overflow_epoch.value), failed_epoch)
        evicted = None
        try:
            evicted = failure_q.get_nowait()
        except queue.Empty:
            pass
        speech_ids = []
        for candidate in (evicted or {}).get("speech_ids", []):
            if str(candidate) not in speech_ids:
                speech_ids.append(str(candidate))
        for candidate in ((evicted or {}).get("speech_id"), speech_id):
            if candidate and str(candidate) not in speech_ids:
                speech_ids.append(str(candidate))
        item = {
            "fail_all": True,
            "speech_id": speech_id,
            "speech_ids": speech_ids[-_SPEAKER_FAILURE_MERGE_LIMIT:],
            "through_epoch": failed_epoch,
            "code": "SPEAKER_FAILURE_QUEUE_OVERFLOW",
            "message": "speaker failure notification queue overloaded",
        }
        seen_ids[speech_id] = None
        while len(seen_ids) > _SPEAKER_FAILURE_DEDUPE_LIMIT:
            del seen_ids[next(iter(seen_ids))]
        try:
            failure_q.put_nowait(item)
        except queue.Full:
            return False
    else:
        seen_ids[speech_id] = None
        while len(seen_ids) > _SPEAKER_FAILURE_DEDUPE_LIMIT:
            del seen_ids[next(iter(seen_ids))]
    return True


def speaker_chunk_from_message(msg) -> dict:
    """Validate an AudioChunk and retain optional utterance metadata."""
    metadata = parse_audio_format(str(msg.format))
    if metadata.base_format not in _SUPPORTED_SPEAKER_FORMATS:
        raise ValueError(f"unsupported audio format: {metadata.base_format!r}")
    if not metadata.valid:
        raise ValueError(metadata.error or "invalid playback metadata")
    return {"pcm": bytes(msg.data), "metadata": metadata}


class BridgeWorker:
    """Subprocess bridge that publishes Q5 sensor snapshots to Domain 42 (FastDDS)."""

    def __init__(self, namespace: str = "q5", debug: bool = False):
        self._ctx = mp.get_context("spawn")
        self._cmd_q = self._ctx.Queue()
        self._sensor_q = self._ctx.Queue()
        # Each stream owns its own latest-frame lane. A shared FIFO lets the
        # 15 Hz RGB stream evict slower depth/pointcloud packets before the
        # bridge has a chance to classify them.
        self._media_qs = {
            "rgb": self._ctx.Queue(maxsize=2),
            "depth_jpeg": self._ctx.Queue(maxsize=2),
            "pointcloud": self._ctx.Queue(maxsize=2),
        }
        self._media_lock = threading.Lock()
        self._audio_q = self._ctx.Queue(maxsize=100)
        self._speaker_q = self._ctx.Queue(maxsize=64)
        self._speaker_fail_q = self._ctx.Queue(maxsize=64)
        self._speaker_fail_epoch = self._ctx.Value("q", -1)
        self._speaker_seen_fail_epoch = -1
        self._pose_q = self._ctx.Queue(maxsize=_POSE_QUEUE_SIZE)
        self._pose_transport_drops = self._ctx.Value("q", 0)
        self._proc = None
        self._debug = debug
        self._namespace = namespace

    def start(self):
        """Spawn the bridge subprocess."""
        self._proc = self._ctx.Process(
            target=_run_bridge_subprocess,
            args=(self._cmd_q, self._sensor_q, self._media_qs, self._audio_q, self._speaker_q,
                  self._speaker_fail_q, self._speaker_fail_epoch,
                  self._pose_q, self._pose_transport_drops,
                  self._debug, self._namespace),
            name="q5_bridge_worker", daemon=True,
        )
        self._proc.start()
        print(f"[BridgeWorker] subprocess started → pid={self._proc.pid}")

    def push_snapshot(self, snap: dict):
        """Push a sensor snapshot to the bridge subprocess (non-blocking)."""
        try:
            self._sensor_q.put_nowait(snap)
        except Exception:
            pass

    def push_media(self, media: dict):
        """Replace only an older frame from the same media stream."""
        kind = media.get("kind") if isinstance(media, dict) else None
        media_q = self._media_qs.get(kind)
        if media_q is None:
            return
        with self._media_lock:
            try:
                media_q.put_nowait(media)
                return
            except queue.Full:
                pass
            # Discard stale data from this stream only, then retry once. The
            # queue remains bounded even if Agent Core is temporarily slower
            # than the camera.
            try:
                media_q.get_nowait()
            except queue.Empty:
                pass
            try:
                media_q.put_nowait(media)
            except queue.Full:
                pass

    def push_audio(self, audio: bytes):
        """Queue ordered PCM audio independently of lossy latest-frame media."""
        try:
            self._audio_q.put_nowait(audio)
        except Exception:
            pass

    def configure_speaker(self, topic: str):
        try:
            self._cmd_q.put_nowait({"kind": "speaker_config", "topic": topic})
        except Exception:
            pass

    def publish_speaker_event(self, event: dict):
        try:
            self._cmd_q.put_nowait({"kind": "speaker_event", "event": dict(event)})
        except Exception:
            pass

    def pop_speaker_failure(self):
        with self._speaker_fail_epoch.get_lock():
            failed_epoch = int(self._speaker_fail_epoch.value)
        if failed_epoch > self._speaker_seen_fail_epoch:
            self._speaker_seen_fail_epoch = failed_epoch
            return {
                "fail_all": True,
                "through_epoch": failed_epoch,
                "code": "SPEAKER_FAILURE_QUEUE_OVERFLOW",
                "message": "speaker failure notification queue overloaded",
            }
        try:
            return self._speaker_fail_q.get_nowait()
        except queue.Empty:
            return None

    def pop_speaker_chunk(self):
        try:
            return self._speaker_q.get_nowait()
        except Exception:
            return None

    def configure_pose(self, topic: str):
        try:
            self._cmd_q.put_nowait({"kind": "pose_config", "topic": str(topic)})
        except Exception:
            pass

    def request_pose_barrier(self, token: str):
        try:
            self._cmd_q.put_nowait({"kind": "pose_barrier", "token": str(token)})
        except Exception:
            pass

    def pop_pose(self):
        try:
            return self._pose_q.get_nowait()
        except queue.Empty:
            return None

    def pose_stats(self) -> dict:
        with self._pose_transport_drops.get_lock():
            drops = int(self._pose_transport_drops.value)
        return {"bridge_transport_drops": drops}

    def shutdown(self):
        """Gracefully stop the bridge subprocess."""
        try:
            self._cmd_q.put_nowait("shutdown")
            self._proc.join(timeout=5)
        except Exception:
            pass
        if self._proc and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=2)
        print("[BridgeWorker] subprocess stopped")


def _run_bridge_subprocess(cmd_q: mp.Queue, sensor_q: mp.Queue, media_qs: dict[str, mp.Queue],
                           audio_q: mp.Queue, speaker_q: mp.Queue,
                          speaker_fail_q: mp.Queue, speaker_fail_epoch,
                          pose_q: mp.Queue, pose_transport_drops,
                          debug: bool, namespace: str):
    """Subprocess entry point — runs in separate process with own DDS domain."""
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError as exc:
        sys.stderr.write(
            f"[BridgeWorker] logsafe unavailable ({exc}); stdout unprotected\n")

    # ── Environment: Force Domain 42 + FastDDS in subprocess ────────────────────
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    # The bridge is in a separate Docker process from Agent Core. Force the
    # bundled loopback-only UDP profile that is verified to carry live PCM
    # across that boundary, regardless of the fleet profile inherited by the
    # container.
    profile = configure_bundled_fastdds_transport()

    import hashlib
    import json
    import signal

    import rclpy
    import rclpy.executors
    import rclpy.qos
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, UInt8MultiArray
    from sensor_msgs.msg import CompressedImage, Image
    from audio_msgs.msg import AudioChunk

    print(f"[BridgeWorker:pid={os.getpid()}] subprocess ready (Domain 42/FastDDS)", flush=True)

    # ── DDS/ROS2 init ──────────────────────────────────────────────────────────
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    node = Node("q5_bridge_worker")

    QOS_SENSOR = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    QOS_MEDIA = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    QOS_AUDIO = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
    )

    # ── Publishers ─────────────────────────────────────────────────────────────
    def _pub(topic):
        return node.create_publisher(String, topic, QOS_SENSOR)

    prefix = f"/{namespace}"
    pub_joint = _pub(f"{prefix}/q5/joints_state")
    pub_joint_json = _pub(f"{prefix}/q5/joints_state_json")
    pub_dynjoint = _pub(f"{prefix}/q5/dynamic_joint_states")
    pub_servo = _pub(f"{prefix}/q5/servo_pose")
    pub_status = _pub(f"{prefix}/q5/robot_status")
    pub_status_json = _pub(f"{prefix}/q5/robot_status_json")
    pub_imu = _pub(f"{prefix}/imu/data")
    pub_batt = _pub(f"{prefix}/battery_state")
    pub_fault = _pub(f"{prefix}/faults")
    pub_hand = _pub(f"{prefix}/hand_sensor")
    pub_odom = _pub(f"{prefix}/odom")
    pub_rgb = node.create_publisher(CompressedImage, f"{prefix}/camera/rgb", QOS_MEDIA)
    pub_depth = node.create_publisher(Image, f"{prefix}/camera/depth", QOS_MEDIA)
    pub_depth_preview = node.create_publisher(CompressedImage, f"{prefix}/camera/depth_preview", QOS_MEDIA)
    pub_pointcloud = node.create_publisher(UInt8MultiArray, f"{prefix}/camera/pointcloud", QOS_MEDIA)
    pub_speaker_events = node.create_publisher(
        String, f"{prefix}/speaker/playback_events", QOS_MEDIA)
    # Audio is a live lossy stream. Match the audio cards used by the other
    # drivers so Agent Core/FastDDS subscribers can request BEST_EFFORT without
    # a reliability negotiation mismatch or unnecessary retransmission delay.
    pub_mic = node.create_publisher(AudioChunk, f"{prefix}/mic/audio", QOS_AUDIO)
    speaker_sub = None
    pose_sub = None
    mic_frames_published = 0
    speaker_frames_received = 0
    speaker_duplicate_frames_dropped = 0
    last_speaker_digest = b""
    last_speaker_frame_at = 0.0
    invalid_format_count = 0
    last_invalid_format_warning_at = None
    speaker_failed_ids = {}
    speaker_epoch = int(speaker_fail_epoch.value) + 1
    pending_pose_barriers = deque(maxlen=64)

    def _on_pose(msg):
        try:
            item = pose_message_item(msg)
        except ValueError:
            _increment_shared_counter(pose_transport_drops)
            return
        _offer_pose(pose_q, item, pose_transport_drops)

    def _on_speaker(msg):
        nonlocal speaker_frames_received, speaker_duplicate_frames_dropped
        nonlocal last_speaker_digest, last_speaker_frame_at
        nonlocal invalid_format_count, last_invalid_format_warning_at
        nonlocal speaker_epoch
        # Agent Core's remote microphone uses pcm_16k_16bit_mono while
        # perception TTS uses audio/pcm-16k. Both carry S16_LE, 16 kHz mono.
        try:
            item = speaker_chunk_from_message(msg)
            if (item["metadata"].tracked
                    and item["metadata"].speech_id in speaker_failed_ids):
                return
            item["speaker_epoch"] = speaker_epoch
            pcm = item["pcm"]
            # Two phanthy_bus_bridge publishers were observed on connected
            # browser-mic topics.  They can relay the exact same packet within
            # milliseconds; submitting both makes live speech garbled.  Do not
            # collapse regular repeated samples (such as silence), only an
            # identical packet that arrives in this short duplicate window.
            if not item["metadata"].tracked:
                now = time.monotonic()
                digest = hashlib.blake2s(pcm, digest_size=8).digest()
                browser_frame = msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
                if (browser_frame and digest == last_speaker_digest
                        and now - last_speaker_frame_at < 0.02):
                    speaker_duplicate_frames_dropped += 1
                    return
                last_speaker_digest = digest
                last_speaker_frame_at = now
            try:
                speaker_q.put_nowait(item)
            except queue.Full:
                # Do not lose tracked EOF or other tracked chunks silently.
                if item["metadata"].tracked:
                    _queue_speaker_failure(speaker_fail_q, {
                        "speech_id": item["metadata"].speech_id,
                        "speaker_epoch": speaker_epoch,
                        "code": "SPEAKER_QUEUE_OVERFLOW",
                        "message": "tracked speaker chunk dropped because the queue is full",
                    }, speaker_failed_ids, speaker_fail_epoch)
                    speaker_epoch += 1
                return
            speaker_frames_received += 1
            if speaker_frames_received == 1:
                node.get_logger().info("speaker received first PCM frame")
            elif speaker_duplicate_frames_dropped and speaker_frames_received % 100 == 0:
                node.get_logger().info(
                    f"speaker dropped {speaker_duplicate_frames_dropped} duplicate PCM frames")
        except ValueError:
            invalid_format_count += 1
            now = time.monotonic()
            if (last_invalid_format_warning_at is None
                    or now - last_invalid_format_warning_at >= 5.0):
                node.get_logger().warning(
                    f"speaker rejected {invalid_format_count} chunks with "
                    "unsupported audio format or invalid metadata")
                invalid_format_count = 0
                last_invalid_format_warning_at = now
        except Exception:
            pass

    node.get_logger().info(f"bridge publishers ready for namespace={namespace}")
    executor.add_node(node)

    # ── Helpers ────────────────────────────────────────────────────────────────
    _clock = node.get_clock()

    def _publish_json(pub, snap):
        msg = String()
        msg.data = json.dumps(snap, ensure_ascii=False)
        pub.publish(msg)

    def _publish_robot_status(snap):
        """Publish robot status as String message."""
        msg = String()
        msg.data = json.dumps({
            "state": snap.get("state", 0),
            "message": snap.get("message", ""),
            "timestamp_ms": snap.get("received_at_ms", int(time.time() * 1000)),
            "fresh": snap.get("fresh", False),
            "age_ms": snap.get("age_ms")
        }, ensure_ascii=False)
        pub_status.publish(msg)
        pub_status_json.publish(msg)

    def _dispatch_snapshot(snap):
        if not snap:
            return

        # /joint_states
        if snap.get("available") and "joints" in snap:
            data = {"source_topic": "/joint_states", **snap}
            _publish_json(pub_joint, data); _publish_json(pub_joint_json, data)
            _publish_json(pub_dynjoint, data); _publish_json(pub_servo, data)

        # /battery_state
        bat = snap.get("_sensor_battery")
        if bat:
            version = snap.get("_sensor_battery_version") or {}
            _publish_json(pub_batt, {"source_topic": "/battery_state", **bat,
                                     "firmware": version.get("components", {}),
                                     "firmware_source": version.get("source_service")})

        # IMU: linear_acceleration → accel topic, angular_velocity → gyro topic
        imu = snap.get("_sensor_imu")
        if imu:
            vec_accel = imu.get("linear_acceleration")
            vec_gyro = imu.get("angular_velocity")
            if vec_accel or vec_gyro:
                _publish_json(pub_imu, {"source_topic": "/camera/camera", **imu})

        # /fault_array
        faults = snap.get("_sensor_faults")
        if faults:
            _publish_json(pub_fault, {"source_topic": "/fault_array", **faults})

        # /hand_sensor
        hand = snap.get("_sensor_hand")
        if hand:
            _publish_json(pub_hand, {"source_topic": "/hand_sensor", **hand})

        # /wr1_base_drive_controller/odom
        odom = snap.get("_sensor_odom")
        if odom:
            _publish_json(pub_odom, {"source_topic": "/wr1_base_drive_controller/odom", **odom})

        # /xbot_state - robot status
        robot_status = snap.get("_sensor_robot_status") or snap.get("_sensor_query_state")
        if robot_status:
            _publish_robot_status(robot_status)

    def _dispatch_media(media):
        kind = media.get("kind")
        if kind == "rgb":
            out = CompressedImage()
            out.header.stamp = node.get_clock().now().to_msg()
            out.format = "jpeg"
            out.data = media["data"]
            pub_rgb.publish(out)
        elif kind == "depth_jpeg":
            out = CompressedImage()
            out.header.stamp = node.get_clock().now().to_msg()
            out.format = "jpeg"
            out.data = media["data"]
            pub_depth_preview.publish(out)
        elif kind == "depth":
            # Backward-compatible raw-depth path; currently unused by Q5.
            out = Image()
            out.header.stamp = node.get_clock().now().to_msg()
            out.height = int(media["height"])
            out.width = int(media["width"])
            out.encoding = str(media["encoding"])
            out.is_bigendian = int(media["is_bigendian"])
            out.step = int(media["step"])
            out.data = media["data"]
            pub_depth.publish(out)
        elif kind == "pointcloud":
            # Agent Core's point-cloud renderer consumes this compact binary
            # envelope: uint32 point_step, uint32 count, float32 xyz * count.
            out = UInt8MultiArray()
            out.data = list(media["data"])
            pub_pointcloud.publish(out)

    # ── Main loop ──────────────────────────────────────────────────────────────
    running = True
    last_log = time.time()

    def _handle_signal(signum, frame):
        nonlocal running
        if debug:
            node.get_logger().info(f"signal {signum} received")
        running = False

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    while running:
        # Commands are rare.  Do not block this loop waiting for one: browser
        # mic uploads arrive as bursts of 1 KiB DDS samples, and a 50 ms wait
        # here caps callback processing below their ~30 Hz stream rate.
        try:
            cmd = cmd_q.get_nowait()
            if cmd == "shutdown":
                node.get_logger().info("shutdown command received")
                running = False
            elif isinstance(cmd, dict) and cmd.get("kind") == "speaker_config":
                if speaker_sub is not None:
                    node.destroy_subscription(speaker_sub)
                while True:
                    try:
                        speaker_q.get_nowait()
                    except Exception:
                        break
                speaker_sub = node.create_subscription(
                    # Perception TTS publishes BEST_EFFORT for low latency.
                    # A RELIABLE request is incompatible and receives zero
                    # samples. Keep a short ordered PCM buffer instead.
                    AudioChunk, str(cmd["topic"]), _on_speaker, QOS_AUDIO)
                node.get_logger().info(
                    f"speaker subscribed to {cmd['topic']} (BEST_EFFORT)")
            elif isinstance(cmd, dict) and cmd.get("kind") == "speaker_event":
                _publish_json(pub_speaker_events, cmd.get("event") or {})
            elif isinstance(cmd, dict) and cmd.get("kind") == "pose_config":
                if pose_sub is not None:
                    node.destroy_subscription(pose_sub)
                    pose_sub = None
                topic = str(cmd.get("topic") or "")
                if topic:
                    pose_sub = node.create_subscription(
                        String, topic, _on_pose, QOS_SENSOR)
                    node.get_logger().info(f"pose subscribed to {topic} (BEST_EFFORT)")
            elif isinstance(cmd, dict) and cmd.get("kind") == "pose_barrier":
                if len(pending_pose_barriers) < pending_pose_barriers.maxlen:
                    pending_pose_barriers.append(str(cmd.get("token") or ""))
            elif debug:
                node.get_logger().debug(f"bridge cmd: {cmd}")
        except Exception:
            pass

        for media_q in media_qs.values():
            newest_media = None
            while True:
                try:
                    newest_media = media_q.get_nowait()
                except Exception:
                    break
            if newest_media is not None:
                _dispatch_media(newest_media)

        # Unlike images, PCM frames must preserve their order. Drain the
        # bounded queue so short bridge delays do not produce audible gaps.
        while True:
            try:
                pcm = audio_q.get_nowait()
            except Exception:
                break
            out = AudioChunk()
            out.header.stamp = node.get_clock().now().to_msg()
            out.format = "audio/pcm-16k"
            out.data = pcm
            pub_mic.publish(out)
            mic_frames_published += 1
            if mic_frames_published == 1:
                node.get_logger().info("mic published first PCM frame to Domain 42")
            elif mic_frames_published % 100 == 0:
                node.get_logger().info(
                    f"mic published {mic_frames_published} PCM frames to Domain 42")

        # Process sensor snapshot (non-blocking, latest only)
        try:
            snap = sensor_q.get_nowait()
            _dispatch_snapshot(snap)
        except Exception:
            pass

        # Health log every 10 seconds
        now = time.time()
        if now - last_log >= 10.0:
            last_log = now
            if debug:
                node.get_logger().info("bridge worker health OK")

        # A short DDS wait avoids a busy loop while draining bursty live PCM
        # fast enough to preserve every 1 KiB browser-microphone frame.
        executor.spin_once(timeout_sec=0.005)

        # Defer the marker until after this executor turn. The guaranteed cut
        # is the ordered Pose IPC queue: callbacks already offered to pose_q
        # precede the marker; later callbacks receive a later Bridge clock.
        while pending_pose_barriers:
            token = pending_pose_barriers.popleft()
            _offer_pose_barrier(
                pose_q,
                pose_barrier_item(
                    token,
                    bridge_transport_drops=_shared_counter_value(
                        pose_transport_drops)),
                pose_transport_drops)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    if debug:
        print(f"[BridgeWorker:pid={os.getpid()}] shutdown complete", flush=True)
