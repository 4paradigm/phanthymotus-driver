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
import os
import sys
import time


class BridgeWorker:
    """Subprocess bridge that publishes Q5 sensor snapshots to Domain 42 (FastDDS)."""

    def __init__(self, namespace: str = "q5", debug: bool = False):
        self._ctx = mp.get_context("spawn")
        self._cmd_q = self._ctx.Queue()
        self._sensor_q = self._ctx.Queue()
        self._media_q = self._ctx.Queue(maxsize=4)
        self._audio_q = self._ctx.Queue(maxsize=100)
        self._speaker_q = self._ctx.Queue(maxsize=64)
        self._proc = None
        self._debug = debug
        self._namespace = namespace

    def start(self):
        """Spawn the bridge subprocess."""
        self._proc = self._ctx.Process(
            target=_run_bridge_subprocess,
            args=(self._cmd_q, self._sensor_q, self._media_q, self._audio_q, self._speaker_q,
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
        """Queue a processed media frame without blocking control state updates."""
        try:
            self._media_q.put_nowait(media)
        except Exception:
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

    def pop_speaker_chunk(self):
        try:
            return self._speaker_q.get_nowait()
        except Exception:
            return None

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


def _run_bridge_subprocess(cmd_q: mp.Queue, sensor_q: mp.Queue, media_q: mp.Queue,
                           audio_q: mp.Queue, speaker_q: mp.Queue, debug: bool, namespace: str):
    """Subprocess entry point — runs in separate process with own DDS domain."""
    # ── Environment: Force Domain 42 + FastDDS in subprocess ────────────────────
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    # UDP-only transport for Docker host networking (shared-memory won't work)
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT")

    import hashlib
    import json
    import signal

    import rclpy
    import rclpy.executors
    import rclpy.qos
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
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
    # Audio is a live lossy stream. Match the audio cards used by the other
    # drivers so Agent Core/FastDDS subscribers can request BEST_EFFORT without
    # a reliability negotiation mismatch or unnecessary retransmission delay.
    pub_mic = node.create_publisher(AudioChunk, f"{prefix}/mic/audio", QOS_AUDIO)
    speaker_sub = None
    mic_frames_published = 0
    speaker_frames_received = 0
    speaker_duplicate_frames_dropped = 0
    last_speaker_digest = b""
    last_speaker_frame_at = 0.0

    supported_audio_formats = {"audio/pcm-16k", "pcm_16k_16bit_mono"}

    def _on_speaker(msg):
        nonlocal speaker_frames_received, speaker_duplicate_frames_dropped
        nonlocal last_speaker_digest, last_speaker_frame_at
        # Agent Core's remote microphone uses pcm_16k_16bit_mono while
        # perception TTS uses audio/pcm-16k. Both carry S16_LE, 16 kHz mono.
        if msg.format not in supported_audio_formats:
            node.get_logger().warning(
                f"speaker ignored unsupported audio format: {msg.format!r}")
            return
        try:
            pcm = bytes(msg.data)
            # Two phanthy_bus_bridge publishers were observed on connected
            # browser-mic topics.  They can relay the exact same packet within
            # milliseconds; submitting both makes live speech garbled.  Do not
            # collapse regular repeated samples (such as silence), only an
            # identical packet that arrives in this short duplicate window.
            now = time.monotonic()
            digest = hashlib.blake2s(pcm, digest_size=8).digest()
            browser_frame = msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
            if (browser_frame and digest == last_speaker_digest
                    and now - last_speaker_frame_at < 0.02):
                speaker_duplicate_frames_dropped += 1
                return
            last_speaker_digest = digest
            last_speaker_frame_at = now
            speaker_q.put_nowait(pcm)
            speaker_frames_received += 1
            if speaker_frames_received == 1:
                node.get_logger().info("speaker received first PCM frame")
            elif speaker_duplicate_frames_dropped and speaker_frames_received % 100 == 0:
                node.get_logger().info(
                    f"speaker dropped {speaker_duplicate_frames_dropped} duplicate PCM frames")
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
            elif debug:
                node.get_logger().debug(f"bridge cmd: {cmd}")
        except Exception:
            pass

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

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    if debug:
        print(f"[BridgeWorker:pid={os.getpid()}] shutdown complete", flush=True)
