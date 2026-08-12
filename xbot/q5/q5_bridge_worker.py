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
    bridge = BridgeWorker()
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

    def __init__(self, debug: bool = False):
        self._ctx = mp.get_context("spawn")
        self._cmd_q = self._ctx.Queue()
        self._sensor_q = self._ctx.Queue()
        self._proc = None
        self._debug = debug

    def start(self):
        """Spawn the bridge subprocess."""
        self._proc = self._ctx.Process(
            target=_run_bridge_subprocess,
            args=(self._cmd_q, self._sensor_q, self._debug),
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


def _run_bridge_subprocess(cmd_q: mp.Queue, sensor_q: mp.Queue, debug: bool):
    """Subprocess entry point — runs in separate process with own DDS domain."""
    # ── Environment: Force Domain 42 + FastDDS in subprocess ────────────────────
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    # UDP-only transport for Docker host networking (shared-memory won't work)
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT")

    import json
    import signal

    import rclpy
    import rclpy.executors
    import rclpy.qos
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import BatteryState, Imu, JointState
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry

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

    # ── Publishers ─────────────────────────────────────────────────────────────
    pub_joint = node.create_publisher(JointState, "/joint_states", QOS_SENSOR)
    pub_batt = node.create_publisher(BatteryState, "/battery_state", QOS_SENSOR)
    pub_imu_accel = node.create_publisher(Imu, "/camera/camera/accel/sample", QOS_SENSOR)
    pub_imu_gyro = node.create_publisher(Imu, "/camera/camera/gyro/sample", QOS_SENSOR)
    pub_fault = node.create_publisher(String, "/fault_array", QOS_SENSOR)
    pub_hand = node.create_publisher(String, "/hand_sensor", QOS_SENSOR)
    pub_odom = node.create_publisher(Odometry, "/wr1_base_drive_controller/odom", QOS_SENSOR)
    pub_status = node.create_publisher(String, "/xbot_state", QOS_SENSOR)

    node.get_logger().info("bridge publishers ready → 8 topics")
    executor.add_node(node)

    # ── Helpers ────────────────────────────────────────────────────────────────
    _clock = node.get_clock()

    def _publish_joint_states(snap):
        msg = JointState()
        msg.header.stamp = _clock.now().to_msg()
        msg.header.frame_id = snap.get("header_frame", "")
        msg.name = list(snap["joints"].keys())
        msg.position = list(snap["joints"].values())
        msg.velocity = list(snap.get("velocities", {}).values())
        msg.effort = list(snap.get("efforts", {}).values())
        pub_joint.publish(msg)

    def _publish_battery(snap):
        msg = BatteryState()
        msg.header.stamp = _clock.now().to_msg()
        msg.voltage = snap.get("voltage", 0.0)
        msg.temperature = snap.get("temperature", 0.0)
        msg.current = snap.get("current", 0.0)
        msg.charge = snap.get("charge", 0.0)
        msg.capacity = snap.get("capacity", 0.0)
        msg.design_capacity = snap.get("design_capacity", 0.0)
        msg.percentage = snap.get("percentage", 0.0)
        msg.power_supply_status = snap.get("power_supply_status", 0)
        msg.power_supply_health = snap.get("power_supply_health", 0)
        msg.power_supply_technology = snap.get("power_supply_technology", 0)
        msg.present = snap.get("present", False)
        msg.location = snap.get("location", "")
        msg.serial_number = snap.get("serial_number", "")
        pub_batt.publish(msg)

    def _publish_imu(vec, publishers):
        msg = Imu()
        msg.header.stamp = _clock.now().to_msg()
        msg.header.frame_id = "imu_link"
        if vec is not None:
            msg.linear_acceleration.x = vec.get("x", 0.0)
            msg.linear_acceleration.y = vec.get("y", 0.0)
            msg.linear_acceleration.z = vec.get("z", 0.0)
        for pub in publishers:
            pub.publish(msg)

    def _publish_json(pub, snap):
        msg = String()
        msg.data = json.dumps(snap, ensure_ascii=False)
        pub.publish(msg)

    def _publish_odom(snap):
        msg = Odometry()
        msg.header.stamp = _clock.now().to_msg()
        msg.header.frame_id = "odom"
        pose = snap.get("position", {})
        ori = snap.get("orientation", {})
        lin = snap.get("linear_velocity", {})
        ang = snap.get("angular_velocity", {})
        msg.pose.pose.position.x = pose.get("x", 0.0)
        msg.pose.pose.position.y = pose.get("y", 0.0)
        msg.pose.pose.position.z = pose.get("z", 0.0)
        msg.pose.pose.orientation.x = ori.get("x", 0.0)
        msg.pose.pose.orientation.y = ori.get("y", 0.0)
        msg.pose.pose.orientation.z = ori.get("z", 0.0)
        msg.pose.pose.orientation.w = ori.get("w", 1.0)
        msg.twist.twist.linear.x = lin.get("x", 0.0)
        msg.twist.twist.linear.y = lin.get("y", 0.0)
        msg.twist.twist.linear.z = lin.get("z", 0.0)
        msg.twist.twist.angular.x = ang.get("x", 0.0)
        msg.twist.twist.angular.y = ang.get("y", 0.0)
        msg.twist.twist.angular.z = ang.get("z", 0.0)
        pub_odom.publish(msg)

    def _dispatch_snapshot(snap):
        if not snap or not snap.get("available"):
            return

        # /joint_states
        if "joints" in snap:
            _publish_joint_states(snap)

        # /battery_state
        bat = snap.get("_sensor_battery")
        if bat:
            _publish_battery(bat)

        # IMU: linear_acceleration → accel topic, angular_velocity → gyro topic
        imu = snap.get("_sensor_imu")
        if imu:
            vec_accel = imu.get("linear_acceleration")
            vec_gyro = imu.get("angular_velocity")
            if vec_accel:
                _publish_imu(vec_accel, [pub_imu_accel])
            if vec_gyro:
                _publish_imu(vec_gyro, [pub_imu_gyro])

        # /fault_array
        faults = snap.get("_sensor_faults")
        if faults:
            _publish_json(pub_fault, faults)

        # /hand_sensor
        hand = snap.get("_sensor_hand")
        if hand:
            _publish_json(pub_hand, hand)

        # /wr1_base_drive_controller/odom
        odom = snap.get("_sensor_odom")
        if odom:
            _publish_odom(odom)

        # /xbot_state
        status = snap.get("_sensor_robot_status")
        if status:
            _publish_json(pub_status, status)

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
        # Process commands (non-blocking)
        try:
            cmd = cmd_q.get(timeout=0.05)
            if cmd == "shutdown":
                node.get_logger().info("shutdown command received")
                running = False
            elif debug:
                node.get_logger().debug(f"bridge cmd: {cmd}")
        except Exception:
            pass

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

        executor.spin_once(timeout_sec=0)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    if debug:
        print(f"[BridgeWorker:pid={os.getpid()}] shutdown complete", flush=True)
