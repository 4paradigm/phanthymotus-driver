"""Read-only Bumi ROS 2 TF Integrity Monitor sensor."""
import json
import threading
import time

from tf_integrity import Integrity, angle, valid_frame

PREFIX = "tf_integrity_monitor"


class TfIntegrityMonitor:
    def __init__(self, cfg, namespace, executor):
        self.cfg = cfg
        self.output = f"/{namespace}/{PREFIX}"
        self.executor = executor
        self.lock = threading.RLock()
        self.node = None
        self.view = Integrity(cfg)
        self.residual = None

    def get_tool(self):
        return dict(name=PREFIX, type="sensor", multiInstance=False,
                    description="Read-only integrity, freshness and continuity of the live ROS TF tree.",
                    inputSchema={"type": "object", "properties": {
                        "action": {"type": "string", "enum": ["start", "stop", "info"]}},
                        "required": ["action"]},
                    topic_out=[{"topic": self.output, "format": "data/json"}])

    def start(self):
        # Bundle initialization registers the sensor. Canvas start acquires ROS resources.
        return True

    def stop(self):
        with self.lock:
            node, self.node = self.node, None
            self.residual = None
        if node is not None:
            try:
                self.executor.remove_node(node)
            finally:
                node.destroy_node()

    def _snapshot(self, node):
        now = node.get_clock().now().nanoseconds
        received = time.monotonic()
        report = self.view.snapshot(now, received)
        if report["state"] == "insufficient_evidence" and self.residual and self.residual[0] == self.view.clock_resets:
            _, stamp, result = self.residual
            if 0 <= (now - stamp) / 1e9 <= self.view.stale_s:
                report["visual_residual"] = result
                report["visual_state"] = "inconsistent" if result["exceeds_tolerance"] else "consistent"
                if "visual_evidence_unavailable" in report["reason"]:
                    report["reason"].remove("visual_evidence_unavailable")
                    report["state"] = "healthy"
                if result["exceeds_tolerance"]:
                    report["state"] = "inconsistent"
                    report["reason"].append("visual_residual_exceeds_tolerance")
        return report

    def _start(self):
        from rclpy.clock import Clock, ClockType
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        with self.lock:
            if self.node is not None:
                return
            self.view = Integrity(self.cfg)
            node = Node(PREFIX, parameter_overrides=[
                Parameter("use_sim_time", value=self.view.sim)])
            try:
                publisher = node.create_publisher(String, self.output, 10)
                dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
                static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                vision = self._setup_vision(node, dynamic)

                def receive(message, info, is_static):
                    with self.lock:
                        if self.node is not node:
                            return
                        now, received = node.get_clock().now().nanoseconds, time.monotonic()
                        gid = (info or {}).get("publisher_gid")
                        source = bytes(gid).hex() if gid else "unknown"
                        accepted = []
                        self.view.clock(now)
                        for entry in message.transforms:
                            t, q = entry.transform.translation, entry.transform.rotation
                            stamp = entry.header.stamp
                            if stamp.nanosec >= 10**9:
                                self.view.issue("invalid_stamp", entry.child_frame_id[:128], received)
                                continue
                            if self.view.observe(entry.header.frame_id, entry.child_frame_id,
                                                 stamp.sec * 10**9 + stamp.nanosec,
                                                 (t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
                                                 is_static, source, now, received):
                                accepted.append(entry)
                        if vision is not None:
                            buffer, epoch = vision
                            if epoch[0] != self.view.clock_resets:
                                buffer.clear()
                                epoch[0] = self.view.clock_resets
                            for entry in accepted:
                                try:
                                    if is_static:
                                        buffer.set_transform_static(entry, source)
                                    else:
                                        buffer.set_transform(entry, source)
                                except ValueError:
                                    continue

                def receive_dynamic(message, info=None):
                    receive(message, info, False)

                def receive_static(message, info=None):
                    receive(message, info, True)

                def publish():
                    with self.lock:
                        if self.node is node:
                            msg = String()
                            msg.data = json.dumps(self._snapshot(node), allow_nan=False)
                            publisher.publish(msg)

                node.create_subscription(TFMessage, "/tf", receive_dynamic, dynamic)
                node.create_subscription(TFMessage, "/tf_static", receive_static, static)
                node.create_timer(1 / self.view.publish_hz, publish,
                                  clock=Clock(clock_type=ClockType.STEADY_TIME))
                if self.executor.add_node(node) is False:
                    raise RuntimeError("executor refused TF monitor node")
                self.node = node
            except Exception:
                try:
                    self.executor.remove_node(node)
                finally:
                    node.destroy_node()
                raise

    def _setup_vision(self, node, qos):
        from math import dist, hypot, isfinite

        if not self.view.visual:
            return None
        cfg = self.cfg.get("vision") or {}
        evidence = ("calibration_id", "survey_id", "time_sync_id",
                    "camera_frame", "surveyed_marker_frame", "observation_topic")
        if not all(isinstance(cfg.get(k), str) and cfg[k].strip() for k in evidence):
            return None
        camera, marker = cfg["camera_frame"], cfg["surveyed_marker_frame"]
        if camera == marker or not all(map(valid_frame, (camera, marker))):
            raise ValueError("Vision requires distinct valid camera and surveyed marker frames")
        limits = tuple(float(cfg.get(key, default)) for key, default in (
            ("translation_tolerance_m", .05), ("rotation_tolerance_rad", .15)))
        if not all(isfinite(v) and v > 0 for v in limits):
            raise ValueError("Visual tolerances must be finite and positive")

        from geometry_msgs.msg import PoseStamped
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformException

        buffer = Buffer()
        epoch = [0]
        previous = None

        def observe(message):
            nonlocal previous
            with self.lock:
                if self.node is not node:
                    return
                self.residual = None
                now = node.get_clock().now().nanoseconds
                self.view.clock(now)
                if message.header.stamp.sec < 0 or message.header.stamp.nanosec >= 10**9:
                    return
                stamp = Time.from_msg(message.header.stamp)
                if (epoch[0] != self.view.clock_resets or message.header.frame_id != camera
                        or stamp.nanoseconds <= 0
                        or not 0 <= (now - stamp.nanoseconds) / 1e9 <= self.view.stale_s
                        or not self._vision_path(camera, marker)):
                    return
                try:
                    # Surveyed frame is independent of the observation being evaluated.
                    expected = buffer.lookup_transform(marker, camera, stamp).transform
                except TransformException:
                    return
                p, q = message.pose.position, message.pose.orientation
                eo, et = expected.rotation, expected.translation
                observed = (q.x, q.y, q.z, q.w)
                norm = hypot(*observed)
                if not all(isfinite(v) and abs(v) < 1e12 for v in (p.x, p.y, p.z, *observed)) or abs(norm - 1) > .01:
                    return
                observed = tuple(v / norm for v in observed)
                # Transform observed origin into the surveyed marker axes.
                x, y, z, w = eo.x, eo.y, eo.z, eo.w
                u, v, s = y*p.z-z*p.y, z*p.x-x*p.z, x*p.y-y*p.x
                vector = (p.x + 2*(w*u+y*s-z*v) + et.x,
                          p.y + 2*(w*v+z*u-x*s) + et.y,
                          p.z + 2*(w*s+x*v-y*u) + et.z)
                a, b, c, d = observed
                rotation = (w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
                            w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c)
                metres, radians = hypot(*vector), angle(rotation, (0, 0, 0, 1))
                stability_m = stability_rad = None
                if previous and previous[0] == epoch[0] and 0 < stamp.nanoseconds - previous[1] <= self.view.stale_s * 1e9:
                    stability_m = dist(vector, previous[2])
                    stability_rad = angle(rotation, previous[3])
                previous = epoch[0], stamp.nanoseconds, vector, rotation
                self.residual = epoch[0], stamp.nanoseconds, dict(
                    translation_m=metres, rotation_rad=radians,
                    successive_translation_m=stability_m, successive_rotation_rad=stability_rad,
                    exceeds_tolerance=metres > limits[0] or radians > limits[1])

        node.create_subscription(PoseStamped, cfg["observation_topic"], observe, qos)
        return buffer, epoch

    def _vision_path(self, camera, marker):
        # Reject ambiguous declarations and require the surveyed marker's own edge to be static.
        marker_edges = [key for key in self.view.edges if key[1] == marker]
        if len(marker_edges) != 1 or not marker_edges[0][2]:
            return False
        report = self.view.snapshot(self.node.get_clock().now().nanoseconds, time.monotonic())
        return (camera in report["checked_frames"] and report["state"] in
                ("healthy", "insufficient_evidence"))

    def dispatch(self, action, args):
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "start":
            try:
                self._start()
            except Exception as exc:
                return {"state": "error", "error": str(exc)}
        if action in ("start", "info", PREFIX):
            with self.lock:
                return dict(state="running" if self.node else "idle",
                            topic_out=self.get_tool()["topic_out"],
                            diagnostics=self._snapshot(self.node) if self.node else None)
        return None
