"""VisionTfPlugin — the `vision_tf` processor card for the Bumi bundle.

Left link:  a vision-localisation card's ``data/json`` topic (AR marker, stamped
            metric pose, pixel+depth, object list).
Right link: a tf-tree card (or anything else that consumes tf-shaped transforms)
            via this card's own ``data/json`` topic.

The card is a pure data adapter.  It converts "where the vision algorithm says
an object is" into tf-shaped ``parent -> child`` edges and publishes them.  It
does **not** publish ``/tf``, does not build a tree, and touches no hardware —
so it is deliberately NOT gated on the high/media controllers in main.py.

Threading mirrors `upper_body_track_card`: subscriptions keep only the newest
message, conversion runs on a worker thread, and a generation counter + run
token fence off any in-flight publish after stop()/start().
"""

from __future__ import annotations

import json
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

import vision_tf_spec as spec

# Machine topics only need the newest sample; a queue would add latency.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class VisionTfPlugin:
    PREFIX = "vision_tf"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        cfg = spec.validate_config(plugin_config)
        self._config = cfg
        self._namespace = namespace
        self._source = cfg["source_name"]
        # Topic carries an instance segment so several vision sources can coexist.
        self._topic = f"/{namespace}/vision_tf/{self._source}/transforms"

        self._node = Node(f"bumi_vision_tf_{self._source}")
        executor.add_node(self._node)
        self._pub = self._node.create_publisher(String, self._topic, _LATEST_QOS)

        self._input_topic = ""
        self._sub = None
        self._worker = None
        self._generation = 0
        self._run_token = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_raw = None
        self._latest_at = 0.0
        # Monotonic callback sequence. A single received message must be
        # converted exactly once — re-feeding it on every worker tick refreshes
        # each target's last-seen time and makes its TTL never expire.
        self._seq = 0
        self._converted_seq = -1
        # child frame -> last-seen monotonic ms, for TTL dropping.
        self._targets = {}

        self._status = "idle"
        self._degraded = "no_input_topic"
        self._received = 0
        self._accepted = 0
        self._rejected = 0
        self._last_error = None
        self._last_payload = None

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "vision_tf",
            "type": "processor",
            "multiInstance": True,
            "description": (
                "把各种视觉定位结果（AR 码 tvec/rvec、带时间戳的位姿、像素+深度、"
                "物体列表）的 JSON 转换成 tf 变换 JSON，供下游 tf 构建卡挂进机器人"
                "全身 tf 树：父帧是相机（camera_optical_frame / camera_link），"
                "子帧是被定位物体（vision_<source>_<id>）。"
                "左联视觉定位卡的 data/json 输出，右联 tf 构建卡。"
                "本卡不做旋转（旋转已在 URDF 的 camera_optical_joint 里，"
                "output_frame=body 时才在卡内转成机体系并挂 camera_link），"
                "不构建 tf 树、不发 /tf、不驱动任何硬件。"
                "四元数统一 xyzw。start 必须带 input_topic，否则如实报 idle。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "卡片生命周期或实时状态查询",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "左联视觉定位卡输出的 data/json 话题（画布连线提供）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"],
                              "description": "订阅左联视觉定位卡并开始转换"},
                    "stop": {"params": [], "description": "停止订阅与发布"},
                    "info": {"params": [],
                             "description": "返回收帧/接受/拒收计数、当前目标、最近错误、父帧约定"},
                },
            },
            "topic_in": [{"format": "data/json"}],
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    # ── Bundle lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        # Real subscription starts in dispatch("start") once the canvas supplies
        # input_topic, exactly like the other processor cards.
        pass

    def stop(self) -> None:
        self._teardown()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "info":
            return self._info()
        return None

    # ── Actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict) -> dict:
        input_topic = str(args.get("input_topic") or "").strip()
        if not input_topic:
            # Never claim running when nothing is wired (the bundle's silent-
            # success trap).  Report idle with an explicit reason.
            self._status = "idle"
            self._degraded = "no_input_topic"
            return {"state": "idle", "degraded": "no_input_topic"}

        self._teardown()
        self._input_topic = input_topic
        self._stop_event = threading.Event()
        self._last_error = None
        self._targets.clear()
        self._converted_seq = -1

        def _on_json(msg: String):
            with self._frame_lock:
                self._latest_raw = msg.data
                self._latest_at = time.time() * 1000.0
                self._seq += 1

        self._sub = self._node.create_subscription(
            String, input_topic, _on_json, _LATEST_QOS)

        self._generation += 1
        run_token = object()
        self._run_token = run_token
        self._worker = threading.Thread(
            target=self._worker_loop, args=(self._generation, run_token),
            daemon=True, name=f"vision_tf_{self._source}")
        self._worker.start()

        self._status = "running"
        self._degraded = ""
        return {
            "state": "running",
            "input_topic": input_topic,
            "topic_in": [{"topic": input_topic, "format": "data/json"}],
        }

    def _stop(self) -> dict:
        self._teardown()
        return {"state": "idle"}

    def _teardown(self) -> None:
        # Fence publication before waiting: an in-flight batch may outlive the
        # bounded join, so clearing the token stops it reaching the output.
        self._run_token = None
        self._stop_event.set()
        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.join(timeout=2.0)
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        with self._frame_lock:
            self._latest_raw = None
        self._targets.clear()
        self._status = "idle"
        self._degraded = "no_input_topic"

    def _info(self) -> dict:
        topic_in = [{"format": "data/json"}]
        if self._input_topic:
            topic_in[0]["topic"] = self._input_topic
        parent = spec.resolve_parent_frame(
            self._config["input_frame"], self._config["output_frame"])
        return {
            "state": self._status,
            "degraded": self._degraded,
            "topic_in": topic_in,
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
            "parent_frame": parent,
            "rotation_order": spec.ROTATION_ORDER,
            "units": "m",
            "source_name": self._source,
            "adapter": self._config["adapter"],
            "input_frame": self._config["input_frame"],
            "output_frame": self._config["output_frame"],
            "pose_direction": self._config["pose_direction"],
            "receive": self._received,
            "accepted": self._accepted,
            "rejected": self._rejected,
            "targets": sorted(self._targets.keys()),
            "last_error": self._last_error,
            "last": self._last_payload,
        }

    # ── Worker ───────────────────────────────────────────────────────────────

    def _tick(self, run_token) -> None:
        """Process at most one new sample, or sweep TTL on a silent source.

        Kept separate from the scheduling loop so offline tests can drive it
        deterministically (and pin the single-message-then-silent behaviour).
        """
        with self._frame_lock:
            raw = self._latest_raw
            received_at = self._latest_at
            seq = self._seq

        # Only convert a message the first time we see its sequence. Feeding
        # the same raw back every tick would refresh every target's last-seen
        # stamp and keep a stale transform alive forever.
        is_new = raw is not None and seq != self._converted_seq
        now_ms = time.time() * 1000.0

        try:
            if is_new:
                parsed = json.loads(raw)
                self._received += 1
                batch = spec.transforms_from_json(
                    parsed, self._config, receive_ms=received_at,
                    targets=self._targets)
                self._converted_seq = seq
                self._accepted += len(batch["transforms"])
                self._rejected += batch["rejected"]
                if batch["last_error"]:
                    self._last_error = batch["last_error"]
            else:
                # No new sample. Pass an empty batch so transforms_from_json
                # only runs its TTL sweep against the wall clock — targets
                # whose last real observation aged out move to `dropped`.
                batch = spec.transforms_from_json(
                    [], self._config, receive_ms=now_ms,
                    targets=self._targets)

            payload = spec.build_payload(self._config, batch)
            if payload["transforms"]:
                self._degraded = ""
            elif not self._targets:
                # No live targets left: either we never got a sample or the
                # last one aged out. Both mean the input is not producing.
                self._degraded = "input_stale"

            # Publish on a fresh sample, or when targets expire (a dropped
            # event tells downstream the vision target just disappeared).
            # Never replay a stale transform on a silent source.
            should_publish = is_new or bool(payload["dropped"])
            if should_publish and run_token is self._run_token:
                msg = String()
                msg.data = json.dumps(payload, ensure_ascii=False)
                self._pub.publish(msg)
                self._last_payload = payload
        except Exception as exc:  # a bad frame is not fatal to the worker
            self._rejected += 1
            self._last_error = f"{type(exc).__name__}: {exc}"
            if self._last_payload is None or self._received <= 1:
                self._node.get_logger().warning(
                    f"vision_tf bad frame: {self._last_error}")

    def _worker_loop(self, generation: int, run_token) -> None:
        interval = 1.0 / self._config["publish_rate_hz"]
        while (not self._stop_event.is_set()
               and generation == self._generation
               and run_token is self._run_token):
            self._tick(run_token)
            self._stop_event.wait(interval)
