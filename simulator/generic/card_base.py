"""Base class for every simulator card.

A card is an MCP tool plus, usually, one ROS publisher. This file owns the parts
that are identical across all of them — the lifecycle verbs the framework sends
to every card, the publisher node lifecycle, and the authoritative `info` reply —
so the individual cards contain only what makes them different.

ROS is imported lazily, the way `common/vendor_runtime.py` does it, so the whole
schema and payload surface tests on a laptop with no rclpy.

## Which `format` strings actually render off the topic bus

Renderers take bus frames through `onData(buffer)` and activity-stream events
through `onEvent(event)`. A renderer with **no `onData` cannot draw a topic at
all** — the card appears, the panel stays blank, and nothing is logged. Measured
against `agent-core/web/js/renderers/` on 2026-09-18:

| format | renderer | reads the bus |
|---|---|---|
| `data/json`, `text/*` | kv-latest, text | yes |
| `image/jpeg` | camera | yes |
| `audio/*` | audio | yes |
| `sensor/mapping` | mapping | yes |
| `sensor/pointcloud` | pointcloud | yes |
| `sensor/skeleton` | skeleton | yes |
| `control/*` | control | yes |
| **`sensor/lidar*`** | lidar | **no — `mcp_result` only** |

So a laser scan goes out as `data/json`, which is also what
`x-humanoid/tianyi2.0`'s `laser_scan` card does. `sensor/imu` has no renderer at
all and is not used anywhere in this repo; IMU likewise goes out as `data/json`.
`tests/test_sim_sensor_formats.py` pins the allowlist.
"""

from __future__ import annotations

import json
import threading

from common.vendor_runtime import action_schema, tool

# Formats whose renderer implements `onData`. Anything outside this set must not
# appear in a `topic_out` — see the module docstring.
BUS_RENDERABLE_FORMATS = frozenset({
    "data/json", "text/plain", "text/markdown",
    "image/jpeg", "image/depth-zlib", "image/depth-z16",
    "audio/pcm-16k",
    "sensor/mapping", "sensor/pointcloud", "sensor/skeleton",
})


class Card:
    """One MCP tool backed by the simulated world."""

    NAME = ""
    KIND = "sensor"                 # sensor | actuator | processor | resource
    DESCRIPTION = ""
    TOPIC = ""                      # suffix under the ROS namespace; "" = no topic
    FORMAT = "data/json"
    HZ = 2.0
    BINARY = False                  # True -> UInt8MultiArray, False -> String(json)
    ACTIONS: dict[str, tuple[list[str], str]] = {}
    PROPERTIES: dict[str, dict] = {}
    RESOURCES: list[str] = []
    HOOKS: dict[str, str] = {}
    COMPLETION: dict | None = None
    CONFIG_SCHEMA: dict | None = None

    def __init__(self, world, config: dict, namespace: str, ros2=None):
        self.world = world
        self.config = config or {}
        self.namespace = namespace
        self._ros2 = ros2
        self._node = None
        self._pub = None
        self._running = False
        self._lock = threading.RLock()

    # ---- identity -----------------------------------------------------

    @property
    def topic(self) -> str:
        return f"/{self.namespace}/{self.TOPIC}" if self.TOPIC else ""

    def topic_out(self) -> list[dict]:
        return [{"topic": self.topic, "format": self.FORMAT}] if self.TOPIC else []

    def get_tool(self) -> dict:
        actions = dict(self.ACTIONS)
        if self.KIND != "resource":
            actions.setdefault("info", ([], "返回本卡片权威的 topic_out 与当前状态"))
            actions.setdefault("start", ([], "开始发布"))
            actions.setdefault("stop", ([], "停止发布"))
        schema = (action_schema(actions, dict(self.PROPERTIES)) if actions
                  else {"type": "object", "properties": {}})
        if self.COMPLETION:
            schema["x-completion"] = dict(self.COMPLETION)
        definition = tool(self.NAME, self.KIND, self.DESCRIPTION, schema,
                          topic_out=self.topic_out() or None)
        if self.RESOURCES:
            definition["x-resource"] = list(self.RESOURCES)
        if self.HOOKS:
            definition["x-hooks"] = dict(self.HOOKS)
        if self.CONFIG_SCHEMA:
            definition["configSchema"] = self.config_schema()
        return definition

    def config_schema(self) -> dict:
        """Overridable so a card can build its enum from disk at call time."""
        return dict(self.CONFIG_SCHEMA or {})

    # ---- lifecycle ----------------------------------------------------

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
        if self.TOPIC:
            self._open_publisher()

    def stop(self) -> None:
        with self._lock:
            self._running = False
        self._close_publisher()

    def _open_publisher(self) -> None:
        if self._ros2 is None or self._node is not None:
            return
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String, UInt8MultiArray

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        node = Node(f"sim_{self.NAME}", context=self._ros2.ctx_core)
        self._pub = node.create_publisher(UInt8MultiArray if self.BINARY else String, self.topic, qos)
        node.create_timer(1.0 / self.HZ, self._publish_once)
        self._ros2.executor_core.add_node(node)
        self._node = node

    def _close_publisher(self) -> None:
        node, self._node = self._node, None
        self._pub = None
        if node is None:
            return
        try:
            self._ros2.executor_core.remove_node(node)
        finally:
            # destroy_node, not just remove_node — otherwise the publisher and
            # the ROS node name leak and a restart collides with itself.
            node.destroy_node()

    def _publish_once(self) -> None:
        if not self._running or self._pub is None:
            return
        try:
            payload = self.payload()
        except Exception as exc:
            print(f"[sim:{self.NAME}] payload failed: {exc}", flush=True)
            return
        if payload is None:
            return
        try:
            if self.BINARY:
                from array import array
                from std_msgs.msg import UInt8MultiArray
                message = UInt8MultiArray()
                message.data = array("B", payload)
            else:
                from std_msgs.msg import String
                message = String()
                message.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(message)
        except Exception as exc:
            print(f"[sim:{self.NAME}] publish failed: {exc}", flush=True)

    # ---- dispatch -----------------------------------------------------

    def dispatch(self, action: str, args: dict) -> dict:
        args = {key: value for key, value in (args or {}).items() if key != "_tool_name"}
        if action == "start":
            self.start()
            return self.info()
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return self.info()
        handler = getattr(self, f"do_{action}", None)
        if handler is None:
            return {"error": f"unknown action: {action}"}
        return handler(**args)

    def info(self) -> dict:
        """Authoritative topic_out — README.md requires every topic-producing
        card to answer `info` with the topics it really publishes, because that
        is what agent-core binds, not what `tools/list` happened to say."""
        return {"state": "running" if self._running else "idle",
                "topic_out": self.topic_out(),
                "format": self.FORMAT if self.TOPIC else None}

    # ---- subclass hook ------------------------------------------------

    def payload(self):
        """Return a JSON-safe dict, or bytes when ``BINARY``. ``None`` skips."""
        return None
