#!/usr/bin/env python3
"""Bumi object-memory card: persist the scene graphs produced by scene_graph.

This variant keeps only the second stage of the position-memory pipeline.  It
subscribes to ``scene_graph/state`` and ``scene_graph/evidence``, and on
``remember`` writes the scene into a small JSON memory plus one evidence JPEG.
It never opens the camera, runs a detector or answers natural-language
questions.  The memory file is the same one the full pipeline's
``object_locator`` reads, and the complete memory is also published on
``/<ns>/object_memory/state``.

Standalone use: with a scene/evidence publisher connected, the card can
remember scenes, list/get entries, forget one or all, and inspect its storage
without any other card.
"""

from __future__ import annotations

import copy
import json
import threading
import time
from pathlib import Path

from object_memory_core import (decode_scene, empty_memory, format_time,
                                load_memory, match_memory, merge_memory,
                                save_evidence, save_memory)

MAX_SCENE_AGE_S = 10.0


class ObjectMemoryPlugin:
    PREFIX = "object_memory"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl=None):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._lock = threading.RLock()
        self._node = None
        self._memory_pub = None
        self._string = None
        self._scene = None
        self._scene_at = 0.0
        self._evidence = None
        self._evidence_at = 0.0

        self._storage = Path(str(plugin_config.get(
            "storage_dir", "/opt/phanthy-motus/data/memory_palace"))).expanduser()
        self._evidence_dir = self._storage / "evidence"
        self._memory_path = self._storage / "memory.json"
        self._scene_topic = (str(plugin_config.get("scene_topic") or "").strip()
                             or f"/{namespace}/scene_graph/state")
        self._evidence_topic = (str(plugin_config.get("evidence_topic") or "").strip()
                                or f"/{namespace}/scene_graph/evidence")
        self._state_topic = f"/{namespace}/object_memory/state"
        self._memory = load_memory(self._memory_path)

        try:
            import rclpy  # noqa: F401
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                                   DurabilityPolicy)
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String
            node = Node("bumi_object_memory")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=2,
                             durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(String, self._scene_topic, self._on_scene, qos)
            node.create_subscription(CompressedImage, self._evidence_topic,
                                     self._on_evidence, qos)
            self._memory_pub = node.create_publisher(String, self._state_topic, qos)
            self._string = String
            executor.add_node(node)
            self._node = node
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[object_memory] ROS setup failed: {exc}", flush=True)
            self._node = None

    # ── Inputs ──────────────────────────────────────────────────────────────
    def _on_scene(self, msg):
        now = time.time()
        try:
            scene = decode_scene(bytes(msg.data), now)
        except ValueError as exc:
            print(f"[object_memory] scene rejected: {exc}", flush=True)
            return
        with self._lock:
            self._scene = scene
            self._scene_at = now

    def _on_evidence(self, msg):
        with self._lock:
            self._evidence = bytes(msg.data)
            self._evidence_at = time.time()

    # ── Output ──────────────────────────────────────────────────────────────
    def _publish_state(self) -> None:
        if self._node is None or self._memory_pub is None or self._string is None:
            return
        try:
            message = self._string()
            message.data = json.dumps(self._memory, ensure_ascii=False)
            self._memory_pub.publish(message)
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[object_memory] publish failed: {exc}", flush=True)

    # ── Actions ─────────────────────────────────────────────────────────────
    def _remember(self, args: dict) -> dict:
        now = time.time()
        with self._lock:
            scene = copy.deepcopy(self._scene)
            scene_at = self._scene_at
            evidence, evidence_at = self._evidence, self._evidence_at
        if scene is None or now - scene_at > MAX_SCENE_AGE_S:
            return {"state": "scene_unavailable",
                    "message": "没有新鲜的场景图；请先执行 scene_graph.snapshot"}
        note = args.get("note")
        note = note.strip()[:200] if isinstance(note, str) and note.strip() else None
        evidence_path = ""
        if evidence is not None and now - evidence_at <= MAX_SCENE_AGE_S:
            evidence_path = save_evidence(self._evidence_dir, "scene", evidence)
        with self._lock:
            merge_memory(self._memory, scene, evidence_path, note)
            try:
                save_memory(self._memory_path, self._memory)
            except OSError as exc:
                return {"state": "error", "message": f"保存失败：{exc}"}
        self._publish_state()
        labels = [node["label"] for node in scene["nodes"]]
        return {
            "state": "remembered",
            "scene_id": scene["id"],
            "captured_at": scene.get("captured_at_text"),
            "count": len(scene["nodes"]),
            "labels": labels,
            "objects": [{"label": node["label"],
                         "distance_m": node.get("distance_m"),
                         "position": node.get("robot"),
                         "pixel": node.get("pixel")}
                        for node in scene["nodes"]],
            "edges": scene.get("edges") or [],
            "evidence_path": evidence_path,
            "memory_path": str(self._memory_path),
        }

    def _list(self) -> dict:
        with self._lock:
            entries = list((self._memory.get("objects") or {}).values())
        entries.sort(key=lambda entry: entry.get("last_seen", 0), reverse=True)
        return {"state": "ok", "count": len(entries),
                "objects": [{"label": entry.get("label"),
                             "last_seen": format_time(entry["last_seen"]),
                             "distance_m": entry.get("distance_m"),
                             "sightings": entry.get("sightings", 1),
                             "evidence": entry.get("evidence")}
                            for entry in entries]}

    def _get(self, args: dict) -> dict:
        label = args.get("label")
        if not isinstance(label, str) or not label.strip():
            return {"state": "error", "message": "get 需要 label"}
        with self._lock:
            memory = copy.deepcopy(self._memory)
        match = match_memory(label, memory)
        if match["state"] == "none":
            return {"state": "not_found", "label": label,
                    "message": f"没有记住“{label}”"}
        if match["state"] == "ambiguous":
            return {"state": "ambiguous", "label": label,
                    "message": "记住了几个相近的标签，请说得更具体",
                    "candidates": match["candidates"]}
        entry = (memory.get("objects") or {}).get(match["key"]) or {}
        return {"state": "found", "label": entry.get("label"),
                "entry": entry, "score": match["score"],
                "memory_path": str(self._memory_path)}

    def _forget(self, args: dict) -> dict:
        label = args.get("label")
        with self._lock:
            if not isinstance(label, str) or not label.strip():
                removed = len(self._memory.get("objects") or {})
                self._memory = empty_memory()
            else:
                match = match_memory(label, self._memory)
                if match["state"] != "matched":
                    return {"state": "not_found", "message": f"没有记住“{label}”"}
                (self._memory.get("objects") or {}).pop(match["key"], None)
                removed = 1
            try:
                save_memory(self._memory_path, self._memory)
            except OSError as exc:
                return {"state": "error", "message": f"保存失败：{exc}"}
        self._publish_state()
        return {"state": "forgotten", "removed": removed}

    # ── Card protocol ───────────────────────────────────────────────────────
    def get_tool(self) -> dict:
        return {
            "name": "object_memory",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "保存 scene_graph 产出的场景图：remember 记住当前场景，"
                "list 列出已记物品，get 按标签取回一条记忆，forget 删除。"
                "记忆写入本地 JSON，并发布完整 memory 到 object_memory/state；"
                "完整流水线的 object_locator 可读取同一份记忆文件。"
                "不读相机、不跑检测、不做自然语言问答。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["remember", "list", "get", "forget",
                                        "info", "start"]},
                    "note": {"type": "string", "maxLength": 200,
                             "description": "remember 时可选的备注"},
                    "label": {"type": "string", "maxLength": 100,
                              "description": "get/forget 用的标签；forget 不填清空全部"},
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "remember": {"params": ["note"],
                                 "description": "把最近一次 scene_graph 场景写入记忆。"},
                    "list": {"params": [], "description": "列出已记住的物品。"},
                    "get": {"params": ["label"],
                            "description": "按标签取回一条记忆（位置、关系、证据路径）。"},
                    "forget": {"params": ["label"],
                               "description": "删除一个物品；不填 label 清空全部。"},
                    "info": {"params": [], "description": "查看输入新鲜度和记忆数量/存储。"},
                    "start": {"params": [], "description": "检查 ROS 输入是否就绪。"},
                },
            },
            "topic_in": [
                {"topic": self._scene_topic, "format": "data/json",
                 "message_type": "std_msgs/msg/String",
                 "desc": "scene_graph 完整场景图"},
                {"topic": self._evidence_topic, "format": "image/jpeg",
                 "message_type": "sensor_msgs/msg/CompressedImage",
                 "desc": "scene_graph 标注截图"},
            ],
            "topic_out": [{"topic": self._state_topic, "format": "data/json"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self):
        if self._node is None:
            return {"state": "unavailable",
                    "message": "ROS 未初始化，object_memory 无法订阅场景图"}
        try:
            self._storage.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return {"state": "unavailable", "message": f"存储目录不可写：{exc}"}
        self._publish_state()
        return {"state": "ready"}

    def stop(self):
        with self._lock:
            try:
                save_memory(self._memory_path, self._memory)
            except OSError as exc:
                print(f"[object_memory] final save failed: {exc}", flush=True)

    def dispatch(self, action: str, args: dict) -> dict | None:
        args = args or {}
        if action == "remember":
            return self._remember(args)
        if action == "list":
            return self._list()
        if action == "get":
            return self._get(args)
        if action == "forget":
            return self._forget(args)
        if action == "start":
            return self.start()
        if action == "info":
            now = time.time()
            with self._lock:
                ages = {"scene_age_s": (round(now - self._scene_at, 2)
                                       if self._scene else None),
                        "evidence_age_s": (round(now - self._evidence_at, 2)
                                           if self._evidence else None)}
                count = len(self._memory.get("objects") or {})
                scenes = len(self._memory.get("scenes") or {})
                updated = self._memory.get("updated_at")
            return {"state": "ready" if self._node is not None else "unavailable",
                    "memory_path": str(self._memory_path),
                    "objects": count, "scenes": scenes,
                    "updated_at": format_time(updated) if updated else None,
                    "topics": {"scene": self._scene_topic,
                               "evidence": self._evidence_topic,
                               "state": self._state_topic},
                    "freshness": ages}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        return None
