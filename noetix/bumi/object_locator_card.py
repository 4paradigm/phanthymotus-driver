#!/usr/bin/env python3
"""Bumi object-locator card: answer "where is my xxx" from the memory.

This variant keeps only the third stage of the position-memory pipeline.  It
reads the memory written by ``object_memory`` — preferably from the live
``object_memory/state`` topic, otherwise from the shared ``memory.json`` — and
answers natural-language questions with the remembered position, spatial
relations and an annotated evidence image.

Standalone use: with a memory file (or a memory-state publisher) present, the
card can answer questions, list remembered labels, recall the last answer and
inspect its storage without the other two cards.
"""

from __future__ import annotations

import json
import threading
import time
from array import array
from pathlib import Path

from object_locator_core import (answer_question, draw_evidence, format_time,
                                 load_memory, save_evidence)

MAX_MEMORY_AGE_S = 30.0


class ObjectLocatorPlugin:
    PREFIX = "object_locator"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl=None):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._lock = threading.RLock()
        self._node = None
        self._answer_pub = None
        self._evidence_pub = None
        self._compressed_image = None
        self._string = None
        self._memory = None
        self._memory_at = 0.0
        self._last_answer = None

        self._storage = Path(str(plugin_config.get(
            "storage_dir", "/opt/phanthy-motus/data/memory_palace"))).expanduser()
        self._evidence_dir = self._storage / "evidence"
        self._memory_path = self._storage / "memory.json"
        self._memory_topic = (str(plugin_config.get("memory_topic") or "").strip()
                              or f"/{namespace}/object_memory/state")
        self._answer_topic = f"/{namespace}/object_locator/answer"
        self._evidence_topic = f"/{namespace}/object_locator/evidence"

        try:
            import rclpy  # noqa: F401
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                                   DurabilityPolicy)
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String
            node = Node("bumi_object_locator")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=2,
                             durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(String, self._memory_topic, self._on_memory, qos)
            self._answer_pub = node.create_publisher(String, self._answer_topic, qos)
            self._evidence_pub = node.create_publisher(
                CompressedImage, self._evidence_topic, qos)
            self._compressed_image = CompressedImage
            self._string = String
            executor.add_node(node)
            self._node = node
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[object_locator] ROS setup failed: {exc}", flush=True)
            self._node = None

    # ── Memory source ───────────────────────────────────────────────────────
    def _on_memory(self, msg):
        try:
            memory = json.loads(bytes(msg.data).decode("utf-8"))
        except Exception as exc:
            print(f"[object_locator] memory rejected: {exc}", flush=True)
            return
        if (not isinstance(memory, dict)
                or not isinstance(memory.get("objects"), dict)):
            print("[object_locator] memory rejected: invalid payload", flush=True)
            return
        if not isinstance(memory.get("scenes"), dict):
            memory["scenes"] = {}
        with self._lock:
            self._memory = memory
            self._memory_at = time.time()

    def _memory_snapshot(self):
        """Prefer a fresh live memory; fall back to the shared memory file."""
        with self._lock:
            memory, at = self._memory, self._memory_at
        if memory is not None and time.time() - at <= MAX_MEMORY_AGE_S:
            return memory, "topic", at
        return load_memory(self._memory_path), "file", None

    # ── Output ──────────────────────────────────────────────────────────────
    def _publish_answer(self, result: dict) -> None:
        if self._node is None or self._answer_pub is None or self._string is None:
            return
        try:
            message = self._string()
            message.data = json.dumps(result, ensure_ascii=False)
            self._answer_pub.publish(message)
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[object_locator] answer publish failed: {exc}", flush=True)

    def _publish_evidence(self, jpeg: bytes) -> None:
        if self._node is None or self._evidence_pub is None or self._compressed_image is None:
            return
        try:
            image = self._compressed_image()
            image.header.stamp = self._node.get_clock().now().to_msg()
            image.format = "jpeg"
            image.data = array("B", jpeg)
            self._evidence_pub.publish(image)
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[object_locator] evidence publish failed: {exc}", flush=True)

    # ── Actions ─────────────────────────────────────────────────────────────
    def _ask(self, args: dict) -> dict:
        question = args.get("question")
        if not isinstance(question, str) or not question.strip():
            return {"state": "error",
                    "message": "ask 需要 question，例如：我的钥匙在哪？"}
        memory, source, memory_at = self._memory_snapshot()
        result = answer_question(memory, question)
        result["memory_source"] = source
        result["memory_count"] = len(memory.get("objects") or {})
        result["memory_at"] = (format_time(memory_at) if memory_at else None)
        result["evidence_topic"] = self._evidence_topic
        if result["state"] == "found":
            scene = (memory.get("scenes") or {}).get(result.get("scene_id"))
            evidence = result.get("evidence_path") or ""
            if scene and evidence and Path(evidence).is_file():
                try:
                    annotated = draw_evidence(Path(evidence).read_bytes(), scene,
                                              result.get("label"))
                    saved = save_evidence(self._evidence_dir / "queries", "query",
                                          annotated)
                    if saved:
                        evidence = saved
                    self._publish_evidence(annotated)
                except Exception as exc:
                    print(f"[object_locator] evidence annotate failed: {exc}",
                          flush=True)
            result["evidence_path"] = evidence
        with self._lock:
            self._last_answer = result
        self._publish_answer(result)
        return result

    def _list(self) -> dict:
        memory, source, memory_at = self._memory_snapshot()
        entries = list((memory.get("objects") or {}).values())
        entries.sort(key=lambda entry: entry.get("last_seen", 0), reverse=True)
        return {"state": "ok", "count": len(entries),
                "memory_source": source,
                "memory_at": format_time(memory_at) if memory_at else None,
                "objects": [{"label": entry.get("label"),
                             "last_seen": format_time(entry.get("last_seen", 0)),
                             "distance_m": entry.get("distance_m"),
                             "sightings": entry.get("sightings", 1),
                             "evidence": entry.get("evidence")}
                            for entry in entries]}

    def _last(self) -> dict:
        with self._lock:
            answer = self._last_answer
        if answer is None:
            return {"state": "no_answer",
                    "message": "还没有回答过问题；请先执行 ask"}
        return {"state": "ok", "answer": answer}

    # ── Card protocol ───────────────────────────────────────────────────────
    def get_tool(self) -> dict:
        return {
            "name": "object_locator",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "从位置记忆回答“我的xxx在哪”：优先读 object_memory/state 实时记忆，"
                "否则读取共享 memory.json；按开放词表标签做中英文匹配，返回位置、"
                "空间关系、时间和带目标标记的截图证据。list 列出已记物品，"
                "last 取回最近答案。只读记忆，不采集、不改记忆、不读相机。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["ask", "list", "last", "info", "start"]},
                    "question": {"type": "string", "maxLength": 200,
                                 "description": "例如：我的钥匙在哪？"},
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "ask": {"params": ["question"],
                            "description": "按问题检索记忆，返回答案和截图证据。"},
                    "list": {"params": [], "description": "列出记忆中的物品。"},
                    "last": {"params": [], "description": "返回最近一次问答结果。"},
                    "info": {"params": [], "description": "查看记忆来源、数量和存储路径。"},
                    "start": {"params": [], "description": "检查 ROS 和记忆输入是否就绪。"},
                },
            },
            "topic_in": [{"topic": self._memory_topic, "format": "data/json",
                          "message_type": "std_msgs/msg/String",
                          "desc": "object_memory 的完整记忆（可选）"}],
            "topic_out": [
                {"topic": self._answer_topic, "format": "data/json"},
                {"topic": self._evidence_topic, "format": "image/jpeg"},
            ],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self):
        if self._node is None:
            return {"state": "unavailable",
                    "message": "ROS 未初始化，object_locator 无法发布答案"}
        return {"state": "ready"}

    def stop(self):
        return None

    def dispatch(self, action: str, args: dict) -> dict | None:
        args = args or {}
        if action == "ask":
            return self._ask(args)
        if action == "list":
            return self._list()
        if action == "last":
            return self._last()
        if action == "start":
            return self.start()
        if action == "info":
            memory, source, memory_at = self._memory_snapshot()
            with self._lock:
                last = self._last_answer
            return {"state": "ready" if self._node is not None else "unavailable",
                    "memory_path": str(self._memory_path),
                    "exists": self._memory_path.is_file(),
                    "memory_source": source,
                    "memory_at": format_time(memory_at) if memory_at else None,
                    "objects": len(memory.get("objects") or {}),
                    "scenes": len(memory.get("scenes") or {}),
                    "topics": {"memory": self._memory_topic,
                               "answer": self._answer_topic,
                               "evidence": self._evidence_topic},
                    "last_answer": (last.get("answer") if last else None)}
        return None
