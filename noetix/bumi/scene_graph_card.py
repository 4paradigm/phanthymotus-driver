#!/usr/bin/env python3
"""Bumi scene-graph card: open-vocabulary detections -> scene graph + evidence.

This variant keeps only the first stage of the position-memory pipeline.  It can
be used on its own (``snapshot`` returns the full scene graph and saved evidence
paths) and it emits exactly the topics the full pipeline's ``object_memory`` and
``object_locator`` cards consume:

* ``/<ns>/scene_graph/state`` — complete scene graph JSON;
* ``/<ns>/scene_graph/evidence`` — annotated JPEG.

VOP is still the open-vocabulary detector; this card neither trains nor ships a
model.  The camera/depth inputs follow the base Bumi ``camera`` card's declared
ports and runtime intrinsics, with the historic topic names only as a fallback.
"""

from __future__ import annotations

import json
import math
import threading
import time
from array import array
from pathlib import Path

from scene_graph_core import (DEFAULT_DEPTH_SCALE, MAX_RESULT_AGE_S,
                              MAX_TIMESTAMP_AGE_S, build_scene, decode_depth,
                              decode_scene, decode_vop, draw_evidence,
                              intrinsics_for, save_evidence)


class SceneGraphPlugin:
    PREFIX = "scene_graph"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 media_ctrl=None, camera_plugin=None):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._camera_plugin = camera_plugin
        self._lock = threading.RLock()
        self._node = None
        self._scene_pub = None
        self._evidence_pub = None
        self._compressed_image = None
        self._string = None
        self._rgb = None
        self._rgb_at = 0.0
        self._depth = None
        self._depth_at = 0.0
        self._objects = None
        self._objects_at = 0.0
        self._objects_error = None
        self._last_scene = None
        self._snapshot_count = 0

        base_topics = self._base_topics(camera_plugin)
        self._rgb_topic = (str(plugin_config.get("rgb_topic") or "").strip()
                           or base_topics.get("camera")
                           or f"/{namespace}/camera/color")
        self._depth_topic = (str(plugin_config.get("depth_topic") or "").strip()
                             or base_topics.get("depth")
                             or f"/{namespace}/camera/depth")
        self._objects_topic = (str(plugin_config.get("objects_topic") or "").strip()
                               or self._rgb_topic + "/objects")
        self._scene_topic = f"/{namespace}/scene_graph/state"
        self._evidence_topic = f"/{namespace}/scene_graph/evidence"
        self._depth_scale = self._bounded_float(
            plugin_config.get("depth_scale"), DEFAULT_DEPTH_SCALE, 0.0, 0.1, True)
        self._intrinsics = self._read_intrinsics(plugin_config)

        self._storage = Path(str(plugin_config.get(
            "storage_dir", "/opt/phanthy-motus/data/scene_graph"))).expanduser()
        self._scenes_dir = self._storage / "scenes"
        self._evidence_dir = self._storage / "evidence"
        self._latest_path = self._storage / "latest.json"
        self._load_last_snapshot()

        try:
            import rclpy  # noqa: F401
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                                   DurabilityPolicy)
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import String
            node = Node("bumi_scene_graph")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=2,
                             durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(CompressedImage, self._rgb_topic, self._on_rgb, qos)
            node.create_subscription(CompressedImage, self._depth_topic, self._on_depth, qos)
            node.create_subscription(String, self._objects_topic, self._on_objects, qos)
            self._scene_pub = node.create_publisher(String, self._scene_topic, qos)
            self._evidence_pub = node.create_publisher(
                CompressedImage, self._evidence_topic, qos)
            self._compressed_image = CompressedImage
            self._string = String
            executor.add_node(node)
            self._node = node
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[scene_graph] ROS setup failed: {exc}", flush=True)
            self._node = None

    # ── Base-camera adaptation ──────────────────────────────────────────────
    @staticmethod
    def _base_topics(camera_plugin):
        """Reuse the base camera card's declared ports when it is available."""
        topics = {}
        if camera_plugin is None:
            return topics
        try:
            for tool in camera_plugin.get_tools():
                name = tool.get("name")
                outputs = tool.get("topic_out") or []
                if name in ("camera", "depth") and outputs:
                    topics[name] = outputs[0]["topic"]
        except Exception:
            pass
        return topics

    @staticmethod
    def _bounded_float(value, default, low, high, exclusive_low=False):
        if isinstance(value, bool):
            return default
        try:
            number = float(value)
        except (TypeError, ValueError):
            return default
        if not math.isfinite(number):
            return default
        if (number <= low if exclusive_low else number < low) or number > high:
            return default
        return number

    @staticmethod
    def _read_intrinsics(config: dict):
        values = {}
        for key in ("fx", "fy", "ppx", "ppy"):
            value = config.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                return None
            values[key] = float(value)
        return values if values["fx"] > 0 and values["fy"] > 0 else None

    def _camera_intrinsics(self, width: int, height: int):
        """Runtime K from the base camera card; fall back to camera_specs."""
        if self._camera_plugin is not None:
            try:
                info = self._camera_plugin.dispatch("info", {"_tool_name": "camera"})
                descriptors = (info or {}).get("camera_info") or []
                if descriptors:
                    descriptor = descriptors[0]
                    k = descriptor.get("K")
                    if (isinstance(k, list) and len(k) == 9
                            and isinstance(k[0], (int, float))
                            and isinstance(k[4], (int, float))
                            and k[0] and k[4]):
                        return {"fx": float(k[0]), "fy": float(k[4]),
                                "ppx": float(k[2]), "ppy": float(k[5])}
                    half_h = descriptor.get("half_fov_rad")
                    half_v = descriptor.get("half_fov_v_rad")
                    if (isinstance(half_h, (int, float))
                            and isinstance(half_v, (int, float))):
                        return {"fx": (width / 2.0) / math.tan(half_h),
                                "fy": (height / 2.0) / math.tan(half_v),
                                "ppx": width / 2.0, "ppy": height / 2.0}
            except Exception:
                pass
        return intrinsics_for(width, height)

    # ── Inputs ──────────────────────────────────────────────────────────────
    def _on_rgb(self, msg):
        with self._lock:
            self._rgb = bytes(msg.data)
            self._rgb_at = time.time()

    def _on_depth(self, msg):
        with self._lock:
            self._depth = bytes(msg.data)
            self._depth_at = time.time()

    def _on_objects(self, msg):
        now = time.time()
        try:
            payload = decode_vop(bytes(msg.data), now)
        except ValueError as exc:
            with self._lock:
                self._objects_error = str(exc)[:200]
            return
        with self._lock:
            self._objects = payload
            self._objects_at = now
            self._objects_error = None

    # ── Snapshot ────────────────────────────────────────────────────────────
    def _snapshot(self):
        now = time.time()
        with self._lock:
            objects, objects_at = self._objects, self._objects_at
            rgb, rgb_at = self._rgb, self._rgb_at
            depth, depth_at = self._depth, self._depth_at
            error = self._objects_error
        if objects is None or now - objects_at > MAX_RESULT_AGE_S:
            return None, "detector_unavailable", (
                f"没有收到新鲜的 VOP 开放词表检测；请把 VOP 输出接到 {self._objects_topic}"
                + (f"（最近错误：{error}）" if error else ""))
        if now - objects["timestamp"] > MAX_TIMESTAMP_AGE_S:
            return None, "detector_unavailable", "VOP 检测结果已过期"
        if rgb is None or now - rgb_at > MAX_RESULT_AGE_S:
            return None, "camera_unavailable", (
                f"相机图像不可用；请确认 camera 卡片在发布 {self._rgb_topic}")
        try:
            import cv2
            import numpy as np
            image = cv2.imdecode(np.frombuffer(rgb, np.uint8), cv2.IMREAD_COLOR)
        except Exception as exc:
            return None, "camera_unavailable", f"无法解码 RGB 图像：{exc}"
        if image is None:
            return None, "camera_unavailable", "无法解码 RGB 图像"
        height, width = image.shape[:2]
        depth_image = None
        if depth is not None and now - depth_at <= MAX_RESULT_AGE_S:
            depth_image = decode_depth(depth, width, height)
        intrinsics = self._intrinsics or self._camera_intrinsics(width, height)
        scene = build_scene(objects["objects"], width, height, depth_image,
                            self._depth_scale, intrinsics, time.time())
        scene["intrinsics"] = intrinsics
        scene["depth_scale"] = self._depth_scale
        scene["source"] = {"rgb_topic": self._rgb_topic,
                           "depth_topic": self._depth_topic,
                           "objects_topic": self._objects_topic}
        return (rgb, scene), None, None

    def _load_last_snapshot(self) -> None:
        """Restore the last snapshot pointer so `last` works after a restart."""
        try:
            latest = json.loads(self._latest_path.read_text(encoding="utf-8"))
            scene_path = latest.get("scene_path")
            if not isinstance(scene_path, str):
                return
            scene = decode_scene(Path(scene_path).read_text(encoding="utf-8"))
        except Exception:
            return
        with self._lock:
            self._last_scene = scene

    def _save_snapshot(self, scene: dict, jpeg: bytes) -> str:
        """Persist evidence + scene JSON + latest pointer; return scene path."""
        evidence_path = ""
        scene["evidence_path"] = ""
        try:
            evidence_path = save_evidence(self._evidence_dir,
                                          f"scene_{scene['id']}", jpeg)
            scene["evidence_path"] = evidence_path
        except Exception as exc:
            print(f"[scene_graph] evidence save failed: {exc}", flush=True)
        scene_path = ""
        try:
            self._scenes_dir.mkdir(parents=True, exist_ok=True)
            path = self._scenes_dir / f"scene_{scene['id']}.json"
            scene["scene_path"] = str(path)
            path.write_text(json.dumps(scene, ensure_ascii=False, indent=1),
                            encoding="utf-8")
            scene_path = str(path)
            latest = {"scene_id": scene["id"],
                      "scene_path": scene_path,
                      "evidence_path": evidence_path,
                      "captured_at": scene["captured_at"],
                      "captured_at_text": scene["captured_at_text"]}
            temporary = self._latest_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(latest, ensure_ascii=False),
                                 encoding="utf-8")
            temporary.replace(self._latest_path)
        except OSError as exc:
            print(f"[scene_graph] scene save failed: {exc}", flush=True)
        return scene_path

    # ── Output ──────────────────────────────────────────────────────────────
    def _publish(self, scene: dict, evidence: bytes) -> None:
        if self._node is None:
            return
        try:
            if self._scene_pub is not None and self._string is not None:
                message = self._string()
                message.data = json.dumps(scene, ensure_ascii=False)
                self._scene_pub.publish(message)
            if self._evidence_pub is not None and self._compressed_image is not None:
                image = self._compressed_image()
                image.header.stamp = self._node.get_clock().now().to_msg()
                image.format = "jpeg"
                image.data = array("B", evidence)
                self._evidence_pub.publish(image)
        except Exception as exc:  # pragma: no cover - robot runtime only
            print(f"[scene_graph] publish failed: {exc}", flush=True)

    def _take_snapshot(self, args: dict) -> dict:
        snapshot, state, message = self._snapshot()
        if snapshot is None:
            return {"state": state, "message": message}
        rgb, scene = snapshot
        label = args.get("label")
        label = label.strip()[:100] if isinstance(label, str) and label.strip() else None
        evidence = draw_evidence(rgb, scene, label)
        scene_path = self._save_snapshot(scene, evidence)
        self._publish(scene, evidence)
        with self._lock:
            self._last_scene = scene
            self._snapshot_count += 1
        labels = [node["label"] for node in scene["nodes"]]
        highlighted = label if label and label.casefold() in {
            item.casefold() for item in labels} else None
        result = {
            "state": "captured",
            "scene_id": scene["id"],
            "captured_at": scene["captured_at_text"],
            "count": len(scene["nodes"]),
            "labels": labels,
            "objects": [{"label": node["label"],
                         "confidence": node["confidence"],
                         "distance_m": node["distance_m"],
                         "position": node["robot"],
                         "pixel": node["pixel"]}
                        for node in scene["nodes"]],
            "edges": scene["edges"],
            "scene": scene,
            "scene_path": scene_path,
            "evidence_path": scene.get("evidence_path", ""),
            "scene_topic": self._scene_topic,
            "evidence_topic": self._evidence_topic,
            "highlighted": highlighted,
        }
        if label and highlighted is None:
            result["message"] = f"场景里没有标签“{label}”；截图标注了所有物体"
        return result

    def _last(self) -> dict:
        with self._lock:
            scene = self._last_scene
        if scene is None:
            return {"state": "no_snapshot",
                    "message": "还没有快照；请先执行 snapshot"}
        return {"state": "ok",
                "scene": scene,
                "scene_path": scene.get("scene_path", ""),
                "evidence_path": scene.get("evidence_path", ""),
                "scene_topic": self._scene_topic,
                "evidence_topic": self._evidence_topic}

    # ── Card protocol ───────────────────────────────────────────────────────
    def get_tool(self) -> dict:
        return {
            "name": "scene_graph",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "用 VOP 开放词表检测和 Bumi RGB/深度构建场景图。"
                "snapshot 返回完整场景图（物体、像素、距离、机器人坐标、空间关系）、"
                "标注截图文件路径，并发布 scene_graph/state + evidence 话题；"
                "与完整流水线的 object_memory / object_locator 卡直接兼容，"
                "单独使用时用 last 取回最近一次场景图和截图。"
                "不保存照片/视频（那仍由 vision_capture 负责）。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["snapshot", "last", "start", "info"]},
                    "label": {"type": "string", "maxLength": 100,
                              "description": "snapshot 可选：在截图中高亮这个标签"},
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "snapshot": {"params": ["label"],
                                 "description": "抓取当前 VOP + 相机帧，生成完整场景图和标注截图；可选中一个标签高亮。"},
                    "last": {"params": [], "description": "返回最近一次场景图和截图路径（重启后也能取）。"},
                    "start": {"params": [], "description": "检查 ROS 输入和存储目录。"},
                    "info": {"params": [], "description": "查看输入话题、数据新鲜度、最近场景和存储路径。"},
                },
            },
            "topic_in": [
                {"topic": self._rgb_topic, "format": "image/jpeg",
                 "message_type": "sensor_msgs/msg/CompressedImage",
                 "desc": "Bumi RGB 图像"},
                {"topic": self._depth_topic, "format": "image/depth-zlib",
                 "message_type": "sensor_msgs/msg/CompressedImage",
                 "desc": "Bumi 深度图（可选，用于距离）"},
                {"topic": self._objects_topic, "format": "data/json",
                 "message_type": "std_msgs/msg/String",
                 "desc": "VOP 开放词表检测结果"},
            ],
            "topic_out": [
                {"topic": self._scene_topic, "format": "data/json"},
                {"topic": self._evidence_topic, "format": "image/jpeg"},
            ],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self):
        if self._node is None:
            return {"state": "unavailable",
                    "message": "ROS 未初始化，scene_graph 无法订阅相机/VOP"}
        try:
            self._storage.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return {"state": "unavailable", "message": f"存储目录不可写：{exc}"}
        if self._last_scene is None:
            self._load_last_snapshot()
        return {"state": "ready"}

    def stop(self):
        return None

    def dispatch(self, action: str, args: dict) -> dict | None:
        args = args or {}
        if action == "snapshot":
            return self._take_snapshot(args)
        if action == "last":
            return self._last()
        if action == "start":
            return self.start()
        if action == "info":
            now = time.time()
            with self._lock:
                ages = {"rgb_age_s": (round(now - self._rgb_at, 2) if self._rgb else None),
                        "depth_age_s": (round(now - self._depth_at, 2) if self._depth else None),
                        "objects_age_s": (round(now - self._objects_at, 2)
                                          if self._objects else None)}
                last = self._last_scene
                count = self._snapshot_count
                error = self._objects_error
            return {"state": "ready" if self._node is not None else "unavailable",
                    "topics": {"rgb": self._rgb_topic, "depth": self._depth_topic,
                               "objects": self._objects_topic,
                               "scene": self._scene_topic,
                               "evidence": self._evidence_topic},
                    "storage": {"root": str(self._storage),
                                "scenes": str(self._scenes_dir),
                                "evidence": str(self._evidence_dir),
                                "latest": str(self._latest_path)},
                    "freshness": ages, "detector_error": error,
                    "snapshots": count,
                    "last_scene": ({"scene_id": last["id"],
                                    "captured_at": last["captured_at_text"],
                                    "count": len(last["nodes"]),
                                    "scene_path": last.get("scene_path", ""),
                                    "evidence_path": last.get("evidence_path", "")}
                                   if last else None)}
        return None
