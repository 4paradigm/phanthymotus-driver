"""Huya-only actuator Card: explicit go_live/end_live, no automatic streaming."""
from __future__ import annotations

import threading

from .config import HuyaCredentialStore, validate_huya_url
from .worker import AudioBuffer, LiveStreamWorker


def attach_audio_subscription(executor, namespace: str, audio: AudioBuffer):
    from audio_msgs.msg import AudioChunk
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

    node = Node("bumi_huya_live_audio")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=64,
        durability=DurabilityPolicy.VOLATILE,
    )
    node.create_subscription(
        AudioChunk, f"/{namespace}/mic/audio",
        lambda msg: audio.push(bytes(msg.data), str(msg.format)),
        qos,
    )
    executor.add_node(node)
    return node


class LiveStreamPlugin:
    PREFIX = "live_stream"

    def __init__(self, cfg: dict, namespace: str, executor, camera,
                 *, store=None, audio=None, worker=None):
        self._lock = threading.RLock()
        self._store = store if store is not None else HuyaCredentialStore(
            cfg.get("credential_path", "/opt/phanthy-motus/data/live_stream/huya-rtmp-url"))
        self._url = self._store.read()
        self._audio = audio if audio is not None else AudioBuffer()
        self._worker = worker if worker is not None else LiveStreamWorker(
            camera, self._audio, fps=cfg.get("fps", 15),
            bitrate_kbps=cfg.get("video_bitrate_kbps", 1200))
        self._node = None
        self._executor = executor
        self._namespace = namespace

    def get_tool(self) -> dict:
        return {
            "name": "live_stream",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "虎牙直播：将 Bumi 的摄像头画面及麦克风声音直播到虎牙。"
                "首次使用：虎牙主播中心→普通/VR远程推流→复制完整地址；"
                "点击本卡片配置齿轮填写保存；执行 go_live 开播、end_live 停播。"
                "启动智能控制不会自动公开直播。仅支持虎牙。"
            ),
            "configSchema": {
                "type": "object",
                "properties": {
                    "rtmp_url": {
                        "type": "string",
                        "title": "虎牙 RTMP 完整推流地址",
                        "description": (
                            "在虎牙主播中心→普通/VR远程推流→普通直播，"
                            "点击复制完整推流地址后粘贴。"
                            "地址包含密钥，勿发送给他人。刷新地址后需重新保存。"
                        ),
                        "format": "password",
                        "x-sensitive": True,
                        "scope": "instance",
                    },
                },
                "required": ["rtmp_url"],
            },
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "go_live", "end_live", "config"],
                        "description": "虎牙直播控制操作",
                    },
                },
                "required": ["action"],
                # Agent Core creates an LLM tool for EVERY x-action-params entry.
                # Keep config internal: the Canvas gear persists configSchema
                # values via /api/canvas/tool-config, which dispatches config
                # directly with rtmp_url (and replays it on start/reconnect).
                # Exposing config here would leak an RTMP credential to the LLM.
                "x-action-params": {
                    "start": {"params": [], "description": "初始化 Card，不开播"},
                    "stop": {"params": [], "description": "画布停止时关闭直播"},
                    "info": {"params": [], "description": "查询直播连接状态"},
                    "go_live": {"params": [], "description": "公开开播：推送 Bumi 摄像头和麦克风"},
                    "end_live": {"params": [], "description": "结束虎牙直播"},
                },
            },
        }

    def start(self) -> dict:
        with self._lock:
            if self._node is None and self._executor is not None:
                self._node = attach_audio_subscription(
                    self._executor, self._namespace, self._audio)
            return {"state": "ready"}

    def stop(self) -> dict:
        with self._lock:
            result = self._worker.stop()
            if result.get("state") == "stopping":
                return result
            if self._node is not None:
                self._executor.remove_node(self._node)
                self._node.destroy_node()
                self._node = None
            self._audio.clear()
            return result

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {**self.start(), "configured": bool(self._url)}
        if action == "stop":
            return self.stop()
        if action == "end_live":
            return self._worker.stop()
        if action == "info":
            return {"platform": "huya", "configured": bool(self._url),
                    **self._worker.info()}
        if action == "config":
            value = args.get("rtmp_url")
            if not isinstance(value, str) or not value.strip():
                return {"ok": False, "error_code": "MISSING_URL",
                        "message": "请点击卡片齿轮填写虎牙完整 RTMP 推流地址"}
            try:
                url = validate_huya_url(value)
            except ValueError:
                return {"ok": False, "error_code": "INVALID_HUYA_URL",
                        "message": "仅支持虎牙远程推流地址，请重新复制"}
            with self._lock:
                if self._worker.info()["state"] in ("starting", "streaming", "stopping"):
                    return {"ok": False, "error_code": "STREAM_ACTIVE",
                            "message": "请先结束直播再更新推流地址"}
                try:
                    self._store.save(url)
                except OSError:
                    return {"ok": False, "error_code": "CONFIG_SAVE_FAILED",
                            "message": "无法安全保存推流地址，请检查机器人数据目录权限"}
                self._url = url
            return {"ok": True, "state": "ready", "configured": True,
                    "message": "推流地址已保存，使用 go_live 开始虎牙直播"}
        if action == "go_live":
            with self._lock:
                if not self._url:
                    return {"ok": False, "error_code": "MISSING_URL",
                            "message": "请先在卡片齿轮填写并保存虎牙 RTMP 推流地址"}
                return self._worker.start(self._url)
        return {"ok": False, "error_code": "UNKNOWN_ACTION"}
