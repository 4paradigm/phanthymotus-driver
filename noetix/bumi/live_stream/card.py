"""Supervised Bumi live-stream Card for approved RTMP destinations."""
from __future__ import annotations

import threading
import time

from .config import HuyaCredentialStore, validate_stream_url
from .approval import verify_live_stream_action
from .worker import AudioBuffer, LiveStreamWorker


def attach_audio_subscription(executor, namespace: str, audio: AudioBuffer):
    from audio_msgs.msg import AudioChunk
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

    node = Node("bumi_live_stream_audio")
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
                 *, store=None, audio=None, worker=None, clock=None,
                 approval_verifier=None):
        self._lock = threading.RLock()
        self._store = store if store is not None else HuyaCredentialStore(
            cfg.get("credential_path", "/opt/phanthy-motus/data/live_stream/huya-rtmp-url"))
        self._url = self._store.read()
        self._platform = validate_stream_url(self._url)[0] if self._url else None
        self._clock = clock if clock is not None else time.monotonic
        self._verify_approval = approval_verifier or verify_live_stream_action
        # Save from the Canvas gear authorizes one start for five minutes.
        # Merely restoring a persisted URL on startup never arms a broadcast.
        self._armed_until = 0.0
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
                "Bumi 直播：支持虎牙和小红书授权 RTMP 地址。"
                "小红书三方推流页面复制服务器及推流码，本地拼接为完整 URL；"
                "在齿轮中填写新推流地址并保存，即授权本场 go_live。"
                "start 不会公开直播，end_live 结束推流。"
            ),
            "configSchema": {
                "type": "object",
                "properties": {
                    "rtmp_url": {
                        "type": "string",
                        "title": "RTMP 完整推流地址（含推流码）",
                        "description": (
                            "小红书：将服务器地址末尾加 / 再拼接推流码；"
                            "虎牙：直接复制完整 RTMP 地址。"
                            "含敏感密钥，仅在本机齿轮填写，勿发送到聊天。"
                        ),
                        "format": "password",
                        "x-sensitive": True,
                        # Keep the instance gear visible in stock Canvas.
                        # The Driver validates missing URLs and persists them.
                        "scope": "instance",
                    },
                },
                # Core cannot resolve an instance-scoped setting when a
                # Canvas action omits instance_id. Let Driver preflight it.
                "required": [],
            },
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "go_live", "end_live", "config"],
                        "description": "受控直播操作",
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
                    "go_live": {"params": [], "description": "本场新地址保存后五分钟内允许一次开播"},
                    "end_live": {"params": [], "description": "结束直播"},
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
            return {"platform": self._platform or "none", "configured": bool(self._url),
                    "ready_to_go_live": bool(self._url) and self._clock() < self._armed_until,
                    "authorization_expires_in_s": max(0, round(self._armed_until - self._clock(), 1)),
                    **self._worker.info()}
        if action == "config":
            value = args.get("rtmp_url")
            if not isinstance(value, str) or not value.strip():
                return {"ok": False, "error_code": "MISSING_URL",
                        "message": "请在卡片齿轮填写完整 RTMP 地址"}
            try:
                platform, url = validate_stream_url(value)
            except ValueError:
                return {"ok": False, "error_code": "INVALID_STREAM_URL",
                        "message": "仅支持已验证的虎牙或小红书官方 RTMP 地址"}
            with self._lock:
                signed = self._verify_approval('config', url, args.get('operator_proof'))
                if not signed and (url != self._url or args.get('authorize_start') is True):
                    return {'ok': False, 'error_code': 'APPROVAL_REQUIRED',
                            'message': '只有通过 Agent Core 用户授权才能设置直播地址'}
                state = self._worker.info()["state"]
                if state in ("starting", "streaming", "stopping"):
                    if url == self._url:
                        # Core replays config before end_live; an identical
                        # replay is a no-op, never a new approval or write.
                        return {"ok": True, "state": state, "configured": True,
                                "ready_to_go_live": False}
                    return {"ok": False, "error_code": "STREAM_ACTIVE",
                            "message": "请先结束直播再更新推流地址"}
                try:
                    self._store.save(url)
                except OSError:
                    return {"ok": False, "error_code": "CONFIG_SAVE_FAILED",
                            "message": "无法安全保存推流地址，请检查机器人数据目录权限"}
                # Replaying the same saved configuration on reconnect does not
                # grant fresh authority to capture and publish camera/microphone.
                # Only an explicit Canvas user action can re-arm an unchanged key.
                # Background config replay must never authorize a broadcast.
                if signed and (url != self._url or args.get("authorize_start") is True):
                    self._armed_until = self._clock() + 300.0
                self._url = url
                self._platform = platform
            armed = self._clock() < self._armed_until
            return {"ok": True, "state": "ready", "configured": True,
                    "ready_to_go_live": armed,
                    "message": ("已授权本场直播，请在五分钟内执行 go_live" if armed
                                else "地址已保存，但重复保存旧地址不会重新授权；请获取新推流码")}
        if action == "go_live":
            with self._lock:
                if not self._url:
                    return {"ok": False, "error_code": "MISSING_URL",
                            "message": "请先在卡片齿轮填写并保存完整 RTMP 地址"}
                if self._clock() >= self._armed_until:
                    return {"ok": False, "error_code": "APPROVAL_REQUIRED",
                            "message": "请在卡片齿轮重新填写本场新推流地址并保存"}
                if not self._verify_approval('go_live', self._url, args.get('operator_proof')):
                    return {'ok': False, 'error_code': 'APPROVAL_REQUIRED',
                            'message': '只有经过 Agent Core 人工确认的开播请求才能执行'}
                # A direct go_live call must not require a separate Canvas start.
                if self._node is None and self._executor is not None:
                    self.start()
                    deadline = time.monotonic() + 2.0
                    while not self._audio.fresh() and time.monotonic() < deadline:
                        time.sleep(0.05)
                result = self._worker.start(self._url)
                if result.get("state") in ("starting", "streaming"):
                    self._armed_until = 0.0  # Consume only after successful preflight.
                return result
        return {"ok": False, "error_code": "UNKNOWN_ACTION"}
