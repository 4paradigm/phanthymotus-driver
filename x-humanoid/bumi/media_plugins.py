#!/usr/bin/env python3
"""
x-humanoid/bumi/media_plugins.py — Bumi MediaController 插件（语音/音视频系统）。

所有插件通过 BumiSDK 共享 MediaController 实例，MediaController 通过 DDS 与
运控板通信，提供语音助手、音视频采集/播放、外部音视频注入等能力。

插件列表：
  MediaSystemStatusPlugin   (sensor) — 语音系统工作状态 (READY/SLEEPED/WAKEUPED/EXIT)
  MediaSystemErrorPlugin    (sensor) — 语音系统最近错误 (code + message)
  MicrophonePlugin          (sensor) — 内部麦克风音频流 (PCM 16kHz)
  SpeakerAudioPlugin        (sensor) — 扬声器播放音频流 (PCM)
  VideoCapturePlugin        (sensor) — 内部摄像头视频流 (JPEG/YUYV)
  VideoDesensedPlugin       (sensor) — 脱敏视频流 (JPEG, 给 AI 大模型)
  WakewordPlugin             (actuator) — 唤醒/休眠/重启 + 唤醒词配置
  VolumePlugin               (actuator) — 音量 get/set
  TimeoutConfigPlugin        (actuator) — 对话超时 get/set
  BeepSwitchPlugin           (actuator) — 提示音开关 get/set
  AudioRoutingPlugin         (actuator) — 音视频路由开关矩阵 (7 个独立开关)
  AudioCaptureControlPlugin  (actuator) — 音频采集 pause/resume
  AudioPlaybackControlPlugin (actuator) — 音频播放 pause/resume
  VideoCaptureControlPlugin  (actuator) — 视频采集 pause/resume
  ExternalAudioInputPlugin   (actuator) — 外部音频注入 AI
  ExternalAudioOutputPlugin  (actuator) — 外部音频推流到扬声器
  ExternalVideoInputPlugin   (actuator) — 外部视频注入 AI
"""

from __future__ import annotations

import time
import threading

from device import _publish


# ══════════════════════════════════════════════════════════════════════════════
# Media Sensor Plugins
# ══════════════════════════════════════════════════════════════════════════════

class _MediaSensorBase:
    """MediaController 传感器基类。"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action, args):
        """Base dispatch — handles start/stop lifecycle. Subclasses should call super().dispatch() as fallback."""
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        return {"state": "running" if self._running else "idle"}

    def _media(self):
        m = self._sdk.media
        if m is None:
            raise RuntimeError("MediaController not initialized")
        return m


class MediaSystemStatusPlugin(_MediaSensorBase):
    """语音系统工作状态"""

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._topic = f"/{namespace}/media/status"
        self._latest = {"work_status": "unknown", "reason": "waiting"}

    def get_tool(self):
        return {
            "name": "media_system_status",
            "type": "sensor",
            "description": "Bumi 语音系统状态 — WorkStatus (READY/SLEEPED/WAKEUPED/EXIT) + StatusChangeReason",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            time.sleep(1.0)
            try:
                status = self._media().get_system_status()
                self._latest = {
                    "work_status": str(status.value),
                    "reason": str(status.reason),
                    "timestamp_us": int(status.header.timestamp_us),
                }
                _publish(self._topic, self._latest)
            except Exception as e:
                print(f"[MediaSystemStatusPlugin] poll error: {e}", flush=True)

    def dispatch(self, action, args):
        if action in ("read", "get", "media_system_status"):
            return dict(self._latest)
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return super().dispatch(action, args)


class MediaSystemErrorPlugin(_MediaSensorBase):
    """语音系统最近错误"""

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._topic = f"/{namespace}/media/error"
        self._latest = {"code": 0, "message": ""}

    def get_tool(self):
        return {
            "name": "media_system_error",
            "type": "sensor",
            "description": "Bumi 语音系统最近错误 (code + message)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            time.sleep(1.0)
            try:
                err = self._media().get_system_error()
                self._latest = {
                    "code": int(err.code),
                    "message": str(err.message),
                    "timestamp_us": int(err.header.timestamp_us),
                }
                _publish(self._topic, self._latest)
            except Exception as e:
                print(f"[MediaSystemErrorPlugin] poll error: {e}", flush=True)

    def dispatch(self, action, args):
        if action in ("read", "get", "media_system_error"):
            return dict(self._latest)
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return super().dispatch(action, args)


class MicrophonePlugin(_MediaSensorBase):
    """内部麦克风音频流 (PCM 16kHz)"""

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._topic = f"/{namespace}/media/mic"
        self._latest = None

    def get_tool(self):
        return {
            "name": "microphone",
            "type": "sensor",
            "description": "Bumi 内部麦克风音频流 (PCM 16kHz)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            time.sleep(0.01)  # 100Hz for audio
            try:
                stream = self._media().get_audio_capture_data()
                self._latest = {
                    "channels": int(stream.channels),
                    "sample_rate": int(stream.sample_rate),
                    "format": int(stream.format),
                    "duration_ms": int(stream.duration_ms),
                    "timestamp_us": int(stream.timestamp_us),
                    # audio_data is List[int16], publish via topic
                }
                _publish(self._topic, self._latest)
            except Exception:
                pass

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return super().dispatch(action, args)


class SpeakerAudioPlugin(_MediaSensorBase):
    """扬声器播放音频流"""

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._topic = f"/{namespace}/media/speaker"
        self._latest = None

    def get_tool(self):
        return {
            "name": "speaker_audio",
            "type": "sensor",
            "description": "Bumi 扬声器当前播放音频流 (PCM)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            time.sleep(0.01)
            try:
                stream = self._media().get_audio_playback_data()
                self._latest = {
                    "channels": int(stream.channels),
                    "sample_rate": int(stream.sample_rate),
                    "format": int(stream.format),
                    "duration_ms": int(stream.duration_ms),
                    "timestamp_us": int(stream.timestamp_us),
                }
                _publish(self._topic, self._latest)
            except Exception:
                pass

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return super().dispatch(action, args)


class VideoCapturePlugin(_MediaSensorBase):
    """内部摄像头视频流

    NOTE: EDU 版本 Bumi 相机硬件直接接算力板，非运控板。
         此卡依赖运控板 DDS 回传视频数据，若运控板无内部摄像头则无数据。
         需实际硬件测试确认可用性。
    """

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._topic = f"/{namespace}/media/video"
        self._latest = None

    def get_tool(self):
        return {
            "name": "video_capture",
            "type": "sensor",
            "description": "Bumi 内部摄像头视频流 (JPEG/YUYV) — 需先通过 audio_routing 开启 camera_to_ai [EDU版本需确认运控板是否有摄像头]",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            time.sleep(0.033)  # ~30Hz
            try:
                stream = self._media().get_video_capture_data()
                self._latest = {
                    "width": int(stream.width),
                    "height": int(stream.height),
                    "format": int(stream.format),
                    "fps": int(stream.fps),
                    "timestamp_us": int(stream.timestamp_us),
                }
                _publish(self._topic, self._latest)
            except Exception:
                pass

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        return super().dispatch(action, args)


class VideoDesensedPlugin(_MediaSensorBase):
    """脱敏视频流 (给 AI 大模型)

    NOTE: EDU 版本 Bumi 相机硬件直接接算力板，非运控板。
         此卡依赖运控板 DDS 回传脱敏视频，若运控板无内部摄像头则无数据。
         需实际硬件测试确认可用性。
    """

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._topic = f"/{namespace}/media/video_desensed"
        self._latest = None

    def get_tool(self):
        return {
            "name": "video_desensed",
            "type": "sensor",
            "description": "Bumi 脱敏视频流 (人脸/隐私脱敏, JPEG) — 给 AI 大模型 [EDU版本需确认运控板是否有摄像头]",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while self._running:
            time.sleep(0.033)
            try:
                stream = self._media().get_video_capture_desensed_data()
                self._latest = {
                    "width": int(stream.width),
                    "height": int(stream.height),
                    "format": int(stream.format),
                    "fps": int(stream.fps),
                    "timestamp_us": int(stream.timestamp_us),
                }
                _publish(self._topic, self._latest)
            except Exception:
                pass

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        return super().dispatch(action, args)


# ══════════════════════════════════════════════════════════════════════════════
# Media Actuator Plugins
# ══════════════════════════════════════════════════════════════════════════════

class _MediaActuatorBase:
    """MediaController 执行器基类。"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action, args):
        """Base dispatch — handles start/stop lifecycle. Subclasses should call super().dispatch() as fallback."""
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _media(self):
        m = self._sdk.media
        if m is None:
            raise RuntimeError("MediaController not initialized")
        return m


class WakewordPlugin(_MediaActuatorBase):
    """语音唤醒管理 — wakeup/sleep/restart + 唤醒词配置"""

    def get_tool(self):
        return {
            "name": "wakeword",
            "type": "actuator",
            "description": "Bumi 语音唤醒管理 — wakeup/sleep/restart + 唤醒词/回复词配置",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["wakeup", "sleep", "restart",
                                 "set_wakeup_response",
                                 "set_sleep_response", "get_wakeup_words"],
                        "default": "wakeup",
                    },
                    "words": {"type": "string", "description": "唤醒词/回复词文本"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "wakeup":
                m.wakeup()
                return {"state": "wakeup_sent"}
            if action == "sleep":
                m.sleep()
                return {"state": "sleep_sent"}
            if action == "restart":
                m.restart()
                return {"state": "restart_sent"}
            if action == "set_wakeup_response":
                m.set_wakeup_response_words(args.get("words", ""))
                return {"state": "ok"}
            if action == "set_sleep_response":
                m.set_sleep_response_words(args.get("words", ""))
                return {"state": "ok"}
            if action == "get_wakeup_words":
                return {"wakeup_words": m.get_wakeup_words()}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class VolumePlugin(_MediaActuatorBase):
    """音量控制 — get/set"""

    def get_tool(self):
        return {
            "name": "volume",
            "type": "actuator",
            "description": "Bumi 音量控制 — get/set",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["get", "set"], "default": "get"},
                    "value": {"type": "integer", "minimum": 0, "maximum": 100,
                              "description": "音量值 (0-100)"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "get":
                return {"volume": m.get_volume()}
            if action == "set":
                value = int(args.get("value", 50))
                m.set_volume(value)
                return {"state": "ok", "volume": value}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class TimeoutConfigPlugin(_MediaActuatorBase):
    """对话超时配置 — get/set"""

    def get_tool(self):
        return {
            "name": "timeout_config",
            "type": "actuator",
            "description": "Bumi 对话超时配置 — get/set (ms, 空闲超时自动休眠)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["get", "set"], "default": "get"},
                    "timeout_ms": {"type": "integer", "minimum": 1000,
                                   "description": "超时时间 (ms)"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "get":
                return {"timeout_ms": m.get_timeout()}
            if action == "set":
                timeout = int(args.get("timeout_ms", 30000))
                m.set_timeout(timeout)
                return {"state": "ok", "timeout_ms": timeout}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class BeepSwitchPlugin(_MediaActuatorBase):
    """提示音开关 — get/set"""

    def get_tool(self):
        return {
            "name": "beep_switch",
            "type": "actuator",
            "description": "Bumi 提示音开关 — get/set",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["get", "set"], "default": "get"},
                    "enable": {"type": "boolean", "description": "开关状态"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "get":
                return {"audio_cue_enable": m.get_audio_cue_enable()}
            if action == "set":
                enable = bool(args.get("enable", True))
                m.set_audio_cue_enable(enable)
                return {"state": "ok", "audio_cue_enable": enable}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class AudioRoutingPlugin(_MediaActuatorBase):
    """音视频路由开关矩阵 — 7 个独立开关"""

    _ROUTES = {
        "mic_to_ai": ("get_internal_capture_audio_data_to_agent_enable",
                      "set_internal_capture_audio_data_to_agent_enable"),
        "ext_audio_to_ai": ("get_external_custom_audio_data_to_agent_enable",
                            "set_external_custom_audio_data_to_agent_enable"),
        "ai_to_speaker": ("get_internal_agent_audio_data_to_playback_enable",
                          "set_internal_agent_audio_data_to_playback_enable"),
        "ext_audio_to_speaker": ("get_external_custom_audio_data_to_playback_enable",
                                 "set_external_custom_audio_data_to_playback_enable"),
        "camera_to_ai": ("get_internal_capture_video_data_to_agent_enable",
                         "set_internal_capture_video_data_to_agent_enable"),
        "ext_video_to_ai": ("get_external_custom_video_data_to_agent_enable",
                            "set_external_custom_video_data_to_agent_enable"),
        "ext_audio_3a": ("get_external_custom_audio_data_to_agent_use_internal_3a",
                         "set_external_custom_audio_data_to_agent_use_internal_3a"),
    }

    def get_tool(self):
        return {
            "name": "audio_routing",
            "type": "actuator",
            "description": "Bumi 音视频路由开关矩阵 — 7 个独立路由 + 3A 开关",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["get", "set"], "default": "get"},
                    "route": {
                        "type": "string",
                        "enum": list(self._ROUTES.keys()),
                        "description": "路由名称",
                    },
                    "enable": {"type": "boolean", "description": "开关状态"},
                },
                "required": ["action", "route"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            route = args.get("route")
            if route not in self._ROUTES:
                return {"error": f"unknown route: {route}"}
            getter, setter = self._ROUTES[route]
            if action == "get":
                return {"route": route, "enable": getattr(m, getter)()}
            if action == "set":
                enable = bool(args.get("enable", True))
                getattr(m, setter)(enable)
                return {"state": "ok", "route": route, "enable": enable}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class AudioCaptureControlPlugin(_MediaActuatorBase):
    """音频采集控制 — pause/resume"""

    def get_tool(self):
        return {
            "name": "audio_capture_control",
            "type": "actuator",
            "description": "Bumi 音频采集控制 — pause/resume",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["pause", "resume"], "default": "pause"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "pause":
                m.pause_audio_capture()
                return {"state": "paused"}
            if action == "resume":
                m.resume_audio_capture()
                return {"state": "resumed"}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class AudioPlaybackControlPlugin(_MediaActuatorBase):
    """音频播放控制 — pause/resume"""

    def get_tool(self):
        return {
            "name": "audio_playback_control",
            "type": "actuator",
            "description": "Bumi 音频播放控制 — pause/resume",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["pause", "resume"], "default": "pause"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "pause":
                m.pause_audio_playback()
                return {"state": "paused"}
            if action == "resume":
                m.resume_audio_playback()
                return {"state": "resumed"}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class VideoCaptureControlPlugin(_MediaActuatorBase):
    """视频采集控制 — pause/resume"""

    def get_tool(self):
        return {
            "name": "video_capture_control",
            "type": "actuator",
            "description": "Bumi 视频采集控制 — pause/resume",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["pause", "resume"], "default": "pause"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        try:
            m = self._media()
            if action == "pause":
                m.pause_video_capture()
                return {"state": "paused"}
            if action == "resume":
                m.resume_video_capture()
                return {"state": "resumed"}
            return super().dispatch(action, args)
        except Exception as e:
            return {"error": str(e)}


class ExternalAudioInputPlugin(_MediaActuatorBase):
    """外部音频注入 AI"""

    def get_tool(self):
        return {
            "name": "external_audio_input",
            "type": "actuator",
            "description": "Bumi 外部音频注入 AI — 需先通过 audio_routing 开启 ext_audio_to_ai",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["publish"], "default": "publish"},
                    "channels": {"type": "integer", "default": 1},
                    "sample_rate": {"type": "integer", "default": 16000},
                    "format": {"type": "integer", "default": 0},
                    "duration_ms": {"type": "integer", "default": 10},
                    "audio_data": {
                        "type": "array", "items": {"type": "integer"},
                        "description": "int16 采样列表",
                    },
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        if action in ("start", "stop"):
            return super().dispatch(action, args)
        try:
            from mediacontrol_py import AudioStream
            stream = AudioStream()
            stream.channels = int(args.get("channels", 1))
            stream.sample_rate = int(args.get("sample_rate", 16000))
            stream.format = int(args.get("format", 0))
            stream.duration_ms = int(args.get("duration_ms", 10))
            stream.audio_data = args.get("audio_data", [])
            self._media().publish_external_audio_stream(stream)
            return {"state": "published"}
        except Exception as e:
            return {"error": str(e)}


class ExternalAudioOutputPlugin(_MediaActuatorBase):
    """外部音频推流到扬声器"""

    def get_tool(self):
        return {
            "name": "external_audio_output",
            "type": "actuator",
            "description": "Bumi 外部音频推流到扬声器 — 需先通过 audio_routing 开启 ext_audio_to_speaker",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["play"], "default": "play"},
                    "channels": {"type": "integer", "default": 1},
                    "sample_rate": {"type": "integer", "default": 16000},
                    "format": {"type": "integer", "default": 0},
                    "duration_ms": {"type": "integer", "default": 10},
                    "audio_data": {
                        "type": "array", "items": {"type": "integer"},
                        "description": "int16 采样列表",
                    },
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        if action in ("start", "stop"):
            return super().dispatch(action, args)
        try:
            from mediacontrol_py import AudioStream
            stream = AudioStream()
            stream.channels = int(args.get("channels", 1))
            stream.sample_rate = int(args.get("sample_rate", 16000))
            stream.format = int(args.get("format", 0))
            stream.duration_ms = int(args.get("duration_ms", 10))
            stream.audio_data = args.get("audio_data", [])
            self._media().publish_external_audio_playback_stream(stream)
            return {"state": "playing"}
        except Exception as e:
            return {"error": str(e)}


class ExternalVideoInputPlugin(_MediaActuatorBase):
    """外部视频注入 AI"""

    def get_tool(self):
        return {
            "name": "external_video_input",
            "type": "actuator",
            "description": "Bumi 外部视频注入 AI — 需先通过 audio_routing 开启 ext_video_to_ai",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["publish"], "default": "publish"},
                    "width": {"type": "integer", "default": 640},
                    "height": {"type": "integer", "default": 480},
                    "format": {"type": "integer", "default": 0},
                    "fps": {"type": "integer", "default": 30},
                    "video_data": {
                        "type": "array", "items": {"type": "integer"},
                        "description": "uint8 视频数据列表",
                    },
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        if action in ("start", "stop"):
            return super().dispatch(action, args)
        try:
            from mediacontrol_py import VideoStream
            stream = VideoStream()
            stream.width = int(args.get("width", 640))
            stream.height = int(args.get("height", 480))
            stream.format = int(args.get("format", 0))
            stream.fps = int(args.get("fps", 30))
            stream.video_data = args.get("video_data", [])
            self._media().publish_external_video_stream(stream)
            return {"state": "published"}
        except Exception as e:
            return {"error": str(e)}
